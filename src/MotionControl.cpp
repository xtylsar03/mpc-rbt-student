#include "mpc_rbt_simulator/RobotConfig.hpp"
#include "MotionControl.hpp"
#include <tf2/utils.h>
#include <cmath>
#include <thread>


//KONSTRUKTOR
//dá ROSu vedet, že ma jmeno motion_control_node, bude v ros node list
MotionControlNode::MotionControlNode() : rclcpp::Node("motion_control_node") {
    //chceme odebirat z /odom, když bude nova zprava tak zavola odomCallback
    odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&MotionControlNode::odomCallback, this, std::placeholders::_1));
    //zapinani lidaru, data z laseru do funkce lidarCallback
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&MotionControlNode::lidarCallback, this, std::placeholders::_1));
    //publicher pro cmd_vel (ovladani
    twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    //tomuto uzlu se budou posilat souradnice
    plan_client_ = this->create_client<nav_msgs::srv::GetPlan>("/plan_path");
    //naslouchani go_to_goal
    nav_server_ = rclcpp_action::create_server<nav2_msgs::action::NavigateToPose>(
        this, "/go_to_goal",
        std::bind(&MotionControlNode::navHandleGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&MotionControlNode::navHandleCancel, this, std::placeholders::_1),
        std::bind(&MotionControlNode::navHandleAccepted, this, std::placeholders::_1));
    //připojovani k planovaci
    while (!plan_client_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) return;
        RCLCPP_INFO(get_logger(), "Čekám na plánovač...");
    }
    RCLCPP_INFO(get_logger(), "Motion control připraven k jízdě.");
}



void MotionControlNode::checkCollision() {
    //když nejsou data z lidaru, a neni cil tak nemusime hlidat kolize
    if (laser_scan_.ranges.empty() || !goal_handle_ || !goal_handle_->is_active()) return;

    //bezpecna vzdalenost, cokoliv co je bliz je "hrozba"
    double thresh = 0.55;
    //paprsky z lidaru
    int total = laser_scan_.ranges.size();
    int center = total / 2;

    // Rozšíř zorné pole: přibližně ±60°
    int search_range = total / 3;

    //skenovani prostoru, prochazi se paprsky
    for (int i = center - search_range; i <= center + search_range; ++i) {
        //pojistka kdyby paprsek byl mimo rozsah, treba zaporny
        if (i < 0 || i >= total) continue;
        //detekce prekazky, pokud jsme menci nez treshold a vzdalenost je realne cislo
        if (std::isfinite(laser_scan_.ranges[i]) && laser_scan_.ranges[i] < thresh) {
            //zastavime robota, vyrovani do terminalu a rychlost 0
            RCLCPP_WARN(get_logger(), "Překážka %.2fm, STOP!", laser_scan_.ranges[i]);
            geometry_msgs::msg::Twist stop;
            twist_publisher_->publish(stop);
            //zruseni ukolu, kdyz cil nebyl splneny
            auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();
            if (goal_handle_->is_executing()) {
                goal_handle_->abort(result);
            }
            return;
        }
    }
}



void MotionControlNode::updateTwist() {
    //Kdyz neni ukol nebo ukol nebezi nebo neni trasa z planovace tak koncime
    if (!goal_handle_ || !goal_handle_->is_executing() || path_.poses.empty()) return;
    //kde se nachazime z odometrie
    double rx = current_pose_.pose.position.x;
    double ry = current_pose_.pose.position.y;
    double yaw = tf2::getYaw(current_pose_.pose.orientation);
    //parametry jizdy
    double lookahead = 0.4;
    double v_max = 0.15;
    double w_max = 0.8;
    //podivame se na lidar a kdyz je krabice tak zacneme brzdit, kdyj je krabice v tresh tak rychlost k nule
    if (!laser_scan_.ranges.empty()) {
        int total = laser_scan_.ranges.size();
        int center = total / 2;
        int range = total / 6;
        double min_dist = std::numeric_limits<double>::max();
        for (int i = center - range; i <= center + range; ++i) {
            if (i >= 0 && i < total && std::isfinite(laser_scan_.ranges[i]))
                min_dist = std::min(min_dist, (double)laser_scan_.ranges[i]);
        }
        // Zpomal lineárně: plná rychlost > 1.0m, nulová < 0.55m
        if (min_dist < 1.0) {
            //interpolace od 1 do 0.55
            double factor = std::clamp((min_dist - 0.55) / (1.0 - 0.55), 0.0, 1.0);
            v_max *= factor;
        }
    }

    // Odstraň waypointy, které už robot minul
    while (path_.poses.size() > 1) {
        double d = std::hypot(path_.poses.front().pose.position.x - rx,
                              path_.poses.front().pose.position.y - ry);
        if (d < lookahead) path_.poses.erase(path_.poses.begin());
        else break;
    }

    //najdeme za care prvni bod ktery je dal nez 0,4 metru. To bude nas cil
    geometry_msgs::msg::Pose target = path_.poses.back().pose;
    for (const auto& wp : path_.poses) {
        double d = std::hypot(wp.pose.position.x - rx, wp.pose.position.y - ry);
        if (d >= lookahead) { target = wp.pose; break; }
    }
    //vypocet uhlu k cili, spocitame vektor k cili a uhel alpha, o ktery se musime otocit abychom mirili na ten bod, nastavime to tak aby byl vzdy od -180 do 180, regulacni odchylka error
    double dx = target.position.x - rx;
    double dy = target.position.y - ry;
    double alpha = std::atan2(dy, dx) - yaw;
    while (alpha >  M_PI) alpha -= 2.0 * M_PI;
    while (alpha < -M_PI) alpha += 2.0 * M_PI;

    geometry_msgs::msg::Twist twist;

    //pokud cil lezi vic jak 45 stupnu tak stojim a jen se otacime na miste
    double alpha_thresh = M_PI / 4.0;
    if (std::abs(alpha) > alpha_thresh) {
        twist.linear.x = 0.0;
    } else {
        //v podstate P regulator
        twist.linear.x = v_max * (1.0 - std::abs(alpha) / alpha_thresh);
    }
    //jak rychle se budeme otacet, vypocet uhlove rychlosti podle vzorce
    twist.angular.z = std::clamp((2.0 * v_max * std::sin(alpha)) / lookahead, -w_max, w_max);
    //povel
    twist_publisher_->publish(twist);
}
//kdyz neni trasa od planovace, vime kam jet ale potrebujeme mapu cesty
rclcpp_action::GoalResponse MotionControlNode::navHandleGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const nav2_msgs::action::NavigateToPose::Goal>) {
    return rclcpp_action::GoalResponse::ACCEPT_AND_DEFER;
}
//kdyz manualne zrusime
rclcpp_action::CancelResponse MotionControlNode::navHandleCancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>>) {
    return rclcpp_action::CancelResponse::ACCEPT;
}

void MotionControlNode::navHandleAccepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle) {
    //ukazatel na ukol
    goal_handle_ = goal_handle;
    //formular pro planovac
    auto request = std::make_shared<nav_msgs::srv::GetPlan::Request>();
    //zacni planovat tam kde stojim z odometrie
    request->start = current_pose_;
    //souradnice cile
    request->goal = goal_handle->get_goal()->pose;
    //posleme to planovaci
    plan_client_->async_send_request(request,
        std::bind(&MotionControlNode::pathCallback, this, std::placeholders::_1));
}

void MotionControlNode::execute() {
    //rychlost kontroly
    rclcpp::Rate loop_rate(10.0);
    //dokud ros bezi mame platny ukol a je aktivni
    while (rclcpp::ok() && goal_handle_ && goal_handle_->is_active()) {
        //kdyz uzivatel zrusil tak skoncime
        if (goal_handle_->is_canceling()) return;
	//vypocet prime vzdalenosti k cili
        double dist = std::hypot(
            goal_handle_->get_goal()->pose.pose.position.x - current_pose_.pose.position.x,
            goal_handle_->get_goal()->pose.pose.position.y - current_pose_.pose.position.y);
	//jsme v cili?
        if (dist < 0.2) {
            //info pro motory ze jsme v cili
            geometry_msgs::msg::Twist stop;
            twist_publisher_->publish(stop);
            //uzavreme ukol v rosu jako success
            goal_handle_->succeed(std::make_shared<nav2_msgs::action::NavigateToPose::Result>());
            RCLCPP_INFO(get_logger(), "Cíle úspěšně dosaženo!");
            return;
        }
        loop_rate.sleep();
    }
}
//spusti se kdyz pozadavek na planovac z navHandleAccepted
void MotionControlNode::pathCallback(
    rclcpp::Client<nav_msgs::srv::GetPlan>::SharedFuture future) {
    //vysledek od planovace
    auto res = future.get();
    //dostali jsme odpoved? a obsauje body cesty?
    if (res && !res->plan.poses.empty()) {
        //ulozime si celou caru
        path_ = res->plan;
        RCLCPP_INFO(get_logger(), "Trasa přijata, vyrážím.");
        //prepneme stav na executing
        goal_handle_->execute();
        //spustime execute funkci (detach = samostatne)
        std::thread(&MotionControlNode::execute, this).detach();
        //pokud planovac cestu nenasel tak abort
    } else {
        RCLCPP_ERROR(get_logger(), "Chyba: Plánovač nenašel cestu!");
        goal_handle_->execute();
        auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();
        goal_handle_->abort(result);
    }
}
//kdyz robot dostane info o svoji poloze
void MotionControlNode::odomCallback(const nav_msgs::msg::Odometry & msg) {
    
    current_pose_.header = msg.header;
    //ulozi svoji pozici
    current_pose_.pose = msg.pose.pose;
    //zkontrolujeme kolize
    checkCollision();
    //pokud je aktivni ukol a jedeme tak updatetwist
    if (goal_handle_ && goal_handle_->is_executing()) updateTwist();
}
//vezmeme zpravu z lidaru a ulozime ji do pameti uzlu
void MotionControlNode::lidarCallback(const sensor_msgs::msg::LaserScan & msg) {
    laser_scan_ = msg;
}
