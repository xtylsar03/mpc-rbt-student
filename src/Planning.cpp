#include "Planning.hpp"


// 1.mapcallback
// 2.dilatemap
// 3.planpath
// 4.astar
// 5.smoothpath

PlanningNode::PlanningNode() :
    rclcpp::Node("planning_node") {

        // Client for map
        map_client_ = this->create_client<nav_msgs::srv::GetMap>("/map_server/map");

        // Service for path
        plan_service_ = this->create_service<nav_msgs::srv::GetPlan>(
            "/plan_path",
            std::bind(&PlanningNode::planPath, this, std::placeholders::_1, std::placeholders::_2)
        );
        
        // Publisher for path
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);

        RCLCPP_INFO(get_logger(), "Planning node started.");

        // Connect to map server
        while (!map_client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the map service. Exiting.");
                return;
            }
            RCLCPP_INFO(get_logger(), "Waiting for map server...");
        }

        // Request map
        auto request = std::make_shared<nav_msgs::srv::GetMap::Request>();
        map_client_->async_send_request(request, std::bind(&PlanningNode::mapCallback, this, std::placeholders::_1));
        
        RCLCPP_INFO(get_logger(), "Trying to fetch map...");
    }

void PlanningNode::mapCallback(rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture future) {
    auto response = future.get();
    if (response) {
        map_ = response->map;
        RCLCPP_INFO(get_logger(), "Map successfully loaded! Width: %d, Height: %d", 
                    map_.info.width, map_.info.height);
        
        
        dilateMap(); 
    }
}

void PlanningNode::planPath(const std::shared_ptr<nav_msgs::srv::GetPlan::Request> request, 
                            std::shared_ptr<nav_msgs::srv::GetPlan::Response> response) {
    RCLCPP_INFO(get_logger(), "Received a plan request!");

    // 1. Spustíme A* (vytvoří zubatou cestu v proměnné path_)
    aStar(request->start, request->goal);

    // 2. Vyhladíme cestu (upraví body v path_ tak, aby čára nebyla zubatá)
    
    smoothPath(); 

    // 3. Naplníme odpověď pro klienta (to, co se vypíše v terminálu 3)
    response->plan = path_;
    
    // 4. Publikujeme cestu pro RViz (to, co uvidíš jako čáru)
    path_pub_->publish(path_);
}



void PlanningNode::dilateMap() {
    RCLCPP_INFO(get_logger(), "Dilating map for safety distance...");
    
    // Vytvoříme kopii, abychom neměnili mapu pod rukama během čtení
    nav_msgs::msg::OccupancyGrid dilatedMap = map_;
    
    // Velikost nafouknutí (4 buňky při 0.05m rozlišení = 20 cm rezerva)
    int dilation_size = 6; 

    for (int y = 0; y < (int)map_.info.height; ++y) {
        for (int x = 0; x < (int)map_.info.width; ++x) {
            
            // Pokud je v původní mapě zeď (> 50)
            if (map_.data[y * map_.info.width + x] > 50) {
                
                // Projdi okolní čtverec
                for (int dy = -dilation_size; dy <= dilation_size; ++dy) {
                    for (int dx = -dilation_size; dx <= dilation_size; ++dx) {
                        int nx = x + dx;
                        int ny = y + dy;

                        // Kontrola mezí mapy
                        if (nx >= 0 && nx < (int)map_.info.width && ny >= 0 && ny < (int)map_.info.height) {
                            dilatedMap.data[ny * map_.info.width + nx] = 100;
                        }
                    }
                }
            }
        }
    }
    map_ = dilatedMap;
    RCLCPP_INFO(get_logger(), "Dilation finished.");
}

void PlanningNode::aStar(const geometry_msgs::msg::PoseStamped &start, const geometry_msgs::msg::PoseStamped &goal) {
    RCLCPP_INFO(get_logger(), "Starting A* algorithm...");

    if (map_.info.resolution == 0.0) {
        RCLCPP_ERROR(get_logger(), "Map resolution is 0! Cannot plan path.");
        return;
    }

    // 1. PŘEVOD SOUŘADNIC Z METRŮ NA INDEXY V MAPĚ
    int start_x = (start.pose.position.x - map_.info.origin.position.x) / map_.info.resolution;
    int start_y = (start.pose.position.y - map_.info.origin.position.y) / map_.info.resolution;
    int goal_x = (goal.pose.position.x - map_.info.origin.position.x) / map_.info.resolution;
    int goal_y = (goal.pose.position.y - map_.info.origin.position.y) / map_.info.resolution;

    Cell cStart(start_x, start_y);
    Cell cGoal(goal_x, goal_y);

    RCLCPP_INFO(get_logger(), "Start grid: [%d, %d], Goal grid: [%d, %d]", start_x, start_y, goal_x, goal_y);

    // Vyčištění předchozí cesty a nastavení hlavičky
    path_.poses.clear();
    path_.header.frame_id = "map";
    path_.header.stamp = this->get_clock()->now();

    // Pokud je start a cíl na stejném místě, rovnou končíme
    if (start_x == goal_x && start_y == goal_y) {
        RCLCPP_INFO(get_logger(), "Start and Goal are the same. Path found!");
        return;
    }

    // 2. INICIALIZACE STRUKTUR
    std::vector<std::shared_ptr<Cell>> openList;
    std::vector<bool> closedList(map_.info.height * map_.info.width, false);

    // Pomocná funkce (lambda) pro převod 2D indexu (x,y) na 1D index pole
    auto getIndex = [&](int x, int y) { return y * map_.info.width + x; };

    openList.push_back(std::make_shared<Cell>(cStart));

    // Směry pohybu (8-okolí)
    std::vector<std::pair<int, int>> directions = {
        {0, 1}, {1, 0}, {0, -1}, {-1, 0},   // Kříž (cena 1.0)
        {1, 1}, {1, -1}, {-1, 1}, {-1, -1}  // Diagonály (cena 1.414)
    };

    // 3. HLAVNÍ SMYČKA ALGORITMU
    while(!openList.empty() && rclcpp::ok()) {
        
        // a) Najdi uzel s nejmenší celkovou cenou 'f' v openListu
        auto current_it = openList.begin();
        for (auto it = openList.begin(); it != openList.end(); ++it) {
            if ((*it)->f < (*current_it)->f) {
            	// oznacime vybrany uzel jako aktual aktualni
                current_it = it;
            }
        }
        std::shared_ptr<Cell> current = *current_it;

        // b) Přesuň ho z openListu do closedListu
        openList.erase(current_it);
        closedList[getIndex(current->x, current->y)] = true;

        // c) Našli jsme cíl? -> Zpětná rekonstrukce cesty
        if (current->x == goal_x && current->y == goal_y) {
            RCLCPP_INFO(get_logger(), "Path found! Reconstructing...");
            
            auto curr = current;
            while (curr != nullptr) {
                geometry_msgs::msg::PoseStamped pose;
                pose.header = path_.header;
                
                // Převod indexů (pixely) zpět na reálné metry
                pose.pose.position.x = curr->x * map_.info.resolution + map_.info.origin.position.x;
                pose.pose.position.y = curr->y * map_.info.resolution + map_.info.origin.position.y;
                pose.pose.position.z = 0.0;
                pose.pose.orientation.w = 1.0;
                
                path_.poses.push_back(pose);
                curr = curr->parent;
            }
            
            // Cestu jsme poskládali od cíle ke startu, takže ji musíme otočit
            std::reverse(path_.poses.begin(), path_.poses.end());
            RCLCPP_INFO(get_logger(), "Path successfully generated with %zu waypoints.", path_.poses.size());
            return;
        }

        // d) Prozkoumej všechny sousedy aktuální buňky
        for (auto dir : directions) {
            int nx = current->x + dir.first;
            int ny = current->y + dir.second;

            // Kontrola, jestli nejsme mimo mapu
            if (nx < 0 || nx >= (int)map_.info.width || ny < 0 || ny >= (int)map_.info.height) continue;

            int n_idx = getIndex(nx, ny);

            // Kontrola closedListu a překážek (hodnoty blížící se 100 jsou zdi, použijeme > 50 jako práh)
            if (closedList[n_idx] || map_.data[n_idx] > 50) continue;

            // Výpočet cen (g = cena od startu sem, h = odhad k cíli)
            float move_cost = (dir.first == 0 || dir.second == 0) ? 1.0 : 1.414; 
            float g_new = current->g + move_cost;
            float h_new = std::sqrt(std::pow(goal_x - nx, 2) + std::pow(goal_y - ny, 2)); // Euklidovská vzdálenost
            float f_new = g_new + h_new;

            // Je už tento soused v openListu s lepší nebo stejnou cenou?
            bool in_open = false;
            for (auto node : openList) {
                if (node->x == nx && node->y == ny) {
                    in_open = true;
                    // Pokud jsme našli lepší cestu k tomuto bodu, zaktualizujeme ho
                    if (g_new < node->g) {
                        node->g = g_new;
                        node->f = f_new;
                        node->parent = current;
                    }
                    break;
                }
            }

            // Pokud není v openListu vůbec, přidáme ho
            if (!in_open) {
                auto neighbor = std::make_shared<Cell>(nx, ny);
                neighbor->g = g_new;
                neighbor->h = h_new;
                neighbor->f = f_new;
                neighbor->parent = current;
                openList.push_back(neighbor);
            }
        }
    }

    RCLCPP_ERROR(get_logger(), "Unable to plan path. Target is unreachable.");
}

void PlanningNode::smoothPath() {
    // Pokud máme jen 2 body (start a cíl), není co vyhlazovat
    if (path_.poses.size() < 3) return;

    RCLCPP_INFO(get_logger(), "Smoothing path...");
    std::vector<geometry_msgs::msg::PoseStamped> newPath = path_.poses;
    
    float weight_data = 0.5;   // Jak moc se držet původní trasy
    float weight_smooth = 0.2; // Jak moc vyhlazovat
    float tolerance = 0.001;   // Kdy zastavit (přesnost)
    float change = tolerance;
    int max_iterations = 1000;
    int iter = 0;

    while (change >= tolerance && iter < max_iterations) {
        change = 0.0;
        // Vyhlazujeme všechny body kromě prvního (start) a posledního (cíl)
        for (size_t i = 1; i < path_.poses.size() - 1; ++i) {
            float old_x = newPath[i].pose.position.x;
            float old_y = newPath[i].pose.position.y;

            // Gradientní úprava pozice
            newPath[i].pose.position.x += weight_data * (path_.poses[i].pose.position.x - newPath[i].pose.position.x) +
                                         weight_smooth * (newPath[i-1].pose.position.x + newPath[i+1].pose.position.x - 2.0 * newPath[i].pose.position.x);
            newPath[i].pose.position.y += weight_data * (path_.poses[i].pose.position.y - newPath[i].pose.position.y) +
                                         weight_smooth * (newPath[i-1].pose.position.y + newPath[i+1].pose.position.y - 2.0 * newPath[i].pose.position.y);

            change += std::abs(old_x - newPath[i].pose.position.x);
            change += std::abs(old_y - newPath[i].pose.position.y);
        }
        iter++;
    }
    path_.poses = newPath;
    RCLCPP_INFO(get_logger(), "Path smoothed in %d iterations.", iter);
}

Cell::Cell(int c, int r) : x(c), y(r), f(0.0), g(0.0), h(0.0), parent(nullptr) {
    // Pomocná struktura pro A* (cena f, g, h a ukazatel na předka)
}
