cmake_minimum_required(VERSION 3.5)
project(mpc_rbt_student)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(mpc_rbt_simulator REQUIRED)
find_package(std_srvs REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(behaviortree_cpp REQUIRED)

include_directories(PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

set(dependencies
        rclcpp
        rclcpp_action
        sensor_msgs
        nav_msgs
        nav2_msgs
        tf2
        tf2_ros
        tf2_geometry_msgs
        mpc_rbt_simulator
)

add_library(${PROJECT_NAME} SHARED src/KeyboardControl.cpp src/Localization.cpp src/Planning.cpp src/MotionControl.cpp)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

# Uzly pro navigaci (Lab 05-07)
foreach(node localization planning motion_control)
    add_executable(${node}_node src/${node}_node.cpp)
    target_link_libraries(${node}_node ${PROJECT_NAME})
    install(TARGETS ${node}_node DESTINATION lib/${PROJECT_NAME})
endforeach()

# Skladník a Server (Lab 08)
add_executable(warehouse_manager src/bt/warehouse_manager_node.cpp src/bt/WarehouseManager.cpp)
ament_target_dependencies(warehouse_manager rclcpp std_srvs)
install(TARGETS warehouse_manager DESTINATION lib/${PROJECT_NAME})

add_executable(bt_server src/bt/bt_server.cpp)
ament_target_dependencies(bt_server rclcpp behaviortree_ros2 behaviortree_cpp)
install(TARGETS bt_server DESTINATION lib/${PROJECT_NAME})

# PLUGINY: do share/package/bt_plugins (BT server hledá přes ament_index)
foreach(plugin NavigateToPoseAction GetTaskService ConfirmLoadingService GetDropoffService LookupPose)
    add_library(${plugin} SHARED src/bt/${plugin}.cpp)
    ament_target_dependencies(${plugin} rclcpp behaviortree_ros2 behaviortree_cpp nav2_msgs std_srvs)
    target_compile_definitions(${plugin} PRIVATE BT_PLUGIN_EXPORT)
    install(TARGETS ${plugin} LIBRARY DESTINATION share/${PROJECT_NAME}/bt_plugins)
endforeach()

install(DIRECTORY launch rviz config behavior_trees DESTINATION share/${PROJECT_NAME}/)

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_export_libraries(${PROJECT_NAME})
ament_package()
