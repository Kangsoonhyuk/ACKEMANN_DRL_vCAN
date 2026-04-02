cmake_minimum_required(VERSION 3.5)
project(turtlebot3_gazebo)

find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${GAZEBO_INCLUDE_DIRS})

# obstacle plugins (used by stage4 / stage5_up worlds)
add_library(obstacle1 SHARED models/turtlebot3_drl_world/obstacle_plugin/obstacle1.cc)
target_link_libraries(obstacle1 ${GAZEBO_LIBRARIES})

add_library(obstacle2 SHARED models/turtlebot3_drl_world/obstacle_plugin/obstacle2.cc)
target_link_libraries(obstacle2 ${GAZEBO_LIBRARIES})

add_library(obstacle3 SHARED models/turtlebot3_drl_world/obstacle_plugin/obstacle3.cc)
target_link_libraries(obstacle3 ${GAZEBO_LIBRARIES})

add_library(obstacle4 SHARED models/turtlebot3_drl_world/obstacle_plugin/obstacle4.cc)
target_link_libraries(obstacle4 ${GAZEBO_LIBRARIES})

add_library(obstacles SHARED models/turtlebot3_drl_world/obstacle_plugin/obstacles.cc)
target_link_libraries(obstacles ${GAZEBO_LIBRARIES})

install(TARGETS obstacle1 obstacle2 obstacle3 obstacle4 obstacles
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch models worlds
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(gazebo_ros_pkgs)
ament_package()
