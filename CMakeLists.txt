cmake_minimum_required(VERSION 3.0.2)
project(dynarm)

find_package(catkin REQUIRED COMPONENTS
  aerial_robot_control
  aerial_robot_dynamics
  aerial_robot_estimation
  aerial_robot_model
  aerial_robot_msgs
  dragon
  motion_planning
  pluginlib
  roscpp
  sensor_msgs
  spinal
)

find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(OsqpEigen REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES manipulator_controller manipulator_model
 CATKIN_DEPENDS aerial_robot_control aerial_robot_dynamics aerial_robot_estimation aerial_robot_model aerial_robot_msgs dragon motion_planning pluginlib roscpp sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OsqpEigen_INCLUDE_DIRS}
  ${pinocchio_INCLUDE_DIRS}
)

add_library(manipulator_model src/model/manipulator_model.cpp)
target_link_libraries(manipulator_model ${catkin_LIBRARIES} ${Eigen3_LIBRARIES} ${OsqpEigen_LIBRARIES})

add_library(manipulator_controller src/control/manipulator_controller.cpp)
target_link_libraries(manipulator_controller manipulator_model ${catkin_LIBRARIES} ${Eigen3_LIBRARIES} ${OsqpEigen_LIBRARIES})
