cmake_minimum_required(VERSION 3.0.2)
project(dynarm)

find_package(catkin REQUIRED COMPONENTS
  aerial_robot_control
  aerial_robot_dynamics
  aerial_robot_estimation
  aerial_robot_model
  aerial_robot_msgs
  motion_planning
  pluginlib
  roscpp
  sensor_msgs
  spinal
)

find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(crocoddyl REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(NLopt REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES manipulator_controller manipulator_model
 CATKIN_DEPENDS aerial_robot_control aerial_robot_dynamics aerial_robot_estimation aerial_robot_model aerial_robot_msgs motion_planning pluginlib roscpp sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OsqpEigen_INCLUDE_DIRS}
  ${pinocchio_INCLUDE_DIRS}
  ${NLOPT_INCLUDE_DIRS}
  )

add_library(nonlinear_inverse_dynamics
  src/model/nonlinear_inverse_dynamics.cpp)
target_link_libraries(nonlinear_inverse_dynamics
  ${catkin_LIBRARIES}
  pinocchio::pinocchio
  ${Eigen3_LIBRARIES}
  ${OsqpEigen_LIBRARIES}
  ${NLOPT_LIBRARIES}
  )

add_library(joint_trajectory_generator
  src/control/joint_trajectory_generator.cpp)
target_link_libraries(joint_trajectory_generator
  ${catkin_LIBRARIES}
  pinocchio::pinocchio
  ${Eigen3_LIBRARIES}
  ${OsqpEigen_LIBRARIES}
  )

add_library(trajectory_generator
  src/control/trajectory_generator.cpp)
target_link_libraries(trajectory_generator
  joint_trajectory_generator
  nonlinear_inverse_dynamics
  ${catkin_LIBRARIES}
  pinocchio::pinocchio
  ${Eigen3_LIBRARIES}
  ${OsqpEigen_LIBRARIES}
  ${NLOPT_LIBRARIES}
  )
add_executable(trajectory_generator_test
  src/control/trajectory_generator.cpp)
target_link_libraries(trajectory_generator_test
  trajectory_generator
  nonlinear_inverse_dynamics
  ${catkin_LIBRARIES}
  pinocchio::pinocchio
  ${Eigen3_LIBRARIES}
  ${OsqpEigen_LIBRARIES}
  ${NLOPT_LIBRARIES}
  )

add_library(manipulator_model
  src/model/manipulator_model.cpp)
target_link_libraries(manipulator_model
  ${catkin_LIBRARIES}
  pinocchio::pinocchio
  ${Eigen3_LIBRARIES}
  ${OsqpEigen_LIBRARIES}
  ${NLOPT_LIBRARIES}
  )

add_library(ddp_hovering_problem
  src/control/ddp_hovering_problem.cpp)
target_link_libraries(ddp_hovering_problem
  ${catkin_LIBRARIES}
  pinocchio::pinocchio
  crocoddyl::crocoddyl
  ${Eigen3_LIBRARIES}
  ${OsqpEigen_LIBRARIES}
  )

add_executable(ddp_hovering_problem_test
  src/control/ddp_hovering_problem.cpp)
target_link_libraries(ddp_hovering_problem_test
  ddp_hovering_problem
  ${catkin_LIBRARIES}
  pinocchio::pinocchio
  crocoddyl::crocoddyl
  ${Eigen3_LIBRARIES}
  ${OsqpEigen_LIBRARIES}
  )

add_library(dynarm_controllib
  src/control/fullbody_flight_controller.cpp
  src/control/manipulator_controller.cpp)
target_link_libraries(dynarm_controllib
  ddp_hovering_problem
  joint_trajectory_generator
  manipulator_model
  nonlinear_inverse_dynamics
  trajectory_generator
  ${catkin_LIBRARIES}
  pinocchio::pinocchio
  ${Eigen3_LIBRARIES}
  ${OsqpEigen_LIBRARIES}
  )
