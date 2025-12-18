#pragma once

#include <dynarm/model/manipulator_model.h>
#include <dynarm/control/joint_trajectory_generator.h>
#include <dynarm/control/trajectory_generator.h>

#include <aerial_robot_control/control/base/base.h>
#include <motion_planning/position_trajectory_generator.hpp>
#include <motion_planning/joint_trajectory_generator.hpp>

#include <ros/ros.h>
#include <Eigen/Core>

#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/JointState.h>
#include <spinal/FourAxisCommand.h>
#include <spinal/ServoTorqueCmd.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

namespace aerial_robot_control
{
class ManipulatorController : public ControlBase
{
public:
  ManipulatorController();
  virtual ~ManipulatorController() = default;

  void virtual initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                          boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                          boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                          boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator, double ctrl_loop_du);

private:
  ros::Publisher four_axis_command_pub_;        // for spinal
  ros::Publisher joints_control_pub_;           // for servo bridge
  ros::Publisher dynamixel_torque_enable_pub_;  // for spinal
  ros::Publisher gimbals_control_pub_;          // for servo bridge
  ros::Publisher robstride_servo_on_pub_;       // for robstride bridge

  ros::Publisher tau_by_thrust_pub_;  // for debug. in real q and lambda
  ros::Publisher rnea_solution_pub_;  // for debug. in real q, dq, and target ddq

  // robot parameter
  std::vector<std::string> joint_names_;
  std::vector<std::string> gimbal_names_;
  std::map<std::string, int> gimbal_index_map_;

  boost::shared_ptr<aerial_robot_model::ManipulatorRobotModel> dragon_arm_robot_model_;
  std::shared_ptr<aerial_robot_dynamics::PinocchioRobotModel> pinocchio_robot_model_;
  std::shared_ptr<pinocchio::Model> pinocchio_model_;
  std::shared_ptr<pinocchio::Data> pinocchio_data_;

  std::shared_ptr<JointTrajectoryGeneratorRos> joint_trajectory_generator_ros_;
  std::shared_ptr<JointTrajectoryGenerator> joint_trajectory_generator_;

  std::shared_ptr<aerial_robot_model::NonlinearInverseDynamicsRos> nonlinear_inverse_dynamics_solver_ros_;
  std::shared_ptr<aerial_robot_model::NonlinearInverseDynamics> nonlinear_inverse_dynamics_solver_;

  std::shared_ptr<TrajectoryGeneratorRos> trajectory_generator_ros_;
  std::shared_ptr<TrajectoryGenerator> trajectory_generator_;

  Eigen::VectorXd rnea_solution_;

  virtual bool update() override;
  virtual void reset() override;
  virtual void activate() override;
  void rosParamInit();
  void loadJointNames();
  void loadGimbalNames();
  void controlCore();
  void sendCmd();
  void sendFourAxisCommand();
  void sendJointCommand();
  void sendGimbalCommand();
};
}  // namespace aerial_robot_control
