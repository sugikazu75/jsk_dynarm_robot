#pragma once

#include <dynarm/model/manipulator_model.h>

#include <aerial_robot_control/control/base/base.h>
#include <motion_planning/position_trajectory_generator.hpp>
#include <motion_planning/joint_trajectory_generator.hpp>

#include <ros/ros.h>
#include <Eigen/Core>

#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/JointState.h>
#include <spinal/FourAxisCommand.h>
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
  ros::Publisher four_axis_command_pub_;
  ros::Publisher joints_control_pub_;
  ros::Publisher gimbals_control_pub_;
  ros::Publisher is_transforming_pub_;
  ros::Publisher id_torque_pub_;
  ros::Publisher id_velocity_pub_;
  ros::Publisher id_acc_pub_;
  ros::Publisher id_time_pub_;
  ros::Publisher tau_by_thrust_pub_;
  ros::Publisher rnea_solution_pub_;
  ros::Publisher rotor_wrench_pub_;
  ros::Publisher target_end_effector_pos_pub_;
  ros::Publisher target_end_effector_vel_pub_;

  ros::Subscriber joint_state_sub_;
  ros::Subscriber target_end_effector_final_pos_sub_;

  std::string robot_ns_;

  sensor_msgs::JointState joint_state_;
  Eigen::Vector3d target_ee_pos_;
  Eigen::Vector3d target_ee_vel_;
  Eigen::VectorXd curr_q_;
  Eigen::VectorXd curr_target_q_;
  Eigen::VectorXd curr_dq_;
  Eigen::VectorXd curr_target_dq_;
  Eigen::VectorXd curr_target_ddq_;
  Eigen::VectorXd curr_tau_;
  Eigen::VectorXd curr_target_tau_;
  Eigen::VectorXd thrusts_;
  Eigen::VectorXd rnea_solution_;

  // debug
  int rotor_wrench_pub_index_;

  // init
  bool first_run_;
  bool is_initialized_;
  Eigen::VectorXd init_target_q_;

  // manipulator param
  motion_planning::PositionTrajectoryGenerator pos_trajectory_generator_;
  double transform_duration_;
  double transform_start_time_;
  double transform_end_time_;
  std::string end_effector_name_;
  bool is_transforming_;

  boost::shared_ptr<aerial_robot_model::ManipulatorRobotModel> dragon_arm_robot_model_;
  std::shared_ptr<aerial_robot_dynamics::PinocchioRobotModel> pinocchio_robot_model_;
  std::shared_ptr<pinocchio::Model> pinocchio_model_;
  std::shared_ptr<pinocchio::Data> pinocchio_data_;

  virtual bool update() override;
  virtual void reset() override;
  void rosParamInit();
  void controlCore();
  void sendCmd();
  void sendFourAxisCommand();
  void sendJointCommand();
  void sendGimbalCommand();

  void jointStateCallback(const sensor_msgs::JointState msg);
  void targetEndEffectorPosCallback(const geometry_msgs::Vector3StampedConstPtr& msg);
};
}  // namespace aerial_robot_control
