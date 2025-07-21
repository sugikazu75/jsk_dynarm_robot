#pragma once

#include <dynarm/model/manipulator_model.h>
#include <dynarm/control/joint_trajectory_generator.h>

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

  std::shared_ptr<aerial_robot_dynamics::PinocchioRobotModel> getPinocchioRobotModel()
  {
    return pinocchio_robot_model_;
  }
  std::shared_ptr<pinocchio::Model> getPinocchioModel()
  {
    return pinocchio_model_;
  }
  std::shared_ptr<pinocchio::Data> getPinocchioData()
  {
    return pinocchio_data_;
  }
  const int getRotorDevider()
  {
    return rotor_devider_;
  }

private:
  ros::Publisher four_axis_command_pub_;
  ros::Publisher joints_control_pub_;
  ros::Publisher gimbals_control_pub_;
  ros::Publisher tau_by_thrust_pub_;
  ros::Publisher rnea_solution_pub_;

  // robot parameter
  int rotor_devider_;
  std::vector<std::string> joint_names_;

  // init
  bool first_run_;
  bool is_initialized_;
  Eigen::VectorXd init_target_q_;

  boost::shared_ptr<aerial_robot_model::ManipulatorRobotModel> dragon_arm_robot_model_;
  std::shared_ptr<aerial_robot_dynamics::PinocchioRobotModel> pinocchio_robot_model_;
  std::shared_ptr<pinocchio::Model> pinocchio_model_;
  std::shared_ptr<pinocchio::Data> pinocchio_data_;
  std::shared_ptr<jointTrajectoryGenerator> joint_trajectory_generator_;

  virtual bool update() override;
  virtual void reset() override;
  void rosParamInit();
  void getJointNames();
  void controlCore();
  void sendCmd();
  void sendFourAxisCommand();
  void sendJointCommand();
  void sendGimbalCommand();
};
}  // namespace aerial_robot_control
