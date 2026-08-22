#pragma once

#include <dynarm/model/manipulator_model.h>
#include <dynarm/model/nonlinear_inverse_dynamics.h>
#include <dynarm/utils/gimbal_util.h>

#include <aerial_robot_control/control/base/base.h>

#include <ros/ros.h>
#include <Eigen/Core>

#include <sensor_msgs/JointState.h>
#include <spinal/FourAxisCommand.h>

namespace aerial_robot_control
{
class WholeBodyForceController : public ControlBase
{
public:
  WholeBodyForceController();
  virtual ~WholeBodyForceController() = default;

  void virtual initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                          boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                          boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                          boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator, double ctrl_loop_du);

private:
  ros::Publisher four_axis_command_pub_;  // for spinal
  ros::Publisher joints_control_pub_;     // for servo bridge
  ros::Publisher gimbals_control_pub_;    // for servo bridge
  ros::Publisher target_q_pub_;           // for debug

  boost::shared_ptr<aerial_robot_model::ManipulatorRobotModel> dynarm_robot_model_;
  std::shared_ptr<aerial_robot_dynamics::PinocchioRobotModel> pinocchio_robot_model_;
  std::shared_ptr<pinocchio::Model> pinocchio_model_;
  std::shared_ptr<pinocchio::Data> pinocchio_data_;

  std::shared_ptr<aerial_robot_model::NonlinearInverseDynamicsRos> nonlinear_inverse_dynamics_solver_ros_;
  std::shared_ptr<aerial_robot_model::NonlinearInverseDynamics> nonlinear_inverse_dynamics_solver_;

  // robot parameter
  std::vector<std::string> joint_names_;
  std::vector<dynarm::GimbalGroupTpl<double> > gimbal_groups_;

  // joint angle trajectory
  Eigen::VectorXd final_target_q_;   // nq. target joint angles from ros param
  Eigen::VectorXd init_target_q_;    // nq. joint angles when the transformation starts
  Eigen::VectorXd curr_target_q_;    // nq
  Eigen::VectorXd curr_target_dq_;   // nv
  double transform_duration_ = 1.0;  // duration to reach final_target_q_. set from ros param
  double transform_start_time_;      // time when the transformation starts
  bool is_transforming_ = false;     // true if the transformation is in progress

  // inverse dynamics solution: [joint torque (nv); thrust (nr); gimbal angle (gimbal_num)].
  // empty until the first solve, which makes it the initial guess of the next one
  Eigen::VectorXd id_result_;

  virtual bool update() override;
  virtual void reset() override;
  void rosParamInit();
  void loadJointNames();
  void loadGimbalGroups();
  Eigen::VectorXd getGimbalNominalAngles(const Eigen::VectorXd& q, bool global_search);
  void resetJointTrajectory();
  void controlCore();
  void updateTargetPose();
  void solveInverseDynamics();
  void sendCmd();
  void sendFourAxisCommand();
  void sendJointCommand();
  void sendGimbalCommand();
};
}  // namespace aerial_robot_control
