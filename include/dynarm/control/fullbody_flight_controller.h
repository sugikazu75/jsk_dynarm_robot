#pragma once

#include <pinocchio/fwd.hpp>
#include <dynarm/model/manipulator_model.h>
#include <dynarm/model/nonlinear_inverse_dynamics.h>
#include <dynarm/control/ddp_hovering_problem.h>

#include <aerial_robot_control/control/base/base.h>

#include <ros/ros.h>
#include <Eigen/Core>

#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>
#include <spinal/FourAxisCommand.h>
#include <tf/transform_broadcaster.h>

namespace aerial_robot_control
{
class FullbodyFlightController : public ControlBase
{
public:
  FullbodyFlightController();
  virtual ~FullbodyFlightController() = default;

  void virtual initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                          boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                          boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                          boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator, double ctrl_loop_du);

private:
  tf::TransformBroadcaster tf_broadcaster_;  // for debug
  ros::Publisher joints_control_pub_;        // for servo bridge
  ros::Publisher gimbals_control_pub_;       // for servo bridge
  ros::Publisher four_axis_command_pub_;     // for spinal
  ros::Publisher rotor_wrench_pub_;          // for debug
  ros::Subscriber joint_state_sub_;
  ros::Subscriber state_command_sub_;

  std::string robot_ns_;
  int rotor_wrench_pub_index_;

  boost::shared_ptr<aerial_robot_model::ManipulatorRobotModel> dragon_arm_robot_model_;
  std::shared_ptr<aerial_robot_dynamics::PinocchioRobotModel> pinocchio_robot_model_;
  std::shared_ptr<pinocchio::Model> pinocchio_model_;
  std::shared_ptr<pinocchio::Data> pinocchio_data_;

  std::shared_ptr<DDPHoveringProblem> hovering_;
  std::shared_ptr<crocoddyl::ShootingProblem> ddp_problem_;
  std::shared_ptr<crocoddyl::SolverAbstract> ddp_solver_;
  std::shared_ptr<aerial_robot_model::NonlinearInverseDynamics> nonlinear_inverse_dynamics_solver_;
  std::vector<Eigen::VectorXd> xs_init_;
  std::vector<Eigen::VectorXd> us_init_;

  Eigen::VectorXd curr_q_;
  Eigen::VectorXd curr_dq_;
  Eigen::VectorXd curr_target_q_;
  Eigen::VectorXd curr_target_dq_;

  Eigen::VectorXd control_input_;

  void rosParamInit();
  void DDPProblemInit();
  virtual void activate() override;
  virtual bool update() override;
  virtual void reset() override;
  void controlCore();
  void sendCmd();
  void publish();
  void sendFourAxisCommand();
  void sendJointCommand();
  void sendGimbalCommand();
  void jointStateCallback(const sensor_msgs::JointStateConstPtr& msg);
  void publishDDPTrajectory();
  void stateCommandCallback(const sensor_msgs::JointStateConstPtr& msg);
};
}  // namespace aerial_robot_control
