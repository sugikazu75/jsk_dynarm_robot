#pragma once

#include <pinocchio/fwd.hpp>
#include <dynarm/model/manipulator_model.h>
#include <dynarm/model/nonlinear_inverse_dynamics.h>
#include <dynarm/control/ddp_hovering_problem.h>

#include <aerial_robot_control/control/base/base.h>

#include <ros/ros.h>
#include <Eigen/Core>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt8.h>
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
  ros::Publisher path_pub_;                  // for debug
  ros::Publisher ddp_solve_time_pub_;        // for debug
  ros::Publisher ddp_iteration_pub_;         // for debug
  ros::Publisher target_root_pose_pub_;      // for debug
  ros::Subscriber joint_state_sub_;
  ros::Subscriber joint_command_sub_;
  ros::Subscriber root_pos_command_sub_;
  ros::Subscriber root_pose_command_sub_;
  ros::Subscriber circle_trajectory_command_sub_;

  std::string robot_ns_;
  int rotor_wrench_pub_index_;

  boost::shared_ptr<aerial_robot_model::ManipulatorRobotModel> dragon_arm_robot_model_;
  std::shared_ptr<aerial_robot_dynamics::PinocchioRobotModel> pinocchio_robot_model_;
  std::shared_ptr<pinocchio::Model> pinocchio_model_;
  std::shared_ptr<pinocchio::Data> pinocchio_data_;

  std::shared_ptr<DDPHoveringProblem> hovering_;
  std::shared_ptr<crocoddyl::ShootingProblem> ddp_problem_;
  std::shared_ptr<crocoddyl::SolverAbstract> ddp_solver_;
  std::vector<Eigen::VectorXd> xs_init_;
  std::vector<Eigen::VectorXd> us_init_;
  double ddp_solve_time_ = 0.0;

  std::shared_ptr<aerial_robot_model::NonlinearInverseDynamics> nonlinear_inverse_dynamics_solver_;

  Eigen::VectorXd xref_;
  Eigen::VectorXd curr_q_;
  Eigen::VectorXd curr_dq_;
  Eigen::VectorXd curr_target_q_;
  Eigen::VectorXd curr_target_dq_;

  Eigen::VectorXd control_input_;

  // circle trajectory flight
  bool circle_trajectory_flight_flag_ = false;
  double circle_radius_;
  double circle_omega_;
  Eigen::Vector3d circle_center_;
  double circle_trajectory_start_time_;
  double circle_trajectory_end_time_;
  double circle_trajectory_pitch_max_ = 1.0;

  void rosParamInit();
  void DDPProblemInit();
  virtual void activate() override;
  virtual bool update() override;
  virtual void reset() override;
  Eigen::VectorXd getCurrentX();
  void controlCore();
  void circleTrajectoryGeneration();
  void sendCmd();
  void publish();
  void sendFourAxisCommand();
  void sendJointCommand();
  void sendGimbalCommand();
  void jointStateCallback(const sensor_msgs::JointStateConstPtr& msg);
  void publishDDPTrajectory();
  void jointCommandCallback(const sensor_msgs::JointStateConstPtr& msg);
  void rootPosCommandCallback(const geometry_msgs::Vector3ConstPtr& msg);
  void rootPoseCommandCallback(const geometry_msgs::PoseConstPtr& msg);
  void circleTrajectoryCommandCallback(const std_msgs::Float32MultiArrayConstPtr& msg);
};
}  // namespace aerial_robot_control
