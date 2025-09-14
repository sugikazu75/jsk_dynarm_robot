#pragma once

#include <aerial_robot_control/flight_navigation.h>
#include <aerial_robot_model/model/transformable_aerial_robot_model.h>
#include <aerial_robot_msgs/PoseControlPid.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/JointState.h>
#include <spinal/DesireCoord.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32MultiArray.h>

namespace aerial_robot_navigation
{
class FullVectoringNavigator : public BaseNavigator
{
public:
  FullVectoringNavigator();
  ~FullVectoringNavigator()
  {
  }

  void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                  boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                  boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator, double loop_du) override;
  void update() override;
  void reset() override;

private:
  boost::shared_ptr<aerial_robot_model::transformable::RobotModel> robot_model_for_plan_;
  sensor_msgs::JointState joint_state_for_plan_;
  std::map<std::string, uint32_t> joint_index_map_without_rotor_;

  ros::Publisher path_pub_;              // for debug
  ros::Publisher joints_control_pub_;    // for servo bridge
  ros::Publisher target_root_pose_pub_;  // for debug
  ros::Publisher root_pose_debug_pub_;   // for debug
  ros::Subscriber desire_coordinate_sub_;
  ros::Subscriber circle_trajectory_command_sub_;
  ros::Subscriber joint_trajectory_command_sub_;

  // circle trajectory flight
  bool circle_trajectory_flight_flag_ = false;
  double circle_radius_;
  double circle_duration_;
  int circle_loop_ = 3;
  Eigen::Vector3d circle_center_;
  double circle_trajectory_initial_yaw_;
  double circle_trajectory_start_time_;
  double circle_trajectory_end_time_;

  // joint trajectory flight
  Eigen::Affine3d world_to_root_initial_;
  bool joint_trajectory_flight_flag_ = false;
  double joint_trajectory_duration_ = 1.0;
  int joint_trajectory_loop_ = 3;
  double joint_trajectory_start_time_;
  double joint_trajectory_end_time_;
  std::vector<std::string> joint_trajectory_names_;
  std::vector<double> joint_trajectory_angle_start_;
  std::vector<double> joint_trajectory_angle_end_;

  void circleTrajectoryGeneration();
  void jointTrajectoryGeneration();

  void publish();

  void desireCoordinateCallback(const spinal::DesireCoordConstPtr& msg);
  void circleTrajectoryCommandCallback(const std_msgs::EmptyConstPtr& msg);
  void jointTrajectoryCommandCallback(const std_msgs::EmptyConstPtr& msg);
};
}  // namespace aerial_robot_navigation
