#pragma once

#include <aerial_robot_control/flight_navigation.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <spinal/DesireCoord.h>
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
  ros::Publisher path_pub_;  // for debug
  ros::Subscriber desire_coordinate_sub_;
  ros::Subscriber circle_trajectory_command_sub_;

  // circle trajectory flight
  bool circle_trajectory_flight_flag_ = false;
  double circle_radius_;
  double circle_omega_;
  Eigen::Vector3d circle_center_;
  double circle_trajectory_initial_yaw_;
  double circle_trajectory_start_time_;
  double circle_trajectory_end_time_;

  void circleTrajectoryGeneration();

  void desireCoordinateCallback(const spinal::DesireCoordConstPtr& msg);
  void circleTrajectoryCommandCallback(const std_msgs::Float32MultiArrayConstPtr& msg);
};
}  // namespace aerial_robot_navigation
