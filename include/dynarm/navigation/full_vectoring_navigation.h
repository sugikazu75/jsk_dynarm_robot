#pragma once

#include <aerial_robot_control/flight_navigation.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <spinal/DesireCoord.h>

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

private:
  ros::Subscriber desire_coordinate_sub_;

  void desireCoordinateCallback(const spinal::DesireCoordConstPtr& msg);
};
}  // namespace aerial_robot_navigation
