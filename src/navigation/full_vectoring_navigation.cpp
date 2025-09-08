#include <dynarm/navigation/full_vectoring_navigation.h>

using namespace aerial_robot_navigation;

FullVectoringNavigator::FullVectoringNavigator() : BaseNavigator()
{
}

void FullVectoringNavigator::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                        boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                        boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                        double loop_du)
{
  /* initialize the flight control */
  BaseNavigator::initialize(nh, nhp, robot_model, estimator, loop_du);

  rosParamInit();

  desire_coordinate_sub_ =
      nh_.subscribe("desire_coordinate", 1, &FullVectoringNavigator::desireCoordinateCallback, this);
  circle_trajectory_command_sub_ =
      nh_.subscribe("circle_trajectory_command", 1, &FullVectoringNavigator::circleTrajectoryCommandCallback, this);
}

void FullVectoringNavigator::reset()
{
  BaseNavigator::reset();

  circle_trajectory_flight_flag_ = false;
}

void FullVectoringNavigator::update()
{
  BaseNavigator::update();

  circleTrajectoryGeneration();
}

void FullVectoringNavigator::circleTrajectoryGeneration()
{
  if (circle_trajectory_flight_flag_)
  {
    if (ros::Time::now().toSec() > circle_trajectory_end_time_)
    {
      Eigen::Vector3d target_pos = circle_center_ + Eigen::Vector3d(circle_radius_, 0.0, 0.0);
      setTargetPosX(target_pos(0));
      setTargetPosY(target_pos(1));
      setTargetVel(0, 0, 0);
      circle_trajectory_flight_flag_ = false;
      ROS_INFO_STREAM("[navigation] finish circle trajectory tracking");
    }
    else if (ros::Time::now().toSec() >= circle_trajectory_start_time_)
    {
      double t = ros::Time::now().toSec() - circle_trajectory_start_time_;
      Eigen::Vector3d target_pos = circle_center_ + Eigen::Vector3d(circle_radius_ * cos(circle_omega_ * t),
                                                                    circle_radius_ * sin(circle_omega_ * t), 0.0);
      setTargetVel(-circle_radius_ * circle_omega_ * sin(circle_omega_ * t),
                   circle_radius_ * circle_omega_ * cos(circle_omega_ * t), 0);
      setTargetPosX(target_pos(0));
      setTargetPosY(target_pos(1));
    }
  }
}

void FullVectoringNavigator::desireCoordinateCallback(const spinal::DesireCoordConstPtr& msg)
{
  KDL::Rotation rot;
  rot = KDL::Rotation::RPY(msg->roll, msg->pitch, 0.0);
  robot_model_->setCogDesireOrientation(rot);
}

void FullVectoringNavigator::circleTrajectoryCommandCallback(const std_msgs::Float32MultiArrayConstPtr& msg)
{
  if (msg->data.size() != 2)
  {
    ROS_ERROR("[navigation] Circle trajectory command size mismatch.");
  }
  else
  {
    setTargetYaw(0.0);
    circle_trajectory_flight_flag_ = true;
    circle_radius_ = msg->data.at(0);
    circle_omega_ = msg->data.at(1);
    circle_trajectory_start_time_ = ros::Time::now().toSec() + 10.0;
    circle_trajectory_end_time_ = circle_trajectory_start_time_ + 6.0 * M_PI / circle_omega_;  // three loops

    tf::Vector3 target_pos = getTargetPos();
    circle_center_ = Eigen::Vector3d(target_pos.x() - circle_radius_, target_pos.y(), target_pos.z());

    ROS_INFO_STREAM("[navigation] circle trajectory center: " << circle_center_.transpose()
                                                              << ", radius: " << circle_radius_);
  }
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_navigation::FullVectoringNavigator, aerial_robot_navigation::BaseNavigator);
