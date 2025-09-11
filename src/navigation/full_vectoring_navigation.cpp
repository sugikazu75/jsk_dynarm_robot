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

  path_pub_ = nh_.advertise<nav_msgs::Path>("trajectory_path", 1);
  joints_control_pub_ = nh_.advertise<sensor_msgs::JointState>("joints_ctrl", 1);
  desire_coordinate_sub_ =
      nh_.subscribe("desire_coordinate", 1, &FullVectoringNavigator::desireCoordinateCallback, this);
  circle_trajectory_command_sub_ =
      nh_.subscribe("circle_trajectory_command", 1, &FullVectoringNavigator::circleTrajectoryCommandCallback, this);
  joint_trajectory_command_sub_ =
      nh_.subscribe("joint_trajectory_command", 1, &FullVectoringNavigator::jointTrajectoryCommandCallback, this);
}

void FullVectoringNavigator::reset()
{
  BaseNavigator::reset();

  circle_trajectory_flight_flag_ = false;
}

void FullVectoringNavigator::update()
{
  BaseNavigator::update();

  if (circle_trajectory_flight_flag_)
  {
    circleTrajectoryGeneration();
  }
  else if (joint_trajectory_flight_flag_)
  {
    jointTrajectoryGeneration();
  }
}

void FullVectoringNavigator::circleTrajectoryGeneration()
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

void FullVectoringNavigator::jointTrajectoryGeneration()
{
  if (ros::Time::now().toSec() > joint_trajectory_end_time_)
  {
    sensor_msgs::JointState joint_cmd_msg;
    joint_cmd_msg.position = joint_trajectory_angle_start_;
    joints_control_pub_.publish(joint_cmd_msg);
    joint_trajectory_flight_flag_ = false;
    ROS_INFO_STREAM("[navigation] finish joint trajectory tracking");
  }
  else if (ros::Time::now().toSec() >= joint_trajectory_start_time_)
  {
    sensor_msgs::JointState joint_cmd_msg;
    double t = ros::Time::now().toSec() - joint_trajectory_start_time_;
    double omega = 2 * M_PI / joint_trajectory_duration_;
    double theta = omega * t;
    for (int i = 0; i < joint_trajectory_angle_start_.size(); i++)
    {
      double target_angle =
          (joint_trajectory_angle_start_.at(i) + joint_trajectory_angle_end_.at(i)) / 2.0 +
          (joint_trajectory_angle_end_.at(i) - joint_trajectory_angle_start_.at(i)) / 2.0 * (-cos(theta));
      joint_cmd_msg.position.push_back(target_angle);
    }
    joints_control_pub_.publish(joint_cmd_msg);
  }
  else
  {
    // hold the start position
    sensor_msgs::JointState joint_cmd_msg;
    joint_cmd_msg.position = joint_trajectory_angle_start_;
    joints_control_pub_.publish(joint_cmd_msg);
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

    nav_msgs::Path path_msg;
    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = "world";
    int N = 120;
    for (int i = 0; i < N; i++)
    {
      geometry_msgs::PoseStamped pose;
      pose.pose.position.x = circle_center_.x() + circle_radius_ * cos(2 * M_PI * i / N);
      pose.pose.position.y = circle_center_.y() + circle_radius_ * sin(2 * M_PI * i / N);
      pose.pose.position.z = circle_center_.z();
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;
      pose.pose.orientation.w = 1.0;
      path_msg.poses.push_back(pose);
    }
    path_pub_.publish(path_msg);
  }
}

void FullVectoringNavigator::jointTrajectoryCommandCallback(const std_msgs::EmptyConstPtr& msg)
{
  ros::NodeHandle joint_traj_nh(nh_, "joint_trajectory");
  joint_traj_nh.param("duration", joint_trajectory_duration_, 1.0);

  joint_traj_nh.getParam("joint_names", joint_trajectory_names_);
  joint_traj_nh.getParam("start_angle", joint_trajectory_angle_start_);
  if (joint_trajectory_angle_start_.size() != joint_trajectory_names_.size())
  {
    ROS_ERROR("[navigation] Joint trajectory command for start angle size mismatch.");
    return;
  }

  joint_traj_nh.getParam("end_angle", joint_trajectory_angle_end_);
  if (joint_trajectory_angle_end_.size() != joint_trajectory_names_.size())
  {
    ROS_ERROR("[navigation] Joint trajectory command for end angle size mismatch.");
    return;
  }

  ROS_INFO_STREAM("[navigation] start joint trajectory tracking, duration: " << joint_trajectory_duration_ << " s");
  joint_trajectory_start_time_ = ros::Time::now().toSec() + 5.0;
  joint_trajectory_end_time_ = joint_trajectory_start_time_ + 3 * joint_trajectory_duration_;  // three loops
  joint_trajectory_flight_flag_ = true;
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_navigation::FullVectoringNavigator, aerial_robot_navigation::BaseNavigator);
