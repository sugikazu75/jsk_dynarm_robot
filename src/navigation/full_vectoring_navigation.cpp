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
  /* initialize base navigator */
  BaseNavigator::initialize(nh, nhp, robot_model, estimator, loop_du);

  /* make a robot model for planning */
  std::map<std::string, uint32_t> joint_index_map = robot_model_->getJointIndexMap();
  robot_model_for_plan_ = boost::make_shared<aerial_robot_model::transformable::RobotModel>();
  joint_state_for_plan_.name.clear();
  joint_state_for_plan_.position.clear();
  joint_state_for_plan_.velocity.clear();
  joint_state_for_plan_.effort.clear();
  joint_state_for_plan_.name = robot_model_->getJointNames();
  joint_state_for_plan_.position.resize(joint_state_for_plan_.name.size(), 0);
  robot_model_for_plan_->updateRobotModel(joint_state_for_plan_);
  for (int i = 0; i < joint_state_for_plan_.name.size(); i++)
  {
    joint_index_map_without_rotor_[joint_state_for_plan_.name.at(i)] = i;
  }

  rosParamInit();

  path_pub_ = nh_.advertise<nav_msgs::Path>("trajectory_path", 1);
  joints_control_pub_ = nh_.advertise<sensor_msgs::JointState>("joints_ctrl", 1);
  target_root_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("debug/target_root_pose", 1);

  desire_coordinate_sub_ =
      nh_.subscribe("desire_coordinate", 1, &FullVectoringNavigator::desireCoordinateCallback, this);
  circle_trajectory_command_sub_ =
      nh_.subscribe("circle_trajectory_command", 1, &FullVectoringNavigator::circleTrajectoryCommandCallback, this);
  joint_trajectory_command_sub_ =
      nh_.subscribe("joint_trajectory_command", 1, &FullVectoringNavigator::jointTrajectoryCommandCallback, this);

  world_to_root_initial_ = Eigen::Affine3d::Identity();
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

  publish();
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
    double omega = 2 * M_PI / circle_duration_;
    Eigen::Vector3d target_pos =
        circle_center_ + Eigen::Vector3d(circle_radius_ * cos(omega * t), circle_radius_ * sin(omega * t), 0.0);
    setTargetVel(-circle_radius_ * omega * sin(omega * t), circle_radius_ * omega * cos(omega * t), 0);
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
    // update joint command
    for (int i = 0; i < joint_trajectory_names_.size(); i++)
    {
      std::string joint_name = joint_trajectory_names_.at(i);
      double target_angle =
          (joint_trajectory_angle_start_.at(i) + joint_trajectory_angle_end_.at(i)) / 2.0 +
          (joint_trajectory_angle_end_.at(i) - joint_trajectory_angle_start_.at(i)) / 2.0 * (-cos(theta));
      double joint_velocity =
          (joint_trajectory_angle_end_.at(i) - joint_trajectory_angle_start_.at(i)) / 2.0 * omega * sin(theta);
      joint_cmd_msg.name.push_back(joint_name);
      joint_cmd_msg.velocity.push_back(joint_velocity);
      joint_cmd_msg.position.push_back(target_angle);
      joint_cmd_msg.effort.push_back(0.0);

      // update joint state for plan
      uint32_t joint_index = joint_index_map_without_rotor_.at(joint_name);
      joint_state_for_plan_.position.at(joint_index) = target_angle;
    }

    // publish joint command
    joints_control_pub_.publish(joint_cmd_msg);

    // update cog trajecotry
    robot_model_for_plan_->updateRobotModel(joint_state_for_plan_);
    Eigen::Affine3d root_to_cog = robot_model_for_plan_->getCog<Eigen::Affine3d>();
    Eigen::Affine3d world_to_cog = world_to_root_initial_ * root_to_cog;
    setTargetPosX(world_to_cog.translation().x());
    setTargetPosY(world_to_cog.translation().y());
    setTargetPosZ(world_to_cog.translation().z());
    Eigen::Matrix3d rot = world_to_cog.rotation();
    Eigen::Vector3d rpy = rot.eulerAngles(0, 1, 2);
    setTargetYaw(rpy.z());
  }
  else
  {
    // hold the start position
    sensor_msgs::JointState joint_cmd_msg;
    joint_cmd_msg.position = joint_trajectory_angle_start_;
    joints_control_pub_.publish(joint_cmd_msg);
  }
}

void FullVectoringNavigator::publish()
{
  // publish the target root pose for debug
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.stamp = ros::Time::now();
  pose_msg.header.frame_id = "world";
  pose_msg.pose.position.x = world_to_root_initial_.translation().x();
  pose_msg.pose.position.y = world_to_root_initial_.translation().y();
  pose_msg.pose.position.z = world_to_root_initial_.translation().z();
  Eigen::Quaterniond quat(world_to_root_initial_.rotation());
  pose_msg.pose.orientation.x = quat.x();
  pose_msg.pose.orientation.y = quat.y();
  pose_msg.pose.orientation.z = quat.z();
  pose_msg.pose.orientation.w = quat.w();
  target_root_pose_pub_.publish(pose_msg);
}

void FullVectoringNavigator::desireCoordinateCallback(const spinal::DesireCoordConstPtr& msg)
{
  KDL::Rotation rot;
  rot = KDL::Rotation::RPY(msg->roll, msg->pitch, 0.0);
  robot_model_->setCogDesireOrientation(rot);
}

void FullVectoringNavigator::circleTrajectoryCommandCallback(const std_msgs::EmptyConstPtr& msg)
{
  ros::NodeHandle circle_traj_nh(nh_, "circle_trajectory");
  circle_traj_nh.param("radius", circle_radius_, 1.0);
  circle_traj_nh.param("duration", circle_duration_, M_PI);
  circle_traj_nh.param("loop", circle_loop_, 3);

  circle_trajectory_flight_flag_ = true;
  circle_trajectory_start_time_ = ros::Time::now().toSec() + 6.0;
  circle_trajectory_end_time_ = circle_trajectory_start_time_ + circle_loop_ * circle_duration_;

  tf::Vector3 target_pos = getTargetPos();
  circle_center_ = Eigen::Vector3d(target_pos.x() - circle_radius_, target_pos.y(), target_pos.z());

  ROS_INFO_STREAM("[navigation] circle trajectory center: " << circle_center_.transpose()
                                                            << ", radius: " << circle_radius_
                                                            << ", duration: " << circle_duration_ << " s"
                                                            << ", loop: " << circle_loop_);

  // visualize path
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

  for (int i = 0; i < joint_trajectory_names_.size(); i++)
  {
    std::string joint_name = joint_trajectory_names_.at(i);
    double start_angle = joint_trajectory_angle_start_.at(i);
    joint_state_for_plan_.position.at(joint_index_map_without_rotor_.at(joint_name)) = start_angle;
  }
  robot_model_for_plan_->updateRobotModel(joint_state_for_plan_);

  Eigen::Vector3d target_pos = Eigen::Vector3d(target_pos_.x(), target_pos_.y(), target_pos_.z());
  Eigen::Matrix3d target_rot = (Eigen::AngleAxisd(target_rpy_.z(), Eigen::Vector3d::UnitZ()) *
                                Eigen::AngleAxisd(target_rpy_.y(), Eigen::Vector3d::UnitY()) *
                                Eigen::AngleAxisd(target_rpy_.x(), Eigen::Vector3d::UnitX()))
                                   .toRotationMatrix();
  Eigen::Affine3d world_to_cog = Eigen::Translation3d(target_pos) * target_rot;
  Eigen::Affine3d root_to_cog = robot_model_for_plan_->getCog<Eigen::Affine3d>();
  Eigen::Affine3d world_to_root = world_to_cog * root_to_cog.inverse();
  world_to_root_initial_ = world_to_root;

  ROS_INFO_STREAM("[navigation] start joint trajectory tracking, duration: " << joint_trajectory_duration_ << " s");
  joint_trajectory_start_time_ = ros::Time::now().toSec() + 6.0;
  joint_trajectory_end_time_ = joint_trajectory_start_time_ + 3 * joint_trajectory_duration_;  // three loops
  joint_trajectory_flight_flag_ = true;
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_navigation::FullVectoringNavigator, aerial_robot_navigation::BaseNavigator);
