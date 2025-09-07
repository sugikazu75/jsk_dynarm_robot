#include <dynarm/control/joint_trajectory_generator.h>
#include <dynarm/model/manipulator_model.h>

using namespace aerial_robot_control;

jointTrajectoryGenerator::jointTrajectoryGenerator(
    ros::NodeHandle nh, std::shared_ptr<aerial_robot_dynamics::PinocchioRobotModel> pinocchio_robot_model)
  : nh_(nh), pinocchio_robot_model_(pinocchio_robot_model)
{
  pinocchio_model_ = pinocchio_robot_model_->getModel();
  pinocchio_data_ = pinocchio_robot_model_->getData();

  loadJointNames();
  loadGimbalNames();

  rosParamInit();

  is_transforming_pub_ = nh_.advertise<std_msgs::UInt8>("debug/is_transforming", 1);
  target_q_pub_ = nh_.advertise<sensor_msgs::JointState>("debug/target_q", 1);
  target_dq_pub_ = nh_.advertise<sensor_msgs::JointState>("debug/target_dq", 1);
  target_ddq_pub_ = nh_.advertise<sensor_msgs::JointState>("debug/target_ddq", 1);
  target_end_effector_pos_pub_ = nh_.advertise<geometry_msgs::Vector3>("debug/target_ee_pos", 1);
  target_end_effector_vel_pub_ = nh_.advertise<geometry_msgs::Vector3>("debug/target_ee_vel", 1);
  target_end_effector_acc_pub_ = nh_.advertise<geometry_msgs::Vector3>("debug/target_ee_acc", 1);
  dummy_joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 10);

  joint_state_sub_ = nh_.subscribe("joint_states", 1, &jointTrajectoryGenerator::jointStateCallback, this);
  target_end_effector_final_pos_sub_ =
      nh_.subscribe("target_ee_final_pos", 1, &jointTrajectoryGenerator::targetEndEffectorPosCallback, this);
  circle_trajectory_sub_ =
      nh_.subscribe("circle_trajectory", 1, &jointTrajectoryGenerator::circleTrajectoryCallback, this);
  direct_joint_angle_sub_ =
      nh_.subscribe("direct_joint_angle", 1, &jointTrajectoryGenerator::directJointAngleCallback, this);

  curr_q_ = Eigen::VectorXd::Zero(pinocchio_robot_model_->getModel()->nq);
  curr_dq_ = Eigen::VectorXd::Zero(pinocchio_robot_model_->getModel()->nv);
  curr_target_q_ = Eigen::VectorXd::Zero(pinocchio_robot_model_->getModel()->nq);
  curr_target_dq_ = Eigen::VectorXd::Zero(pinocchio_robot_model_->getModel()->nv);
  curr_target_ddq_ = Eigen::VectorXd::Zero(pinocchio_robot_model_->getModel()->nv);

  // joint angle in reset pose
  std::vector<double> init_target_q;
  ros::NodeHandle control_nh(nh_, "controller");
  control_nh.getParam("init_target_q", init_target_q);
  if (init_target_q.size() != pinocchio_model_->nq)
    ROS_ERROR_STREAM("[dragon_arm] nq: " << pinocchio_model_->nq << " and initial joint angle size: "
                                         << init_target_q.size() << " in ros parameter is not same");
  reset_target_q_.resize(pinocchio_model_->nq);
  for (int i = 0; i < pinocchio_model_->nq; i++)
  {
    reset_target_q_(i) = init_target_q.at(i);
  }
  ROS_INFO_STREAM("[dragon_arm][control] initial target joint angle: " << reset_target_q_.transpose());
  curr_target_q_ = getGimbalNominalAngles(reset_target_q_);

  // initialize target end-effector position, velocity, and acceleration
  pinocchio::framesForwardKinematics(*pinocchio_model_, *pinocchio_data_, curr_target_q_);
  pinocchio::FrameIndex frame_id = pinocchio_model_->getFrameId(end_effector_name_);
  target_ee_pos_ = pinocchio_data_->oMf[frame_id].translation();
  target_ee_vel_.setZero();
  target_ee_acc_.setZero();
}

void jointTrajectoryGenerator::rosParamInit()
{
  ros::NodeHandle model_nh(nh_, "model");
  getParam<int>(model_nh, "rotor_devider", rotor_devider_, 1);

  ros::NodeHandle control_nh(nh_, "controller");
  getParam<std::string>(control_nh, "end_effector_name", end_effector_name_, "");
  getParam<double>(control_nh, "transform_duration", transform_duration_, 1.0);

  double ctm_gain;
  getParam<double>(control_nh, "ctm_p_gain", ctm_gain, 1.0);
  ctm_p_gain_ =
      Eigen::MatrixXd::Identity(pinocchio_robot_model_->getModel()->nv, pinocchio_robot_model_->getModel()->nv);
  ctm_p_gain_ *= ctm_gain;

  getParam<bool>(control_nh, "quasi_static_mode", quasi_static_mode_, false);
  getParam<double>(control_nh, "ctm_d_gain", ctm_gain, 1.0);
  if (quasi_static_mode_)
    ctm_gain = 0.0;  // set d gain to zero in quasi-static mode
  ctm_d_gain_ =
      Eigen::MatrixXd::Identity(pinocchio_robot_model_->getModel()->nv, pinocchio_robot_model_->getModel()->nv);
  ctm_d_gain_ *= ctm_gain;

  // set feedback gains for gimbal to zero
  for (int i = 0; i < gimbal_names_.size(); i++)
  {
    int gimbal_index_v = pinocchio_model_->joints[pinocchio_model_->getJointId(gimbal_names_.at(i))].idx_v();
    ctm_p_gain_(gimbal_index_v, gimbal_index_v) = 0.0;
    ctm_d_gain_(gimbal_index_v, gimbal_index_v) = 0.0;
  }
}

void jointTrajectoryGenerator::loadJointNames()
{
  joint_names_.clear();
  for (int i = 0; i < pinocchio_model_->njoints; i++)
  {
    std::string joint_name = pinocchio_model_->names[i];
    if (joint_name.find("joint") != std::string::npos)
    {
      joint_names_.push_back(joint_name);
    }
  }
}

void jointTrajectoryGenerator::loadGimbalNames()
{
  gimbal_names_.clear();
  for (int i = 0; i < pinocchio_model_->njoints; i++)
  {
    std::string joint_name = pinocchio_model_->names[i];
    if (joint_name.find("gimbal") != std::string::npos)
    {
      gimbal_names_.push_back(joint_name);
    }
  }
}

void jointTrajectoryGenerator::reset()
{
  // set private variables to initial smooth deformation
  is_transforming_ = 3;
  transform_start_time_ = ros::Time::now().toSec();
  transform_duration_ = 5.0;
  final_target_q_ = getGimbalNominalAngles(reset_target_q_);
  init_target_q_ = getGimbalNominalAngles(curr_q_);

  pos_trajectory_generator_.reset();
  curr_target_dq_ = Eigen::VectorXd::Zero(pinocchio_model_->nv);
  curr_target_ddq_ = Eigen::VectorXd::Zero(pinocchio_model_->nv);
}

void jointTrajectoryGenerator::update()
{
  generateEndEffectorTrajectory();
  generateJointTrajectory();
  stateTransition();
}

Eigen::VectorXd jointTrajectoryGenerator::getGimbalNominalAngles(Eigen::VectorXd q)
{
  pinocchio::framesForwardKinematics(*pinocchio_model_, *pinocchio_data_, q);

  Eigen::VectorXd gimbal_processed_q = q;

  for (int i = 0; i < pinocchio_robot_model_->getRotorNum() / rotor_devider_; i++)
  {
    std::string link_i_name = "link" + std::to_string(i + 1);
    pinocchio::FrameIndex link_i_frame_id = pinocchio_model_->getFrameId(link_i_name);
    pinocchio::SE3 link_i_frame = pinocchio_data_->oMf[link_i_frame_id];
    Eigen::Vector3d rpy = pinocchio::rpy::matrixToRpy(link_i_frame.rotation());

    std::string gimbal_roll_name = "gimbal" + std::to_string(i + 1) + "_roll";
    std::string gimbal_pitch_name = "gimbal" + std::to_string(i + 1) + "_pitch";

    int gimbal_roll_index = pinocchio_model_->joints[pinocchio_model_->getJointId(gimbal_roll_name)].idx_q();
    int gimbal_pitch_index = pinocchio_model_->joints[pinocchio_model_->getJointId(gimbal_pitch_name)].idx_q();

    gimbal_processed_q(gimbal_roll_index) = -rpy(0);   // roll
    gimbal_processed_q(gimbal_pitch_index) = -rpy(1);  // pitch
  }
  return gimbal_processed_q;
}

void jointTrajectoryGenerator::generateEndEffectorTrajectory()
{
  switch (is_transforming_)
  {
    case 0:  // no transform
    {
      break;
    }
    case 1:  // linear transform
    {
      double curr_time = ros::Time::now().toSec() - transform_start_time_;
      Eigen::Vector3d pos, vel, acc;
      pos_trajectory_generator_.eval(curr_time, pos, vel, acc);
      target_ee_pos_ = pos;
      target_ee_vel_ = vel;
      target_ee_acc_ = acc;
      break;
    }
    case 2:  // circle trajectory
    {
      double curr_time = ros::Time::now().toSec() - transform_start_time_;
      double angle = circle_trajectory_angular_velocity_ * curr_time;
      int reverse = 1;
      if (std::fmod(angle, 4 * M_PI) > 2 * M_PI)
        reverse = -1;  // reverse direction after 2pi

      target_ee_pos_ = circle_trajectory_center_ +
                       circle_trajectory_radius_ *
                           (Eigen::Vector3d(0, reverse, 0) + Eigen::Vector3d(0.0, -reverse * cos(angle), sin(angle)));
      target_ee_vel_ = circle_trajectory_radius_ * circle_trajectory_angular_velocity_ *
                       Eigen::Vector3d(0.0, reverse * sin(angle), cos(angle));
      target_ee_acc_ = -circle_trajectory_radius_ * circle_trajectory_angular_velocity_ *
                       circle_trajectory_angular_velocity_ * Eigen::Vector3d(0.0, reverse * cos(angle), sin(angle));

      break;
    }
    case 3: {
      break;
    }
    default:
      ROS_ERROR_STREAM("[dragon_arm][control] is_transforming_ is not valid: " << is_transforming_);
      break;
  }
}

void jointTrajectoryGenerator::generateJointTrajectory()
{
  if (is_transforming_ == 0)
  {
    curr_target_dq_ = Eigen::VectorXd::Zero(pinocchio_model_->nv);
    curr_target_ddq_ = Eigen::VectorXd::Zero(pinocchio_model_->nv);
  }
  else if ((is_transforming_ == 1) ||
           (is_transforming_ == 2))  // solve IK and generate dq, ddq from target end-effector trajectory
  {
    Eigen::VectorXd ik_initial_q = curr_target_q_;
    bool solved =
        motion_planning::solveIK(*pinocchio_model_, *pinocchio_data_, pinocchio_model_->getFrameId(end_effector_name_),
                                 target_ee_pos_, ik_initial_q, curr_target_q_, false, 1000, 1e-4);
    if (!solved)
    {
      ROS_ERROR_STREAM(
          "[dragon_arm][control] IK solution not found for target position: " << target_ee_pos_.transpose());
    }

    pinocchio::FrameIndex frame_id = pinocchio_model_->getFrameId(end_effector_name_);

    // calculate target dq
    Eigen::MatrixXd J6 = Eigen::MatrixXd::Zero(6, pinocchio_model_->nv);
    pinocchio::computeFrameJacobian(*pinocchio_model_, *pinocchio_data_, curr_q_, frame_id, pinocchio::WORLD,
                                    J6);  // world frame. q is (target or current)
    Eigen::MatrixXd J = J6.topRows(3);    // position
    Eigen::MatrixXd JJt = J * J.transpose() + 1e-12 * Eigen::MatrixXd::Identity(3, 3);
    curr_target_dq_ = J.transpose() * JJt.ldlt().solve(target_ee_vel_);  // target velocity

    // calculate target ddq
    pinocchio::forwardKinematics(*pinocchio_model_, *pinocchio_data_, curr_q_,
                                 curr_dq_);  // q and dq are (target or current)
    pinocchio::computeJointJacobiansTimeVariation(*pinocchio_model_, *pinocchio_data_, curr_q_,
                                                  curr_dq_);  // q is (target or current)
    Eigen::MatrixXd Jdot6 = Eigen::MatrixXd::Zero(6, pinocchio_model_->nv);
    pinocchio::getFrameJacobianTimeVariation(*pinocchio_model_, *pinocchio_data_, frame_id, pinocchio::WORLD, Jdot6);
    Eigen::MatrixXd Jdot = Jdot6.topRows(3);  // position
    curr_target_ddq_ = J.transpose() * JJt.ldlt().solve(target_ee_acc_ - Jdot * curr_dq_);
  }
  else if (is_transforming_ == 3)  // direct command. linear interpolation
  {
    double curr_time = ros::Time::now().toSec() - transform_start_time_;
    if (transform_duration_ == 0.0)
    {
      curr_target_q_ = final_target_q_;
      return;
    }

    Eigen::VectorXd delta_q = pinocchio::difference(*pinocchio_model_, init_target_q_, final_target_q_);  // nv
    // set gimbal delta to zero
    for (int i = 0; i < gimbal_names_.size(); i++)
    {
      int gimbal_index_v = pinocchio_model_->joints[pinocchio_model_->getJointId(gimbal_names_.at(i))].idx_v();
      delta_q(gimbal_index_v) = 0.0;
    }
    curr_target_q_ =
        pinocchio::integrate(*pinocchio_model_, init_target_q_, delta_q * (curr_time / transform_duration_));
    curr_target_dq_ = delta_q / transform_duration_;
    curr_target_ddq_ = Eigen::VectorXd::Zero(pinocchio_model_->nv);
  }

  // add feedback term to target ddq
  curr_target_ddq_ = curr_target_ddq_ +
                     ctm_p_gain_ * pinocchio::difference(*pinocchio_model_, curr_q_, curr_target_q_) +
                     ctm_d_gain_ * (curr_target_dq_ - curr_dq_);

  if (quasi_static_mode_)
  {
    curr_target_dq_ = Eigen::VectorXd::Zero(pinocchio_model_->nv);
    curr_target_ddq_ = Eigen::VectorXd::Zero(pinocchio_model_->nv);
  }
}

void jointTrajectoryGenerator::stateTransition()
{
  switch (is_transforming_)
  {
    case 0: {
      break;
    }
    case 1:  // linear transform
    {
      if (ros::Time::now().toSec() >= transform_start_time_ + transform_duration_)
      {
        is_transforming_ = 0;
        ROS_INFO_STREAM("[dragon_arm][control] end effector position transformation completed.");
      }
      break;
    }
    case 2:  // circular trajectory
    {
      break;
    }
    case 3:  // direct joint angle command
    {
      if (ros::Time::now().toSec() >= transform_start_time_ + transform_duration_)
      {
        is_transforming_ = 0;
        ROS_INFO_STREAM("[dragon_arm][control] direct joint angle transformation completed.");
      }
      break;
    }
    default: {
      break;
    }
  }
}

void jointTrajectoryGenerator::publish()
{
  // for debug: send target q, dq, and ddq
  sensor_msgs::JointState target_q_msg;
  sensor_msgs::JointState target_dq_msg;
  sensor_msgs::JointState target_ddq_msg;
  target_q_msg.header.stamp = ros::Time::now();
  target_dq_msg.header.stamp = ros::Time::now();
  target_ddq_msg.header.stamp = ros::Time::now();

  for (int i = 0; i < joint_names_.size(); i++)
  {
    if (joint_names_.at(i).find("root") != std::string::npos)
      continue;  // skip root joint

    int joint_index_q = pinocchio_model_->joints[pinocchio_model_->getJointId(joint_names_.at(i))].idx_q();
    int joint_index_v = pinocchio_model_->joints[pinocchio_model_->getJointId(joint_names_.at(i))].idx_v();

    if (joint_index_q < 0 || joint_index_v < 0)
      continue;  // skip if joint index is invalid

    target_q_msg.name.push_back(joint_names_.at(i));
    target_q_msg.position.push_back(curr_target_q_(joint_index_q));

    target_dq_msg.name.push_back(joint_names_.at(i));
    target_dq_msg.velocity.push_back(curr_target_dq_(joint_index_v));

    target_ddq_msg.name.push_back(joint_names_.at(i));
    target_ddq_msg.effort.push_back(curr_target_ddq_(joint_index_v));
  }
  target_q_pub_.publish(target_q_msg);
  target_dq_pub_.publish(target_dq_msg);
  target_ddq_pub_.publish(target_ddq_msg);

  // for debug: send target end effector position and velocity
  geometry_msgs::Vector3 target_ee_pos_msg;
  geometry_msgs::Vector3 target_ee_vel_msg;
  geometry_msgs::Vector3 target_ee_acc_msg;
  target_ee_pos_msg.x = target_ee_pos_(0);
  target_ee_pos_msg.y = target_ee_pos_(1);
  target_ee_pos_msg.z = target_ee_pos_(2);
  target_ee_vel_msg.x = target_ee_vel_(0);
  target_ee_vel_msg.y = target_ee_vel_(1);
  target_ee_vel_msg.z = target_ee_vel_(2);
  target_ee_acc_msg.x = target_ee_acc_(0);
  target_ee_acc_msg.y = target_ee_acc_(1);
  target_ee_acc_msg.z = target_ee_acc_(2);
  target_end_effector_pos_pub_.publish(target_ee_pos_msg);
  target_end_effector_vel_pub_.publish(target_ee_vel_msg);
  target_end_effector_acc_pub_.publish(target_ee_acc_msg);

  // for motion planner: is transforming or not
  std_msgs::UInt8 is_transforming_msg;
  is_transforming_msg.data = is_transforming_;
  is_transforming_pub_.publish(is_transforming_msg);
}

void jointTrajectoryGenerator::jointStateCallback(const sensor_msgs::JointState msg)
{
  for (int i = 0; i < msg.name.size(); i++)
  {
    std::string joint_name = msg.name[i];
    int joint_id = pinocchio_model_->getJointId(joint_name);
    if (joint_id == pinocchio_model_->njoints)
      continue;  // skip this joint because there is not in kinematic tree

    // position
    if (msg.name.size() == msg.position.size())
    {
      int joint_index_q = pinocchio_model_->joints[joint_id].idx_q();
      curr_q_(joint_index_q) = msg.position[i];
    }

    // velocity
    if (msg.name.size() == msg.velocity.size())
    {
      int joint_index_v = pinocchio_model_->joints[joint_id].idx_v();
      curr_dq_(joint_index_v) = msg.velocity[i];
    }
  }
}

void jointTrajectoryGenerator::targetEndEffectorPosCallback(const geometry_msgs::Vector3StampedConstPtr& msg)
{
  pinocchio::FrameIndex frame_id = pinocchio_model_->getFrameId(end_effector_name_);
  pinocchio::framesForwardKinematics(*pinocchio_model_, *pinocchio_data_, curr_q_);
  Eigen::Vector3d x_curr = pinocchio_data_->oMf[frame_id].translation();
  Eigen::Vector3d x_des(msg->vector.x, msg->vector.y, msg->vector.z);

  if (msg->header.stamp.sec != 0 || msg->header.stamp.nsec != 0)
  {
    transform_duration_ = msg->header.stamp.sec + msg->header.stamp.nsec / 1000000000.0;
  }

  ROS_INFO_STREAM("[dragon_arm][control] target end effector position: " << x_des.transpose() << " with "
                                                                         << transform_duration_ << "s");
  ROS_INFO_STREAM("[dragon_arm][control] current end effector position: " << x_curr.transpose());

  pos_trajectory_generator_.reset();
  pos_trajectory_generator_.generateTrajectory(x_curr, x_des, transform_duration_);

  is_transforming_ = 1;  // linear mode
  transform_start_time_ = ros::Time::now().toSec();
}

void jointTrajectoryGenerator::circleTrajectoryCallback(const std_msgs::Float32MultiArrayConstPtr& msg)
{
  if (msg->data.size() != 2)
  {
    ROS_INFO_STREAM("[dragon_arm][control] switch back to linear trajectory mode");
    is_transforming_ = 0;  // not transforming
    curr_target_q_ = curr_q_;
    curr_target_dq_ = Eigen::VectorXd::Zero(pinocchio_robot_model_->getModel()->nv);
    curr_target_ddq_ = Eigen::VectorXd::Zero(pinocchio_robot_model_->getModel()->nv);
    return;
  }

  pinocchio::FrameIndex frame_id = pinocchio_model_->getFrameId(end_effector_name_);
  pinocchio::framesForwardKinematics(*pinocchio_model_, *pinocchio_data_, curr_q_);
  circle_trajectory_center_ = pinocchio_data_->oMf[frame_id].translation();
  circle_trajectory_radius_ = msg->data[0];
  circle_trajectory_angular_velocity_ = msg->data[1];

  ROS_INFO_STREAM("[dragon_arm][control] circle trajectory with radius: "
                  << circle_trajectory_radius_ << " and angular velocity: " << circle_trajectory_angular_velocity_
                  << " center: " << circle_trajectory_center_.transpose());

  is_transforming_ = 2;  // circular mode
  transform_start_time_ = ros::Time::now().toSec();
}

void jointTrajectoryGenerator::directJointAngleCallback(const sensor_msgs::JointStateConstPtr& msg)
{
  if (msg->name.size() != msg->position.size())
  {
    ROS_ERROR_STREAM("[dynarm][control] size of joint name: " << msg->name.size() << " and joint position: "
                                                              << msg->position.size() << " is not same");
    return;
  }

  transform_duration_ = msg->header.stamp.sec + msg->header.stamp.nsec / 1000000000.0;

  init_target_q_ = curr_q_;
  final_target_q_ = curr_q_;

  for (int i = 0; i < msg->name.size(); i++)
  {
    std::string joint_name = msg->name.at(i);
    int joint_id = pinocchio_model_->getJointId(joint_name);
    if (joint_id == pinocchio_model_->njoints)
    {
      ROS_WARN_STREAM("[dynarm][control] there is not joint named \"" << joint_name << "\"");
      continue;  // skip this joint because there is not in kinematic tree
    }

    int joint_index_q = pinocchio_model_->joints[joint_id].idx_q();
    final_target_q_(joint_index_q) = msg->position[i];
  }

  ROS_INFO_STREAM("[dynarm][control] move to " << final_target_q_.transpose() << " with " << transform_duration_
                                               << "s");

  is_transforming_ = 3;
  transform_start_time_ = ros::Time::now().toSec();
}
