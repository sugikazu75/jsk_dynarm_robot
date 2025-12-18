#include <dynarm/control/joint_trajectory_generator.h>
#include <dynarm/model/manipulator_model.h>
#include <aerial_robot_control/util/joy_parser.h>

using namespace aerial_robot_control;

JointTrajectoryGenerator::JointTrajectoryGenerator(
    std::shared_ptr<aerial_robot_dynamics::PinocchioRobotModel> pinocchio_robot_model)
  : pinocchio_robot_model_(pinocchio_robot_model)
{
  pinocchio_model_ = pinocchio_robot_model_->getModel();
  pinocchio_data_ = pinocchio_robot_model_->getData();

  loadJointNames();
  loadGimbalNames();

  curr_q_ = Eigen::VectorXd::Zero(pinocchio_robot_model_->getModel()->nq);
  curr_dq_ = Eigen::VectorXd::Zero(pinocchio_robot_model_->getModel()->nv);

  curr_target_q_ = Eigen::VectorXd::Zero(pinocchio_robot_model_->getModel()->nq);
  curr_target_dq_ = Eigen::VectorXd::Zero(pinocchio_robot_model_->getModel()->nv);
  curr_target_ddq_ = Eigen::VectorXd::Zero(pinocchio_robot_model_->getModel()->nv);

  target_ee_pos_ = Eigen::Vector3d::Zero();
  target_ee_vel_ = Eigen::Vector3d::Zero();
  target_ee_acc_ = Eigen::Vector3d::Zero();
}

void JointTrajectoryGenerator::loadJointNames()
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

void JointTrajectoryGenerator::loadGimbalNames()
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

void JointTrajectoryGenerator::reset()
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

void JointTrajectoryGenerator::update()
{
  generateEndEffectorTrajectory();
  generateJointTrajectory();
  stateTransition();
}

Eigen::VectorXd JointTrajectoryGenerator::getGimbalNominalAngles(Eigen::VectorXd q)
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

void JointTrajectoryGenerator::generateEndEffectorTrajectory()
{
  switch (is_transforming_)
  {
    case 0:  // no transform
    {
      target_ee_vel_.setZero();
      target_ee_acc_.setZero();
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
    case 4: {  // joy control
      target_ee_vel_(0) = joy_msg_.axes[JOY_AXIS_STICK_LEFT_UPWARDS];
      target_ee_vel_(1) = joy_msg_.axes[JOY_AXIS_STICK_LEFT_LEFTWARDS];
      target_ee_vel_(2) = joy_msg_.axes[JOY_AXIS_STICK_RIGHT_UPWARDS];
      target_ee_pos_ = target_ee_pos_ + target_ee_vel_ * ctrl_loop_du_;
      break;
    }
    default:
      ROS_ERROR_STREAM("[JointTrajectoryGenerator] is_transforming_ is not valid: " << is_transforming_);
      break;
  }
}

void JointTrajectoryGenerator::generateJointTrajectory()
{
  if (is_transforming_ == 0)
  {
    curr_target_dq_ = Eigen::VectorXd::Zero(pinocchio_model_->nv);
    curr_target_ddq_ = Eigen::VectorXd::Zero(pinocchio_model_->nv);
  }
  else if ((is_transforming_ == 1) || (is_transforming_ == 2) ||
           (is_transforming_ == 4))  // solve IK and generate dq, ddq from target end-effector trajectory
  {
    Eigen::VectorXd ik_initial_q = curr_target_q_;
    bool solved =
        motion_planning::solveIK(*pinocchio_model_, *pinocchio_data_, pinocchio_model_->getFrameId(end_effector_name_),
                                 target_ee_pos_, ik_initial_q, curr_target_q_, false, 1000, 1e-4);
    if (!solved)
    {
      ROS_ERROR_STREAM(
          "[JointTrajectoryGenerator] IK solution not found for target position: " << target_ee_pos_.transpose());
    }

    pinocchio::FrameIndex frame_id = pinocchio_model_->getFrameId(end_effector_name_);

    // calculate target dq
    Eigen::MatrixXd J6 = Eigen::MatrixXd::Zero(6, pinocchio_model_->nv);
    pinocchio::computeFrameJacobian(*pinocchio_model_, *pinocchio_data_, curr_q_, frame_id,
                                    pinocchio::LOCAL_WORLD_ALIGNED,
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
    pinocchio::getFrameJacobianTimeVariation(*pinocchio_model_, *pinocchio_data_, frame_id,
                                             pinocchio::LOCAL_WORLD_ALIGNED, Jdot6);
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

void JointTrajectoryGenerator::stateTransition()
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
        ROS_INFO_STREAM("[JointTrajectoryGenerator] end effector position transformation completed.");
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
        ROS_INFO_STREAM("[JointTrajectoryGenerator] direct joint angle transformation completed.");
      }
      break;
    }
    default: {
      break;
    }
  }
}

JointTrajectoryGeneratorRos::JointTrajectoryGeneratorRos(
    ros::NodeHandle nh, std::shared_ptr<aerial_robot_dynamics::PinocchioRobotModel> pinocchio_robot_model)
  : nh_(nh)
{
  joint_trajectory_generator_ = std::make_shared<JointTrajectoryGenerator>(pinocchio_robot_model);

  is_transforming_pub_ = nh_.advertise<std_msgs::UInt8>("debug/is_transforming", 1);
  target_q_pub_ = nh_.advertise<sensor_msgs::JointState>("debug/target_q", 1);
  target_dq_pub_ = nh_.advertise<sensor_msgs::JointState>("debug/target_dq", 1);
  target_ddq_pub_ = nh_.advertise<sensor_msgs::JointState>("debug/target_ddq", 1);
  target_end_effector_pos_pub_ = nh_.advertise<geometry_msgs::Vector3>("debug/target_ee_pos", 1);
  target_end_effector_vel_pub_ = nh_.advertise<geometry_msgs::Vector3>("debug/target_ee_vel", 1);
  target_end_effector_acc_pub_ = nh_.advertise<geometry_msgs::Vector3>("debug/target_ee_acc", 1);

  joint_state_sub_ = nh_.subscribe("joint_states", 1, &JointTrajectoryGeneratorRos::jointStateCallback, this);
  target_end_effector_final_pos_sub_ =
      nh_.subscribe("target_ee_final_pos", 1, &JointTrajectoryGeneratorRos::targetEndEffectorPosCallback, this);
  circle_trajectory_sub_ =
      nh_.subscribe("circle_trajectory", 1, &JointTrajectoryGeneratorRos::circleTrajectoryCallback, this);
  direct_joint_angle_sub_ =
      nh_.subscribe("direct_joint_angle", 1, &JointTrajectoryGeneratorRos::directJointAngleCallback, this);
  joy_sub_ = nh_.subscribe("joy", 1, &JointTrajectoryGeneratorRos::joyCallback, this);

  rosParamInit();
}

void JointTrajectoryGeneratorRos::rosParamInit()
{
  ros::NodeHandle model_nh(nh_, "model");
  ros::NodeHandle control_nh(nh_, "controller");

  // number of rotors in one gimbal
  getParam<int>(model_nh, "rotor_devider", joint_trajectory_generator_->rotor_devider_, 1);

  // joint angle in reset pose
  std::vector<double> init_target_q;
  control_nh.getParam("init_target_q", init_target_q);
  if (init_target_q.size() != joint_trajectory_generator_->getPinocchioModel()->nq)
    ROS_ERROR_STREAM("[JointTrajectoryGeneratorRos] nq: " << joint_trajectory_generator_->getPinocchioModel()->nq
                                                          << " and initial joint angle size: " << init_target_q.size()
                                                          << " in ros parameter is not same");

  Eigen::VectorXd reset_target_q = Eigen::VectorXd::Zero(joint_trajectory_generator_->getPinocchioModel()->nq);
  for (int i = 0; i < joint_trajectory_generator_->getPinocchioModel()->nq; i++)
  {
    reset_target_q(i) = init_target_q.at(i);
  }
  ROS_INFO_STREAM("[JointTrajectoryGeneratorRos] initial target joint angle: " << reset_target_q.transpose());
  joint_trajectory_generator_->setResetTargetQ(reset_target_q);
  joint_trajectory_generator_->setCurrentTargetQ(joint_trajectory_generator_->getGimbalNominalAngles(reset_target_q));

  getParam<std::string>(control_nh, "end_effector_name", joint_trajectory_generator_->end_effector_name_, "");
  getParam<double>(control_nh, "transform_duration", joint_trajectory_generator_->transform_duration_, 1.0);

  // initialize target end-effector position
  pinocchio::framesForwardKinematics(*(joint_trajectory_generator_->getPinocchioModel()),
                                     *(joint_trajectory_generator_->getPinocchioData()),
                                     joint_trajectory_generator_->getCurrentTargetQ());
  pinocchio::FrameIndex frame_id =
      joint_trajectory_generator_->getPinocchioModel()->getFrameId(joint_trajectory_generator_->end_effector_name_);
  joint_trajectory_generator_->setTargetEndEffectorPos(
      joint_trajectory_generator_->getPinocchioData()->oMf[frame_id].translation());

  if (!control_nh.getParam("ctrl_loop_du", joint_trajectory_generator_->ctrl_loop_du_))
    ROS_ERROR_STREAM("[JointTrajectoryGeneratorRos] Failed to get ctrl_loop_du from ros parameter server");

  getParam<bool>(control_nh, "quasi_static_mode", joint_trajectory_generator_->quasi_static_mode_, false);

  // set feedback gains for computed torque method
  double ctm_gain;
  getParam<double>(control_nh, "ctm_p_gain", ctm_gain, 1.0);
  joint_trajectory_generator_->ctm_p_gain_ =
      ctm_gain * Eigen::MatrixXd::Identity(joint_trajectory_generator_->getPinocchioModel()->nv,
                                           joint_trajectory_generator_->getPinocchioModel()->nv);

  getParam<double>(control_nh, "ctm_d_gain", ctm_gain, 1.0);
  if (joint_trajectory_generator_->quasi_static_mode_)
    ctm_gain = 0.0;  // set d gain to zero in quasi-static mode
  joint_trajectory_generator_->ctm_d_gain_ =
      ctm_gain * Eigen::MatrixXd::Identity(joint_trajectory_generator_->getPinocchioModel()->nv,
                                           joint_trajectory_generator_->getPinocchioModel()->nv);

  // set feedback gains for gimbal to zero
  std::vector<std::string> gimbal_names = joint_trajectory_generator_->getGimbalNames();
  for (int i = 0; i < gimbal_names.size(); i++)
  {
    int gimbal_index_v = joint_trajectory_generator_->getPinocchioModel()
                             ->joints[joint_trajectory_generator_->getPinocchioModel()->getJointId(gimbal_names.at(i))]
                             .idx_v();
    joint_trajectory_generator_->ctm_p_gain_(gimbal_index_v, gimbal_index_v) = 0.0;
    joint_trajectory_generator_->ctm_d_gain_(gimbal_index_v, gimbal_index_v) = 0.0;
  }
}

void JointTrajectoryGeneratorRos::publish()
{
  // for debug: send target q, dq, and ddq
  sensor_msgs::JointState target_q_msg;
  sensor_msgs::JointState target_dq_msg;
  sensor_msgs::JointState target_ddq_msg;
  target_q_msg.header.stamp = ros::Time::now();
  target_dq_msg.header.stamp = ros::Time::now();
  target_ddq_msg.header.stamp = ros::Time::now();

  std::vector<std::string> joint_names = joint_trajectory_generator_->getJointNames();
  Eigen::VectorXd curr_target_q = joint_trajectory_generator_->getCurrentTargetQ();
  Eigen::VectorXd curr_target_dq = joint_trajectory_generator_->getCurrentTargetDQ();
  Eigen::VectorXd curr_target_ddq = joint_trajectory_generator_->getCurrentTargetDDQ();
  for (int i = 0; i < joint_names.size(); i++)
  {
    if (joint_names.at(i).find("root") != std::string::npos)
      continue;  // skip root joint

    int joint_index_q = joint_trajectory_generator_->getPinocchioModel()
                            ->joints[joint_trajectory_generator_->getPinocchioModel()->getJointId(joint_names.at(i))]
                            .idx_q();
    int joint_index_v = joint_trajectory_generator_->getPinocchioModel()
                            ->joints[joint_trajectory_generator_->getPinocchioModel()->getJointId(joint_names.at(i))]
                            .idx_v();

    if (joint_index_q < 0 || joint_index_v < 0)
      continue;  // skip if joint index is invalid

    target_q_msg.name.push_back(joint_names.at(i));
    target_q_msg.position.push_back(curr_target_q(joint_index_q));

    target_dq_msg.name.push_back(joint_names.at(i));
    target_dq_msg.velocity.push_back(curr_target_dq(joint_index_v));

    target_ddq_msg.name.push_back(joint_names.at(i));
    target_ddq_msg.effort.push_back(curr_target_ddq(joint_index_v));
  }
  target_q_pub_.publish(target_q_msg);
  target_dq_pub_.publish(target_dq_msg);
  target_ddq_pub_.publish(target_ddq_msg);

  // for debug: send target end effector position and velocity
  geometry_msgs::Vector3 target_ee_pos_msg;
  geometry_msgs::Vector3 target_ee_vel_msg;
  geometry_msgs::Vector3 target_ee_acc_msg;
  Eigen::Vector3d target_ee_pos = joint_trajectory_generator_->getTargetEndEffectorPos();
  Eigen::Vector3d target_ee_vel = joint_trajectory_generator_->getTargetEndEffectorVel();
  Eigen::Vector3d target_ee_acc = joint_trajectory_generator_->getTargetEndEffectorAcc();
  target_ee_pos_msg.x = target_ee_pos(0);
  target_ee_pos_msg.y = target_ee_pos(1);
  target_ee_pos_msg.z = target_ee_pos(2);
  target_ee_vel_msg.x = target_ee_vel(0);
  target_ee_vel_msg.y = target_ee_vel(1);
  target_ee_vel_msg.z = target_ee_vel(2);
  target_ee_acc_msg.x = target_ee_acc(0);
  target_ee_acc_msg.y = target_ee_acc(1);
  target_ee_acc_msg.z = target_ee_acc(2);
  target_end_effector_pos_pub_.publish(target_ee_pos_msg);
  target_end_effector_vel_pub_.publish(target_ee_vel_msg);
  target_end_effector_acc_pub_.publish(target_ee_acc_msg);

  // for motion planner: is transforming or not
  std_msgs::UInt8 is_transforming_msg;
  is_transforming_msg.data = joint_trajectory_generator_->is_transforming_;
  is_transforming_pub_.publish(is_transforming_msg);
}

void JointTrajectoryGeneratorRos::jointStateCallback(const sensor_msgs::JointState msg)
{
  for (int i = 0; i < msg.name.size(); i++)
  {
    std::string joint_name = msg.name[i];
    int joint_id = joint_trajectory_generator_->getPinocchioModel()->getJointId(joint_name);
    if (joint_id == joint_trajectory_generator_->getPinocchioModel()->njoints)
      continue;  // skip this joint because there is not in kinematic tree

    // position
    if (msg.name.size() == msg.position.size())
    {
      int joint_index_q = joint_trajectory_generator_->getPinocchioModel()->joints[joint_id].idx_q();
      joint_trajectory_generator_->curr_q_(joint_index_q) = msg.position[i];
    }

    // velocity
    if (msg.name.size() == msg.velocity.size())
    {
      int joint_index_v = joint_trajectory_generator_->getPinocchioModel()->joints[joint_id].idx_v();
      joint_trajectory_generator_->curr_dq_(joint_index_v) = msg.velocity[i];
    }
  }
}

void JointTrajectoryGeneratorRos::targetEndEffectorPosCallback(const geometry_msgs::Vector3StampedConstPtr& msg)
{
  pinocchio::FrameIndex frame_id =
      joint_trajectory_generator_->getPinocchioModel()->getFrameId(joint_trajectory_generator_->end_effector_name_);
  pinocchio::framesForwardKinematics(*(joint_trajectory_generator_->getPinocchioModel()),
                                     *joint_trajectory_generator_->getPinocchioData(),
                                     joint_trajectory_generator_->getCurrentQ());
  Eigen::Vector3d x_curr = joint_trajectory_generator_->getPinocchioData()->oMf[frame_id].translation();
  Eigen::Vector3d x_des(msg->vector.x, msg->vector.y, msg->vector.z);

  if (msg->header.stamp.sec != 0 || msg->header.stamp.nsec != 0)
  {
    joint_trajectory_generator_->transform_duration_ = msg->header.stamp.sec + msg->header.stamp.nsec / 1000000000.0;
  }

  ROS_INFO_STREAM("[JointTrajectoryGeneratorRos] target end effector position: "
                  << x_des.transpose() << " with " << joint_trajectory_generator_->transform_duration_ << "s");
  ROS_INFO_STREAM("[JointTrajectoryGeneratorRos] current end effector position: " << x_curr.transpose());

  joint_trajectory_generator_->pos_trajectory_generator_.reset();
  joint_trajectory_generator_->pos_trajectory_generator_.generateTrajectory(
      x_curr, x_des, joint_trajectory_generator_->transform_duration_);

  joint_trajectory_generator_->is_transforming_ = 1;  // linear mode
  joint_trajectory_generator_->transform_start_time_ = ros::Time::now().toSec();
}

void JointTrajectoryGeneratorRos::circleTrajectoryCallback(const std_msgs::Float32MultiArrayConstPtr& msg)
{
  if (msg->data.size() != 2)
  {
    ROS_INFO_STREAM("[JointTrajectoryGeneratorRos] switch back to linear trajectory mode");
    joint_trajectory_generator_->is_transforming_ = 0;  // not transforming
    joint_trajectory_generator_->setCurrentTargetQ(joint_trajectory_generator_->getCurrentQ());
    joint_trajectory_generator_->setCurrentTargetDQ(
        Eigen::VectorXd::Zero(joint_trajectory_generator_->getPinocchioModel()->nv));
    joint_trajectory_generator_->setCurrentTargetDDQ(
        Eigen::VectorXd::Zero(joint_trajectory_generator_->getPinocchioModel()->nv));
    return;
  }

  pinocchio::FrameIndex frame_id =
      joint_trajectory_generator_->getPinocchioModel()->getFrameId(joint_trajectory_generator_->end_effector_name_);
  pinocchio::framesForwardKinematics(*(joint_trajectory_generator_->getPinocchioModel()),
                                     *joint_trajectory_generator_->getPinocchioData(),
                                     joint_trajectory_generator_->getCurrentQ());
  joint_trajectory_generator_->circle_trajectory_center_ =
      joint_trajectory_generator_->getPinocchioData()->oMf[frame_id].translation();
  joint_trajectory_generator_->circle_trajectory_radius_ = msg->data[0];
  joint_trajectory_generator_->circle_trajectory_angular_velocity_ = msg->data[1];

  ROS_INFO_STREAM("[JointTrajectoryGeneratorRos] circle trajectory with radius: "
                  << joint_trajectory_generator_->circle_trajectory_radius_
                  << " and angular velocity: " << joint_trajectory_generator_->circle_trajectory_angular_velocity_
                  << " center: " << joint_trajectory_generator_->circle_trajectory_center_.transpose());

  joint_trajectory_generator_->is_transforming_ = 2;  // circular mode
  joint_trajectory_generator_->transform_start_time_ = ros::Time::now().toSec();
}

void JointTrajectoryGeneratorRos::directJointAngleCallback(const sensor_msgs::JointStateConstPtr& msg)
{
  if (msg->name.size() != msg->position.size())
  {
    ROS_ERROR_STREAM("[JointTrajectoryGeneratorRos] size of joint name: " << msg->name.size() << " and joint position: "
                                                                          << msg->position.size() << " is not same");
    return;
  }

  joint_trajectory_generator_->transform_duration_ = msg->header.stamp.sec + msg->header.stamp.nsec / 1000000000.0;

  joint_trajectory_generator_->init_target_q_ = joint_trajectory_generator_->getCurrentQ();
  joint_trajectory_generator_->final_target_q_ = joint_trajectory_generator_->getCurrentQ();

  for (int i = 0; i < msg->name.size(); i++)
  {
    std::string joint_name = msg->name.at(i);
    int joint_id = joint_trajectory_generator_->getPinocchioModel()->getJointId(joint_name);
    if (joint_id == joint_trajectory_generator_->getPinocchioModel()->njoints)
    {
      ROS_WARN_STREAM("[JointTrajectoryGeneratorRos] there is not joint named \"" << joint_name << "\"");
      continue;  // skip this joint because there is not in kinematic tree
    }

    int joint_index_q = joint_trajectory_generator_->getPinocchioModel()->joints[joint_id].idx_q();
    joint_trajectory_generator_->final_target_q_(joint_index_q) = msg->position[i];
  }

  ROS_INFO_STREAM("[JointTrajectoryGeneratorRos] move to " << joint_trajectory_generator_->final_target_q_.transpose()
                                                           << " with "
                                                           << joint_trajectory_generator_->transform_duration_ << "s");

  joint_trajectory_generator_->is_transforming_ = 3;
  joint_trajectory_generator_->transform_start_time_ = ros::Time::now().toSec();
}

void JointTrajectoryGeneratorRos::joyCallback(const sensor_msgs::JoyConstPtr& msg)
{
  sensor_msgs::Joy joy_cmd = joyParse(*msg);
  joint_trajectory_generator_->joy_msg_ = joy_cmd;
  if (joy_cmd.axes.size() == 0 || joy_cmd.buttons.size() == 0)
  {
    ROS_WARN("[JointTrajectoryGeneratorRos] the joystick type is not supported (buttons: %d, axes: %d)",
             (int)msg->buttons.size(), (int)msg->axes.size());
    return;
  }

  /* mode switch */
  if (joy_cmd.buttons[JOY_BUTTON_REAR_RIGHT_2])
  {
    if (joint_trajectory_generator_->is_transforming_ != 4)
    {
      joint_trajectory_generator_->is_transforming_ = 4;  // joy control mode
      ROS_INFO_STREAM("[JointTrajectoryGeneratorRos] switch to joy control mode");
    }
  }
  else
  {
    if (joint_trajectory_generator_->is_transforming_ == 4)
    {
      joint_trajectory_generator_->is_transforming_ = 0;  // not transforming
      ROS_INFO_STREAM("[JointTrajectoryGeneratorRos] switch back to no transform mode");
    }
  }
}
