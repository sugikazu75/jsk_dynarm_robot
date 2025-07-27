#include <dynarm/control/joint_trajectory_generator.h>
#include <dynarm/model/manipulator_model.h>

using namespace aerial_robot_control;

jointTrajectoryGenerator::jointTrajectoryGenerator(
    ros::NodeHandle nh, std::shared_ptr<aerial_robot_dynamics::PinocchioRobotModel> pinocchio_robot_model)
  : nh_(nh), pinocchio_robot_model_(pinocchio_robot_model)
{
  pinocchio_model_ = pinocchio_robot_model_->getModel();
  pinocchio_data_ = pinocchio_robot_model_->getData();

  rosParamInit();

  id_torque_pub_ = nh_.advertise<sensor_msgs::JointState>("debug/id_debug/torque", 1);
  id_velocity_pub_ = nh_.advertise<sensor_msgs::JointState>("debug/id_debug/velocity", 1);
  id_acc_pub_ = nh_.advertise<sensor_msgs::JointState>("debug/id_debug/acceleration", 1);
  id_tau_by_thrust_pub_ = nh_.advertise<sensor_msgs::JointState>("debug/id_debug/tau_by_thrust", 1);
  is_transforming_pub_ = nh_.advertise<std_msgs::UInt8>("is_transforming", 1);
  id_time_pub_ = nh_.advertise<std_msgs::Float32>("debug/id_debug/solve_time", 1);
  id_result_torque_pub_ = nh_.advertise<std_msgs::Float32>("debug/id_debug/result/torque", 1);
  id_result_thrust_pub_ = nh_.advertise<std_msgs::Float32>("debug/id_debug/result/thrust", 1);
  thrust_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("debug/id_debug/thrust", 1);
  rotor_wrench_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("rotor_wrench", 1);
  target_end_effector_pos_pub_ = nh_.advertise<geometry_msgs::Vector3>("debug/target_ee_pos", 1);
  target_end_effector_vel_pub_ = nh_.advertise<geometry_msgs::Vector3>("debug/target_ee_vel", 1);
  target_end_effector_acc_pub_ = nh_.advertise<geometry_msgs::Vector3>("debug/target_ee_acc", 1);
  dummy_joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);

  joint_state_sub_ = nh_.subscribe("joint_states", 1, &jointTrajectoryGenerator::jointStateCallback, this);
  target_end_effector_final_pos_sub_ =
      nh_.subscribe("target_ee_final_pos", 1, &jointTrajectoryGenerator::targetEndEffectorPosCallback, this);
  circle_trajectory_sub_ =
      nh_.subscribe("circle_trajectory", 1, &jointTrajectoryGenerator::circleTrajectoryCallback, this);
  direct_joint_angle_sub_ =
      nh_.subscribe("direct_joint_angle", 1, &jointTrajectoryGenerator::directJointAngleCallback, this);

  // for rotor wrench visualization
  robot_ns_ = ros::this_node::getNamespace();
  if (!robot_ns_.empty() && robot_ns_[0] == '/')
    robot_ns_ = robot_ns_.substr(1);
  rotor_wrench_pub_index_ = 0;

  is_transforming_ = 0;

  target_ee_pos_.setZero();
  target_ee_vel_.setZero();
  target_ee_acc_.setZero();

  curr_q_ = Eigen::VectorXd::Zero(pinocchio_robot_model_->getModel()->nq);
  curr_target_q_ = Eigen::VectorXd::Zero(pinocchio_robot_model_->getModel()->nq);
  curr_target_dq_ = Eigen::VectorXd::Zero(pinocchio_robot_model_->getModel()->nv);
  curr_target_ddq_ = Eigen::VectorXd::Zero(pinocchio_robot_model_->getModel()->nv);

  curr_target_tau_ = Eigen::VectorXd::Zero(pinocchio_robot_model_->getModel()->nv);
  curr_target_thrust_ = Eigen::VectorXd::Zero(pinocchio_robot_model_->getRotorNum());
  curr_target_gimbal_angle_ =
      Eigen::VectorXd::Zero(pinocchio_robot_model_->getRotorNum() / rotor_devider_ * 2);  // roll and pitch

  // initial joint angle
  std::vector<double> init_target_q;
  ros::NodeHandle control_nh(nh_, "controller");
  control_nh.getParam("init_target_q", init_target_q);
  if (init_target_q.size() != pinocchio_model_->nq)
    ROS_ERROR_STREAM("[dragon_arm] nq: " << pinocchio_model_->nq << " and initial joint angle size: "
                                         << init_target_q.size() << " in ros parameter is not same");
  for (int i = 0; i < pinocchio_model_->nq; i++)
  {
    curr_target_q_(i) = init_target_q.at(i);
  }
  ROS_INFO_STREAM("[dragon_arm][control] initial target joint angle: " << curr_target_q_.transpose());
}

void jointTrajectoryGenerator::rosParamInit()
{
  ros::NodeHandle control_nh(nh_, "controller");
  getParam<bool>(control_nh, "nonlinear_mode", nonlinear_mode_, true);
  getParam<std::string>(control_nh, "end_effector_name", end_effector_name_, "");
  getParam<double>(control_nh, "transform_duration", transform_duration_, 1.0);
  getParam<double>(control_nh, "gimbal_delta_max", gimbal_delta_max_, M_PI);

  ros::NodeHandle model_nh(nh_, "model");
  getParam<int>(model_nh, "rotor_devider", rotor_devider_, 1);
}

void jointTrajectoryGenerator::reset()
{
  is_transforming_ = 0;
  nlp_first_run_ = true;
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
      if (curr_time < (2 * M_PI / circle_trajectory_angular_velocity_))
        curr_time = 0.0;  // wait init
      target_ee_pos_ =
          circle_trajectory_center_ +
          Eigen::Vector3d(0.0, circle_trajectory_radius_ * cos(circle_trajectory_angular_velocity_ * curr_time),
                          circle_trajectory_radius_ * sin(circle_trajectory_angular_velocity_ * curr_time));
      target_ee_vel_ = Eigen::Vector3d(0.0,
                                       -circle_trajectory_radius_ * circle_trajectory_angular_velocity_ *
                                           sin(circle_trajectory_angular_velocity_ * curr_time),
                                       circle_trajectory_radius_ * circle_trajectory_angular_velocity_ *
                                           cos(circle_trajectory_angular_velocity_ * curr_time));
      target_ee_acc_ = Eigen::Vector3d(
          0.0,
          -circle_trajectory_radius_ * circle_trajectory_angular_velocity_ * circle_trajectory_angular_velocity_ *
              cos(circle_trajectory_angular_velocity_ * curr_time),
          -circle_trajectory_radius_ * circle_trajectory_angular_velocity_ * circle_trajectory_angular_velocity_ *
              sin(circle_trajectory_angular_velocity_ * curr_time));
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
    pinocchio::computeFrameJacobian(*pinocchio_model_, *pinocchio_data_, curr_target_q_, frame_id, pinocchio::WORLD,
                                    J6);  // world frame. q is (target or current)
    Eigen::MatrixXd J = J6.topRows(3);    // position
    Eigen::MatrixXd JJt = J * J.transpose() + 1e-12 * Eigen::MatrixXd::Identity(3, 3);
    curr_target_dq_ = J.transpose() * JJt.ldlt().solve(target_ee_vel_);  // target velocity

    // calculate target ddq
    pinocchio::forwardKinematics(*pinocchio_model_, *pinocchio_data_, curr_target_q_,
                                 curr_target_dq_);  // q is (target or current)
    pinocchio::computeJointJacobiansTimeVariation(*pinocchio_model_, *pinocchio_data_, curr_target_q_,
                                                  curr_target_dq_);  // q is (target or current)
    Eigen::MatrixXd Jdot6 = Eigen::MatrixXd::Zero(6, pinocchio_model_->nv);
    pinocchio::getFrameJacobianTimeVariation(*pinocchio_model_, *pinocchio_data_, frame_id, pinocchio::WORLD, Jdot6);
    Eigen::MatrixXd Jdot = Jdot6.topRows(3);  // position
    curr_target_ddq_ = J.transpose() * JJt.ldlt().solve(target_ee_acc_ - Jdot * curr_target_dq_);
  }
  else if (is_transforming_ == 3)  // direct command. linear interpolation
  {
    double curr_time = ros::Time::now().toSec() - transform_start_time_;
    if (transform_duration_ == 0.0)
    {
      curr_target_q_ = final_target_q_;
      return;
    }
    Eigen::VectorXd delta_q = final_target_q_ - init_target_q_;
    curr_target_q_ = init_target_q_ + delta_q * (curr_time / transform_duration_);
    curr_target_dq_ = delta_q / transform_duration_;
  }
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

    int gimbal_roll_index =
        pinocchio_model_->joints[pinocchio_model_->getJointId("gimbal" + std::to_string(i + 1) + "_roll")].idx_q();
    int gimbal_pitch_index =
        pinocchio_model_->joints[pinocchio_model_->getJointId("gimbal" + std::to_string(i + 1) + "_pitch")].idx_q();

    gimbal_processed_q(gimbal_roll_index) = -rpy(0);   // roll
    gimbal_processed_q(gimbal_pitch_index) = -rpy(1);  // pitch
  }
  return gimbal_processed_q;
}

void jointTrajectoryGenerator::getGimbalNominalAngles()
{
  curr_target_q_ = getGimbalNominalAngles(curr_target_q_);
}

bool jointTrajectoryGenerator::solveInverseDynamics()
{
  Eigen::VectorXd id_result;
  bool solved;
  if (nonlinear_mode_)
  {
    solved = nonlinearInverseDynamics(curr_target_q_, curr_target_dq_, curr_target_ddq_, id_result);
    curr_target_tau_ = id_result.head(pinocchio_model_->nv);
    curr_target_thrust_ = id_result.segment(pinocchio_model_->nv, pinocchio_robot_model_->getRotorNum());
    curr_target_gimbal_angle_ = id_result.tail(pinocchio_robot_model_->getRotorNum() / rotor_devider_ * 2);
  }
  else
  {
    solved = pinocchio_robot_model_->inverseDynamics(curr_target_q_, curr_target_dq_, curr_target_ddq_, id_result);
    curr_target_tau_ = id_result.head(pinocchio_model_->nv);
    curr_target_thrust_ = id_result.tail(pinocchio_robot_model_->getRotorNum());
  }

  if (!solved)
  {
    ROS_ERROR_STREAM("[dragon_arm][control] Inverse dynamics failed to solve"
                     << "\n Current target q: " << curr_target_q_.transpose() << "\n Current target dq: "
                     << curr_target_dq_.transpose() << "\n Current target ddq: " << curr_target_ddq_.transpose());
  }

  return solved;
}

void jointTrajectoryGenerator::updateTargetGimbalAngle()
{
  for (int i = 0; i < pinocchio_robot_model_->getRotorNum() / rotor_devider_; i++)
  {
    std::string gimbal_roll_name = "gimbal" + std::to_string(i + 1) + "_roll";
    std::string gimbal_pitch_name = "gimbal" + std::to_string(i + 1) + "_pitch";

    int gimbal_roll_index = pinocchio_model_->joints[pinocchio_model_->getJointId(gimbal_roll_name)].idx_q();
    int gimbal_pitch_index = pinocchio_model_->joints[pinocchio_model_->getJointId(gimbal_pitch_name)].idx_q();

    curr_target_q_(gimbal_roll_index) = curr_target_gimbal_angle_(2 * i + 0);
    curr_target_q_(gimbal_pitch_index) = curr_target_gimbal_angle_(2 * i + 1);
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
  // for debug: rotor wrench
  std_msgs::Float32MultiArray thrust_msg;
  for (int i = 0; i < pinocchio_robot_model_->getRotorNum(); i++)
    thrust_msg.data.push_back(curr_target_thrust_(i));
  thrust_pub_.publish(thrust_msg);

  geometry_msgs::WrenchStamped rotor_wrench_msg;
  rotor_wrench_msg.header.stamp = ros::Time::now();
  rotor_wrench_msg.header.frame_id = robot_ns_ + "/thrust" + std::to_string(rotor_wrench_pub_index_ + 1);
  rotor_wrench_msg.wrench.force.x = 0.0;
  rotor_wrench_msg.wrench.force.y = 0.0;
  rotor_wrench_msg.wrench.force.z = curr_target_thrust_(rotor_wrench_pub_index_);
  rotor_wrench_msg.wrench.torque.x = 0.0;
  rotor_wrench_msg.wrench.torque.y = 0.0;
  rotor_wrench_msg.wrench.torque.z = pinocchio_robot_model_->getRotorDirection(rotor_wrench_pub_index_) *
                                     pinocchio_robot_model_->getMFRate() * curr_target_thrust_(rotor_wrench_pub_index_);
  rotor_wrench_pub_.publish(rotor_wrench_msg);
  rotor_wrench_pub_index_ = (rotor_wrench_pub_index_ + 1) % pinocchio_robot_model_->getRotorNum();

  // for debug: send target torque, velocity, acceleration, and torque generated by thrust
  sensor_msgs::JointState id_torque_msg;
  id_torque_msg.header.stamp = ros::Time::now();
  sensor_msgs::JointState id_velocity_msg;
  id_velocity_msg.header.stamp = ros::Time::now();
  sensor_msgs::JointState id_acc_msg;
  id_acc_msg.header.stamp = ros::Time::now();
  sensor_msgs::JointState id_tau_by_thrust_msg;

  Eigen::MatrixXd tauext_partial_thrust = pinocchio_robot_model_->computeTauExtByThrustDerivative(curr_target_q_);
  Eigen::VectorXd tauext = tauext_partial_thrust * curr_target_thrust_;

  for (int i = 0; i < pinocchio_model_->njoints; i++)
  {
    int joint_index_q = pinocchio_model_->joints[pinocchio_model_->getJointId(pinocchio_model_->names[i])].idx_q();
    int joint_index_v = pinocchio_model_->joints[pinocchio_model_->getJointId(pinocchio_model_->names[i])].idx_v();

    if (joint_index_q < 0 || joint_index_v < 0)
      continue;  // skip if joint index is invalid

    id_torque_msg.name.push_back(pinocchio_model_->names[i]);
    id_torque_msg.position.push_back(curr_target_q_(joint_index_q));
    id_torque_msg.effort.push_back(curr_target_tau_(joint_index_v));

    id_velocity_msg.name.push_back(pinocchio_model_->names[i]);
    id_velocity_msg.velocity.push_back(curr_target_dq_(joint_index_v));

    id_acc_msg.name.push_back(pinocchio_model_->names[i]);
    id_acc_msg.effort.push_back(curr_target_ddq_(joint_index_v));

    id_tau_by_thrust_msg.name.push_back(pinocchio_model_->names[i]);
    id_tau_by_thrust_msg.effort.push_back(tauext(joint_index_v));
  }

  id_torque_pub_.publish(id_torque_msg);
  id_velocity_pub_.publish(id_velocity_msg);
  id_acc_pub_.publish(id_acc_msg);
  id_tau_by_thrust_pub_.publish(id_tau_by_thrust_msg);

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

  // for debug: send ID solve time
  std_msgs::Float32 id_time_msg;
  if (nonlinear_mode_)
    id_time_msg.data = nlp_solve_time_;
  else
    id_time_msg.data = pinocchio_robot_model_->getLatestIdSolveTime();
  id_time_pub_.publish(id_time_msg);

  // for debug: ID result.
  std_msgs::Float32 id_result_msg;
  id_result_msg.data = curr_target_tau_.dot(curr_target_tau_);
  id_result_torque_pub_.publish(id_result_msg);
  id_result_msg.data = curr_target_thrust_.dot(curr_target_thrust_);
  id_result_thrust_pub_.publish(id_result_msg);
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
  transform_end_time_ = ros::Time::now().toSec() + transform_duration_;
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
  transform_end_time_ = ros::Time::now().toSec() + transform_duration_;
}
