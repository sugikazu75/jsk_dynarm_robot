#include <dynarm/control/manipulator_controller.h>

using namespace aerial_robot_control;

ManipulatorController::ManipulatorController() : ControlBase()
{
}

void ManipulatorController::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                       boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                       boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                       boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                       double ctrl_loop_du)
{
  ControlBase::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_du);
  rosParamInit();

  four_axis_command_pub_ = nh_.advertise<spinal::FourAxisCommand>("four_axes/command", 1);
  joints_control_pub_ = nh_.advertise<sensor_msgs::JointState>("joints_ctrl", 1);
  gimbals_control_pub_ = nh_.advertise<sensor_msgs::JointState>("gimbals_ctrl", 1);
  id_torque_pub_ = nh_.advertise<sensor_msgs::JointState>("debug/id_debug/torque", 1);
  id_velocity_pub_ = nh_.advertise<sensor_msgs::JointState>("debug/id_debug/velocity", 1);
  id_acc_pub_ = nh_.advertise<sensor_msgs::JointState>("debug/id_debug/acceleration", 1);
  rotor_wrench_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("rotor_wrench", 1);

  target_end_effector_pos_sub_ =
      nh_.subscribe("target_ee_pos", 1, &ManipulatorController::targetEndEffectorPosCallback, this);

  robot_ns_ = ros::this_node::getNamespace();
  if (!robot_ns_.empty() && robot_ns_[0] == '/')
    robot_ns_ = robot_ns_.substr(1);

  rotor_wrench_pub_index_ = 0;

  dragon_arm_robot_model_ = boost::dynamic_pointer_cast<aerial_robot_model::ManipulatorRobotModel>(robot_model);
  pinocchio_robot_model_ = dragon_arm_robot_model_->getPinocchioRobotModel();
  pinocchio_model_ = pinocchio_robot_model_->getModel();
  pinocchio_data_ = pinocchio_robot_model_->getData();

  curr_target_q_.resize(pinocchio_robot_model_->getModel()->nq);
  prev_target_q_.resize(pinocchio_robot_model_->getModel()->nq);
  curr_target_dq_.resize(pinocchio_robot_model_->getModel()->nv);
  prev_target_dq_.resize(pinocchio_robot_model_->getModel()->nv);
  thrusts_.resize(pinocchio_robot_model_->getRotorNum());

  is_initialized_ = false;
  init_target_q_.resize(pinocchio_robot_model_->getModel()->nq);

  init_target_q_ << 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.3, 0.0, 0.0, 0.0;
}

void ManipulatorController::rosParamInit()
{
  ros::NodeHandle control_nh(nh_, "controller");
  getParam<std::string>(control_nh, "end_effector_name", end_effector_name_, "");
  getParam<double>(control_nh, "transform_duration", transform_duration_, 1.0);
}

void ManipulatorController::reset()
{
  ControlBase::reset();

  first_run_ = true;
  is_transforming_ = false;
  is_initialized_ = false;

  ROS_INFO_STREAM("[dragon_arm][control] reset");
}

bool ManipulatorController::update()
{
  if (!ControlBase::update())
    return false;

  controlCore();
  sendCmd();

  return true;
}

void ManipulatorController::controlCore()
{
  if (first_run_)
  {
    curr_target_q_ = dragon_arm_robot_model_->getCurrentJointPositions();
    curr_target_dq_ = Eigen::VectorXd::Zero(pinocchio_robot_model_->getModel()->nv);
    first_run_ = false;
  }
  prev_target_q_ = curr_target_q_;
  prev_target_dq_ = curr_target_dq_;

  // determine q to calculate inverse dynamics
  if (!is_initialized_)
  {
    curr_target_q_ = dragon_arm_robot_model_->getCurrentJointPositions();
  }
  else
  {
    if (is_transforming_)  // calculate target joint angle by inverse kinematics
    {
      double curr_time = ros::Time::now().toSec() - transform_start_time_;
      Eigen::Vector3d pos, vel, acc;
      pos_trajectory_generator_.eval(curr_time, pos, vel, acc);

      Eigen::VectorXd curr_q = dragon_arm_robot_model_->getCurrentJointPositions();
      Eigen::VectorXd curr_target_q;
      bool solved = motion_planning::solveIK(*pinocchio_model_, *pinocchio_data_,
                                             pinocchio_model_->getFrameId(end_effector_name_), pos, curr_q,
                                             curr_target_q_, false, 1000, 1e-4);

      if (!solved)
      {
        ROS_ERROR_STREAM("[dragon_arm][control] IK solution not found for target position: " << pos.transpose());
      }
    }
  }

  // process gimbal angles
  curr_target_q_ = dragon_arm_robot_model_->getGimbalNominalAngles(curr_target_q_);

  // calculate target joint velocities and accelerations
  curr_target_dq_ = Eigen::VectorXd::Zero(pinocchio_robot_model_->getModel()->nv);
  curr_target_ddq_ = Eigen::VectorXd::Zero(pinocchio_robot_model_->getModel()->nv);

  if (is_transforming_)
  {
    pinocchio::difference(*pinocchio_model_, prev_target_q_, curr_target_q_, curr_target_dq_);
    curr_target_dq_ = curr_target_dq_ / ctrl_loop_du_;

    curr_target_ddq_ = (curr_target_dq_ - prev_target_dq_) / ctrl_loop_du_;

    // std::cout << "[dragon_arm][control] current target dq: " << curr_target_dq_.transpose() << std::endl;
    // std::cout << "[dragon_arm][control] current target ddq: " << curr_target_ddq_.transpose() << std::endl;
  }

  // calculate inverse dynamics
  Eigen::VectorXd curr_target_full_tau =
      Eigen::VectorXd::Zero(pinocchio_robot_model_->getModel()->nv + pinocchio_robot_model_->getRotorNum());
  bool solved =
      pinocchio_robot_model_->inverseDynamics(curr_target_q_, curr_target_dq_, curr_target_ddq_, curr_target_full_tau);

  if (solved)
  {
    curr_target_tau_ = curr_target_full_tau.head(pinocchio_robot_model_->getModel()->nv);
    thrusts_ = curr_target_full_tau.tail(pinocchio_robot_model_->getRotorNum());
  }
  else
  {
    ROS_ERROR_STREAM("[dragon_arm][control] Inverse dynamics failed to solve"
                     << "\n Current target q: " << curr_target_q_.transpose() << "\n Current target dq: "
                     << curr_target_dq_.transpose() << "\n Current target ddq: " << curr_target_ddq_.transpose());
  }

  // state transition
  if (!is_initialized_)
  {
    // check joint angle convergence
    bool is_converged = true;
    Eigen::VectorXd curr_q = dragon_arm_robot_model_->getCurrentJointPositions();
    for (int i = 0; i < robot_model_->getRotorNum() - 1; i++)
    {
      std::string joint_pitch_name = "joint" + std::to_string(i + 1) + "_pitch";
      std::string joint_yaw_name = "joint" + std::to_string(i + 1) + "_yaw";

      int joint_pitch_index_q = pinocchio_model_->joints[pinocchio_model_->getJointId(joint_pitch_name)].idx_q();
      int joint_yaw_index_q = pinocchio_model_->joints[pinocchio_model_->getJointId(joint_yaw_name)].idx_q();

      if (fabs(curr_q(joint_pitch_index_q) - init_target_q_(joint_pitch_index_q)) > 0.05 ||
          fabs(curr_q(joint_yaw_index_q) - init_target_q_(joint_yaw_index_q)) > 0.05)
      {
        is_converged = false;
        break;
      }
    }
    if (is_converged)
    {
      is_initialized_ = true;
      ROS_INFO_STREAM("[dragon_arm][control] initialized with target q: " << curr_target_q_.transpose());
      curr_target_q_ = init_target_q_;
      curr_target_q_ = dragon_arm_robot_model_->getGimbalNominalAngles(curr_target_q_);
    }
  }
  else
  {
    if (is_transforming_)
    {
      if (ros::Time::now().toSec() >= transform_end_time_)
      {
        is_transforming_ = false;
        ROS_INFO_STREAM("[dragon_arm][control] end effector position transformation completed.");
      }
    }
  }
}

void ManipulatorController::sendCmd()
{
  sendFourAxisCommand();
  sendJointCommand();
  sendGimbalCommand();

  // for debug: send target torque
  sensor_msgs::JointState id_torque_msg;
  sensor_msgs::JointState id_velocity_msg;
  sensor_msgs::JointState id_acc_msg;
  for (int i = 0; i < pinocchio_model_->njoints; i++)
  {
    int joint_index_q = pinocchio_model_->joints[pinocchio_model_->getJointId(pinocchio_model_->names[i])].idx_q();
    int joint_index_dq = pinocchio_model_->joints[pinocchio_model_->getJointId(pinocchio_model_->names[i])].idx_v();

    if (joint_index_q < 0)
      continue;  // skip if joint index is invalid

    id_torque_msg.name.push_back(pinocchio_model_->names[i]);
    id_torque_msg.position.push_back(curr_target_q_(joint_index_q));
    id_torque_msg.effort.push_back(curr_target_tau_(joint_index_q));

    id_velocity_msg.name.push_back(pinocchio_model_->names[i]);
    id_velocity_msg.velocity.push_back(curr_target_dq_(joint_index_dq));

    id_acc_msg.name.push_back(pinocchio_model_->names[i]);
    id_acc_msg.effort.push_back(curr_target_ddq_(joint_index_dq));
  }
  id_torque_pub_.publish(id_torque_msg);
  id_velocity_pub_.publish(id_velocity_msg);
  id_acc_pub_.publish(id_acc_msg);
}

void ManipulatorController::sendFourAxisCommand()
{
  // send target thrust
  spinal::FourAxisCommand four_axis_command_msg;
  for (int i = 0; i < robot_model_->getRotorNum(); i++)
  {
    four_axis_command_msg.base_thrust.push_back(thrusts_(i));
  }
  four_axis_command_pub_.publish(four_axis_command_msg);

  // for debug: rotor wrench
  geometry_msgs::WrenchStamped rotor_wrench_msg;
  rotor_wrench_msg.header.stamp = ros::Time::now();
  rotor_wrench_msg.header.frame_id = robot_ns_ + "/thrust" + std::to_string(rotor_wrench_pub_index_ + 1);
  rotor_wrench_msg.wrench.force.x = 0.0;
  rotor_wrench_msg.wrench.force.y = 0.0;
  rotor_wrench_msg.wrench.force.z = thrusts_(rotor_wrench_pub_index_);
  rotor_wrench_msg.wrench.torque.x = 0.0;
  rotor_wrench_msg.wrench.torque.y = 0.0;
  rotor_wrench_msg.wrench.torque.z = pinocchio_robot_model_->getMFRate() * thrusts_(rotor_wrench_pub_index_);
  rotor_wrench_pub_.publish(rotor_wrench_msg);
  rotor_wrench_pub_index_ = (rotor_wrench_pub_index_ + 1) % pinocchio_robot_model_->getRotorNum();
}

void ManipulatorController::sendJointCommand()
{
  // send target joint angle, velocity and torque
  sensor_msgs::JointState joint_state_msg;
  joint_state_msg.header.stamp = ros::Time::now();

  // if not initialized, set target joint angle to init target q
  if (!is_initialized_)
  {
    curr_target_q_ = init_target_q_;
    curr_target_dq_ = Eigen::VectorXd::Zero(pinocchio_model_->nv);
    curr_target_tau_ = Eigen::VectorXd::Zero(pinocchio_model_->nv);
  }

  for (int i = 0; i < robot_model_->getRotorNum() - 1; i++)
  {
    std::string joint_pitch_name = "joint" + std::to_string(i + 1) + "_pitch";
    std::string joint_yaw_name = "joint" + std::to_string(i + 1) + "_yaw";

    int joint_pitch_index_q = pinocchio_model_->joints[pinocchio_model_->getJointId(joint_pitch_name)].idx_q();
    int joint_yaw_index_q = pinocchio_model_->joints[pinocchio_model_->getJointId(joint_yaw_name)].idx_q();
    int joint_pitch_index_dq = pinocchio_model_->joints[pinocchio_model_->getJointId(joint_pitch_name)].idx_v();
    int joint_yaw_index_dq = pinocchio_model_->joints[pinocchio_model_->getJointId(joint_yaw_name)].idx_v();

    joint_state_msg.name.push_back(joint_pitch_name);
    joint_state_msg.name.push_back(joint_yaw_name);
    joint_state_msg.position.push_back(curr_target_q_(joint_pitch_index_q));
    joint_state_msg.position.push_back(curr_target_q_(joint_yaw_index_q));
    joint_state_msg.velocity.push_back(curr_target_dq_(joint_pitch_index_dq));
    joint_state_msg.velocity.push_back(curr_target_dq_(joint_yaw_index_dq));
    joint_state_msg.effort.push_back(curr_target_tau_(joint_pitch_index_q));
    joint_state_msg.effort.push_back(curr_target_tau_(joint_yaw_index_q));
  }

  joints_control_pub_.publish(joint_state_msg);
}

void ManipulatorController::sendGimbalCommand()
{
  // send gimbal nominal angles
  Eigen::VectorXd curr_q = dragon_arm_robot_model_->getCurrentJointPositions();
  Eigen::VectorXd curr_q_gimbal_processed = dragon_arm_robot_model_->getGimbalNominalAngles(curr_q);
  Eigen::VectorXd target_gimbal_angles_q = curr_q_gimbal_processed;

  sensor_msgs::JointState joint_state_msg;
  joint_state_msg.header.stamp = ros::Time::now();
  for (int i = 0; i < robot_model_->getRotorNum(); i++)
  {
    std::string gimbal_roll_name = "gimbal" + std::to_string(i + 1) + "_roll";
    std::string gimbal_pitch_name = "gimbal" + std::to_string(i + 1) + "_pitch";

    int gimbal_roll_index = pinocchio_model_->joints[pinocchio_model_->getJointId(gimbal_roll_name)].idx_q();
    int gimbal_pitch_index = pinocchio_model_->joints[pinocchio_model_->getJointId(gimbal_pitch_name)].idx_q();

    joint_state_msg.name.push_back(gimbal_roll_name);
    joint_state_msg.name.push_back(gimbal_pitch_name);
    joint_state_msg.position.push_back(target_gimbal_angles_q(gimbal_roll_index));
    joint_state_msg.position.push_back(target_gimbal_angles_q(gimbal_pitch_index));
  }
  gimbals_control_pub_.publish(joint_state_msg);
}

void ManipulatorController::targetEndEffectorPosCallback(const geometry_msgs::Vector3ConstPtr& msg)
{
  pinocchio::FrameIndex frame_id = pinocchio_model_->getFrameId(end_effector_name_);
  Eigen::Vector3d x_curr = pinocchio_data_->oMf[frame_id].translation();
  Eigen::Vector3d x_des(msg->x, msg->y, msg->z);

  ROS_INFO_STREAM("[dragon_arm][control] target end effector position received: " << x_des.transpose());
  ROS_INFO_STREAM("[dragon_arm][control] current end effector position: " << x_curr.transpose());

  pos_trajectory_generator_.reset();
  pos_trajectory_generator_.generateTrajectory(x_curr, x_des, transform_duration_);

  is_transforming_ = true;
  transform_start_time_ = ros::Time::now().toSec();
  transform_end_time_ = ros::Time::now().toSec() + transform_duration_;
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::ManipulatorController, aerial_robot_control::ControlBase);
