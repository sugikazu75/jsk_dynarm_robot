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

  dragon_arm_robot_model_ = boost::dynamic_pointer_cast<aerial_robot_model::ManipulatorRobotModel>(robot_model);
  pinocchio_robot_model_ = dragon_arm_robot_model_->getPinocchioRobotModel();
  pinocchio_model_ = pinocchio_robot_model_->getModel();
  pinocchio_data_ = pinocchio_robot_model_->getData();

  joint_trajectory_generator_ros_ = std::make_shared<JointTrajectoryGeneratorRos>(nh_, pinocchio_robot_model_);
  joint_trajectory_generator_ = joint_trajectory_generator_ros_->getJointTrajectoryGenerator();

  nonlinear_inverse_dynamics_solver_ros_ =
      std::make_shared<aerial_robot_model::NonlinearInverseDynamicsRos>(nh_, pinocchio_robot_model_);
  nonlinear_inverse_dynamics_solver_ = nonlinear_inverse_dynamics_solver_ros_->getNonlinearInverseDynamicsSolver();

  trajectory_generator_ros_ = std::make_shared<TrajectoryGeneratorRos>(
      nh_, pinocchio_robot_model_, joint_trajectory_generator_ros_, nonlinear_inverse_dynamics_solver_ros_);
  trajectory_generator_ = trajectory_generator_ros_->getTrajectoryGenerator();

  rosParamInit();

  four_axis_command_pub_ = nh_.advertise<spinal::FourAxisCommand>("four_axes/command", 1);
  joints_control_pub_ = nh_.advertise<sensor_msgs::JointState>("joints_ctrl", 1);
  dynamixel_torque_enable_pub_ = nh_.advertise<spinal::ServoTorqueCmd>("servo/torque_enable", 1);
  gimbals_control_pub_ = nh_.advertise<sensor_msgs::JointState>("gimbals_ctrl", 1);
  robstride_servo_on_pub_ = nh_.advertise<std_msgs::Empty>("robstride_servo_on", 1);

  tau_by_thrust_pub_ = nh_.advertise<sensor_msgs::JointState>("debug/id_debug/tau_by_thrust", 1);
  rnea_solution_pub_ = nh_.advertise<sensor_msgs::JointState>("debug/id_debug/rnea_solution", 1);

  loadJointNames();
  loadGimbalNames();
}

void ManipulatorController::loadJointNames()
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

void ManipulatorController::loadGimbalNames()
{
  gimbal_names_.clear();
  gimbal_index_map_.clear();
  for (int i = 0; i < pinocchio_model_->njoints; i++)
  {
    std::string joint_name = pinocchio_model_->names[i];
    if (joint_name.find("gimbal") != std::string::npos)
    {
      gimbal_names_.push_back(joint_name);
      gimbal_index_map_[joint_name] = gimbal_names_.size() - 1;
    }
  }
}

void ManipulatorController::rosParamInit()
{
}

void ManipulatorController::reset()
{
  ControlBase::reset();

  joint_trajectory_generator_->reset();
  nonlinear_inverse_dynamics_solver_->reset();
  trajectory_generator_->reset();
}

void ManipulatorController::activate()
{
  ControlBase::activate();

  // enable torque for gimbals
  spinal::ServoTorqueCmd gimbal_torque_enable_msg;
  gimbal_torque_enable_msg.index.resize(gimbal_names_.size());
  gimbal_torque_enable_msg.torque_enable.resize(gimbal_names_.size());
  std::iota(gimbal_torque_enable_msg.index.begin(), gimbal_torque_enable_msg.index.end(), 0);
  std::fill(gimbal_torque_enable_msg.torque_enable.begin(), gimbal_torque_enable_msg.torque_enable.end(), 1);
  dynamixel_torque_enable_pub_.publish(gimbal_torque_enable_msg);
  ROS_INFO_STREAM("[dynarm][control] torque for " << gimbal_names_.size() << " gimbals enabled");

  // send robstride servo on command
  robstride_servo_on_pub_.publish(std_msgs::Empty());
  ROS_INFO("[dynarm][control] robstride servo on command sent");
}

bool ManipulatorController::update()
{
  if (!ControlBase::update())
  {
    joint_trajectory_generator_->reset();  // reset for initial smooth deformation
    return false;
  }

  controlCore();
  sendCmd();

  return true;
}

void ManipulatorController::controlCore()
{
  trajectory_generator_->update();

  rnea_solution_ = pinocchio::rnea(*pinocchio_model_, *pinocchio_data_, joint_trajectory_generator_->getCurrentQ(),
                                   joint_trajectory_generator_->getCurrentDQ(),
                                   joint_trajectory_generator_->getCurrentTargetDDQ());  // for debug
}

void ManipulatorController::sendCmd()
{
  sendFourAxisCommand();
  sendJointCommand();
  sendGimbalCommand();

  joint_trajectory_generator_ros_->publish();

  // publish inverse dynamics solver info if nonlinear mode
  if (trajectory_generator_->nonlinear_mode_)
    nonlinear_inverse_dynamics_solver_ros_->publish();

  trajectory_generator_ros_->publish();

  // for debug: torque generated by thrust and normal rnea solution
  Eigen::VectorXd curr_q = joint_trajectory_generator_->getCurrentQ();
  Eigen::VectorXd curr_target_thrust = trajectory_generator_->getCurrentTargetThrust();
  sensor_msgs::JointState tau_by_thrust_msg;  // in real q and lambda
  sensor_msgs::JointState rnea_solution_msg;  // in real q, dq, and target ddq. calculated in ControlCore()
  tau_by_thrust_msg.header.stamp = ros::Time::now();
  rnea_solution_msg.header.stamp = ros::Time::now();
  Eigen::VectorXd tau_by_thrust = pinocchio_robot_model_->computeTauExtByThrustDerivative(curr_q) * curr_target_thrust;

  for (int i = 0; i < pinocchio_model_->njoints; i++)
  {
    int joint_index_q = pinocchio_model_->joints[pinocchio_model_->getJointId(pinocchio_model_->names[i])].idx_q();
    int joint_index_v = pinocchio_model_->joints[pinocchio_model_->getJointId(pinocchio_model_->names[i])].idx_v();

    if (joint_index_q < 0 || joint_index_v < 0)
      continue;  // skip if joint index is invalid

    tau_by_thrust_msg.name.push_back(pinocchio_model_->names[i]);
    tau_by_thrust_msg.effort.push_back(tau_by_thrust(joint_index_v));

    rnea_solution_msg.name.push_back(pinocchio_model_->names[i]);
    rnea_solution_msg.effort.push_back(rnea_solution_(joint_index_v));
  }
  tau_by_thrust_pub_.publish(tau_by_thrust_msg);
  rnea_solution_pub_.publish(rnea_solution_msg);
}

void ManipulatorController::sendFourAxisCommand()
{
  // send target thrust
  Eigen::VectorXd thrust_upper_limits = pinocchio_robot_model_->getThrustUpperLimits();
  Eigen::VectorXd thrust_lower_limits = pinocchio_robot_model_->getThrustLowerLimits();
  spinal::FourAxisCommand four_axis_command_msg;
  Eigen::VectorXd curr_target_thrust = trajectory_generator_->getCurrentTargetThrust();

  for (int i = 0; i < robot_model_->getRotorNum(); i++)
  {
    four_axis_command_msg.base_thrust.push_back(
        std::min(std::max(thrust_lower_limits(i), curr_target_thrust(i)), thrust_upper_limits(i)));
  }
  four_axis_command_pub_.publish(four_axis_command_msg);
}

void ManipulatorController::sendJointCommand()
{
  // send target joint angle, velocity and torque
  sensor_msgs::JointState joint_state_msg;
  joint_state_msg.header.stamp = ros::Time::now();

  Eigen::VectorXd curr_target_q = joint_trajectory_generator_->getCurrentTargetQ();
  Eigen::VectorXd curr_target_dq = joint_trajectory_generator_->getCurrentTargetDQ();
  Eigen::VectorXd curr_target_tau = trajectory_generator_->getCurrentTargetTau();

  for (int i = 0; i < joint_names_.size(); i++)
  {
    int joint_index_q = pinocchio_model_->joints[pinocchio_model_->getJointId(joint_names_.at(i))].idx_q();
    int joint_index_v = pinocchio_model_->joints[pinocchio_model_->getJointId(joint_names_.at(i))].idx_v();

    joint_state_msg.name.push_back(joint_names_.at(i));
    joint_state_msg.position.push_back(curr_target_q(joint_index_q));
    joint_state_msg.velocity.push_back(curr_target_dq(joint_index_v));
    joint_state_msg.effort.push_back(curr_target_tau(joint_index_v));
  }

  joints_control_pub_.publish(joint_state_msg);
}

void ManipulatorController::sendGimbalCommand()
{
  // send gimbal nominal angles
  Eigen::VectorXd curr_target_gimbal_angle = trajectory_generator_->getCurrentTargetGimbalAngle();
  std::vector<std::string> gimbal_names = trajectory_generator_->getGimbalNames();
  std::map<std::string, int> gimbal_index_map = trajectory_generator_->getGimbalIndexMap();

  sensor_msgs::JointState joint_state_msg;
  joint_state_msg.header.stamp = ros::Time::now();
  for (int i = 0; i < gimbal_names.size(); i++)
  {
    std::string gimbal_name = gimbal_names.at(i);
    joint_state_msg.name.push_back(gimbal_name);
    joint_state_msg.position.push_back(curr_target_gimbal_angle(gimbal_index_map.at(gimbal_name)));
  }
  gimbals_control_pub_.publish(joint_state_msg);
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::ManipulatorController, aerial_robot_control::ControlBase);
