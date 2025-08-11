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

  joint_trajectory_generator_ = std::make_shared<jointTrajectoryGenerator>(nh_, pinocchio_robot_model_);

  rosParamInit();

  four_axis_command_pub_ = nh_.advertise<spinal::FourAxisCommand>("four_axes/command", 1);
  joints_control_pub_ = nh_.advertise<sensor_msgs::JointState>("joints_ctrl", 1);
  gimbals_control_pub_ = nh_.advertise<sensor_msgs::JointState>("gimbals_ctrl", 1);
  tau_by_thrust_pub_ = nh_.advertise<sensor_msgs::JointState>("debug/id_debug/tau_by_thrust", 1);
  rnea_solution_pub_ = nh_.advertise<sensor_msgs::JointState>("debug/id_debug/rnea_solution", 1);

  is_initialized_ = false;

  // initial joint angle
  init_target_q_.resize(pinocchio_robot_model_->getModel()->nq);
  std::vector<double> init_target_q;
  ros::NodeHandle control_nh(nh_, "controller");
  control_nh.getParam("init_target_q", init_target_q);
  if (init_target_q.size() != pinocchio_model_->nq)
    ROS_ERROR_STREAM("[dragon_arm] nq: " << pinocchio_model_->nq << " and initial joint angle size: "
                                         << init_target_q.size() << " in ros parameter is not same");
  for (int i = 0; i < pinocchio_model_->nq; i++)
    init_target_q_(i) = init_target_q.at(i);
  ROS_INFO_STREAM("[dragon_arm][control] initial target joint angle: " << init_target_q_.transpose());
}

void ManipulatorController::rosParamInit()
{
  ros::NodeHandle model_nh(nh_, "model");
  getParam<int>(model_nh, "rotor_devider", rotor_devider_, 1);

  Eigen::VectorXd joint_p_gain = Eigen::VectorXd::Zero(pinocchio_model_->nv);
  Eigen::VectorXd joint_d_gain = Eigen::VectorXd::Zero(pinocchio_model_->nv);
  bool simulation_mode;
  nh_.param("/use_sim_time", simulation_mode, false);
  if (simulation_mode)
  {
    while (!ros::service::waitForService(nh_.getNamespace() + std::string("/controller_manager/load_controller")))
    {
      ROS_INFO_STREAM_THROTTLE(1.0, "[dynarm][control] wait for controller manager for joints");
    }

    XmlRpc::XmlRpcValue all_servos_params;
    nh_.getParam("servo_controller", all_servos_params);
    for (auto servo_group_params : all_servos_params)
    {
      if (servo_group_params.second.getType() != XmlRpc::XmlRpcValue::TypeStruct)
        continue;

      double servo_group_p_gain = -1;
      double servo_group_d_gain = -1;

      if (servo_group_params.second.hasMember("simulation") &&
          servo_group_params.second["simulation"].hasMember("pid"))  // PD gains are defined for all servos in group
      {
        servo_group_p_gain = servo_group_params.second["simulation"]["pid"]["p"];
        servo_group_d_gain = servo_group_params.second["simulation"]["pid"]["d"];
      }

      for (auto servo_params : servo_group_params.second)  // each servo
      {
        bool has_pd_gain = false;
        if (servo_params.first.find("controller") != std::string::npos)
        {
          std::string joint_name = servo_params.second["name"];
          int joint_id = pinocchio_model_->getJointId(joint_name);
          double p_gain = servo_group_p_gain;
          double d_gain = servo_group_d_gain;
          if (joint_id == pinocchio_model_->njoints)
          {
            ROS_WARN_STREAM("[dynarm][control] there is not joint named \"" << joint_name << "\"");
            continue;
          }
          else
          {
            int joint_index_v = pinocchio_model_->joints[joint_id].idx_v();

            has_pd_gain =
                (servo_params.second.hasMember("simulation") && servo_params.second["simulation"].hasMember("pid"));
            if (has_pd_gain)
            {
              p_gain = servo_params.second["simulation"]["pid"]["p"];
              d_gain = servo_params.second["simulation"]["pid"]["d"];
            }

            if (p_gain < 0 || d_gain < 0)
              ROS_ERROR_STREAM("[dynarm][control] PD gain for " << joint_name << " is " << p_gain << " " << d_gain);

            joint_p_gain(joint_index_v) = p_gain;
            joint_d_gain(joint_index_v) = d_gain;
          }
        }
      }
    }
    ROS_INFO_STREAM("[dynarm][control] joint P gain: " << joint_p_gain.transpose());
    ROS_INFO_STREAM("[dynarm][control] joint D gain: " << joint_d_gain.transpose());
    joint_p_gain_ = joint_p_gain.asDiagonal();
    joint_d_gain_ = joint_d_gain.asDiagonal();
  }
}

void ManipulatorController::reset()
{
  ControlBase::reset();
  joint_trajectory_generator_->reset();

  first_run_ = true;
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
    joint_trajectory_generator_->setCurrentTargetQ(dragon_arm_robot_model_->getCurrentJointPositions());
    joint_trajectory_generator_->setCurrentTargetDQ(Eigen::VectorXd::Zero(pinocchio_model_->nv));
    joint_trajectory_generator_->setCurrentTargetDDQ(Eigen::VectorXd::Zero(pinocchio_model_->nv));
    first_run_ = false;
  }

  // determine q to calculate inverse dynamics
  if (!is_initialized_)
  {
    joint_trajectory_generator_->setCurrentTargetQ(dragon_arm_robot_model_->getCurrentJointPositions());
  }
  else
  {
    joint_trajectory_generator_->generateEndEffectorTrajectory();
    joint_trajectory_generator_->generateJointTrajectory();
  }

  // process gimbal angles if linear mode because gimbal angle is nominal state
  if (!joint_trajectory_generator_->getNonlinearMode())
    joint_trajectory_generator_->getGimbalNominalAngles();

  // calculate inverse dynamics
  joint_trajectory_generator_->solveInverseDynamics();

  // update target q by target gimbal angle if nonlinear mode
  if (joint_trajectory_generator_->getNonlinearMode())
    joint_trajectory_generator_->updateTargetGimbalAngle();

  // state transition
  if (!is_initialized_)
  {
    // check joint angle convergence
    bool is_converged = true;
    Eigen::VectorXd curr_q = dragon_arm_robot_model_->getCurrentJointPositions();
    std::vector<std::string> joint_names = dragon_arm_robot_model_->getJointNames();
    for (int i = 0; i < joint_names.size(); i++)
    {
      int joint_index_q = pinocchio_model_->joints[pinocchio_model_->getJointId(joint_names.at(i))].idx_q();
      if (fabs(curr_q(joint_index_q) - init_target_q_(joint_index_q)) > 0.05)
      {
        is_converged = false;
        break;
      }
    }
    if (is_converged)
    {
      is_initialized_ = true;
      ROS_INFO_STREAM("[dragon_arm][control] initialized with target q: "
                      << joint_trajectory_generator_->getCurrentTargetQ().transpose());
      joint_trajectory_generator_->setCurrentTargetQ(
          joint_trajectory_generator_->getGimbalNominalAngles(init_target_q_));
    }
  }
  else
  {
    joint_trajectory_generator_->stateTransition();
  }
}

void ManipulatorController::sendCmd()
{
  sendFourAxisCommand();
  sendJointCommand();
  sendGimbalCommand();

  joint_trajectory_generator_->publish();

  // for debug: send target torque
  sensor_msgs::JointState tau_by_thrust_msg;
  tau_by_thrust_msg.header.stamp = ros::Time::now();
  // sensor_msgs::JointState rnea_solution_msg;
  // rnea_solution_msg.header.stamp = ros::Time::now();
  Eigen::VectorXd curr_q = dragon_arm_robot_model_->getCurrentJointPositions();
  Eigen::VectorXd curr_target_thrust = joint_trajectory_generator_->getCurrentTargetThrust();
  Eigen::VectorXd tau_by_thrust = pinocchio_robot_model_->computeTauExtByThrustDerivative(curr_q) * curr_target_thrust;

  for (int i = 0; i < pinocchio_model_->njoints; i++)
  {
    int joint_index_q = pinocchio_model_->joints[pinocchio_model_->getJointId(pinocchio_model_->names[i])].idx_q();
    int joint_index_v = pinocchio_model_->joints[pinocchio_model_->getJointId(pinocchio_model_->names[i])].idx_v();

    if (joint_index_q < 0 || joint_index_v < 0)
      continue;  // skip if joint index is invalid

    tau_by_thrust_msg.name.push_back(pinocchio_model_->names[i]);
    tau_by_thrust_msg.effort.push_back(tau_by_thrust(joint_index_v));

    // rnea_solution_msg.name.push_back(pinocchio_model_->names[i]);
    // rnea_solution_msg.effort.push_back(rnea_solution_(joint_index_v));
  }
  tau_by_thrust_pub_.publish(tau_by_thrust_msg);
  // rnea_solution_pub_.publish(rnea_solution_msg);
}

void ManipulatorController::sendFourAxisCommand()
{
  // send target thrust
  Eigen::VectorXd thrust_upper_limits = pinocchio_robot_model_->getThrustUpperLimits();
  Eigen::VectorXd thrust_lower_limits = pinocchio_robot_model_->getThrustLowerLimits();
  spinal::FourAxisCommand four_axis_command_msg;
  Eigen::VectorXd curr_target_thrust = joint_trajectory_generator_->getCurrentTargetThrust();

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
  Eigen::VectorXd curr_target_tau = joint_trajectory_generator_->getCurrentTargetTau();

  // if not initialized, set target joint angle to init target q
  if (!is_initialized_)
  {
    curr_target_q = init_target_q_;
  }

  std::vector<std::string> joint_names = dragon_arm_robot_model_->getJointNames();
  for (int i = 0; i < joint_names.size(); i++)
  {
    int joint_index_q = pinocchio_model_->joints[pinocchio_model_->getJointId(joint_names.at(i))].idx_q();
    int joint_index_v = pinocchio_model_->joints[pinocchio_model_->getJointId(joint_names.at(i))].idx_v();

    joint_state_msg.name.push_back(joint_names.at(i));
    joint_state_msg.position.push_back(curr_target_q(joint_index_q));
    joint_state_msg.velocity.push_back(curr_target_dq(joint_index_v));
    joint_state_msg.effort.push_back(curr_target_tau(joint_index_v));
  }

  joints_control_pub_.publish(joint_state_msg);
}

void ManipulatorController::sendGimbalCommand()
{
  // send gimbal nominal angles
  Eigen::VectorXd curr_q = dragon_arm_robot_model_->getCurrentJointPositions();
  Eigen::VectorXd curr_q_gimbal_processed = joint_trajectory_generator_->getGimbalNominalAngles(curr_q);
  Eigen::VectorXd target_gimbal_angles_q = curr_q_gimbal_processed;
  Eigen::VectorXd curr_target_gimbal_angle = joint_trajectory_generator_->getCurrentTargetGimbalAngle();

  sensor_msgs::JointState joint_state_msg;
  joint_state_msg.header.stamp = ros::Time::now();
  for (int i = 0; i < robot_model_->getRotorNum() / rotor_devider_; i++)
  {
    std::string gimbal_roll_name = "gimbal" + std::to_string(i + 1) + "_roll";
    std::string gimbal_pitch_name = "gimbal" + std::to_string(i + 1) + "_pitch";

    int gimbal_roll_index = pinocchio_model_->joints[pinocchio_model_->getJointId(gimbal_roll_name)].idx_q();
    int gimbal_pitch_index = pinocchio_model_->joints[pinocchio_model_->getJointId(gimbal_pitch_name)].idx_q();

    joint_state_msg.name.push_back(gimbal_roll_name);
    joint_state_msg.name.push_back(gimbal_pitch_name);
    if (joint_trajectory_generator_->getNonlinearMode())
    {
      joint_state_msg.position.push_back(curr_target_gimbal_angle(2 * i));      // roll
      joint_state_msg.position.push_back(curr_target_gimbal_angle(2 * i + 1));  // pitch
    }
    else
    {
      joint_state_msg.position.push_back(target_gimbal_angles_q(gimbal_roll_index));
      joint_state_msg.position.push_back(target_gimbal_angles_q(gimbal_pitch_index));
    }
  }
  gimbals_control_pub_.publish(joint_state_msg);
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::ManipulatorController, aerial_robot_control::ControlBase);
