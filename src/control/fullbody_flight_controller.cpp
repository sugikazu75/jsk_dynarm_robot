#include <dynarm/control/fullbody_flight_controller.h>

using namespace aerial_robot_control;

FullbodyFlightController::FullbodyFlightController() : ControlBase()
{
}

void FullbodyFlightController::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
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

  four_axis_command_pub_ = nh_.advertise<spinal::FourAxisCommand>("four_axes/command", 1);
  joints_control_pub_ = nh_.advertise<sensor_msgs::JointState>("joints_ctrl", 1);
  gimbals_control_pub_ = nh_.advertise<sensor_msgs::JointState>("gimbals_ctrl", 1);
  rotor_wrench_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("rotor_wrench", 1);

  state_command_sub_ = nh_.subscribe("state_command", 1, &FullbodyFlightController::stateCommandCallback, this);

  robot_ns_ = ros::this_node::getNamespace();
  if (!robot_ns_.empty() && robot_ns_[0] == '/')
    robot_ns_ = robot_ns_.substr(1);
  rotor_wrench_pub_index_ = 0;

  joint_state_sub_ = nh_.subscribe("joint_states", 1, &FullbodyFlightController::jointStateCallback, this);

  nonlinear_inverse_dynamics_solver_ =
      std::make_shared<aerial_robot_model::NonlinearInverseDynamics>(nh, pinocchio_robot_model_);

  control_input_ = Eigen::VectorXd::Zero(pinocchio_model_->nv + pinocchio_robot_model_->getRotorNum() +
                                         nonlinear_inverse_dynamics_solver_->getGimbalNames().size());

  curr_q_ = Eigen::VectorXd::Zero(pinocchio_model_->nq);
  curr_dq_ = Eigen::VectorXd::Zero(pinocchio_model_->nv);

  rosParamInit();
  DDPProblemInit();
}

void FullbodyFlightController::rosParamInit()
{
}

void FullbodyFlightController::DDPProblemInit()
{
  std::vector<int> rotor_frame_indices = pinocchio_robot_model_->getRotorFrameIndices();
  double m_f_rate = pinocchio_robot_model_->getMFRate();
  std::vector<crocoddyl::Thruster> thrusters;
  for (int i = 0; i < rotor_frame_indices.size(); i++)
  {
    int rotor_direction = pinocchio_robot_model_->getRotorDirection(i);
    double thrust_lower_limit = pinocchio_robot_model_->getThrustLowerLimits()(i);
    double thrust_upper_limit = pinocchio_robot_model_->getThrustUpperLimits()(i);
    thrusters.emplace_back(rotor_frame_indices.at(i), (float)(abs(m_f_rate)),
                           ((rotor_direction == 1) ? crocoddyl::ThrusterType::CCW : crocoddyl::ThrusterType::CW),
                           (float)thrust_lower_limit, (float)thrust_upper_limit);
  }

  ros::NodeHandle ddp_nh(nh_, "ddp");
  bool fwddyn;
  {
    ddp_nh.getParam("fwddyn", fwddyn);
    std::cout << "fwddyn: " << fwddyn << std::endl;
  }

  DDPHoveringProblem::CostWeight cost_weight;
  {
    double whatever_double;
    ddp_nh.getParam("state_weight", whatever_double);
    cost_weight.state_weight = whatever_double;

    ddp_nh.getParam("state_bound_weight", whatever_double);
    cost_weight.state_bound_weight = whatever_double;

    ddp_nh.getParam("control_weight", whatever_double);
    cost_weight.control_weight = whatever_double;

    std::vector<double> whatever_vector;
    ddp_nh.getParam("x_state_weights", whatever_vector);
    cost_weight.x_weights = Eigen::Map<Eigen::VectorXd>(whatever_vector.data(), whatever_vector.size());

    if (fwddyn)
      ddp_nh.getParam("u_weight_fwddyn", whatever_vector);
    else
      ddp_nh.getParam("u_weight_invdyn", whatever_vector);
    cost_weight.u_weights = Eigen::Map<Eigen::VectorXd>(whatever_vector.data(), whatever_vector.size());

    std::cout << "state_weight: " << cost_weight.state_weight << std::endl;
    std::cout << "state_bound_weight: " << cost_weight.state_bound_weight << std::endl;
    std::cout << "control_weight: " << cost_weight.control_weight << std::endl;
    std::cout << "x_weights: " << cost_weight.x_weights.transpose() << std::endl;
    std::cout << "u_weights: " << cost_weight.u_weights.transpose() << std::endl;
  }
  DDPHoveringProblem::OptimizationParam optimization_param;
  std::cout << "horizon: " << optimization_param.horizon << std::endl;
  std::cout << "dt: " << optimization_param.dt << std::endl;
  std::cout << "max_iter: " << optimization_param.max_iter << std::endl;

  hovering_ =
      std::make_shared<DDPHoveringProblem>(pinocchio_model_, thrusters, fwddyn, cost_weight, optimization_param);
}

void FullbodyFlightController::activate()
{
  ControlBase::activate();

  control_input_ = Eigen::VectorXd::Zero(pinocchio_model_->nv + pinocchio_robot_model_->getRotorNum() +
                                         nonlinear_inverse_dynamics_solver_->getGimbalNames().size());

  sendGimbalCommand();
}

void FullbodyFlightController::reset()
{
  ControlBase::reset();

  nonlinear_inverse_dynamics_solver_->reset();

  control_input_ = Eigen::VectorXd::Zero(pinocchio_model_->nv + pinocchio_robot_model_->getRotorNum() +
                                         nonlinear_inverse_dynamics_solver_->getGimbalNames().size());

  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(pinocchio_model_->nq + pinocchio_model_->nv);
  Eigen::VectorXd curr_q = dragon_arm_robot_model_->getCurrentJointPositions();
  x0.head(pinocchio_model_->nq) = curr_q;  // includes root pose and joint positions
  x0.head(3) << estimator_->getPos(Frame::BASELINK, estimate_mode_).x(),
      estimator_->getPos(Frame::BASELINK, estimate_mode_).y(),
      estimator_->getPos(Frame::BASELINK, estimate_mode_).z();  // root position
  tf::Matrix3x3 root_rot = estimator_->getOrientation(Frame::BASELINK, estimate_mode_);
  tf::Quaternion root_quat;
  root_rot.getRotation(root_quat);
  x0.segment(3, 4) << root_quat.x(), root_quat.y(), root_quat.z(), root_quat.w();  // root rotation
  std::cout << "[ddp] x0: " << x0.transpose() << std::endl;

  Eigen::VectorXd xref = Eigen::VectorXd::Zero(pinocchio_model_->nq + pinocchio_model_->nv);
  xref.head(pinocchio_model_->nq) = curr_q;  // includes root pose and joint positions
  xref.head(3) << estimator_->getPos(Frame::BASELINK, estimate_mode_).x(),
      estimator_->getPos(Frame::BASELINK, estimate_mode_).y(), navigator_->getTargetPos().z();  // root position
  tf::Quaternion root_quat_des(navigator_->getTargetRPY().x(), navigator_->getTargetRPY().y(),
                               navigator_->getTargetRPY().z());
  xref.segment(3, 4) << root_quat_des.x(), root_quat_des.y(), root_quat_des.z(), root_quat_des.w();  // root rotation
  std::vector<std::string> gimbal_names = nonlinear_inverse_dynamics_solver_->getGimbalNames();
  for (int i = 0; i < gimbal_names.size(); i++)
  {
    int gimbal_id = pinocchio_model_->getJointId(gimbal_names[i]);
    int gimbal_index_q = pinocchio_model_->joints[gimbal_id].idx_q();
    xref(gimbal_index_q) = 0.0;  // set target gimbal angles to 0
  }
  std::cout << "[ddp] xref: " << xref.transpose() << std::endl;

  curr_target_q_ = xref.head(pinocchio_model_->nq);
  curr_target_dq_ = xref.tail(pinocchio_model_->nv);

  ddp_problem_ = hovering_->createHoveringProblem(x0, xref);
  ddp_solver_ = std::make_shared<crocoddyl::SolverBoxFDDP>(ddp_problem_);

  int N = hovering_->optimization_param_.horizon / hovering_->optimization_param_.dt;
  xs_init_.resize(N, x0);
  us_init_ = ddp_solver_->get_problem()->quasiStatic_xs(xs_init_);
  xs_init_.push_back(x0);
}

bool FullbodyFlightController::update()
{
  if (!ControlBase::update())
    return false;

  controlCore();
  sendCmd();

  return true;
}

void FullbodyFlightController::controlCore()
{
  crocoddyl::Timer timer;
  std::cout << "q0: " << ddp_problem_->get_x0().head(pinocchio_model_->nq).transpose() << std::endl;
  std::cout << "v0: " << ddp_problem_->get_x0().tail(pinocchio_model_->nv).transpose() << std::endl;
  ddp_solver_->solve(xs_init_, us_init_);
  double time = timer.get_duration();
  std::cout << "total calculation time: " << time << "[ms]" << std::endl;
  std::cout << "Number of iterations: " << ddp_solver_->get_iter() << std::endl;
  std::cout << "time per iterate: " << time / ddp_solver_->get_iter() << std::endl;
  std::cout << "Total cost: " << ddp_solver_->get_cost() << std::endl;
  std::cout << "Gradient norm: " << ddp_solver_->get_stop() << std::endl;

  xs_init_ = ddp_solver_->get_xs();
  us_init_ = ddp_solver_->get_us();

  std::cout << "root ddq: " << us_init_.at(0).head(6).transpose() << std::endl;
  std::cout << "joint ddq: " << us_init_.at(0).tail(us_init_.at(0).size() - 6).transpose() << std::endl;
  std::cout << std::endl;
  std::cout << "q final: " << xs_init_.back().head(pinocchio_model_->nq).transpose() << std::endl;
  std::cout << "v final: " << xs_init_.back().tail(pinocchio_model_->nv).transpose() << std::endl;
  std::cout << "root ddq final: " << us_init_.back().head(6).transpose() << std::endl;
  std::cout << "joint ddq final: " << us_init_.back().tail(us_init_.back().size() - 6).transpose() << std::endl;
  std::cout << std::endl;

  // set current state as next initial state
  Eigen::VectorXd current_x = Eigen::VectorXd::Zero(pinocchio_model_->nq + pinocchio_model_->nv);

  // joint positions
  current_x.head(pinocchio_model_->nq) = curr_q_;

  // root position
  current_x.head(3) << estimator_->getPos(Frame::BASELINK, estimate_mode_).x(),
      estimator_->getPos(Frame::BASELINK, estimate_mode_).y(), estimator_->getPos(Frame::BASELINK, estimate_mode_).z();

  // root orientation
  tf::Matrix3x3 root_rot = estimator_->getOrientation(Frame::BASELINK, estimate_mode_);
  tf::Quaternion root_quat;
  root_rot.getRotation(root_quat);
  current_x.segment(3, 4) << root_quat.x(), root_quat.y(), root_quat.z(), root_quat.w();

  // joint velocities
  current_x.tail(pinocchio_model_->nv) = curr_dq_;

  // root linear velocity in local frame
  tf::Vector3 root_vel = root_rot.inverse() * estimator_->getVel(Frame::BASELINK, estimate_mode_);
  current_x.segment(pinocchio_model_->nq, 3) << root_vel.x(), root_vel.y(), root_vel.z();

  // root angular velocity
  current_x.segment(pinocchio_model_->nq + 3, 3) << estimator_->getAngularVel(Frame::BASELINK, estimate_mode_).x(),
      estimator_->getAngularVel(Frame::BASELINK, estimate_mode_).y(),
      estimator_->getAngularVel(Frame::BASELINK, estimate_mode_).z();

  ddp_problem_->set_x0(current_x);

  // initial guess in next loop
  xs_init_.erase(xs_init_.begin());
  xs_init_.push_back(xs_init_.back());
  us_init_.erase(us_init_.begin());
  us_init_.push_back(us_init_.back());

  // solve inverse dynamics to get control input
  Eigen::VectorXd target_ddq = us_init_.at(0);
  Eigen::VectorXd control_input;
  bool solved = nonlinear_inverse_dynamics_solver_->solve(
      current_x.head(pinocchio_model_->nq), current_x.tail(pinocchio_model_->nv), target_ddq, control_input);
  if (!solved)
  {
    ROS_ERROR("[ddp] Nonlinear inverse dynamics solver failed to solve.");
    return;
  }
  std::cout << "generalized force: " << control_input.head(pinocchio_model_->nv).transpose() << std::endl;
  std::cout << "thrust:"
            << control_input.segment(pinocchio_model_->nv, pinocchio_robot_model_->getRotorNum()).transpose()
            << std::endl;
  std::cout << "gimbal angles: "
            << control_input.tail(nonlinear_inverse_dynamics_solver_->getGimbalNames().size()).transpose() << std::endl;
  control_input_ = control_input;
  std::cout << std::endl;
  std::cout << std::endl;
  std::cout << std::endl;
}

void FullbodyFlightController::sendCmd()
{
  sendFourAxisCommand();
  sendJointCommand();
  sendGimbalCommand();
  publish();
  publishDDPTrajectory();
}

void FullbodyFlightController::publish()
{
  // for debug: rotor wrench
  Eigen::VectorXd curr_target_thrust =
      control_input_.segment(pinocchio_model_->nv, pinocchio_robot_model_->getRotorNum());
  geometry_msgs::WrenchStamped rotor_wrench_msg;
  rotor_wrench_msg.header.stamp = ros::Time::now();
  rotor_wrench_msg.header.frame_id = robot_ns_ + "/thrust" + std::to_string(rotor_wrench_pub_index_ + 1);
  rotor_wrench_msg.wrench.force.x = 0.0;
  rotor_wrench_msg.wrench.force.y = 0.0;
  rotor_wrench_msg.wrench.force.z = curr_target_thrust(rotor_wrench_pub_index_);
  rotor_wrench_msg.wrench.torque.x = 0.0;
  rotor_wrench_msg.wrench.torque.y = 0.0;
  rotor_wrench_msg.wrench.torque.z = pinocchio_robot_model_->getRotorDirection(rotor_wrench_pub_index_) *
                                     pinocchio_robot_model_->getMFRate() * curr_target_thrust(rotor_wrench_pub_index_);
  rotor_wrench_pub_.publish(rotor_wrench_msg);
  rotor_wrench_pub_index_ = (rotor_wrench_pub_index_ + 1) % pinocchio_robot_model_->getRotorNum();
}

void FullbodyFlightController::sendFourAxisCommand()
{
  Eigen::VectorXd thrust_upper_limits = pinocchio_robot_model_->getThrustUpperLimits();
  Eigen::VectorXd thrust_lower_limits = pinocchio_robot_model_->getThrustLowerLimits();
  spinal::FourAxisCommand four_axis_command_msg;
  for (int i = 0; i < pinocchio_robot_model_->getRotorNum(); i++)
  {
    four_axis_command_msg.base_thrust.push_back(
        std::min(std::max(thrust_lower_limits(i), control_input_(pinocchio_model_->nv + i)), thrust_upper_limits(i)));
  }
  four_axis_command_pub_.publish(four_axis_command_msg);
}

void FullbodyFlightController::sendJointCommand()
{
  sensor_msgs::JointState joint_state_msg;
  joint_state_msg.header.stamp = ros::Time::now();

  Eigen::VectorXd curr_target_q = curr_target_q_;
  Eigen::VectorXd curr_target_dq = curr_target_dq_;
  Eigen::VectorXd curr_target_tau = control_input_.head(
      pinocchio_model_->nv);  // this vector includes target root generalized force(should be 0) and joint torques

  std::vector<std::string> joint_names = nonlinear_inverse_dynamics_solver_->getJointNames();

  for (int i = 0; i < joint_names.size(); i++)
  {
    if (joint_names.at(i).find("root") != std::string::npos)
      continue;  // skip root joint

    int joint_index_q = pinocchio_model_->joints[pinocchio_model_->getJointId(joint_names.at(i))].idx_q();
    int joint_index_v = pinocchio_model_->joints[pinocchio_model_->getJointId(joint_names.at(i))].idx_v();

    joint_state_msg.name.push_back(joint_names.at(i));
    joint_state_msg.position.push_back(curr_target_q(joint_index_q));
    joint_state_msg.velocity.push_back(curr_target_dq(joint_index_v));
    joint_state_msg.effort.push_back(curr_target_tau(joint_index_v));
  }
  joints_control_pub_.publish(joint_state_msg);
}

void FullbodyFlightController::sendGimbalCommand()
{
  sensor_msgs::JointState gimbal_state_msg;
  gimbal_state_msg.header.stamp = ros::Time::now();

  std::vector<std::string> gimbal_names = nonlinear_inverse_dynamics_solver_->getGimbalNames();
  Eigen::VectorXd target_gimbal_angles =
      control_input_.tail(gimbal_names.size());  // Assuming gimbal angles are at the end of control_input_

  for (int i = 0; i < gimbal_names.size(); i++)
  {
    double gimbal_angle = target_gimbal_angles(i);

    gimbal_state_msg.name.push_back(gimbal_names.at(i));
    gimbal_state_msg.position.push_back(gimbal_angle);
  }

  gimbals_control_pub_.publish(gimbal_state_msg);
}

void FullbodyFlightController::jointStateCallback(const sensor_msgs::JointStateConstPtr& msg)
{
  for (size_t i = 0; i < msg->name.size(); ++i)
  {
    int joint_id = pinocchio_model_->getJointId(msg->name[i]);
    if (joint_id >= 0 && joint_id < pinocchio_model_->njoints)
    {
      int joint_index_q = pinocchio_model_->joints[joint_id].idx_q();
      int joint_index_v = pinocchio_model_->joints[joint_id].idx_v();
      if (msg->position.size() == msg->name.size())
      {
        curr_q_(joint_index_q) = msg->position[i];
      }
      if (msg->velocity.size() == msg->name.size())
      {
        curr_dq_(joint_index_v) = msg->velocity[i];
      }
    }
  }
}

void FullbodyFlightController::stateCommandCallback(const sensor_msgs::JointStateConstPtr& msg)
{
  if (msg->name.size() != msg->position.size())
  {
    ROS_ERROR("[ddp] Joint state command size mismatch.");
    return;
  }

  if (msg->name.size() == msg->position.size())
  {
    for (size_t i = 0; i < msg->name.size(); ++i)
    {
      int joint_id = pinocchio_model_->getJointId(msg->name.at(i));
      if (joint_id < 0 || joint_id >= pinocchio_model_->njoints)
      {
        ROS_ERROR("[ddp] Joint name '%s' not found in the model.", msg->name.at(i).c_str());
        continue;
      }
      int joint_index_q = pinocchio_model_->joints[joint_id].idx_q();
      curr_target_q_(joint_index_q) = msg->position.at(i);
    }
  }
  if (msg->name.size() == msg->velocity.size())
  {
    for (size_t i = 0; i < msg->name.size(); ++i)
    {
      int joint_id = pinocchio_model_->getJointId(msg->name.at(i));
      if (joint_id < 0 || joint_id >= pinocchio_model_->njoints)
      {
        ROS_ERROR("[ddp] Joint name '%s' not found in the model.", msg->name.at(i).c_str());
        continue;
      }
      int joint_index_v = pinocchio_model_->joints[joint_id].idx_v();
      curr_target_dq_(joint_index_v) = msg->velocity.at(i);
    }
  }

  // set the reference for state residuals
  Eigen::VectorXd reference_x = Eigen::VectorXd::Zero(pinocchio_model_->nq + pinocchio_model_->nv);
  reference_x.head(pinocchio_model_->nq) = curr_target_q_;   // root pose and joint positions
  reference_x.tail(pinocchio_model_->nv) = curr_target_dq_;  // root linear and angular velocities
  for (int i = 0; i < hovering_->state_residuals_.size(); i++)
  {
    hovering_->state_residuals_.at(i)->set_reference(reference_x);
  }
}

void FullbodyFlightController::publishDDPTrajectory()
{
  geometry_msgs::TransformStamped robot_base_transform;
  robot_base_transform.header.stamp = ros::Time::now();
  for (int i = 0; i < xs_init_.size(); i++)
  {
    robot_base_transform.header.frame_id = "world";
    robot_base_transform.child_frame_id = tf::resolve(robot_ns_, "root") + "_trajectory_" + std::to_string(i);
    robot_base_transform.transform.translation.x = xs_init_[i](0);
    robot_base_transform.transform.translation.y = xs_init_[i](1);
    robot_base_transform.transform.translation.z = xs_init_[i](2);
    robot_base_transform.transform.rotation.x = xs_init_[i](3);
    robot_base_transform.transform.rotation.y = xs_init_[i](4);
    robot_base_transform.transform.rotation.z = xs_init_[i](5);
    robot_base_transform.transform.rotation.w = xs_init_[i](6);

    tf_broadcaster_.sendTransform(robot_base_transform);
  }
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::FullbodyFlightController, aerial_robot_control::ControlBase);
