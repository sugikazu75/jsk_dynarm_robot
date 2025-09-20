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
  path_pub_ = nh_.advertise<nav_msgs::Path>("trajectory_path", 1);
  ddp_solve_time_pub_ = nh_.advertise<std_msgs::Float64>("debug/ddp_solve_time", 1);
  ddp_iteration_pub_ = nh_.advertise<std_msgs::UInt8>("debug/ddp_iteration", 1);
  target_root_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("debug/target_root_pose", 1);
  pid_debug_pub_ = nh_.advertise<aerial_robot_msgs::PoseControlPid>("debug/pose/pid", 1);

  joint_command_sub_ = nh_.subscribe("joint_command", 1, &FullbodyFlightController::jointCommandCallback, this);
  root_pos_command_sub_ = nh_.subscribe("root_pos_command", 1, &FullbodyFlightController::rootPosCommandCallback, this);
  root_pose_command_sub_ =
      nh_.subscribe("root_pose_command", 1, &FullbodyFlightController::rootPoseCommandCallback, this);
  circle_trajectory_command_sub_ =
      nh_.subscribe("circle_trajectory_command", 1, &FullbodyFlightController::circleTrajectoryCommandCallback, this);
  joint_trajectory_command_sub_ =
      nh_.subscribe("joint_trajectory_command", 1, &FullbodyFlightController::jointTrajectoryCommandCallback, this);
  transforming_tracking_command_sub_ = nh_.subscribe(
      "transforming_tracking_command", 1, &FullbodyFlightController::transformingTrackingCommandCallback, this);

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
  ros::NodeHandle ddp_nh(nh_, "ddp");

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

    ddp_nh.getParam("u_weight_invdyn", whatever_vector);
    cost_weight.u_weights = Eigen::Map<Eigen::VectorXd>(whatever_vector.data(), whatever_vector.size());

    std::cout << "state_weight: " << cost_weight.state_weight << std::endl;
    std::cout << "state_bound_weight: " << cost_weight.state_bound_weight << std::endl;
    std::cout << "control_weight: " << cost_weight.control_weight << std::endl;
    std::cout << "x_weights: " << cost_weight.x_weights.transpose() << std::endl;
    std::cout << "u_weights: " << cost_weight.u_weights.transpose() << std::endl;
  }
  DDPHoveringProblem::OptimizationParam optimization_param;
  {
    double whatever_double;
    ddp_nh.param("horizon", whatever_double, optimization_param.horizon);
    optimization_param.horizon = whatever_double;

    ddp_nh.param("dt", whatever_double, optimization_param.dt);
    optimization_param.dt = whatever_double;

    int whatever_int;
    ddp_nh.param("max_iter", whatever_int, optimization_param.max_iter);
    optimization_param.max_iter = whatever_int;

    ddp_nh.param("num_threads", whatever_int, optimization_param.num_threads);
    optimization_param.num_threads = whatever_int;

    std::cout << "horizon: " << optimization_param.horizon << std::endl;
    std::cout << "dt: " << optimization_param.dt << std::endl;
    std::cout << "max_iter: " << optimization_param.max_iter << std::endl;
    std::cout << "num_threads: " << optimization_param.num_threads << std::endl;
  }

  hovering_ = std::make_shared<DDPHoveringProblem>(pinocchio_model_, cost_weight, optimization_param);
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

  // initial state
  Eigen::VectorXd x0 = getCurrentX();
  std::cout << "[ddp] x0: " << x0.transpose() << std::endl;

  // reference state
  Eigen::VectorXd xref = Eigen::VectorXd::Zero(pinocchio_model_->nq + pinocchio_model_->nv);
  xref.head(pinocchio_model_->nq) = curr_q_;  // includes root pose and joint positions
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
  xref_ = xref;
  std::cout << "[ddp] xref: " << xref.transpose() << std::endl;

  curr_target_q_ = xref.head(pinocchio_model_->nq);
  curr_target_dq_ = xref.tail(pinocchio_model_->nv);

  ddp_problem_ = hovering_->createHoveringProblem(x0, xref);
  ddp_solver_ = std::make_shared<crocoddyl::SolverBoxFDDP>(ddp_problem_);

  int N = hovering_->optimization_param_.horizon / hovering_->optimization_param_.dt;
  xs_init_.resize(N, x0);
  us_init_ = ddp_solver_->get_problem()->quasiStatic_xs(xs_init_);
  xs_init_.push_back(x0);

  circle_trajectory_flight_flag_ = false;
}

bool FullbodyFlightController::update()
{
  if (!ControlBase::update())
    return false;

  controlCore();
  sendCmd();

  return true;
}

Eigen::VectorXd FullbodyFlightController::getCurrentX()
{
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

  return current_x;
}

void FullbodyFlightController::circleTrajectoryGeneration()
{
  if (ros::Time::now().toSec() > circle_trajectory_end_time_)
  {
    xref_.head(3) = circle_center_ + Eigen::Vector3d(circle_radius_, 0.0, 0.0);
    xref_.segment(3, 4) << 0, 0, 0, 1;
    circle_trajectory_flight_flag_ = false;
    ROS_INFO_STREAM("[ddp] finish circle trajectory tracking");
  }
  else if (ros::Time::now().toSec() > circle_trajectory_start_time_)
  {
    double t = ros::Time::now().toSec() - circle_trajectory_start_time_;
    double omega = 2 * M_PI / circle_duration_;
    for (int i = 0; i < xs_init_.size(); i++)
    {
      double ti = t + i * hovering_->optimization_param_.dt;
      Eigen::Vector3d target_pos =
          circle_center_ + Eigen::Vector3d(circle_radius_ * cos(omega * ti), circle_radius_ * sin(omega * ti), 0.0);
      tf::Quaternion target_root_quat(0.0, 0.0, 0.0);
      tf::Matrix3x3 target_root_rot(target_root_quat);

      tf::Vector3 target_vel_local =
          target_root_rot.inverse() *
          tf::Vector3(-circle_radius_ * omega * sin(omega * ti), circle_radius_ * omega * cos(omega * ti), 0.0);
      tf::Vector3 target_omega_local(0.0, 0.0, 0.0);

      if (ti + circle_trajectory_start_time_ >= circle_trajectory_end_time_)
      {
        target_pos = circle_center_ + Eigen::Vector3d(circle_radius_, 0, 0);
        target_root_quat = tf::Quaternion(0, 0, 0);
        target_vel_local.setZero();
        target_omega_local.setZero();
      }

      Eigen::VectorXd reference_i = hovering_->state_residuals_.at(i)->get_reference();
      reference_i.head(3) = target_pos;
      reference_i.segment(3, 4) << target_root_quat.x(), target_root_quat.y(), target_root_quat.z(),
          target_root_quat.w();
      reference_i.segment(pinocchio_model_->nq, 3) << target_vel_local.x(), target_vel_local.y(), target_vel_local.z();
      reference_i.segment(pinocchio_model_->nq + 3, 3) << target_omega_local.x(), target_omega_local.y(),
          target_omega_local.z();
      hovering_->state_residuals_.at(i)->set_reference(reference_i);
    }
  }
  else
  {
    for (int i = 0; i < xs_init_.size(); i++)
    {
      hovering_->state_residuals_.at(i)->set_reference(xref_);
    }
  }
}

void FullbodyFlightController::jointTrajectoryGeneration()
{
  if (ros::Time::now().toSec() > joint_trajectory_end_time_)
  {
    for (int i = 0; i < joint_trajectory_names_.size(); i++)
    {
      int joint_id = pinocchio_model_->getJointId(joint_trajectory_names_.at(i));
      int joint_index_q = pinocchio_model_->joints[joint_id].idx_q();
      int joint_index_v = pinocchio_model_->joints[joint_id].idx_v();
      curr_target_q_(joint_index_q) = joint_trajectory_angle_start_.at(i);
      curr_target_dq_(joint_index_v) = 0.0;
      xref_(joint_index_q) = joint_trajectory_angle_start_.at(i);
      xref_(pinocchio_model_->nq + joint_index_v) = 0.0;
    }
    joint_trajectory_flight_flag_ = false;
    ROS_INFO_STREAM("[ddp] finish joint trajectory tracking");
  }
  else if (ros::Time::now().toSec() >= joint_trajectory_start_time_)
  {
    double t = ros::Time::now().toSec() - joint_trajectory_start_time_;
    double omega = 2 * M_PI / joint_trajectory_duration_;
    for (int i = 0; i < xs_init_.size(); i++)  // time step
    {
      double ti = t + i * hovering_->optimization_param_.dt;
      double theta = omega * ti;
      Eigen::VectorXd reference_i = hovering_->state_residuals_.at(i)->get_reference();
      for (int j = 0; j < joint_trajectory_names_.size(); j++)  // joint
      {
        int joint_id = pinocchio_model_->getJointId(joint_trajectory_names_.at(j));
        int joint_index_q = pinocchio_model_->joints[joint_id].idx_q();
        int joint_index_v = pinocchio_model_->joints[joint_id].idx_v();
        double target_q =
            (joint_trajectory_angle_start_.at(j) + joint_trajectory_angle_end_.at(j)) / 2.0 +
            (joint_trajectory_angle_end_.at(j) - joint_trajectory_angle_start_.at(j)) / 2.0 * (-cos(theta));

        double target_dq =
            (joint_trajectory_angle_end_.at(j) - joint_trajectory_angle_start_.at(j)) / 2.0 * omega * sin(theta);

        if (i == 0)
        {
          curr_target_q_(joint_index_q) = target_q;
          curr_target_dq_(joint_index_v) = target_dq;
        }

        reference_i(joint_index_q) = target_q;
        reference_i(pinocchio_model_->nq + joint_index_v) = target_dq;
      }
      hovering_->state_residuals_.at(i)->set_reference(reference_i);
    }
  }
  else
  {
    for (int i = 0; i < xs_init_.size(); i++)
    {
      hovering_->state_residuals_.at(i)->set_reference(xref_);
    }
  }
}

void FullbodyFlightController::controlCore()
{
  // update reference state
  if (circle_trajectory_flight_flag_)
  {
    circleTrajectoryGeneration();
  }
  if (joint_trajectory_flight_flag_)
  {
    jointTrajectoryGeneration();
  }

  if ((!circle_trajectory_flight_flag_) && (!joint_trajectory_flight_flag_))
  {
    for (int i = 0; i < xs_init_.size(); i++)
    {
      hovering_->state_residuals_.at(i)->set_reference(xref_);
    }
  }
  Eigen::VectorXd curr_target_x = hovering_->state_residuals_.at(0)->get_reference();
  curr_target_q_.head(7) = curr_target_x.head(7);

  crocoddyl::Timer timer;
  ddp_solver_->solve(xs_init_, us_init_);
  ddp_solve_time_ = timer.get_duration();

  xs_init_ = ddp_solver_->get_xs();
  us_init_ = ddp_solver_->get_us();

  Eigen::VectorXd current_x = getCurrentX();
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
  control_input_ = control_input;
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

  // for debug: target root pose
  geometry_msgs::PoseStamped target_root_pose_msg;
  target_root_pose_msg.header.stamp = ros::Time::now();
  target_root_pose_msg.header.frame_id = "world";
  target_root_pose_msg.pose.position.x = curr_target_q_(0);
  target_root_pose_msg.pose.position.y = curr_target_q_(1);
  target_root_pose_msg.pose.position.z = curr_target_q_(2);
  target_root_pose_msg.pose.orientation.x = curr_target_q_(3);
  target_root_pose_msg.pose.orientation.y = curr_target_q_(4);
  target_root_pose_msg.pose.orientation.z = curr_target_q_(5);
  target_root_pose_msg.pose.orientation.w = curr_target_q_(6);
  target_root_pose_pub_.publish(target_root_pose_msg);

  Eigen::VectorXd curr_q = getCurrentX().head(pinocchio_model_->nq);
  Eigen::VectorXd q_diff = pinocchio::difference(*pinocchio_model_, curr_q, curr_target_q_);
  aerial_robot_msgs::PoseControlPid pid_debug_msg;
  pid_debug_msg.header.stamp = ros::Time::now();
  pid_debug_msg.x.err_p = q_diff(0);
  pid_debug_msg.y.err_p = q_diff(1);
  pid_debug_msg.z.err_p = q_diff(2);
  pid_debug_msg.roll.err_p = q_diff(3);
  pid_debug_msg.pitch.err_p = q_diff(4);
  pid_debug_msg.yaw.err_p = q_diff(5);
  pid_debug_pub_.publish(pid_debug_msg);

  // for debug: ddp solve time and iteration
  std_msgs::Float64 ddp_solve_time_msg;
  ddp_solve_time_msg.data = ddp_solve_time_;
  ddp_solve_time_pub_.publish(ddp_solve_time_msg);

  std_msgs::UInt8 ddp_iteration_msg;
  ddp_iteration_msg.data = ddp_solver_->get_iter();
  ddp_iteration_pub_.publish(ddp_iteration_msg);

  // publish nonlinear inverse dynamics solver info
  nonlinear_inverse_dynamics_solver_->publish();
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

void FullbodyFlightController::jointCommandCallback(const sensor_msgs::JointStateConstPtr& msg)
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
  Eigen::VectorXd reference_x = hovering_->state_residuals_.at(0)->get_reference();
  std::vector<std::string> joint_names = nonlinear_inverse_dynamics_solver_->getJointNames();
  for (int i = 0; i < joint_names.size(); i++)
  {
    if (joint_names.at(i).find("root") != std::string::npos)
      continue;  // skip root joint
    int joint_index_q = pinocchio_model_->joints[pinocchio_model_->getJointId(joint_names.at(i))].idx_q();
    int joint_index_v = pinocchio_model_->joints[pinocchio_model_->getJointId(joint_names.at(i))].idx_v();

    reference_x(joint_index_q) = curr_target_q_(joint_index_q);
    reference_x(pinocchio_model_->nq + joint_index_v) = curr_target_dq_(joint_index_v);
  }
  xref_ = reference_x;
}

void FullbodyFlightController::rootPosCommandCallback(const geometry_msgs::Vector3ConstPtr& msg)
{
  xref_.head(3) << msg->x, msg->y, msg->z;
  ROS_INFO_STREAM("[ddp] receive root position command: " << xref_.head(3).transpose());

  // visualize path
  nav_msgs::Path path_msg;
  path_msg.header.stamp = ros::Time::now();
  path_msg.header.frame_id = "world";
  geometry_msgs::PoseStamped pose;
  // current position
  pose.pose.position.x = estimator_->getPos(Frame::BASELINK, estimate_mode_).x();
  pose.pose.position.y = estimator_->getPos(Frame::BASELINK, estimate_mode_).y();
  pose.pose.position.z = estimator_->getPos(Frame::BASELINK, estimate_mode_).z();
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 0.0;
  pose.pose.orientation.w = 1.0;
  path_msg.poses.push_back(pose);
  // target position
  pose.pose.position.x = msg->x;
  pose.pose.position.y = msg->y;
  pose.pose.position.z = msg->z;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 0.0;
  pose.pose.orientation.w = 1.0;
  path_msg.poses.push_back(pose);
  path_pub_.publish(path_msg);
}

void FullbodyFlightController::rootPoseCommandCallback(const geometry_msgs::PoseConstPtr& msg)
{
  xref_.head(3) << msg->position.x, msg->position.y, msg->position.z;
  xref_.segment(3, 4) << msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w;
  ROS_INFO_STREAM("[ddp] receive root pose command: " << xref_.head(7).transpose());

  // visualize path
  nav_msgs::Path path_msg;
  path_msg.header.stamp = ros::Time::now();
  path_msg.header.frame_id = "world";
  geometry_msgs::PoseStamped pose;
  // current position
  pose.pose.position.x = estimator_->getPos(Frame::BASELINK, estimate_mode_).x();
  pose.pose.position.y = estimator_->getPos(Frame::BASELINK, estimate_mode_).y();
  pose.pose.position.z = estimator_->getPos(Frame::BASELINK, estimate_mode_).z();
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 0.0;
  pose.pose.orientation.w = 1.0;
  path_msg.poses.push_back(pose);
  // target position
  pose.pose.position.x = msg->position.x;
  pose.pose.position.y = msg->position.y;
  pose.pose.position.z = msg->position.z;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 0.0;
  pose.pose.orientation.w = 1.0;
  path_msg.poses.push_back(pose);
  path_pub_.publish(path_msg);
}

void FullbodyFlightController::circleTrajectoryCommandCallback(const std_msgs::EmptyConstPtr& msg)
{
  ros::NodeHandle circle_traj_nh(nh_, "circle_trajectory");
  circle_traj_nh.param("radius", circle_radius_, 1.0);
  circle_traj_nh.param("duration", circle_duration_, M_PI);
  circle_traj_nh.param("loop", circle_loop_, 3);

  Eigen::Vector3d target_pos = hovering_->state_residuals_.at(0)->get_reference().head(3);
  circle_center_ = Eigen::Vector3d(target_pos(0) - circle_radius_, target_pos(1), target_pos(2));

  xref_.head(3) = target_pos;

  ROS_INFO_STREAM("[ddp] circle trajectory center: " << circle_center_.transpose() << ", radius: " << circle_radius_
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

  circle_trajectory_start_time_ = ros::Time::now().toSec() + 6.0;
  circle_trajectory_end_time_ = circle_trajectory_start_time_ + circle_loop_ * circle_duration_;
  circle_trajectory_flight_flag_ = true;
}

void FullbodyFlightController::jointTrajectoryCommandCallback(const std_msgs::EmptyConstPtr& msg)
{
  ros::NodeHandle joint_traj_nh(nh_, "joint_trajectory");
  joint_traj_nh.param("duration", joint_trajectory_duration_, 1.0);
  joint_traj_nh.param("loop", joint_trajectory_loop_, 3);

  joint_trajectory_names_ = nonlinear_inverse_dynamics_solver_->getJointNames();
  joint_traj_nh.getParam("start_angle", joint_trajectory_angle_start_);
  if (joint_trajectory_angle_start_.size() != joint_trajectory_names_.size())
  {
    ROS_ERROR("[ddp] Joint trajectory command for start angle size mismatch.");
    return;
  }

  joint_traj_nh.getParam("end_angle", joint_trajectory_angle_end_);
  if (joint_trajectory_angle_end_.size() != joint_trajectory_names_.size())
  {
    ROS_ERROR("[ddp] Joint trajectory command for end angle size mismatch.");
    return;
  }

  for (int i = 0; i < joint_trajectory_names_.size(); i++)
  {
    int joint_id = pinocchio_model_->getJointId(joint_trajectory_names_.at(i));
    int joint_index_q = pinocchio_model_->joints[joint_id].idx_q();
    curr_target_q_(joint_index_q) = joint_trajectory_angle_start_.at(i);
    xref_(joint_index_q) = joint_trajectory_angle_start_.at(i);
  }

  ROS_INFO_STREAM("[ddp] start joint trajectory tracking with duration: " << joint_trajectory_duration_ << " s"
                                                                          << ", loop: " << joint_trajectory_loop_);

  joint_trajectory_start_time_ = ros::Time::now().toSec() + 6.0;
  joint_trajectory_end_time_ = joint_trajectory_start_time_ + joint_trajectory_loop_ * joint_trajectory_duration_;
  joint_trajectory_flight_flag_ = true;
}

void FullbodyFlightController::transformingTrackingCommandCallback(const std_msgs::EmptyConstPtr& msg)
{
  ros::NodeHandle circle_traj_nh(nh_, "circle_trajectory");
  circle_traj_nh.param("radius", circle_radius_, 1.0);
  circle_traj_nh.param("duration", circle_duration_, M_PI);
  circle_traj_nh.param("loop", circle_loop_, 3);

  ros::NodeHandle joint_traj_nh(nh_, "joint_trajectory");
  joint_traj_nh.param("duration", joint_trajectory_duration_, 1.0);
  joint_traj_nh.param("loop", joint_trajectory_loop_, 3);

  // circle trajecotry of root
  Eigen::Vector3d target_pos = hovering_->state_residuals_.at(0)->get_reference().head(3);
  circle_center_ = Eigen::Vector3d(target_pos(0) - circle_radius_, target_pos(1), target_pos(2));

  xref_.head(3) = target_pos;

  ROS_INFO_STREAM("[ddp] circle trajectory center: " << circle_center_.transpose() << ", radius: " << circle_radius_
                                                     << ", duration: " << circle_duration_ << " s"
                                                     << ", loop: " << circle_loop_);

  // joint trajecotry
  joint_trajectory_names_ = nonlinear_inverse_dynamics_solver_->getJointNames();
  joint_traj_nh.getParam("start_angle", joint_trajectory_angle_start_);
  if (joint_trajectory_angle_start_.size() != joint_trajectory_names_.size())
  {
    ROS_ERROR("[ddp] Joint trajectory command for start angle size mismatch.");
    return;
  }

  joint_traj_nh.getParam("end_angle", joint_trajectory_angle_end_);
  if (joint_trajectory_angle_end_.size() != joint_trajectory_names_.size())
  {
    ROS_ERROR("[ddp] Joint trajectory command for end angle size mismatch.");
    return;
  }

  for (int i = 0; i < joint_trajectory_names_.size(); i++)
  {
    int joint_id = pinocchio_model_->getJointId(joint_trajectory_names_.at(i));
    int joint_index_q = pinocchio_model_->joints[joint_id].idx_q();
    curr_target_q_(joint_index_q) = joint_trajectory_angle_start_.at(i);
    xref_(joint_index_q) = joint_trajectory_angle_start_.at(i);
  }

  ROS_INFO_STREAM("[ddp] start joint trajectory tracking with duration: " << joint_trajectory_duration_ << " s"
                                                                          << ", loop: " << joint_trajectory_loop_);

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

  circle_trajectory_start_time_ = ros::Time::now().toSec() + 6.0;
  circle_trajectory_end_time_ = circle_trajectory_start_time_ + circle_loop_ * circle_duration_;

  joint_trajectory_start_time_ = circle_trajectory_start_time_;
  joint_trajectory_end_time_ = joint_trajectory_start_time_ + joint_trajectory_loop_ * joint_trajectory_duration_;

  joint_trajectory_flight_flag_ = true;
  circle_trajectory_flight_flag_ = true;
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
