#include <dynarm/control/whole_body_force_controller.h>

using namespace aerial_robot_control;

WholeBodyForceController::WholeBodyForceController() : ControlBase()
{
}

void WholeBodyForceController::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                          boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                          boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                          boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                          double ctrl_loop_du)
{
  ControlBase::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_du);

  dynarm_robot_model_ = boost::dynamic_pointer_cast<aerial_robot_model::ManipulatorRobotModel>(robot_model);
  pinocchio_robot_model_ = dynarm_robot_model_->getPinocchioRobotModel();
  pinocchio_model_ = pinocchio_robot_model_->getModel();
  pinocchio_data_ = std::make_shared<pinocchio::Data>(*pinocchio_model_);

  curr_target_q_ = Eigen::VectorXd::Zero(pinocchio_model_->nq);
  curr_target_dq_ = Eigen::VectorXd::Zero(pinocchio_model_->nv);
  init_target_q_ = Eigen::VectorXd::Zero(pinocchio_model_->nq);
  final_target_q_ = Eigen::VectorXd::Zero(pinocchio_model_->nq);

  loadJointNames();
  loadGimbalGroups();
  rosParamInit();

  joints_control_pub_ = nh_.advertise<sensor_msgs::JointState>("joints_ctrl", 1);
  gimbals_control_pub_ = nh_.advertise<sensor_msgs::JointState>("gimbals_ctrl", 1);
  target_q_pub_ = nh_.advertise<sensor_msgs::JointState>("debug/target_q", 1);
}

void WholeBodyForceController::loadJointNames()
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

void WholeBodyForceController::loadGimbalGroups()
{
  gimbal_groups_ = dynarm::findGimbalGroups<double>(*pinocchio_model_, false);

  std::stringstream ss;
  for (const auto& group : gimbal_groups_)
  {
    ss << " " << group.rotor_frame_name << "(";
    for (int i = 0; i < group.joint_names.size(); i++)
      ss << (i == 0 ? "" : ", ") << group.joint_names.at(i);
    ss << ")";
  }
  ROS_INFO_STREAM("[dynarm][whole_body_force_control] " << gimbal_groups_.size()
                                                        << " gimbal groups found:" << ss.str());
}

Eigen::VectorXd WholeBodyForceController::getGimbalNominalAngles(const Eigen::VectorXd& q, bool global_search)
{
  Eigen::VectorXd gimbal_processed_q = q;

  dynarm::AlignUpwardOptionsTpl<double> options;
  options.verbose = false;
  if (!global_search)
  {
    options.max_seed_combinations = 1;  // start from the gimbal angles already in q
    options.max_iterations = 10;
    options.max_damping_steps = 10;
  }

  for (const auto& group : gimbal_groups_)
    dynarm::alignRotorUpward<double>(*pinocchio_model_, *pinocchio_data_, gimbal_processed_q, group, options);

  return gimbal_processed_q;
}

void WholeBodyForceController::rosParamInit()
{
  ros::NodeHandle control_nh(nh_, "controller");

  // show the order of the configuration vector, which init_target_q has to follow
  std::stringstream q_order;
  for (int i = 1; i < pinocchio_model_->njoints; i++)
    q_order << " " << pinocchio_model_->joints[i].idx_q() << ":" << pinocchio_model_->names[i];
  ROS_INFO_STREAM("[dynarm][whole_body_force_control] order of init_target_q:" << q_order.str());

  // target joint angle after takeoff
  std::vector<double> init_target_q;
  control_nh.getParam("init_target_q", init_target_q);
  if (init_target_q.size() != pinocchio_model_->nq)
  {
    ROS_ERROR_STREAM("[dynarm][whole_body_force_control] nq: "
                     << pinocchio_model_->nq << " and initial joint angle size: " << init_target_q.size()
                     << " in ros parameter is not same");
  }
  else
  {
    for (int i = 0; i < pinocchio_model_->nq; i++)
    {
      final_target_q_(i) = init_target_q.at(i);
    }
  }
  // set gimbal nominal angles for the target joint angle
  final_target_q_ = getGimbalNominalAngles(final_target_q_, true);
  ROS_INFO_STREAM("[dynarm][whole_body_force_control] initial target joint angle: " << final_target_q_.transpose());

  // duration to reach the target joint angle
  getParam<double>(control_nh, "transform_duration", transform_duration_, 1.0);
}

void WholeBodyForceController::resetJointTrajectory()
{
  // start the transformation from the current joint angles
  Eigen::VectorXd curr_q = dynarm_robot_model_->getCurrentJointPositions();

  is_transforming_ = true;
  transform_start_time_ = ros::Time::now().toSec();
  init_target_q_ = getGimbalNominalAngles(curr_q, true);
  curr_target_q_ = init_target_q_;
  curr_target_dq_ = Eigen::VectorXd::Zero(pinocchio_model_->nv);
}

void WholeBodyForceController::reset()
{
  ControlBase::reset();

  resetJointTrajectory();
}

bool WholeBodyForceController::update()
{
  if (!ControlBase::update())
  {
    resetJointTrajectory();  // keep the start pose updated until takeoff
    return false;
  }

  controlCore();
  sendCmd();

  return true;
}

void WholeBodyForceController::controlCore()
{
  if (!is_transforming_)
  {
    curr_target_q_ = final_target_q_;
    curr_target_dq_ = Eigen::VectorXd::Zero(pinocchio_model_->nv);
    return;
  }

  double curr_time = ros::Time::now().toSec() - transform_start_time_;

  if (transform_duration_ <= 0.0 || curr_time >= transform_duration_)
  {
    curr_target_q_ = final_target_q_;
    curr_target_dq_ = Eigen::VectorXd::Zero(pinocchio_model_->nv);
    is_transforming_ = false;
    ROS_INFO_STREAM("[dynarm][whole_body_force_control] transformation to the target joint angle completed");
    return;
  }

  // linear interpolation from init_target_q_ to final_target_q_
  Eigen::VectorXd delta_q = pinocchio::difference(*pinocchio_model_, init_target_q_, final_target_q_);  // nv

  // the gimbals are not interpolated: they follow the nominal angle of the interpolated pose
  for (const auto& group : gimbal_groups_)
    for (const int idx_v : group.idx_vs)
      delta_q(idx_v) = 0.0;

  curr_target_q_ = pinocchio::integrate(*pinocchio_model_, init_target_q_, delta_q * (curr_time / transform_duration_));
  curr_target_dq_ = delta_q / transform_duration_;
  curr_target_q_ = getGimbalNominalAngles(curr_target_q_, false);
}

void WholeBodyForceController::sendCmd()
{
  sendJointCommand();
  sendGimbalCommand();
}

void WholeBodyForceController::sendJointCommand()
{
  // send target joint angle, velocity and feedforward torque
  sensor_msgs::JointState joint_state_msg;
  joint_state_msg.header.stamp = ros::Time::now();

  for (int i = 0; i < joint_names_.size(); i++)
  {
    int joint_index_q = pinocchio_model_->joints[pinocchio_model_->getJointId(joint_names_.at(i))].idx_q();
    int joint_index_v = pinocchio_model_->joints[pinocchio_model_->getJointId(joint_names_.at(i))].idx_v();

    joint_state_msg.name.push_back(joint_names_.at(i));
    joint_state_msg.position.push_back(curr_target_q_(joint_index_q));
    joint_state_msg.velocity.push_back(curr_target_dq_(joint_index_v));
    joint_state_msg.effort.push_back(0.0);  // TODO: feedforward torque from the whole body force control
  }

  joints_control_pub_.publish(joint_state_msg);
  target_q_pub_.publish(joint_state_msg);
}

void WholeBodyForceController::sendGimbalCommand()
{
  // send the nominal gimbal angles of the target pose
  sensor_msgs::JointState joint_state_msg;
  joint_state_msg.header.stamp = ros::Time::now();

  for (const auto& group : gimbal_groups_)
  {
    for (int i = 0; i < group.joint_names.size(); i++)
    {
      joint_state_msg.name.push_back(group.joint_names.at(i));
      joint_state_msg.position.push_back(curr_target_q_(group.idx_qs.at(i)));
    }
  }

  gimbals_control_pub_.publish(joint_state_msg);
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::WholeBodyForceController, aerial_robot_control::ControlBase);
