#include <dynarm/control/full_vectoring_pid_control.h>

using namespace aerial_robot_control;

FullVectoringPIDController::FullVectoringPIDController() : PoseLinearController()
{
}

void FullVectoringPIDController::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                            boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                            boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                            boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                            double ctrl_loop_rate)
{
  PoseLinearController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);
  rosParamInit();

  full_vectoring_robot_model_ = boost::dynamic_pointer_cast<FullVectoringRobotModel>(robot_model);

  gimbal_control_pub_ = nh_.advertise<sensor_msgs::JointState>("gimbals_ctrl", 1);
  flight_cmd_pub_ = nh_.advertise<spinal::FourAxisCommand>("four_axes/command", 1);
  rotor_wrench_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("rotor_wrench", 1);

  target_base_thrust_.resize(motor_num_, 0.0);
  target_gimbal_angles_.resize(motor_num_ * 2, 0.0);

  robot_ns_ = ros::this_node::getNamespace();
  if (!robot_ns_.empty() && robot_ns_[0] == '/')
    robot_ns_ = robot_ns_.substr(1);
  rotor_wrench_pub_index_ = 0;
}

void FullVectoringPIDController::rosParamInit()
{
}

void FullVectoringPIDController::controlCore()
{
  PoseLinearController::controlCore();

  tf::Matrix3x3 uav_rot = estimator_->getOrientation(Frame::COG, estimate_mode_);
  tf::Vector3 target_acc_w(pid_controllers_.at(X).result(), pid_controllers_.at(Y).result(),
                           pid_controllers_.at(Z).result());
  tf::Vector3 target_acc_cog = uav_rot.inverse() * target_acc_w;
  Eigen::VectorXd target_wrench_acc_cog = Eigen::VectorXd::Zero(6);
  target_wrench_acc_cog.head(3) = Eigen::Vector3d(target_acc_cog.x(), target_acc_cog.y(), target_acc_cog.z());

  double target_ang_acc_x = pid_controllers_.at(ROLL).result();
  double target_ang_acc_y = pid_controllers_.at(PITCH).result();
  double target_ang_acc_z = pid_controllers_.at(YAW).result();
  target_wrench_acc_cog.tail(3) = Eigen::Vector3d(target_ang_acc_x, target_ang_acc_y, target_ang_acc_z);

  pid_msg_.roll.total.at(0) = target_ang_acc_x;
  pid_msg_.roll.p_term.at(0) = pid_controllers_.at(ROLL).getPTerm();
  pid_msg_.roll.i_term.at(0) = pid_controllers_.at(ROLL).getITerm();
  pid_msg_.roll.d_term.at(0) = pid_controllers_.at(ROLL).getDTerm();
  pid_msg_.roll.target_p = target_rpy_.x();
  pid_msg_.roll.err_p = pid_controllers_.at(ROLL).getErrP();
  pid_msg_.roll.target_d = target_omega_.x();
  pid_msg_.roll.err_d = pid_controllers_.at(ROLL).getErrD();
  pid_msg_.pitch.total.at(0) = target_ang_acc_y;
  pid_msg_.pitch.p_term.at(0) = pid_controllers_.at(PITCH).getPTerm();
  pid_msg_.pitch.i_term.at(0) = pid_controllers_.at(PITCH).getITerm();
  pid_msg_.pitch.d_term.at(0) = pid_controllers_.at(PITCH).getDTerm();
  pid_msg_.pitch.target_p = target_rpy_.y();
  pid_msg_.pitch.err_p = pid_controllers_.at(PITCH).getErrP();
  pid_msg_.pitch.target_d = target_omega_.y();
  pid_msg_.pitch.err_d = pid_controllers_.at(PITCH).getErrD();

  /* calculate the allocation matrix */
  const auto links_rotation_from_cog = full_vectoring_robot_model_->getLinksRotationFromCog<Eigen::Matrix3d>();
  Eigen::MatrixXd full_q_mat(6, motor_num_ * 3);
  for (int i = 0; i < motor_num_; i++)
  {
    std::vector<Eigen::Vector3d> rotors_origin_from_cog = robot_model_->getRotorsOriginFromCog<Eigen::Vector3d>();

    Eigen::MatrixXd wrench_map = Eigen::MatrixXd::Zero(6, 3);
    wrench_map.block(0, 0, 3, 3) = Eigen::MatrixXd::Identity(3, 3);
    wrench_map.block(3, 0, 3, 3) = aerial_robot_model::skew(rotors_origin_from_cog.at(i));

    full_q_mat.block(0, i * 3, 6, 3) = wrench_map * links_rotation_from_cog.at(i);
  }

  Eigen::Matrix3d inertia_inv = robot_model_->getInertia<Eigen::Matrix3d>().inverse();  // update
  double mass_inv = 1 / robot_model_->getMass();
  full_q_mat.topRows(3) = mass_inv * full_q_mat.topRows(3);
  full_q_mat.bottomRows(3) = inertia_inv * full_q_mat.bottomRows(3);

  Eigen::MatrixXd full_q_mat_inv = aerial_robot_model::pseudoinverse(full_q_mat);
  Eigen::VectorXd target_vectoring_f = full_q_mat_inv * target_wrench_acc_cog;

  for (int i = 0; i < motor_num_; i++)
  {
    Eigen::Vector3d f_i = target_vectoring_f.segment(i * 3, 3);
    target_base_thrust_.at(i) = f_i.norm();

    double gimbal_i_roll = atan2(-f_i.y(), f_i.z());
    double gimbal_i_pitch = atan2(f_i.x(), -f_i.y() * sin(gimbal_i_roll) + f_i.z() * cos(gimbal_i_roll));
    target_gimbal_angles_.at(2 * i) = gimbal_i_roll;
    target_gimbal_angles_.at(2 * i + 1) = gimbal_i_pitch;
  }
}

void FullVectoringPIDController::sendCmd()
{
  PoseLinearController::sendCmd();

  spinal::FourAxisCommand flight_command_data;
  flight_command_data.base_thrust = target_base_thrust_;
  flight_cmd_pub_.publish(flight_command_data);

  sensor_msgs::JointState gimbal_control_msg;
  gimbal_control_msg.header.stamp = ros::Time::now();
  for (int i = 0; i < motor_num_ * 2; i++)
    gimbal_control_msg.position.push_back(target_gimbal_angles_.at(i));
  gimbal_control_pub_.publish(gimbal_control_msg);

  // for debug: rotor wrench
  double m_f_rate = robot_model_->getMFRate();
  auto rotor_direction = robot_model_->getRotorDirection();
  geometry_msgs::WrenchStamped rotor_wrench_msg;
  rotor_wrench_msg.header.stamp = ros::Time::now();
  rotor_wrench_msg.header.frame_id = robot_ns_ + "/thrust" + std::to_string(rotor_wrench_pub_index_ + 1);
  rotor_wrench_msg.wrench.force.x = 0.0;
  rotor_wrench_msg.wrench.force.y = 0.0;
  rotor_wrench_msg.wrench.force.z = target_base_thrust_.at(rotor_wrench_pub_index_);
  rotor_wrench_msg.wrench.torque.x = 0.0;
  rotor_wrench_msg.wrench.torque.y = 0.0;
  rotor_wrench_msg.wrench.torque.z =
      rotor_direction.at(rotor_wrench_pub_index_ + 1) * m_f_rate * target_base_thrust_.at(rotor_wrench_pub_index_);
  rotor_wrench_pub_.publish(rotor_wrench_msg);
  rotor_wrench_pub_index_ = (rotor_wrench_pub_index_ + 1) % robot_model_->getRotorNum();
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::FullVectoringPIDController, aerial_robot_control::ControlBase);
