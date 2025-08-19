#include <dynarm/control/trajectory_generator.h>

using namespace aerial_robot_control;

TrajectoryGenerator::TrajectoryGenerator(
    ros::NodeHandle nh, std::shared_ptr<aerial_robot_dynamics::PinocchioRobotModel> pinocchio_robot_model)
  : nh_(nh), pinocchio_robot_model_(pinocchio_robot_model)
{
  pinocchio_model_ = pinocchio_robot_model_->getModel();
  pinocchio_data_ = pinocchio_robot_model_->getData();

  rosParamInit();
  loadGimbalNames();

  joint_trajectory_generator_ =
      std::make_shared<aerial_robot_control::jointTrajectoryGenerator>(nh_, pinocchio_robot_model_);
  nonlinear_inverse_dynamics_solver_ =
      std::make_shared<aerial_robot_model::NonlinearInverseDynamics>(nh_, pinocchio_robot_model_);

  target_q_pub_ = nh_.advertise<sensor_msgs::JointState>("debug/target_q", 1);
  target_joint_torque_pub_ = nh_.advertise<sensor_msgs::JointState>("debug/id_debug/torque", 1);
  tau_by_thrust_pub_ = nh_.advertise<sensor_msgs::JointState>("debug/id_debug/tau_by_thrust", 1);
  rotor_wrench_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("rotor_wrench", 1);
  thrust_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("debug/id_debug/thrust", 1);
  id_solve_time_pub_ = nh_.advertise<std_msgs::Float32>("debug/id_debug/solve_time", 1);
  dummy_joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 10);

  // for rotor wrench visualization
  robot_ns_ = ros::this_node::getNamespace();
  if (!robot_ns_.empty() && robot_ns_[0] == '/')
    robot_ns_ = robot_ns_.substr(1);
  rotor_wrench_pub_index_ = 0;

  curr_target_tau_ = Eigen::VectorXd::Zero(pinocchio_robot_model_->getModel()->nv);
  curr_target_thrust_ = Eigen::VectorXd::Zero(pinocchio_robot_model_->getRotorNum());
  curr_target_gimbal_angle_ = Eigen::VectorXd::Zero(gimbal_names_.size());  // gimbal names must be loaded beforehand
}

void TrajectoryGenerator::rosParamInit()
{
  ros::NodeHandle control_nh(nh_, "controller");
  getParam<bool>(control_nh, "nonlinear_mode", nonlinear_mode_, true);
}

void TrajectoryGenerator::loadGimbalNames()
{
  gimbal_names_.clear();
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

void TrajectoryGenerator::reset()
{
  joint_trajectory_generator_->reset();
  nonlinear_inverse_dynamics_solver_->reset();
}

void TrajectoryGenerator::timer(ros::TimerEvent const& event)
{
  joint_trajectory_generator_->setCurrentQ(joint_trajectory_generator_->getCurrentTargetQ());
  joint_trajectory_generator_->setCurrentDQ(joint_trajectory_generator_->getCurrentTargetDQ());

  update();

  joint_trajectory_generator_->publish();
  this->publish();
  this->publishDummyJointState();
}

void TrajectoryGenerator::update()
{
  joint_trajectory_generator_->update();
  Eigen::VectorXd q = joint_trajectory_generator_->getCurrentQ();
  Eigen::VectorXd dq = joint_trajectory_generator_->getCurrentDQ();
  Eigen::VectorXd ddq = joint_trajectory_generator_->getCurrentTargetDDQ();

  if (!nonlinear_mode_)
    q = joint_trajectory_generator_->getGimbalNominalAngles(q);

  solveInverseDynamics(q, dq, ddq);
}

bool TrajectoryGenerator::solveInverseDynamics(Eigen::VectorXd q, Eigen::VectorXd dq, Eigen::VectorXd ddq)
{
  Eigen::VectorXd id_result;
  bool solved;
  if (nonlinear_mode_)
  {
    solved = nonlinear_inverse_dynamics_solver_->solve(q, dq, ddq, id_result);
    curr_target_tau_ = id_result.head(pinocchio_model_->nv);
    curr_target_thrust_ = id_result.segment(pinocchio_model_->nv, pinocchio_robot_model_->getRotorNum());
    curr_target_gimbal_angle_ = id_result.tail(nonlinear_inverse_dynamics_solver_->getGimbalNames().size());
  }
  else
  {
    solved = pinocchio_robot_model_->inverseDynamics(q, dq, ddq, id_result);
    curr_target_tau_ = id_result.head(pinocchio_model_->nv);
    curr_target_thrust_ = id_result.tail(pinocchio_robot_model_->getRotorNum());
    for (int i = 0; i < gimbal_names_.size(); i++)
    {
      int gimbal_index_q = pinocchio_model_->joints[pinocchio_model_->getJointId(gimbal_names_.at(i))].idx_q();
      curr_target_gimbal_angle_(i) = q(gimbal_index_q);
    }
  }

  if (!solved)
  {
    ROS_ERROR_STREAM("[dragon_arm][control] Inverse dynamics failed to solve"
                     << "\n   q: " << q.transpose() << "\n  dq: " << dq.transpose() << "\n ddq: " << ddq.transpose());
  }
  return solved;
}

void TrajectoryGenerator::publish()
{
  // for debug: send target q for gimbal part
  sensor_msgs::JointState target_q_msg;
  target_q_msg.header.stamp = ros::Time::now();
  for (int i = 0; i < gimbal_names_.size(); i++)
  {
    int gimbal_index_q = pinocchio_model_->joints[pinocchio_model_->getJointId(gimbal_names_.at(i))].idx_q();
    target_q_msg.name.push_back(gimbal_names_.at(i));
    target_q_msg.position.push_back(curr_target_gimbal_angle_(gimbal_index_map_.at(gimbal_names_.at(i))));
  }
  target_q_pub_.publish(target_q_msg);

  // for debug: send torque
  sensor_msgs::JointState target_joint_torque_msg;
  target_joint_torque_msg.header.stamp = ros::Time::now();
  sensor_msgs::JointState tau_by_thrust_msg;
  // Eigen::MatrixXd tauext_partial_thrust = pinocchio_robot_model_->computeTauExtByThrustDerivative(curr_target_q_);
  // Eigen::VectorXd tauext = tauext_partial_thrust * curr_target_thrust_;
  for (int i = 0; i < pinocchio_model_->njoints; i++)
  {
    int joint_index_q = pinocchio_model_->joints[pinocchio_model_->getJointId(pinocchio_model_->names[i])].idx_q();
    int joint_index_v = pinocchio_model_->joints[pinocchio_model_->getJointId(pinocchio_model_->names[i])].idx_v();
    if (joint_index_q < 0 || joint_index_v < 0)
      continue;  // skip if joint index is invalid

    target_joint_torque_msg.name.push_back(pinocchio_model_->names[i]);
    target_joint_torque_msg.effort.push_back(curr_target_tau_(joint_index_v));

    // tau_by_thrust_msg.name.push_back(pinocchio_model_->names[i]);
    // tau_by_thrust_msg.effort.push_back(tauext(joint_index_v));
  }
  target_joint_torque_pub_.publish(target_joint_torque_msg);
  // tau_by_thrust_pub_.publish(tau_by_thrust_msg);

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

  // for debug: send ID solve time
  std_msgs::Float32 id_solve_time_msg;
  if (nonlinear_mode_)
    id_solve_time_msg.data = nonlinear_inverse_dynamics_solver_->getSolveTime();
  else
    id_solve_time_msg.data = pinocchio_robot_model_->getLatestIdSolveTime();
  id_solve_time_pub_.publish(id_solve_time_msg);
}

void TrajectoryGenerator::publishDummyJointState()
{
  // for debug: publish joint position effort
  sensor_msgs::JointState joint_state_msg;
  joint_state_msg.header.stamp = ros::Time::now();
  Eigen::VectorXd curr_target_q = joint_trajectory_generator_->getCurrentTargetQ();

  // overwrite gimbal angles with current target gimbal angles
  for (int i = 0; i < gimbal_names_.size(); i++)
  {
    int gimbal_index_q = pinocchio_model_->joints[pinocchio_model_->getJointId(gimbal_names_.at(i))].idx_q();
    curr_target_q(gimbal_index_q) = curr_target_gimbal_angle_(gimbal_index_map_.at(gimbal_names_.at(i)));
  }

  for (int i = ((pinocchio_robot_model_->getIsFloatingBase()) ? 2 : 1); i < pinocchio_model_->njoints; i++)
  {
    int joint_index_q = pinocchio_model_->joints[pinocchio_model_->getJointId(pinocchio_model_->names[i])].idx_q();
    int joint_index_v = pinocchio_model_->joints[pinocchio_model_->getJointId(pinocchio_model_->names[i])].idx_v();
    joint_state_msg.name.push_back(pinocchio_model_->names[i]);
    joint_state_msg.position.push_back(curr_target_q(joint_index_q));
    joint_state_msg.effort.push_back(curr_target_tau_(joint_index_v));
  }
  dummy_joint_state_pub_.publish(joint_state_msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "trajectory_generation_test");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

  bool is_floating_base;
  nhp.param("dynamics/is_floating_base", is_floating_base, false);
  std::shared_ptr<aerial_robot_dynamics::PinocchioRobotModel> pinocchio_robot_model =
      std::make_shared<aerial_robot_dynamics::PinocchioRobotModel>(is_floating_base);

  TrajectoryGenerator trajectory_generator(nh, pinocchio_robot_model);

  ros::Timer timer =
      nh.createTimer(ros::Duration(0.01), boost::bind(&TrajectoryGenerator::timer, &trajectory_generator, _1));

  ros::spin();
  return 0;
}
