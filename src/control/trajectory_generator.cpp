#include <dynarm/control/trajectory_generator.h>

using namespace aerial_robot_control;

TrajectoryGenerator::TrajectoryGenerator(
    std::shared_ptr<aerial_robot_dynamics::PinocchioRobotModel> pinocchio_robot_model,
    std::shared_ptr<aerial_robot_control::jointTrajectoryGenerator> joint_trajectory_generator,
    std::shared_ptr<aerial_robot_model::NonlinearInverseDynamics> nonlinear_inverse_dynamics_solver)
  : pinocchio_robot_model_(pinocchio_robot_model)
  , joint_trajectory_generator_(joint_trajectory_generator)
  , nonlinear_inverse_dynamics_solver_(nonlinear_inverse_dynamics_solver)
{
  pinocchio_model_ = pinocchio_robot_model_->getModel();
  pinocchio_data_ = pinocchio_robot_model_->getData();

  loadGimbalNames();

  curr_target_tau_ = Eigen::VectorXd::Zero(pinocchio_robot_model_->getModel()->nv);
  curr_target_thrust_ = Eigen::VectorXd::Zero(pinocchio_robot_model_->getRotorNum());
  curr_target_gimbal_angle_ = Eigen::VectorXd::Zero(gimbal_names_.size());  // gimbal names must be loaded beforehand
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
  curr_target_tau_ = Eigen::VectorXd::Zero(pinocchio_robot_model_->getModel()->nv);
  curr_target_thrust_ = Eigen::VectorXd::Zero(pinocchio_robot_model_->getRotorNum());
  curr_target_gimbal_angle_ = Eigen::VectorXd::Zero(gimbal_names_.size());
}

void TrajectoryGenerator::update()
{
  joint_trajectory_generator_->update();
  Eigen::VectorXd q = joint_trajectory_generator_->getCurrentQ();
  Eigen::VectorXd dq = joint_trajectory_generator_->getCurrentDQ();
  Eigen::VectorXd ddq = joint_trajectory_generator_->getCurrentTargetDDQ();

  if (!nonlinear_mode_)
    q = joint_trajectory_generator_->getGimbalNominalAngles(q);

  if (quasi_static_mode_)
  {
    dq = Eigen::VectorXd::Zero(pinocchio_model_->nv);
    ddq = Eigen::VectorXd::Zero(pinocchio_model_->nv);
  }

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
    ROS_ERROR_STREAM("[TrajectoryGenerator] Inverse dynamics failed to solve"
                     << "\n   q: " << q.transpose() << "\n  dq: " << dq.transpose() << "\n ddq: " << ddq.transpose());
  }
  return solved;
}

TrajectoryGeneratorRos::TrajectoryGeneratorRos(
    ros::NodeHandle nh, std::shared_ptr<aerial_robot_dynamics::PinocchioRobotModel> pinocchio_robot_model,
    std::shared_ptr<aerial_robot_control::jointTrajectoryGeneratorRos> joint_trajectory_generator_ros,
    std::shared_ptr<aerial_robot_model::NonlinearInverseDynamicsRos> nonlinear_inverse_dynamics_solver_ros)
  : nh_(nh)
  , joint_trajectory_generator_ros_(joint_trajectory_generator_ros)
  , nonlinear_inverse_dynamics_solver_ros_(nonlinear_inverse_dynamics_solver_ros)
{
  std::shared_ptr<aerial_robot_control::jointTrajectoryGenerator> joint_trajectory_generator =
      joint_trajectory_generator_ros->getJointTrajectoryGenerator();
  std::shared_ptr<aerial_robot_model::NonlinearInverseDynamics> nonlinear_inverse_dynamics_solver =
      nonlinear_inverse_dynamics_solver_ros->getNonlinearInverseDynamicsSolver();

  trajectory_generator_ = std::make_shared<aerial_robot_control::TrajectoryGenerator>(
      pinocchio_robot_model, joint_trajectory_generator, nonlinear_inverse_dynamics_solver);

  target_q_pub_ = nh_.advertise<sensor_msgs::JointState>("debug/target_q", 1);
  rotor_wrench_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("rotor_wrench", 1);
  id_solve_time_pub_ = nh_.advertise<std_msgs::Float32>("debug/id_debug/solve_time", 1);

  tau_by_thrust_pub_ = nh_.advertise<sensor_msgs::JointState>("debug/id_debug/tau_by_thrust", 1);
  thrust_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("debug/id_debug/thrust", 1);
  target_joint_torque_pub_ = nh_.advertise<sensor_msgs::JointState>("debug/id_debug/torque", 1);
  dummy_joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 10);

  rosParamInit();

  // for rotor wrench visualization
  robot_ns_ = ros::this_node::getNamespace();
  if (!robot_ns_.empty() && robot_ns_[0] == '/')
    robot_ns_ = robot_ns_.substr(1);
  rotor_wrench_pub_index_ = 0;
}

void TrajectoryGeneratorRos::rosParamInit()
{
  ros::NodeHandle control_nh(nh_, "controller");
  getParam<bool>(control_nh, "nonlinear_mode", trajectory_generator_->nonlinear_mode_, true);
  getParam<bool>(control_nh, "quasi_static_mode", trajectory_generator_->quasi_static_mode_, false);
}

void TrajectoryGeneratorRos::timer(ros::TimerEvent const& event)
{
  trajectory_generator_->getJointTrajectoryGenerator()->setCurrentQ(
      trajectory_generator_->getJointTrajectoryGenerator()->getCurrentTargetQ());
  trajectory_generator_->getJointTrajectoryGenerator()->setCurrentDQ(
      trajectory_generator_->getJointTrajectoryGenerator()->getCurrentTargetDQ());

  trajectory_generator_->update();

  publishAll();
}

void TrajectoryGeneratorRos::publish()
{
  // for debug: send target q for gimbal part
  sensor_msgs::JointState target_q_msg;
  std::vector<std::string> gimbal_names = trajectory_generator_->getGimbalNames();
  std::map<std::string, int> gimbal_index_map = trajectory_generator_->getGimbalIndexMap();
  Eigen::VectorXd curr_target_gimbal_angle = trajectory_generator_->getCurrentTargetGimbalAngle();
  target_q_msg.header.stamp = ros::Time::now();
  for (int i = 0; i < gimbal_names.size(); i++)
  {
    int joint_id = trajectory_generator_->getPinocchioModel()->getJointId(gimbal_names.at(i));
    int gimbal_index_q = trajectory_generator_->getPinocchioModel()->joints[joint_id].idx_q();
    target_q_msg.name.push_back(gimbal_names.at(i));
    target_q_msg.position.push_back(curr_target_gimbal_angle(gimbal_index_map.at(gimbal_names.at(i))));
  }
  target_q_pub_.publish(target_q_msg);

  // for debug: rotor wrench
  geometry_msgs::WrenchStamped rotor_wrench_msg;
  rotor_wrench_msg.header.stamp = ros::Time::now();
  rotor_wrench_msg.header.frame_id = robot_ns_ + "/thrust" + std::to_string(rotor_wrench_pub_index_ + 1);
  rotor_wrench_msg.wrench.force.x = 0.0;
  rotor_wrench_msg.wrench.force.y = 0.0;
  rotor_wrench_msg.wrench.force.z = trajectory_generator_->getCurrentTargetThrust()(rotor_wrench_pub_index_);
  rotor_wrench_msg.wrench.torque.x = 0.0;
  rotor_wrench_msg.wrench.torque.y = 0.0;
  rotor_wrench_msg.wrench.torque.z =
      trajectory_generator_->getPinocchioRobotModel()->getRotorDirection(rotor_wrench_pub_index_) *
      trajectory_generator_->getPinocchioRobotModel()->getMFRate() *
      trajectory_generator_->getCurrentTargetThrust()(rotor_wrench_pub_index_);
  rotor_wrench_pub_.publish(rotor_wrench_msg);
  rotor_wrench_pub_index_ =
      (rotor_wrench_pub_index_ + 1) % (trajectory_generator_->getPinocchioRobotModel()->getRotorNum());

  // publish id solve time if not in nonlinear mode
  if (!trajectory_generator_->nonlinear_mode_)
  {
    std_msgs::Float32 id_solve_time_msg;
    id_solve_time_msg.data = trajectory_generator_->getPinocchioRobotModel()->getLatestIdSolveTime();
    id_solve_time_pub_.publish(id_solve_time_msg);
  }
}

void TrajectoryGeneratorRos::publishDummyJointState()
{
  // for debug: publish joint position and effort
  sensor_msgs::JointState joint_state_msg;
  joint_state_msg.header.stamp = ros::Time::now();
  Eigen::VectorXd curr_target_q = trajectory_generator_->getJointTrajectoryGenerator()->getCurrentTargetQ();
  Eigen::VectorXd curr_target_tau = trajectory_generator_->getCurrentTargetTau();

  // overwrite gimbal angles with current target gimbal angles
  Eigen::VectorXd curr_target_gimbal_angle = trajectory_generator_->getCurrentTargetGimbalAngle();
  std::vector<std::string> gimbal_names = trajectory_generator_->getGimbalNames();
  std::map<std::string, int> gimbal_index_map = trajectory_generator_->getGimbalIndexMap();
  for (int i = 0; i < gimbal_names.size(); i++)
  {
    int gimbal_index_q = trajectory_generator_->getPinocchioModel()
                             ->joints[trajectory_generator_->getPinocchioModel()->getJointId(gimbal_names.at(i))]
                             .idx_q();
    curr_target_q(gimbal_index_q) = curr_target_gimbal_angle(gimbal_index_map.at(gimbal_names.at(i)));
  }

  // publish
  for (int i = ((trajectory_generator_->getPinocchioRobotModel()->getIsFloatingBase()) ? 2 : 1);
       i < trajectory_generator_->getPinocchioModel()->njoints; i++)
  {
    int joint_id =
        trajectory_generator_->getPinocchioModel()->getJointId(trajectory_generator_->getPinocchioModel()->names[i]);
    int joint_index_q = trajectory_generator_->getPinocchioModel()->joints[joint_id].idx_q();
    int joint_index_v = trajectory_generator_->getPinocchioModel()->joints[joint_id].idx_v();

    joint_state_msg.name.push_back(trajectory_generator_->getPinocchioModel()->names[i]);
    joint_state_msg.position.push_back(curr_target_q(joint_index_q));
    joint_state_msg.effort.push_back(curr_target_tau(joint_index_v));
  }
  dummy_joint_state_pub_.publish(joint_state_msg);
}

void TrajectoryGeneratorRos::publishAll()
{
  joint_trajectory_generator_ros_->publish();

  // for debug: ID solve time and iteration
  if (trajectory_generator_->nonlinear_mode_)
  {
    nonlinear_inverse_dynamics_solver_ros_->publish();
  }

  publish();
  publishDummyJointState();

  // for debug: target joint torque and tau by thrust
  sensor_msgs::JointState target_joint_torque_msg;
  sensor_msgs::JointState tau_by_thrust_msg;
  target_joint_torque_msg.header.stamp = ros::Time::now();
  tau_by_thrust_msg.header.stamp = ros::Time::now();
  Eigen::MatrixXd tauext_partial_thrust =
      trajectory_generator_->getPinocchioRobotModel()->computeTauExtByThrustDerivative(
          trajectory_generator_->getJointTrajectoryGenerator()->getCurrentQ());
  Eigen::VectorXd tauext = tauext_partial_thrust * trajectory_generator_->getCurrentTargetThrust();
  for (int i = 0; i < trajectory_generator_->getPinocchioModel()->njoints; i++)
  {
    int joint_id =
        trajectory_generator_->getPinocchioModel()->getJointId(trajectory_generator_->getPinocchioModel()->names[i]);
    int joint_index_q = trajectory_generator_->getPinocchioModel()->joints[joint_id].idx_q();
    int joint_index_v = trajectory_generator_->getPinocchioModel()->joints[joint_id].idx_v();
    if (joint_index_q < 0 || joint_index_v < 0)
      continue;  // skip if joint index is invalid

    target_joint_torque_msg.name.push_back(trajectory_generator_->getPinocchioModel()->names[i]);
    target_joint_torque_msg.effort.push_back(trajectory_generator_->getCurrentTargetTau()(joint_index_v));

    tau_by_thrust_msg.name.push_back(trajectory_generator_->getPinocchioModel()->names[i]);
    tau_by_thrust_msg.effort.push_back(tauext(joint_index_v));
  }
  target_joint_torque_pub_.publish(target_joint_torque_msg);
  tau_by_thrust_pub_.publish(tau_by_thrust_msg);

  // for debug: thrust
  std_msgs::Float32MultiArray thrust_msg;
  for (int i = 0; i < trajectory_generator_->getPinocchioRobotModel()->getRotorNum(); i++)
  {
    thrust_msg.data.push_back(trajectory_generator_->getCurrentTargetThrust()(i));
  }
  thrust_pub_.publish(thrust_msg);
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

  std::shared_ptr<aerial_robot_control::jointTrajectoryGeneratorRos> joint_trajectory_generator_ros =
      std::make_shared<aerial_robot_control::jointTrajectoryGeneratorRos>(nh, pinocchio_robot_model);
  std::shared_ptr<aerial_robot_model::NonlinearInverseDynamicsRos> nonlinear_inverse_dynamics_solver_ros =
      std::make_shared<aerial_robot_model::NonlinearInverseDynamicsRos>(nh, pinocchio_robot_model);

  std::shared_ptr<aerial_robot_control::TrajectoryGeneratorRos> trajectory_generator_ros =
      std::make_shared<aerial_robot_control::TrajectoryGeneratorRos>(
          nh, pinocchio_robot_model, joint_trajectory_generator_ros, nonlinear_inverse_dynamics_solver_ros);

  ros::Timer timer = nh.createTimer(ros::Duration(0.01),
                                    boost::bind(&TrajectoryGeneratorRos::timer, trajectory_generator_ros.get(), _1));

  ros::spin();
  return 0;
}
