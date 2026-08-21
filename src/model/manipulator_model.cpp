#include <dynarm/model/manipulator_model.h>

#include <aerial_robot_dynamics/robot_model_ros.h>

using namespace aerial_robot_model;

ManipulatorRobotModel::ManipulatorRobotModel(bool init_with_rosparam, bool verbose)
  : aerial_robot_model::transformable::RobotModel(init_with_rosparam, verbose)
{
  ros::NodeHandle nh;
  aerial_robot_dynamics::PinocchioRobotModelRos pinocchio_robot_model_ros(nh);
  pinocchio_robot_model_ = pinocchio_robot_model_ros.getPinocchioRobotModel();
  pinocchio_model_ = pinocchio_robot_model_->getModel();
  pinocchio_data_ = pinocchio_robot_model_->getData();

  curr_q_ = Eigen::VectorXd::Zero(pinocchio_model_->nq);
  curr_dq_ = Eigen::VectorXd::Zero(pinocchio_model_->nv);
}

void ManipulatorRobotModel::updateRobotModel(const sensor_msgs::JointState& state)
{
  // hold the joint positions and velocities before the velocity is dropped by jointMsgToKdl()
  parseJointState(state);

  aerial_robot_model::RobotModel::updateRobotModel(state);
}

void ManipulatorRobotModel::updateRobotModelImpl(const KDL::JntArray& joint_positions)
{
  aerial_robot_model::RobotModel::updateRobotModelImpl(joint_positions);
}

Eigen::VectorXd ManipulatorRobotModel::parseJointState(const sensor_msgs::JointState& joint_state)
{
  // Update the current joint positions and velocities in the robot model
  for (int i = 0; i < joint_state.name.size(); i++)
  {
    int joint_id = pinocchio_model_->getJointId(joint_state.name.at(i));
    if (joint_id == pinocchio_model_->njoints)
      continue;  // skip this joint because it is not in the kinematic tree

    if (joint_state.name.size() == joint_state.position.size())
      curr_q_(pinocchio_model_->joints[joint_id].idx_q()) = joint_state.position.at(i);

    if (joint_state.name.size() == joint_state.velocity.size())
      curr_dq_(pinocchio_model_->joints[joint_id].idx_v()) = joint_state.velocity.at(i);
  }
  return curr_q_;
}

Eigen::VectorXd ManipulatorRobotModel::parseJointState(const KDL::JntArray& joint_positions)
{
  // Update the current joint positions in the robot model
  const auto joint_index_map = getJointIndexMap();

  for (const auto& actuator : joint_index_map)
  {
    int index = pinocchio_model_->joints[pinocchio_model_->getJointId(actuator.first)].idx_q();
    curr_q_(index) = joint_positions(actuator.second);
  }
  return curr_q_;
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_model::ManipulatorRobotModel, aerial_robot_model::RobotModel);
