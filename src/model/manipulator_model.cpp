#include <dynarm/model/manipulator_model.h>

#include <aerial_robot_dynamics/robot_model_ros.h>

using namespace aerial_robot_model;

ManipulatorRobotModel::ManipulatorRobotModel(bool init_with_rosparam, bool verbose)
  : aerial_robot_model::transformable::RobotModel(init_with_rosparam, verbose)
{
  // PinocchioRobotModel no longer reads the parameter server by itself.
  // PinocchioRobotModelRos fetches robot_description, pinocchio_robot_description and
  // dynamics/{is_floating_base,thrust_hessian_weight}, and the returned model outlives it.
  ros::NodeHandle nh;
  aerial_robot_dynamics::PinocchioRobotModelRos pinocchio_robot_model_ros(nh);
  pinocchio_robot_model_ = pinocchio_robot_model_ros.getPinocchioRobotModel();
  pinocchio_model_ = pinocchio_robot_model_->getModel();
  pinocchio_data_ = pinocchio_robot_model_->getData();

  curr_q_.resize(pinocchio_robot_model_->getModel()->nq);
}

void ManipulatorRobotModel::updateRobotModelImpl(const KDL::JntArray& joint_positions)
{
  aerial_robot_model::RobotModel::updateRobotModelImpl(joint_positions);

  Eigen::VectorXd curr_q = parseJointState(joint_positions);
}

Eigen::VectorXd ManipulatorRobotModel::parseJointState(const sensor_msgs::JointState& joint_state)
{
  // Update the current joint positions in the robot model
  for (int i = 0; i < joint_state.name.size(); i++)
  {
    int index = pinocchio_model_->joints[pinocchio_model_->getJointId(joint_state.name.at(i))].idx_q();
    curr_q_(index) = joint_state.position.at(i);
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
