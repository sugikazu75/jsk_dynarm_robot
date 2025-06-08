#include <dynarm/model/manipulator_model.h>

using namespace aerial_robot_model;

ManipulatorRobotModel::ManipulatorRobotModel(bool init_with_rosparam, bool verbose)
  : aerial_robot_model::transformable::RobotModel(init_with_rosparam, verbose)
{
  pinocchio_robot_model_ = std::make_shared<aerial_robot_dynamics::PinocchioRobotModel>(false);
  pinocchio_model_ = pinocchio_robot_model_->getModel();
  pinocchio_data_ = pinocchio_robot_model_->getData();

  curr_q_.resize(pinocchio_robot_model_->getModel()->nq);
}

void ManipulatorRobotModel::updateRobotModelImpl(const KDL::JntArray& joint_positions)
{
  aerial_robot_model::RobotModel::updateRobotModelImpl(joint_positions);

  Eigen::VectorXd curr_q = parseJointState(joint_positions);

  pinocchio::framesForwardKinematics(*pinocchio_model_, *pinocchio_data_, curr_q_);
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

Eigen::VectorXd ManipulatorRobotModel::getGimbalNominalAngles(Eigen::VectorXd q)
{
  pinocchio::framesForwardKinematics(*pinocchio_model_, *pinocchio_data_, q);

  Eigen::VectorXd gimbal_processed_q = q;

  for (int i = 0; i < getRotorNum(); i++)
  {
    std::string link_i_name = "link" + std::to_string(i + 1);
    pinocchio::FrameIndex link_i_frame_id = pinocchio_model_->getFrameId(link_i_name);
    pinocchio::SE3 link_i_frame = pinocchio_data_->oMf[link_i_frame_id];
    Eigen::Vector3d rpy = pinocchio::rpy::matrixToRpy(link_i_frame.rotation());

    int gimbal_roll_index =
        pinocchio_model_->joints[pinocchio_model_->getJointId("gimbal" + std::to_string(i + 1) + "_roll")].idx_q();
    int gimbal_pitch_index =
        pinocchio_model_->joints[pinocchio_model_->getJointId("gimbal" + std::to_string(i + 1) + "_pitch")].idx_q();

    gimbal_processed_q(gimbal_roll_index) = -rpy(0);   // roll
    gimbal_processed_q(gimbal_pitch_index) = -rpy(1);  // pitch
  }
  return gimbal_processed_q;
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_model::ManipulatorRobotModel, aerial_robot_model::RobotModel);
