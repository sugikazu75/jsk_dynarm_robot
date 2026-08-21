#pragma once

#include <aerial_robot_dynamics/robot_model.h>
#include <pinocchio/math/rpy.hpp>
#include <aerial_robot_model/model/transformable_aerial_robot_model.h>
#include <sensor_msgs/JointState.h>
#include <memory>

namespace aerial_robot_model
{
class ManipulatorRobotModel : public aerial_robot_model::transformable::RobotModel
{
public:
  ManipulatorRobotModel(bool init_with_rosparam = true, bool verbose = false);
  virtual ~ManipulatorRobotModel() = default;

  using aerial_robot_model::transformable::RobotModel::updateRobotModel;

  void updateRobotModel(const sensor_msgs::JointState& state) override;

  std::shared_ptr<aerial_robot_dynamics::PinocchioRobotModel> getPinocchioRobotModel()
  {
    return pinocchio_robot_model_;
  }

  Eigen::VectorXd getCurrentJointPositions()
  {
    return curr_q_;
  }

  Eigen::VectorXd getCurrentJointVelocities()
  {
    return curr_dq_;
  }

private:
  std::shared_ptr<aerial_robot_dynamics::PinocchioRobotModel> pinocchio_robot_model_;
  std::shared_ptr<pinocchio::Model> pinocchio_model_;
  std::shared_ptr<pinocchio::Data> pinocchio_data_;
  Eigen::VectorXd curr_q_;   // nq
  Eigen::VectorXd curr_dq_;  // nv

  void updateRobotModelImpl(const KDL::JntArray& joint_positions) override;
  Eigen::VectorXd parseJointState(const sensor_msgs::JointState& joint_state);
  Eigen::VectorXd parseJointState(const KDL::JntArray& joint_positions);
};
}  // namespace aerial_robot_model
