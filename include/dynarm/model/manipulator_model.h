#pragma once

#include <aerial_robot_dynamics/robot_model.h>
#include <pinocchio/math/rpy.hpp>
#include <aerial_robot_model/model/transformable_aerial_robot_model.h>
#include <memory>

namespace aerial_robot_model
{
class ManipulatorRobotModel : public aerial_robot_model::transformable::RobotModel
{
public:
  ManipulatorRobotModel(bool init_with_rosparam = true, bool verbose = false);
  virtual ~ManipulatorRobotModel() = default;

  std::shared_ptr<aerial_robot_dynamics::PinocchioRobotModel> getPinocchioRobotModel()
  {
    return pinocchio_robot_model_;
  }

  Eigen::VectorXd getCurrentJointPositions()
  {
    return curr_q_;
  }

  const std::vector<std::string>& getJointNames()
  {
    return joint_names_;
  }

  const std::vector<std::string>& getGimbalNames()
  {
    return gimbal_names_;
  }

private:
  std::shared_ptr<aerial_robot_dynamics::PinocchioRobotModel> pinocchio_robot_model_;
  std::shared_ptr<pinocchio::Model> pinocchio_model_;
  std::shared_ptr<pinocchio::Data> pinocchio_data_;
  Eigen::VectorXd curr_q_;
  std::vector<std::string> joint_names_;
  std::vector<std::string> gimbal_names_;
  bool is_floating_base_ = false;

  void updateRobotModelImpl(const KDL::JntArray& joint_positions) override;
  Eigen::VectorXd parseJointState(const sensor_msgs::JointState& joint_state);
  Eigen::VectorXd parseJointState(const KDL::JntArray& joint_positions);

  void getParamFromRos();
  void loadJointNames();
  void loadGimbalNames();
};
}  // namespace aerial_robot_model
