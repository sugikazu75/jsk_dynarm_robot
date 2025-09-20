#pragma once

#include <aerial_robot_model/model/transformable_aerial_robot_model.h>

class FullVectoringRobotModel : public aerial_robot_model::transformable::RobotModel
{
public:
  FullVectoringRobotModel(bool init_with_rosparam = true, bool verbose = false);
  ~FullVectoringRobotModel()
  {
  }

  template <class T>
  std::vector<T> getLinksRotationFromCog();

protected:
  void updateRobotModelImpl(const KDL::JntArray& joint_positions) override;

private:
  std::vector<KDL::Rotation> links_rotation_from_cog_;
  std::mutex links_rotation_mutex_;
};

template <>
inline std::vector<KDL::Rotation> FullVectoringRobotModel::getLinksRotationFromCog()
{
  std::lock_guard<std::mutex> lock(links_rotation_mutex_);
  return links_rotation_from_cog_;
}

template <>
inline std::vector<Eigen::Matrix3d> FullVectoringRobotModel::getLinksRotationFromCog()
{
  return aerial_robot_model::kdlToEigen(getLinksRotationFromCog<KDL::Rotation>());
}
