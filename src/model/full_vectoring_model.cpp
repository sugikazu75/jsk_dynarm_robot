#include <dynarm/model/full_vectoring_model.h>

FullVectoringRobotModel::FullVectoringRobotModel(bool init_with_rosparam, bool verbose)
  : aerial_robot_model::transformable::RobotModel(init_with_rosparam, verbose)
{
  links_rotation_from_cog_.resize(getRotorNum());
}

void FullVectoringRobotModel::updateRobotModelImpl(const KDL::JntArray& joint_positions)
{
  aerial_robot_model::transformable::RobotModel::updateRobotModelImpl(joint_positions);

  const auto& seg_tf_map = getSegmentsTf();

  KDL::Frame cog = getCog<KDL::Frame>();

  /* link and rotor information in each link */
  for (int i = 0; i < getRotorNum(); ++i)
  {
    std::string s = std::to_string(i + 1);

    KDL::Frame link_f = seg_tf_map.at("link" + s);
    links_rotation_from_cog_.at(i) = (cog.Inverse() * link_f).M;
  }
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(FullVectoringRobotModel, aerial_robot_model::RobotModel);
