#include <dynarm/navigation/full_vectoring_navigation.h>

using namespace aerial_robot_navigation;

FullVectoringNavigator::FullVectoringNavigator() : BaseNavigator()
{
}

void FullVectoringNavigator::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                        boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                        boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                        double loop_du)
{
  /* initialize the flight control */
  BaseNavigator::initialize(nh, nhp, robot_model, estimator, loop_du);

  rosParamInit();

  desire_coordinate_sub_ =
      nh_.subscribe("desire_coordinate", 1, &FullVectoringNavigator::desireCoordinatedCallback, this);
}

void FullVectoringNavigator::desireCoordinateCallback(const spinal::DesireCoordConstPtr& msg)
{
  KDL::Rotation rot;
  rot = KDL::Rotation::RPY(msg->roll, msg->pitch, 0.0);
  robot_model_->setCogDesireOrientation(rot);
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_navigation::FullVectoringNavigator, aerial_robot_navigation::BaseNavigator);
