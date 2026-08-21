#include <dynarm/control/whole_body_force_controller.h>

using namespace aerial_robot_control;

WholeBodyForceController::WholeBodyForceController() : ControlBase()
{
}

void WholeBodyForceController::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                          boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                          boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                          boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                          double ctrl_loop_du)
{
  ControlBase::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_du);
}

bool WholeBodyForceController::update()
{
  if (!ControlBase::update())
  {
    return false;
  }

  controlCore();
  sendCmd();

  return true;
}

void WholeBodyForceController::controlCore()
{
}

void WholeBodyForceController::sendCmd()
{
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::WholeBodyForceController, aerial_robot_control::ControlBase);
