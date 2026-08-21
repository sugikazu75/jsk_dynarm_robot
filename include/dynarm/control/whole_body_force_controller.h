#pragma once

#include <aerial_robot_control/control/base/base.h>

#include <ros/ros.h>
#include <Eigen/Core>

namespace aerial_robot_control
{
class WholeBodyForceController : public ControlBase
{
public:
  WholeBodyForceController();
  virtual ~WholeBodyForceController() = default;

  void virtual initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                          boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                          boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                          boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator, double ctrl_loop_du);

private:
  virtual bool update() override;
  void controlCore();
  void sendCmd();
};
}  // namespace aerial_robot_control
