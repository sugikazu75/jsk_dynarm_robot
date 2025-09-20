#pragma once

#include <aerial_robot_control/control/base/pose_linear_controller.h>
#include <dynarm/model/full_vectoring_model.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/JointState.h>
#include <spinal/FourAxisCommand.h>

namespace aerial_robot_control
{
class FullVectoringPIDController : public PoseLinearController
{
public:
  FullVectoringPIDController();
  ~FullVectoringPIDController()
  {
  }

  void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                  boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                  boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                  boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator, double ctrl_loop_rate) override;

private:
  ros::Publisher flight_cmd_pub_;      // for spinal
  ros::Publisher gimbal_control_pub_;  // for servo bridge
  ros::Publisher rotor_wrench_pub_;    // for debug
  std::vector<float> target_base_thrust_;
  std::vector<double> target_gimbal_angles_;
  boost::shared_ptr<FullVectoringRobotModel> full_vectoring_robot_model_;
  void controlCore() override;
  void rosParamInit();
  void sendCmd();

  std::string robot_ns_;
  int rotor_wrench_pub_index_;
};
}  // namespace aerial_robot_control
