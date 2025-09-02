#pragma once

#include <pinocchio/fwd.hpp>
#include <pinocchio/math/rpy.hpp>
#include <aerial_robot_dynamics/robot_model.h>
#include <ros/ros.h>
#include <Eigen/Core>

#include <dynarm/control/joint_trajectory_generator.h>
#include <dynarm/model/nonlinear_inverse_dynamics.h>

#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/JointState.h>

namespace aerial_robot_control
{
class TrajectoryGenerator
{
public:
  TrajectoryGenerator(ros::NodeHandle nh,
                      std::shared_ptr<aerial_robot_dynamics::PinocchioRobotModel> pinocchio_robot_model);
  ~TrajectoryGenerator() = default;

  std::shared_ptr<aerial_robot_control::jointTrajectoryGenerator> getJointTrajectoryGenerator()
  {
    return joint_trajectory_generator_;
  }

  void reset();
  void timer(ros::TimerEvent const& event);
  void update();
  bool solveInverseDynamics(Eigen::VectorXd q, Eigen::VectorXd dq, Eigen::VectorXd ddq);
  void publish();
  void publishDummyJointState();

  std::vector<std::string> getGimbalNames()
  {
    return gimbal_names_;
  }

  std::map<std::string, int> getGimbalIndexMap()
  {
    return gimbal_index_map_;
  }

  Eigen::VectorXd getCurrentTargetTau()
  {
    return curr_target_tau_;
  }
  Eigen::VectorXd getCurrentTargetThrust()
  {
    return curr_target_thrust_;
  }
  Eigen::VectorXd getCurrentTargetGimbalAngle()
  {
    return curr_target_gimbal_angle_;
  }

private:
  std::shared_ptr<aerial_robot_dynamics::PinocchioRobotModel> pinocchio_robot_model_;
  std::shared_ptr<pinocchio::Model> pinocchio_model_;
  std::shared_ptr<pinocchio::Data> pinocchio_data_;

  std::shared_ptr<aerial_robot_control::jointTrajectoryGenerator> joint_trajectory_generator_;
  std::shared_ptr<aerial_robot_model::NonlinearInverseDynamics> nonlinear_inverse_dynamics_solver_;

  ros::NodeHandle nh_;
  ros::Publisher target_q_pub_;             // for debug: gimbal
  ros::Publisher target_joint_torque_pub_;  // for debug
  ros::Publisher tau_by_thrust_pub_;        // for debug
  ros::Publisher rotor_wrench_pub_;         // for debug
  ros::Publisher thrust_pub_;               // for debug
  ros::Publisher id_solve_time_pub_;        // for debug
  ros::Publisher dummy_joint_state_pub_;    // for debug

  std::vector<std::string> gimbal_names_;
  std::map<std::string, int> gimbal_index_map_;

  Eigen::VectorXd curr_target_tau_;
  Eigen::VectorXd curr_target_thrust_;
  Eigen::VectorXd curr_target_gimbal_angle_;

  // debug
  std::string robot_ns_;
  int rotor_wrench_pub_index_;

  // inverse dynamics param
  bool nonlinear_mode_;
  bool quasi_static_mode_;

  void loadGimbalNames();
  void rosParamInit();

  template <class T>
  void getParam(ros::NodeHandle nh, std::string param_name, T& param, T default_value)
  {
    nh.param<T>(param_name, param, default_value);
  }
};
}  // namespace aerial_robot_control
