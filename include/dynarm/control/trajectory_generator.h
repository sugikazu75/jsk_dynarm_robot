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
  TrajectoryGenerator(std::shared_ptr<aerial_robot_dynamics::PinocchioRobotModel> pinocchio_robot_model,
                      std::shared_ptr<aerial_robot_control::jointTrajectoryGenerator> joint_trajectory_generator,
                      std::shared_ptr<aerial_robot_model::NonlinearInverseDynamics> nonlinear_inverse_dynamics_solver);
  ~TrajectoryGenerator() = default;

  std::shared_ptr<aerial_robot_dynamics::PinocchioRobotModel> getPinocchioRobotModel()
  {
    return pinocchio_robot_model_;
  }
  std::shared_ptr<pinocchio::Model> getPinocchioModel()
  {
    return pinocchio_model_;
  }
  std::shared_ptr<pinocchio::Data> getPinocchioData()
  {
    return pinocchio_data_;
  }

  std::shared_ptr<aerial_robot_control::jointTrajectoryGenerator> getJointTrajectoryGenerator()
  {
    return joint_trajectory_generator_;
  }

  std::shared_ptr<aerial_robot_model::NonlinearInverseDynamics> getNonlinearInverseDynamicsSolver()
  {
    return nonlinear_inverse_dynamics_solver_;
  }

  void reset();
  void update();
  bool solveInverseDynamics(Eigen::VectorXd q, Eigen::VectorXd dq, Eigen::VectorXd ddq);

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

  // inverse dynamics param
  bool nonlinear_mode_;
  bool quasi_static_mode_;

private:
  std::shared_ptr<aerial_robot_dynamics::PinocchioRobotModel> pinocchio_robot_model_;
  std::shared_ptr<pinocchio::Model> pinocchio_model_;
  std::shared_ptr<pinocchio::Data> pinocchio_data_;

  std::shared_ptr<aerial_robot_control::jointTrajectoryGenerator> joint_trajectory_generator_;
  std::shared_ptr<aerial_robot_model::NonlinearInverseDynamics> nonlinear_inverse_dynamics_solver_;

  std::vector<std::string> gimbal_names_;
  std::map<std::string, int> gimbal_index_map_;

  Eigen::VectorXd curr_target_tau_;
  Eigen::VectorXd curr_target_thrust_;
  Eigen::VectorXd curr_target_gimbal_angle_;

  void loadGimbalNames();
};

class TrajectoryGeneratorRos
{
public:
  TrajectoryGeneratorRos(
      ros::NodeHandle nh, std::shared_ptr<aerial_robot_dynamics::PinocchioRobotModel> pinocchio_robot_model,
      std::shared_ptr<aerial_robot_control::jointTrajectoryGeneratorRos> joint_trajectory_generator_ros,
      std::shared_ptr<aerial_robot_model::NonlinearInverseDynamicsRos> nonlinear_inverse_dynamics_solver_ros);
  virtual ~TrajectoryGeneratorRos() = default;

  std::shared_ptr<TrajectoryGenerator> getTrajectoryGenerator()
  {
    return trajectory_generator_;
  }

  void publish();
  void timer(ros::TimerEvent const& event);

private:
  ros::NodeHandle nh_;
  ros::Publisher target_q_pub_;       // for debug. only gimbal part
  ros::Publisher rotor_wrench_pub_;   // for debug. rotor wrench visualization
  ros::Publisher id_solve_time_pub_;  // for debug. linear ID solve time if necessary

  ros::Publisher tau_by_thrust_pub_;        // for debug. in desired state. published in test mode.
  ros::Publisher thrust_pub_;               // for debug. published in test mode
  ros::Publisher target_joint_torque_pub_;  // for debug. published in test mode
  ros::Publisher dummy_joint_state_pub_;    // for debug. published in test mode

  std::shared_ptr<TrajectoryGenerator> trajectory_generator_;

  std::shared_ptr<aerial_robot_control::jointTrajectoryGeneratorRos> joint_trajectory_generator_ros_;
  std::shared_ptr<aerial_robot_model::NonlinearInverseDynamicsRos> nonlinear_inverse_dynamics_solver_ros_;

  // debug
  std::string robot_ns_;
  int rotor_wrench_pub_index_;

  void publishDummyJointState();
  void publishAll();
  void rosParamInit();

  template <class T>
  void getParam(ros::NodeHandle nh, std::string param_name, T& param, T default_value)
  {
    nh.param<T>(param_name, param, default_value);
  }
};
}  // namespace aerial_robot_control
