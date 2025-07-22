#pragma once

#include <dynarm/model/manipulator_model.h>
#include <motion_planning/position_trajectory_generator.hpp>
#include <motion_planning/inverse_kinematics_3d.hpp>
#include <ros/ros.h>
#include <Eigen/Core>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt8.h>

namespace aerial_robot_control
{
class jointTrajectoryGenerator
{
public:
  jointTrajectoryGenerator(ros::NodeHandle nh,
                           std::shared_ptr<aerial_robot_dynamics::PinocchioRobotModel> pinocchio_robot_model);
  virtual ~jointTrajectoryGenerator() = default;

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
  const int getRotorDevider()
  {
    return rotor_devider_;
  }
  const int getIsTransforming()
  {
    return is_transforming_;
  }

  void update(const ros::TimerEvent& event);
  void publish();
  void publishDummyJointState();
  void generateEndEffectorTrajectory();
  void generateJointTrajectory();
  Eigen::VectorXd getGimbalNominalAngles(Eigen::VectorXd q);
  void getGimbalNominalAngles();
  bool solveInverseDynamics();
  void updateTargetGimbalAngle();
  void stateTransition();
  bool nonlinearInverseDynamics(const Eigen::VectorXd& q, const Eigen::VectorXd& v, const Eigen::VectorXd& a,
                                Eigen::VectorXd& tau_thrust_gimbal);
  void rosParamInit();
  void reset();

  const bool getNonlinearMode()
  {
    return nonlinear_mode_;
  }
  void setCurrentTargetQ(const Eigen::VectorXd curr_target_q)
  {
    curr_target_q_ = curr_target_q;
  }
  Eigen::VectorXd getCurrentTargetQ()
  {
    return curr_target_q_;
  }
  void setCurrentTargetDQ(const Eigen::VectorXd curr_target_dq)
  {
    curr_target_dq_ = curr_target_dq;
  }
  Eigen::VectorXd getCurrentTargetDQ()
  {
    return curr_target_dq_;
  }
  void setCurrentTargetDDQ(const Eigen::VectorXd curr_target_ddq)
  {
    curr_target_ddq_ = curr_target_ddq;
  }
  Eigen::VectorXd getCurrentTargetDDQ()
  {
    return curr_target_ddq_;
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

  const int getGimbalNumForOpt()
  {
    return gimbal_num_;
  }
  const Eigen::VectorXd getCurrentTargetQForOpt()
  {
    return nlp_curr_target_q_;
  }
  const Eigen::VectorXd getCurrentTargetDqForOpt()
  {
    return nlp_curr_target_dq_;
  }
  const Eigen::VectorXd getCurrentTargetDdqForOpt()
  {
    return nlp_curr_target_ddq_;
  }

private:
  std::shared_ptr<aerial_robot_dynamics::PinocchioRobotModel> pinocchio_robot_model_;
  std::shared_ptr<pinocchio::Model> pinocchio_model_;
  std::shared_ptr<pinocchio::Data> pinocchio_data_;

  ros::NodeHandle nh_;
  ros::Publisher id_torque_pub_;
  ros::Publisher id_velocity_pub_;
  ros::Publisher id_acc_pub_;
  ros::Publisher is_transforming_pub_;
  ros::Publisher id_time_pub_;
  ros::Publisher rotor_wrench_pub_;
  ros::Publisher target_end_effector_pos_pub_;
  ros::Publisher target_end_effector_vel_pub_;
  ros::Publisher dummy_joint_state_pub_;

  ros::Subscriber joint_state_sub_;
  ros::Subscriber target_end_effector_final_pos_sub_;
  ros::Subscriber circle_trajectory_sub_;
  ros::Subscriber direct_joint_angle_sub_;

  // debug
  std::string robot_ns_;
  int rotor_wrench_pub_index_;

  int is_transforming_ = 0;  // 0: not transforming, 1: linear transform, 2: circle trajectory

  Eigen::Vector3d target_ee_pos_;
  Eigen::Vector3d target_ee_vel_;
  Eigen::Vector3d target_ee_acc_;

  Eigen::VectorXd curr_q_;

  Eigen::VectorXd curr_target_q_;
  Eigen::VectorXd curr_target_dq_;
  Eigen::VectorXd curr_target_ddq_;

  Eigen::VectorXd curr_target_tau_;
  Eigen::VectorXd curr_target_thrust_;
  Eigen::VectorXd curr_target_gimbal_angle_;

  Eigen::VectorXd final_target_q_;
  Eigen::VectorXd init_target_q_;

  // manipulation param
  motion_planning::PositionTrajectoryGenerator pos_trajectory_generator_;
  double transform_duration_;
  double transform_start_time_;
  double transform_end_time_;
  std::string end_effector_name_;
  double circle_trajectory_radius_;
  double circle_trajectory_angular_velocity_;
  Eigen::Vector3d circle_trajectory_center_;

  // nlp param
  bool nonlinear_mode_;
  int gimbal_num_;
  int rotor_devider_ = 1;
  std::vector<int> gimbal_q_indices_;
  std::vector<int> gimbal_v_indices_;
  bool nlp_first_run_ = true;
  double nlp_solve_time_ = 0.0;
  double gimbal_delta_max_;
  Eigen::VectorXd nlp_curr_target_q_;
  Eigen::VectorXd nlp_curr_target_dq_;
  Eigen::VectorXd nlp_curr_target_ddq_;

  void jointStateCallback(const sensor_msgs::JointState msg);
  void targetEndEffectorPosCallback(const geometry_msgs::Vector3StampedConstPtr& msg);
  void circleTrajectoryCallback(const std_msgs::Float32MultiArrayConstPtr& msg);
  void directJointAngleCallback(const sensor_msgs::JointStateConstPtr& msg);

  template <class T>
  void getParam(ros::NodeHandle nh, std::string param_name, T& param, T default_value)
  {
    nh.param<T>(param_name, param, default_value);
  }
};
}  // namespace aerial_robot_control
