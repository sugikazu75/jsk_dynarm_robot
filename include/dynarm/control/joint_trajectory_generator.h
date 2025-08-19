#pragma once

#include <pinocchio/fwd.hpp>
#include <dynarm/model/nonlinear_inverse_dynamics.h>
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

  void update();
  void publish();
  void publishDummyJointState();
  Eigen::VectorXd getGimbalNominalAngles(Eigen::VectorXd q);
  void generateEndEffectorTrajectory();
  void generateJointTrajectory();
  void stateTransition();
  void reset();

  void setCurrentQ(const Eigen::VectorXd curr_q)
  {
    curr_q_ = curr_q;
  }
  void setCurrentDQ(const Eigen::VectorXd curr_dq)
  {
    curr_dq_ = curr_dq;
  }
  Eigen::VectorXd getCurrentQ()
  {
    return curr_q_;
  }
  Eigen::VectorXd getCurrentDQ()
  {
    return curr_dq_;
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
  void setFinalTargetQ(const Eigen::VectorXd final_target_q)
  {
    final_target_q_ = final_target_q;
  }
  Eigen::VectorXd getResetTargetQ()
  {
    return reset_target_q_;
  }
  void setTransformDuration(double duration)
  {
    transform_duration_ = duration;
  }

private:
  std::shared_ptr<aerial_robot_dynamics::PinocchioRobotModel> pinocchio_robot_model_;
  std::shared_ptr<pinocchio::Model> pinocchio_model_;
  std::shared_ptr<pinocchio::Data> pinocchio_data_;

  ros::NodeHandle nh_;
  ros::Publisher is_transforming_pub_;
  ros::Publisher target_q_pub_;
  ros::Publisher target_dq_pub_;
  ros::Publisher target_ddq_pub_;
  ros::Publisher target_end_effector_pos_pub_;
  ros::Publisher target_end_effector_vel_pub_;
  ros::Publisher target_end_effector_acc_pub_;
  ros::Publisher dummy_joint_state_pub_;
  ros::Subscriber joint_state_sub_;
  ros::Subscriber target_end_effector_final_pos_sub_;
  ros::Subscriber circle_trajectory_sub_;
  ros::Subscriber direct_joint_angle_sub_;

  int is_transforming_ = 0;  // 0: not transforming, 1: linear transform, 2: circle trajectory

  std::vector<std::string> joint_names_;
  std::vector<std::string> gimbal_names_;
  int rotor_devider_ = 1;

  Eigen::VectorXd curr_q_;
  Eigen::VectorXd curr_dq_;

  Eigen::VectorXd curr_target_q_;
  Eigen::VectorXd curr_target_dq_;
  Eigen::VectorXd curr_target_ddq_;

  Eigen::VectorXd final_target_q_;
  Eigen::VectorXd init_target_q_;
  Eigen::VectorXd reset_target_q_;

  Eigen::Vector3d target_ee_pos_;
  Eigen::Vector3d target_ee_vel_;
  Eigen::Vector3d target_ee_acc_;

  Eigen::MatrixXd ctm_p_gain_;
  Eigen::MatrixXd ctm_d_gain_;

  // manipulation param
  motion_planning::PositionTrajectoryGenerator pos_trajectory_generator_;
  double transform_duration_;
  double transform_start_time_;
  std::string end_effector_name_;
  double circle_trajectory_radius_;
  double circle_trajectory_angular_velocity_;
  Eigen::Vector3d circle_trajectory_center_;

  void loadJointNames();
  void loadGimbalNames();
  void rosParamInit();

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
