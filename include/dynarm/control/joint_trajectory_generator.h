#pragma once

#include <pinocchio/fwd.hpp>
#include <dynarm/model/nonlinear_inverse_dynamics.h>
#include <motion_planning/position_trajectory_generator.hpp>
#include <motion_planning/inverse_kinematics_3d.hpp>
#include <ros/ros.h>
#include <Eigen/Core>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt8.h>

namespace aerial_robot_control
{
class JointTrajectoryGenerator
{
public:
  JointTrajectoryGenerator(std::shared_ptr<aerial_robot_dynamics::PinocchioRobotModel> pinocchio_robot_model);
  virtual ~JointTrajectoryGenerator() = default;

  void update();
  Eigen::VectorXd getGimbalNominalAngles(Eigen::VectorXd q);
  void generateEndEffectorTrajectory();
  void generateJointTrajectory();
  void stateTransition();
  void reset();

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
  void setCurrentQ(const Eigen::VectorXd curr_q)
  {
    curr_q_ = curr_q;
  }
  Eigen::VectorXd getCurrentQ()
  {
    return curr_q_;
  }
  void setCurrentDQ(const Eigen::VectorXd curr_dq)
  {
    curr_dq_ = curr_dq;
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

  void setResetTargetQ(const Eigen::VectorXd reset_target_q)
  {
    reset_target_q_ = reset_target_q;
  }

  Eigen::VectorXd getResetTargetQ()
  {
    return reset_target_q_;
  }

  void setTargetEndEffectorPos(const Eigen::Vector3d target_ee_pos)
  {
    target_ee_pos_ = target_ee_pos;
  }

  Eigen::Vector3d getTargetEndEffectorPos()
  {
    return target_ee_pos_;
  }

  Eigen::Vector3d getTargetEndEffectorVel()
  {
    return target_ee_vel_;
  }

  Eigen::Vector3d getTargetEndEffectorAcc()
  {
    return target_ee_acc_;
  }

  std::vector<std::string>& getJointNames()
  {
    return joint_names_;
  }

  std::vector<std::string>& getGimbalNames()
  {
    return gimbal_names_;
  }

  void setTransformDuration(double duration)
  {
    transform_duration_ = duration;
  }

  motion_planning::PositionTrajectoryGenerator pos_trajectory_generator_;

  double ctrl_loop_du_;    // for interpolation in joy control mode. set from ros param
  int rotor_devider_ = 1;  // number of rotors in one gimbal. set from ros param
  int is_transforming_ =
      0;  // 0: not transforming, 1: linear transform, 2: circle trajectory, 3: direct joint command, 4: joy control

  // manipulation parameters
  std::string end_effector_name_;  // end effector name. set from ros param
  double transform_duration_;
  double transform_start_time_;

  // circle trajectory parameters
  double circle_trajectory_radius_;
  double circle_trajectory_angular_velocity_;
  Eigen::Vector3d circle_trajectory_center_;

  bool quasi_static_mode_;
  Eigen::MatrixXd ctm_p_gain_;
  Eigen::MatrixXd ctm_d_gain_;

  Eigen::VectorXd final_target_q_;
  Eigen::VectorXd init_target_q_;
  Eigen::VectorXd reset_target_q_;
  Eigen::VectorXd curr_q_;
  Eigen::VectorXd curr_dq_;

  // joy control
  sensor_msgs::Joy joy_msg_;

private:
  std::shared_ptr<aerial_robot_dynamics::PinocchioRobotModel> pinocchio_robot_model_;
  std::shared_ptr<pinocchio::Model> pinocchio_model_;
  std::shared_ptr<pinocchio::Data> pinocchio_data_;

  std::vector<std::string> joint_names_;
  std::vector<std::string> gimbal_names_;

  Eigen::VectorXd curr_target_q_;
  Eigen::VectorXd curr_target_dq_;
  Eigen::VectorXd curr_target_ddq_;

  Eigen::Vector3d target_ee_pos_;
  Eigen::Vector3d target_ee_vel_;
  Eigen::Vector3d target_ee_acc_;

  void loadJointNames();
  void loadGimbalNames();
};

class JointTrajectoryGeneratorRos
{
public:
  JointTrajectoryGeneratorRos(ros::NodeHandle nh,
                              std::shared_ptr<aerial_robot_dynamics::PinocchioRobotModel> pinocchio_robot_model);
  virtual ~JointTrajectoryGeneratorRos() = default;

  std::shared_ptr<JointTrajectoryGenerator> getJointTrajectoryGenerator()
  {
    return joint_trajectory_generator_;
  }

  void publish();

private:
  ros::NodeHandle nh_;
  ros::Publisher is_transforming_pub_;                 // for debug
  ros::Publisher target_q_pub_;                        // for debug. only joint part
  ros::Publisher target_dq_pub_;                       // for debug. only joint part
  ros::Publisher target_ddq_pub_;                      // for debug. only joint part
  ros::Publisher target_end_effector_pos_pub_;         // for debug
  ros::Publisher target_end_effector_vel_pub_;         // for debug
  ros::Publisher target_end_effector_acc_pub_;         // for debug
  ros::Subscriber joint_state_sub_;                    // to get current joint states. TODO: integrate with robot_model
  ros::Subscriber target_end_effector_final_pos_sub_;  // to receive target end effector position
  ros::Subscriber circle_trajectory_sub_;              // to receive circle trajectory command
  ros::Subscriber direct_joint_angle_sub_;             // to receive direct joint angle command
  ros::Subscriber joy_sub_;                            // to receive joy command

  std::shared_ptr<JointTrajectoryGenerator> joint_trajectory_generator_;

  void rosParamInit();

  void jointStateCallback(const sensor_msgs::JointState msg);
  void targetEndEffectorPosCallback(const geometry_msgs::Vector3StampedConstPtr& msg);
  void circleTrajectoryCallback(const std_msgs::Float32MultiArrayConstPtr& msg);
  void directJointAngleCallback(const sensor_msgs::JointStateConstPtr& msg);
  void joyCallback(const sensor_msgs::JoyConstPtr& msg);

  template <class T>
  void getParam(ros::NodeHandle nh, std::string param_name, T& param, T default_value)
  {
    nh.param<T>(param_name, param, default_value);
  }
};

}  // namespace aerial_robot_control
