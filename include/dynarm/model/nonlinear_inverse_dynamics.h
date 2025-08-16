#pragma once

#include <pinocchio/fwd.hpp>
#include <aerial_robot_dynamics/robot_model.h>

#include <nlopt.hpp>

namespace aerial_robot_model
{
class NonlinearInverseDynamics
{
public:
  NonlinearInverseDynamics(ros::NodeHandle nh,
                           std::shared_ptr<aerial_robot_dynamics::PinocchioRobotModel> pinocchio_robot_model);
  virtual ~NonlinearInverseDynamics() = default;

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

  const std::vector<std::string>& getJointNames()
  {
    return joint_names_;
  }
  const std::vector<std::string>& getGimbalNames()
  {
    return gimbal_names_;
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

  const double getSolveTime()
  {
    return solve_time_;
  }

  void reset();
  bool solve(const Eigen::VectorXd& q, const Eigen::VectorXd& v, const Eigen::VectorXd& a,
             Eigen::VectorXd& tau_thrust_gimbal);

private:
  ros::NodeHandle nh_;
  std::shared_ptr<aerial_robot_dynamics::PinocchioRobotModel> pinocchio_robot_model_;
  std::shared_ptr<pinocchio::Model> pinocchio_model_;
  std::shared_ptr<pinocchio::Data> pinocchio_data_;
  std::vector<std::string> joint_names_;
  std::vector<std::string> gimbal_names_;

  nlopt::opt nlp_solver_;
  std::vector<double> nlp_lb_;
  std::vector<double> nlp_ub_;
  int nlp_n_variables_;
  int nlp_n_constraints_;

  Eigen::VectorXd nlp_last_solution_;
  Eigen::VectorXd nlp_curr_target_q_;
  Eigen::VectorXd nlp_curr_target_dq_;
  Eigen::VectorXd nlp_curr_target_ddq_;
  double gimbal_delta_max_;
  double solve_time_ = 0.0;

  void rosParamInit();
  void loadJointNames();
  void loadGimbalNames();

  template <class T>
  void getParam(ros::NodeHandle nh, std::string param_name, T& param, T default_value)
  {
    nh.param<T>(param_name, param, default_value);
  }
};
}  // namespace aerial_robot_model
