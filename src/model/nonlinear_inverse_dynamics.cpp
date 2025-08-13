#include <dynarm/model/nonlinear_inverse_dynamics.h>

#include <nlopt.hpp>

using namespace aerial_robot_model;

namespace
{
double torqueThrustMinimize(const std::vector<double>& x, std::vector<double>& grad, void* ptr)
{
  /* variables (n)
     0 ~ nv: generalized force
     nv ~ nv + nr: thrust
     nv + nr ~ nv + nr + gimbal_num: gimbal angles
  */
  NonlinearInverseDynamics* robot_model = reinterpret_cast<NonlinearInverseDynamics*>(ptr);

  std::shared_ptr<aerial_robot_dynamics::PinocchioRobotModel> pinocchio_robot_model =
      robot_model->getPinocchioRobotModel();
  std::shared_ptr<pinocchio::Model> pinocchio_model = robot_model->getPinocchioModel();
  std::shared_ptr<pinocchio::Data> pinocchio_data = robot_model->getPinocchioData();

  int gimbal_num = robot_model->getGimbalNames().size();
  double thrust_hessian_weight = pinocchio_robot_model->getThrustHessianWeight();

  double cost = 0.0;

  // torque
  for (int i = 0; i < pinocchio_model->nv; i++)
    cost += x[i] * x[i];  // minimize joint torque

  // thrust
  for (int i = 0; i < pinocchio_robot_model->getRotorNum(); i++)
    cost += thrust_hessian_weight * x[pinocchio_model->nv + i] * x[pinocchio_model->nv + i];  // minimize thrust

  if (grad.empty())
    return cost;

  // gradient
  // torque
  for (int i = 0; i < pinocchio_model->nv; i++)
    grad[i] = 2.0 * x[i];

  // thrust
  for (int i = 0; i < pinocchio_robot_model->getRotorNum(); i++)
    grad[pinocchio_model->nv + i] = 2.0 * thrust_hessian_weight * x[pinocchio_model->nv + i];

  // gimbal angles
  for (int i = 0; i < gimbal_num; i++)
    grad[pinocchio_model->nv + pinocchio_robot_model->getRotorNum() + i] =
        0.0;  // gimbal angles do not contribute to the cost

  return cost;
}

void rneaConstraint(unsigned m, double* result, unsigned n, const double* x, double* grad, void* ptr)
{
  /* constraints (m)
     0 ~ nv: 0 = rnea - generalized_force - tauext
  */
  NonlinearInverseDynamics* robot_model = reinterpret_cast<NonlinearInverseDynamics*>(ptr);

  std::shared_ptr<aerial_robot_dynamics::PinocchioRobotModel> pinocchio_robot_model =
      robot_model->getPinocchioRobotModel();
  std::shared_ptr<pinocchio::Model> pinocchio_model = robot_model->getPinocchioModel();
  std::shared_ptr<pinocchio::Data> pinocchio_data = robot_model->getPinocchioData();

  Eigen::VectorXd curr_target_q = robot_model->getCurrentTargetQForOpt();
  Eigen::VectorXd curr_target_dq = robot_model->getCurrentTargetDqForOpt();
  Eigen::VectorXd curr_target_ddq = robot_model->getCurrentTargetDdqForOpt();

  // overwrite q with current gimbal angles
  std::vector<std::string> gimbal_names = robot_model->getGimbalNames();
  int gimbal_num = gimbal_names.size();
  std::vector<int> gimbal_q_indices(0);
  std::vector<int> gimbal_v_indices(0);
  for (int i = 0; i < gimbal_num; i++)
  {
    std::string gimbal_name = gimbal_names.at(i);
    int gimbal_index_q = pinocchio_model->joints[pinocchio_model->getJointId(gimbal_name)].idx_q();
    int gimbal_index_v = pinocchio_model->joints[pinocchio_model->getJointId(gimbal_name)].idx_v();

    gimbal_q_indices.push_back(gimbal_index_q);
    gimbal_v_indices.push_back(gimbal_index_v);

    curr_target_q(gimbal_index_q) = x[pinocchio_model->nv + pinocchio_robot_model->getRotorNum() + i];
  }

  // calculate tauext by thrust
  Eigen::MatrixXd tauext_partial_thrust =
      pinocchio_robot_model->computeTauExtByThrustDerivative(curr_target_q);  // nv * nr
  Eigen::VectorXd thrusts = Eigen::VectorXd::Zero(pinocchio_robot_model->getRotorNum());
  for (int i = 0; i < pinocchio_robot_model->getRotorNum(); i++)
  {
    thrusts(i) = x[pinocchio_model->nv + i];
  }
  Eigen::VectorXd tauext = tauext_partial_thrust * thrusts;

  // calculate rnea
  Eigen::VectorXd rnea_solution =
      pinocchio::rnea(*pinocchio_model, *pinocchio_data, curr_target_q, curr_target_dq, curr_target_ddq);

  // set result
  for (unsigned i = 0; i < pinocchio_model->nv; i++)
  {
    result[i] = rnea_solution(i) - x[i] - tauext(i);  // 0 = rnea - joint torque - tauext
  }

  if (grad == NULL)
    return;

  // gradient
  // torque (0 ~ nv)
  for (int i = 0; i < m; i++)  // contraint (nv)
  {
    for (int j = 0; j < pinocchio_model->nv; j++)  // variable (nv)
    {
      grad[i * n + j] = (i == j) ? -1.0 : 0.0;  // -1 for joint torque
    }
  }

  // thrust (nv ~ nv + nr)
  for (int i = 0; i < m; i++)  // constraint (nv)
  {
    for (int j = 0; j < pinocchio_robot_model->getRotorNum(); j++)  // variable (nr)
    {
      grad[i * n + pinocchio_model->nv + j] = -tauext_partial_thrust(i, j);  // partial derivative of tauext by thrust
    }
  }

  // gimbal angles (nv+nr ~ nv+nr+gimbal_num)
  pinocchio::computeRNEADerivatives(*pinocchio_model, *pinocchio_data, curr_target_q, curr_target_dq, curr_target_ddq);
  Eigen::MatrixXd rnea_partial_q = pinocchio_data->dtau_dq;  // nv * nv
  for (int i = 0; i < m; i++)                                // constraint (nv)
  {
    for (int j = 0; j < gimbal_num; j++)  // variable (gimbal_num)
    {
      grad[i * n + pinocchio_model->nv + pinocchio_robot_model->getRotorNum() + j] =
          rnea_partial_q(i, gimbal_v_indices.at(j));  // partial derivative of rnea by gimbal angles
    }
  }

  std::vector<Eigen::MatrixXd> tauext_partial_thrust_partial_q =
      pinocchio_robot_model->computeTauExtByThrustDerivativeQDerivatives(curr_target_q);

  for (int j = 0; j < gimbal_num; j++)  // variable (gimbal_num)
  {
    Eigen::VectorXd tauext_partial_q_j = tauext_partial_thrust_partial_q.at(gimbal_v_indices.at(j)) *
                                         thrusts;  // (nv * nr) * (nr * 1) = (nv * 1). j-th col.
    for (int i = 0; i < m; i++)                    // constraint (nv)
    {
      grad[i * n + pinocchio_model->nv + pinocchio_robot_model->getRotorNum() + j] += -tauext_partial_q_j(i);  // minus
    }
  }
}
}  // namespace

NonlinearInverseDynamics::NonlinearInverseDynamics(
    ros::NodeHandle nh, std::shared_ptr<aerial_robot_dynamics::PinocchioRobotModel> pinocchio_robot_model)
  : nh_(nh), pinocchio_robot_model_(pinocchio_robot_model)
{
  pinocchio_model_ = pinocchio_robot_model_->getModel();
  pinocchio_data_ = pinocchio_robot_model_->getData();

  rosParamInit();
  loadJointNames();
  loadGimbalNames();

  nlp_last_solution_ =
      Eigen::VectorXd::Zero(pinocchio_model_->nv + pinocchio_robot_model_->getRotorNum() + gimbal_names_.size());
}

void NonlinearInverseDynamics::reset()
{
  nlp_last_solution_ =
      Eigen::VectorXd::Zero(pinocchio_model_->nv + pinocchio_robot_model_->getRotorNum() + gimbal_names_.size());
  solve_time_ = 0.0;
}

void NonlinearInverseDynamics::rosParamInit()
{
  ros::NodeHandle control_nh(nh_, "controller");
  getParam<double>(control_nh, "gimbal_delta_max", gimbal_delta_max_, M_PI);

  std::cout << "gimbal_delta_max: " << gimbal_delta_max_ << std::endl;
}

void NonlinearInverseDynamics::loadJointNames()
{
  joint_names_.clear();
  for (int i = 0; i < pinocchio_model_->njoints; i++)
  {
    std::string joint_name = pinocchio_model_->names[i];
    if (joint_name.find("joint") != std::string::npos)
    {
      joint_names_.push_back(joint_name);
    }
  }
}

void NonlinearInverseDynamics::loadGimbalNames()
{
  gimbal_names_.clear();
  for (int i = 0; i < pinocchio_model_->njoints; i++)
  {
    std::string joint_name = pinocchio_model_->names[i];
    if (joint_name.find("gimbal") != std::string::npos)
    {
      gimbal_names_.push_back(joint_name);
    }
  }
}

bool NonlinearInverseDynamics::solve(const Eigen::VectorXd& q, const Eigen::VectorXd& v, const Eigen::VectorXd& a,
                                     Eigen::VectorXd& tau)
{
  nlp_curr_target_q_ = q;
  nlp_curr_target_dq_ = v;
  nlp_curr_target_ddq_ = a;

  // // Initialize the nonlinear programming problem
  int gimbal_num = gimbal_names_.size();
  int n_variables = pinocchio_model_->nv + pinocchio_robot_model_->getRotorNum() +
                    gimbal_num;             // generalized_force + thrust + gimbal_angles
  int n_constrints = pinocchio_model_->nv;  // rnea

  // bounds
  std::vector<double> lb(n_variables, -std::numeric_limits<double>::infinity());
  std::vector<double> ub(n_variables, std::numeric_limits<double>::infinity());
  Eigen::VectorXd joint_torque_limits = pinocchio_robot_model_->getJointTorqueLimits();
  Eigen::VectorXd thrust_upper_limits = pinocchio_robot_model_->getThrustUpperLimits();
  Eigen::VectorXd thrust_lower_limits = pinocchio_robot_model_->getThrustLowerLimits();
  for (int i = 0; i < pinocchio_model_->nv; ++i)
  {
    lb[i] = -joint_torque_limits(i);
    ub[i] = joint_torque_limits(i);
  }
  for (int i = 0; i < pinocchio_robot_model_->getRotorNum(); ++i)
  {
    lb[pinocchio_model_->nv + i] = thrust_lower_limits(i);
    ub[pinocchio_model_->nv + i] = thrust_upper_limits(i);
  }
  for (int i = 0; i < gimbal_num; i++)
  {
    int gimbal_index_q = pinocchio_model_->joints[pinocchio_model_->getJointId(gimbal_names_.at(i))].idx_q();
    lb[pinocchio_model_->nv + pinocchio_robot_model_->getRotorNum() + i] =
        std::clamp(q(gimbal_index_q) - gimbal_delta_max_, pinocchio_model_->lowerPositionLimit(gimbal_index_q),
                   pinocchio_model_->upperPositionLimit(gimbal_index_q));  // gimbal angles lower bound
    ub[pinocchio_model_->nv + pinocchio_robot_model_->getRotorNum() + i] =
        std::clamp(q(gimbal_index_q) + gimbal_delta_max_, pinocchio_model_->lowerPositionLimit(gimbal_index_q),
                   pinocchio_model_->upperPositionLimit(gimbal_index_q));  // gimbal angles upper bound
  }

  // // Set initial guess
  std::vector<double> x(n_variables);
  for (int i = 0; i < pinocchio_model_->nv; ++i)  // initial guess for generalized force.
  {
    if (tau.size() == n_variables)  // initial guess is passed
      x[i] = std::clamp(tau(i), lb[i], ub[i]);
    else
      x[i] = std::clamp(nlp_last_solution_(i), lb[i], ub[i]);
  }
  for (int i = 0; i < pinocchio_robot_model_->getRotorNum(); ++i)  // initial guess for thrust.
  {
    if (tau.size() == n_variables)  // initial guess is passed
      x[pinocchio_model_->nv + i] =
          std::clamp(tau(pinocchio_model_->nv + i), lb[pinocchio_model_->nv + i], ub[pinocchio_model_->nv + i]);
    else
      x[pinocchio_model_->nv + i] = std::clamp(nlp_last_solution_(pinocchio_model_->nv + i),
                                               lb[pinocchio_model_->nv + i], ub[pinocchio_model_->nv + i]);
  }
  for (int i = 0; i < gimbal_num; ++i)  // initial guess for gimbal angles
  {
    int gimbal_index_q = pinocchio_model_->joints[pinocchio_model_->getJointId(gimbal_names_.at(i))].idx_q();
    x[pinocchio_model_->nv + pinocchio_robot_model_->getRotorNum() + i] =
        std::clamp(q(gimbal_index_q), lb[pinocchio_model_->nv + pinocchio_robot_model_->getRotorNum() + i],
                   ub[pinocchio_model_->nv + pinocchio_robot_model_->getRotorNum() + i]);
  }

  // Solve the optimization problem
  nlopt::opt opt(nlopt::LD_SLSQP, n_variables);
  opt.set_min_objective(torqueThrustMinimize, this);
  opt.add_equality_mconstraint(rneaConstraint, this, std::vector<double>(n_constrints, 1e-4));  // rnea constraints
  opt.set_lower_bounds(lb);
  opt.set_upper_bounds(ub);
  opt.set_ftol_rel(1e-6);
  opt.set_xtol_rel(1e-6);
  opt.set_maxeval(1000);

  double minf;
  nlopt::result result;
  try
  {
    auto start = std::chrono::high_resolution_clock::now();
    result = opt.optimize(x, minf);
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    solve_time_ = duration.count();  // microseconds
  }
  catch (std::runtime_error error)
  {
  }
  if (result < 0)
    ROS_ERROR_STREAM_THROTTLE(1.0, "[nlopt] failed to solve. result is " << result);

  // print
  tau.resize(n_variables);
  for (int i = 0; i < n_variables; i++)
  {
    tau(i) = x.at(i);
  }

  nlp_last_solution_ = tau;

  return (result >= 0) ? true : false;
}
