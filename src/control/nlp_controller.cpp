#include <dynarm/control/joint_trajectory_generator.h>
#include <nlopt.hpp>

using namespace aerial_robot_control;

namespace
{
double torqueThrustMinimize(const std::vector<double>& x, std::vector<double>& grad, void* ptr)
{
  /* variables (n)
     0 ~ nv: joint torque
     nv ~ nv + nr: thrust
     nv + nr ~ nv + nr + gimbal_num: gimbal angles
  */
  jointTrajectoryGenerator* controller = reinterpret_cast<jointTrajectoryGenerator*>(ptr);

  std::shared_ptr<aerial_robot_dynamics::PinocchioRobotModel> pinocchio_robot_model =
      controller->getPinocchioRobotModel();
  std::shared_ptr<pinocchio::Model> pinocchio_model = controller->getPinocchioModel();
  std::shared_ptr<pinocchio::Data> pinocchio_data = controller->getPinocchioData();

  int gimbal_num = controller->getGimbalNumForOpt();
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
     0 ~ nv: 0 = rnea - joint_torque - tauext
  */
  jointTrajectoryGenerator* controller = reinterpret_cast<jointTrajectoryGenerator*>(ptr);

  std::shared_ptr<aerial_robot_dynamics::PinocchioRobotModel> pinocchio_robot_model =
      controller->getPinocchioRobotModel();
  std::shared_ptr<pinocchio::Model> pinocchio_model = controller->getPinocchioModel();
  std::shared_ptr<pinocchio::Data> pinocchio_data = controller->getPinocchioData();

  Eigen::VectorXd curr_target_q = controller->getCurrentTargetQForOpt();
  Eigen::VectorXd curr_target_dq = controller->getCurrentTargetDqForOpt();
  Eigen::VectorXd curr_target_ddq = controller->getCurrentTargetDdqForOpt();

  // overwrite q with current gimbal angles
  int gimbal_num = controller->getGimbalNumForOpt();
  std::vector<int> gimbal_q_indices(0);
  std::vector<int> gimbal_v_indices(0);
  for (int i = 0; i < pinocchio_robot_model->getRotorNum() / controller->getRotorDevider(); i++)
  {
    // assume gimbal_num = 2*rotor_num and their names are gimbal*_roll and gimbal*_pitch in this order
    std::string gimbal_roll_name = "gimbal" + std::to_string(i + 1) + "_roll";
    std::string gimbal_pitch_name = "gimbal" + std::to_string(i + 1) + "_pitch";
    int gimbal_roll_index_q = pinocchio_model->joints[pinocchio_model->getJointId(gimbal_roll_name)].idx_q();
    int gimbal_pitch_index_q = pinocchio_model->joints[pinocchio_model->getJointId(gimbal_pitch_name)].idx_q();
    int gimbal_roll_index_v = pinocchio_model->joints[pinocchio_model->getJointId(gimbal_roll_name)].idx_v();
    int gimbal_pitch_index_v = pinocchio_model->joints[pinocchio_model->getJointId(gimbal_pitch_name)].idx_v();

    gimbal_q_indices.push_back(gimbal_roll_index_q);
    gimbal_q_indices.push_back(gimbal_pitch_index_q);
    gimbal_v_indices.push_back(gimbal_roll_index_v);
    gimbal_v_indices.push_back(gimbal_pitch_index_v);

    curr_target_q(gimbal_roll_index_q) = x[pinocchio_model->nv + pinocchio_robot_model->getRotorNum() + 2 * i];
    curr_target_q(gimbal_pitch_index_q) = x[pinocchio_model->nv + pinocchio_robot_model->getRotorNum() + 2 * i + 1];
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

bool jointTrajectoryGenerator::nonlinearInverseDynamics(const Eigen::VectorXd& q, const Eigen::VectorXd& v,
                                                        const Eigen::VectorXd& a, Eigen::VectorXd& tau)
{
  nlp_curr_target_q_ = q;
  nlp_curr_target_dq_ = v;
  nlp_curr_target_ddq_ = a;

  // calculate gimbal num for optimization
  if (nlp_first_run_)
  {
    gimbal_num_ = 0;
    gimbal_q_indices_.clear();
    gimbal_v_indices_.clear();
    for (int i = 0; i < pinocchio_robot_model_->getRotorNum() / rotor_devider_; i++)
    {
      // assume gimbal_num = rotor_num and their names are gimbal*_roll and gimbal*_pitch in this order
      std::string gimbal_roll_name = "gimbal" + std::to_string(i + 1) + "_roll";
      std::string gimbal_pitch_name = "gimbal" + std::to_string(i + 1) + "_pitch";

      gimbal_num_ += 2;  // each gimbal has two angles (roll and pitch)

      int gimbal_roll_index_q = pinocchio_model_->joints[pinocchio_model_->getJointId(gimbal_roll_name)].idx_q();
      int gimbal_pitch_index_q = pinocchio_model_->joints[pinocchio_model_->getJointId(gimbal_pitch_name)].idx_q();
      int gimbal_roll_index_v = pinocchio_model_->joints[pinocchio_model_->getJointId(gimbal_roll_name)].idx_v();
      int gimbal_pitch_index_v = pinocchio_model_->joints[pinocchio_model_->getJointId(gimbal_pitch_name)].idx_v();

      gimbal_q_indices_.push_back(gimbal_roll_index_q);
      gimbal_q_indices_.push_back(gimbal_pitch_index_q);
      gimbal_v_indices_.push_back(gimbal_roll_index_v);
      gimbal_v_indices_.push_back(gimbal_pitch_index_v);
    }
    ROS_INFO_STREAM("[dynarm][control][nlopt] gibmal num is " << gimbal_num_);
    nlp_first_run_ = false;
  }

  // // Initialize the nonlinear programming problem
  int n_variables = pinocchio_model_->nv + pinocchio_robot_model_->getRotorNum() +
                    gimbal_num_;            // joint torque + thrust + gimbal angles
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
  for (int i = 0; i < gimbal_num_; i++)
  {
    lb[pinocchio_model_->nv + pinocchio_robot_model_->getRotorNum() + i] = std::clamp(
        curr_q_(gimbal_q_indices_[i]) - gimbal_delta_max_, pinocchio_model_->lowerPositionLimit(gimbal_q_indices_[i]),
        pinocchio_model_->upperPositionLimit(gimbal_q_indices_[i]));  // gimbal angles lower bound
    ub[pinocchio_model_->nv + pinocchio_robot_model_->getRotorNum() + i] = std::clamp(
        curr_q_(gimbal_q_indices_[i]) + gimbal_delta_max_, pinocchio_model_->lowerPositionLimit(gimbal_q_indices_[i]),
        pinocchio_model_->upperPositionLimit(gimbal_q_indices_[i]));  // gimbal angles upper bound
  }

  // // Set initial guess
  std::vector<double> x(n_variables);
  for (int i = 0; i < pinocchio_model_->nv; ++i)
  {
    x[i] = std::clamp(curr_target_tau_(i), lb[i], ub[i]);  // initial guess for joint torque
  }
  for (int i = 0; i < pinocchio_robot_model_->getRotorNum(); ++i)
  {
    x[pinocchio_model_->nv + i] = std::clamp(curr_target_thrust_(i), lb[pinocchio_model_->nv + i],
                                             ub[pinocchio_model_->nv + i]);  // initial guess for thrust
  }
  for (int i = 0; i < gimbal_num_; ++i)
  {
    // assume gimbal angles are initialized to zero
    x[pinocchio_model_->nv + pinocchio_robot_model_->getRotorNum() + i] = std::clamp(
        curr_q_(gimbal_q_indices_[i]), lb[pinocchio_model_->nv + pinocchio_robot_model_->getRotorNum() + i],
        ub[pinocchio_model_->nv + pinocchio_robot_model_->getRotorNum() + i]);  // initial guess for gimbal angles
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
    nlp_solve_time_ = duration.count();  // microseconds
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

  return (result >= 0) ? true : false;
}
