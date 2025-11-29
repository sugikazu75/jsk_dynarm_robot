#pragma once

#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>

#include <crocoddyl/core/action-base.hpp>
#include <crocoddyl/core/activations/weighted-quadratic.hpp>
#include <crocoddyl/core/activations/quadratic-barrier.hpp>
#include <crocoddyl/core/costs/cost-sum.hpp>
#include <crocoddyl/core/costs/residual.hpp>
#include <crocoddyl/core/diff-action-base.hpp>
#include <crocoddyl/core/integ-action-base.hpp>
#include <crocoddyl/core/integrator/euler.hpp>
#include <crocoddyl/core/integrator/rk.hpp>
#include <crocoddyl/core/optctrl/shooting.hpp>
#include <crocoddyl/core/residuals/control.hpp>
#include <crocoddyl/core/solvers/fddp.hpp>
#include <crocoddyl/core/solvers/box-fddp.hpp>
#include <crocoddyl/core/state-base.hpp>
#include <crocoddyl/core/utils/timer.hpp>
#include <crocoddyl/multibody/actions/free-fwddyn.hpp>
#include <crocoddyl/multibody/actions/free-thrust-fwddyn.hpp>
#include <crocoddyl/multibody/actions/free-invdyn.hpp>
#include <crocoddyl/multibody/actuations/floating-base.hpp>
#include <crocoddyl/multibody/actuations/floating-base-thrusters.hpp>
#include <crocoddyl/multibody/states/multibody.hpp>
#include <crocoddyl/multibody/residuals/com-position.hpp>
#include <crocoddyl/multibody/residuals/frame-placement.hpp>
#include <crocoddyl/multibody/residuals/frame-rotation.hpp>
#include <crocoddyl/multibody/residuals/state.hpp>

class DDPHoveringProblem
{
public:
  struct CostWeight
  {
    double state_weight;
    Eigen::VectorXd x_weights;
    double state_bound_weight;
    double control_weight;
    Eigen::VectorXd u_weights;
  };
  struct OptimizationParam
  {
    double horizon = 2.0;
    double dt = 0.1;
    int max_iter = 100;
    int num_threads = 1;
  };

  DDPHoveringProblem(std::shared_ptr<pinocchio::Model> pinocchio_model, std::vector<crocoddyl::Rotor> rotors,
                     bool fwddyn, const CostWeight& cost_weight, const OptimizationParam& optimization_param);
  ~DDPHoveringProblem(){};

  std::shared_ptr<crocoddyl::ActionModelAbstract> createActionModel(Eigen::VectorXd x0, Eigen::VectorXd xref);
  std::shared_ptr<crocoddyl::ShootingProblem> createHoveringProblem(Eigen::VectorXd x0, Eigen::VectorXd xref);

  CostWeight cost_weight_;
  OptimizationParam optimization_param_;
  std::vector<std::shared_ptr<crocoddyl::ResidualModelState>> state_residuals_;

protected:
  std::shared_ptr<pinocchio::Model> pinocchio_model_;
  std::shared_ptr<pinocchio::Data> pinocchio_data_;
  std::shared_ptr<crocoddyl::StateMultibody> state_;
  std::shared_ptr<crocoddyl::ActuationModelAbstract> actuation_;
  std::vector<crocoddyl::Rotor> rotors_;
  bool fwddyn_;
  int nu_;
};
