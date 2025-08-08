#include <pinocchio/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <aerial_robot_dynamics/robot_model.h>

#include "crocoddyl/core/action-base.hpp"
#include "crocoddyl/core/activations/weighted-quadratic.hpp"
#include "crocoddyl/core/activations/quadratic-barrier.hpp"
#include "crocoddyl/core/costs/cost-sum.hpp"
#include "crocoddyl/core/costs/residual.hpp"
#include <crocoddyl/core/diff-action-base.hpp>
#include "crocoddyl/core/integ-action-base.hpp"
#include "crocoddyl/core/integrator/euler.hpp"
#include "crocoddyl/core/integrator/rk.hpp"
#include "crocoddyl/core/optctrl/shooting.hpp"
#include "crocoddyl/core/residuals/control.hpp"
#include "crocoddyl/core/solvers/fddp.hpp"
#include "crocoddyl/core/solvers/box-fddp.hpp"
#include <crocoddyl/core/state-base.hpp>
#include "crocoddyl/core/utils/timer.hpp"
#include <crocoddyl/multibody/actions/free-fwddyn.hpp>
#include <crocoddyl/multibody/actions/free-invdyn.hpp>
#include <crocoddyl/multibody/actuations/floating-base-thrusters.hpp>
#include <crocoddyl/multibody/states/multibody.hpp>
#include "crocoddyl/multibody/residuals/com-position.hpp"
#include "crocoddyl/multibody/residuals/frame-placement.hpp"
#include "crocoddyl/multibody/residuals/frame-rotation.hpp"
#include "crocoddyl/multibody/residuals/state.hpp"

#include <sensor_msgs/JointState.h>

#include <vector>
#include <limits>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "crocoddyl_test");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  tf::TransformBroadcaster robot_base_broadcaster;
  ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);

  std::string fileName = "";
  nhp.getParam("filename", fileName);

  std::shared_ptr<aerial_robot_dynamics::PinocchioRobotModel> pinocchio_robot_model =
      std::make_shared<aerial_robot_dynamics::PinocchioRobotModel>(true);
  std::shared_ptr<pinocchio::Model> pinocchio_model = pinocchio_robot_model->getModel();
  std::shared_ptr<pinocchio::Data> pinocchio_data = pinocchio_robot_model->getData();

  std::vector<int> rotor_frame_indices = pinocchio_robot_model->getRotorFrameIndices();
  double m_f_rate = pinocchio_robot_model->getMFRate();

  // state
  std::shared_ptr<crocoddyl::StateMultibody> state = std::make_shared<crocoddyl::StateMultibody>(pinocchio_model);
  std::cout << "state.nx: " << state->get_nx() << std::endl;
  std::cout << "state.lb: " << state->get_lb().transpose() << std::endl;
  std::cout << "state.ub: " << state->get_ub().transpose() << std::endl;

  // actuation
  std::vector<crocoddyl::Thruster> thrusters;
  for (int i = 0; i < rotor_frame_indices.size(); i++)
  {
    int rotor_direction = pinocchio_robot_model->getRotorDirection(i);
    double thrust_lower_limit = pinocchio_robot_model->getThrustLowerLimits()(i);
    double thrust_upper_limit = pinocchio_robot_model->getThrustUpperLimits()(i);
    thrusters.emplace_back(rotor_frame_indices.at(i), (float)(abs(m_f_rate)),
                           ((rotor_direction == 1) ? crocoddyl::ThrusterType::CCW : crocoddyl::ThrusterType::CW),
                           (float)thrust_lower_limit, (float)thrust_upper_limit);
  }
  std::shared_ptr<crocoddyl::ActuationModelFloatingBaseThrusters> actuation =
      std::make_shared<crocoddyl::ActuationModelFloatingBaseThrusters>(state, thrusters);

  bool fwddyn = true;
  nhp.getParam("fwddyn", fwddyn);
  int nu;
  if (fwddyn)
  {
    nu = actuation->get_nu();
  }
  else
  {
    nu = state->get_nv();
  }

  std::shared_ptr<crocoddyl::CostModelSum> running_cost_model = std::make_shared<crocoddyl::CostModelSum>(state, nu);
  std::shared_ptr<crocoddyl::CostModelSum> terminal_cost_model = std::make_shared<crocoddyl::CostModelSum>(state, nu);
  std::cout << "nu: " << nu << std::endl;

  // reference state
  Eigen::VectorXd xref;
  {
    std::vector<double> whatever(pinocchio_model->nq + pinocchio_model->nv);
    nhp.getParam("x_ref", whatever);
    xref = Eigen::Map<Eigen::VectorXd>(whatever.data(), whatever.size());
    xref.segment<4>(3).normalize();
    std::cout << "xref: " << xref.transpose() << std::endl;
    std::cout << "xref size: " << xref.size() << std::endl;
  }

  for (int i = 0; i < pinocchio_robot_model->getRotorNum(); i++)
  {
    Eigen::MatrixXd rotor_i_jacobian = Eigen::MatrixXd::Zero(6, pinocchio_model->nv);
    pinocchio::computeFrameJacobian(*pinocchio_model, *pinocchio_data, xref.head(pinocchio_model->nq),
                                    rotor_frame_indices.at(i), pinocchio::LOCAL, rotor_i_jacobian);
    std::cout << "rotor " << i + 1 << " jacobian: \n" << rotor_i_jacobian << std::endl;

    rotor_i_jacobian.setZero();
    Eigen::VectorXd noise_dq = Eigen::VectorXd::Random(pinocchio_model->nv) * 0.05;
    noise_dq.tail(pinocchio_model->nv - 6).setZero();
    pinocchio::computeFrameJacobian(*pinocchio_model, *pinocchio_data,
                                    pinocchio::integrate(*pinocchio_model, xref.head(pinocchio_model->nq), noise_dq),
                                    rotor_frame_indices.at(i), pinocchio::LOCAL, rotor_i_jacobian);
    std::cout << "rotor " << i + 1 << " jacobian with noise: \n" << rotor_i_jacobian << std::endl;
    std::cout << "noise dq: " << noise_dq.transpose() << std::endl;
  }

  pinocchio::framesForwardKinematics(*pinocchio_model, *pinocchio_data, xref.head(pinocchio_model->nq));
  std::cout << "root: " << pinocchio_data->oMi[1] << std::endl;
  for (int i = 0; i < pinocchio_robot_model->getRotorNum(); i++)
  {
    std::cout << "rotor " << i + 1 << " frame: " << rotor_frame_indices.at(i) << ": "
              << pinocchio_data->oMf[rotor_frame_indices.at(i)] << std::endl;
    std::cout << "root to rotor " << i + 1 << ": \n"
              << pinocchio_data->oMi[1].inverse() * pinocchio_data->oMf[rotor_frame_indices.at(i)] << std::endl;
  }

  // cost
  // com track
  Eigen::Vector3d com_ref = pinocchio::centerOfMass(*pinocchio_model, *pinocchio_data, xref.head(pinocchio_model->nq));
  com_ref(2) = xref(2);
  std::cout << "com_ref: " << com_ref.transpose() << std::endl;
  std::shared_ptr<crocoddyl::CostModelAbstract> com_cost = std::make_shared<crocoddyl::CostModelResidual>(
      state, std::make_shared<crocoddyl::ResidualModelCoMPosition>(state, com_ref, nu));

  // root frame rotation
  std::shared_ptr<crocoddyl::ResidualModelAbstract> root_rotation_residual =
      std::make_shared<crocoddyl::ResidualModelFrameRotation>(state, pinocchio_model->getFrameId("root"),
                                                              Eigen::Matrix3d::Identity(), nu);
  std::shared_ptr<crocoddyl::CostModelAbstract> root_rotation_cost =
      std::make_shared<crocoddyl::CostModelResidual>(state, root_rotation_residual);

  // state bounds
  std::shared_ptr<crocoddyl::ResidualModelAbstract> x_bounds_residual =
      std::make_shared<crocoddyl::ResidualModelState>(state, xref, nu);
  std::shared_ptr<crocoddyl::ActivationModelAbstract> activation_x_bounds =
      std::make_shared<crocoddyl::ActivationModelQuadraticBarrier>(crocoddyl::ActivationBounds(
          state->get_lb().tail(2 * pinocchio_model->nv), state->get_ub().tail(2 * pinocchio_model->nv)));
  std::shared_ptr<crocoddyl::CostModelAbstract> x_bounds_cost =
      std::make_shared<crocoddyl::CostModelResidual>(state, activation_x_bounds, x_bounds_residual);

  // state point
  std::shared_ptr<crocoddyl::ResidualModelAbstract> x_residual =
      std::make_shared<crocoddyl::ResidualModelState>(state, xref, nu);
  Eigen::VectorXd x_activation_weights;
  {
    std::vector<double> whatever(pinocchio_model->nv + pinocchio_model->nv);
    nhp.getParam("x_state_weights", whatever);
    x_activation_weights = Eigen::Map<Eigen::VectorXd>(whatever.data(), whatever.size());
    std::cout << "x_activation_weights: " << x_activation_weights.transpose() << std::endl;
  }
  std::shared_ptr<crocoddyl::CostModelAbstract> x_reg_cost = std::make_shared<crocoddyl::CostModelResidual>(
      state, std::make_shared<crocoddyl::ActivationModelWeightedQuad>(x_activation_weights), x_residual);

  // control input
  Eigen::VectorXd uref = Eigen::VectorXd::Zero(nu);
  std::shared_ptr<crocoddyl::ResidualModelAbstract> u_residual =
      std::make_shared<crocoddyl::ResidualModelControl>(state, uref);
  Eigen::VectorXd u_activation_weights;
  {
    std::vector<double> whatever(nu);
    if (fwddyn)
      nhp.getParam("u_weight_fwddyn", whatever);
    else
      nhp.getParam("u_weight_invdyn", whatever);

    u_activation_weights = Eigen::Map<Eigen::VectorXd>(whatever.data(), whatever.size());
    std::cout << "u_activation_weights: " << u_activation_weights.transpose() << std::endl;
  }
  std::shared_ptr<crocoddyl::CostModelAbstract> u_reg_cost = std::make_shared<crocoddyl::CostModelResidual>(
      state, std::make_shared<crocoddyl::ActivationModelWeightedQuad>(u_activation_weights), u_residual);

  {
    double whatever = 0;
    nhp.getParam("com_weight", whatever);
    std::cout << "com_weight: " << whatever << std::endl;
    if (whatever > 0)
      running_cost_model->addCost("com_track", com_cost, whatever);
    else
      std::cout << "\033[31m"
                << "com_track cost not added, weight is 0"
                << "\033[0m" << std::endl;

    whatever = 0;
    nhp.getParam("root_rotation_weight", whatever);
    std::cout << "root_rotation_weight: " << whatever << std::endl;
    if (whatever > 0)
      running_cost_model->addCost("root_rotation", root_rotation_cost, whatever);
    else
      std::cout << "\033[31m"
                << "root_rotation cost not added, weight is 0"
                << "\033[0m" << std::endl;

    whatever = 0;
    nhp.getParam("x_reg_weight", whatever);
    std::cout << "x_reg_weight: " << whatever << std::endl;
    if (whatever > 0)
      running_cost_model->addCost("x_reg", x_reg_cost, whatever);
    else
      std::cout << "\033[31m"
                << "x_reg cost not added, weight is 0"
                << "\033[0m" << std::endl;

    whatever = 0;
    nhp.getParam("x_bounds_weight", whatever);
    std::cout << "x_bounds_weight: " << whatever << std::endl;
    if (whatever > 0)
      running_cost_model->addCost("x_bounds", x_bounds_cost, whatever);
    else
      std::cout << "\033[31m"
                << "x_bounds cost not added, weight is 0"
                << "\033[0m" << std::endl;

    whatever = 0;
    nhp.getParam("u_reg_weight", whatever);
    std::cout << "u_reg_weight: " << whatever << std::endl;
    if (whatever > 0)
      running_cost_model->addCost("u_reg", u_reg_cost, whatever);
    else
      std::cout << "\033[31m"
                << "u_reg cost not added, weight is 0"
                << "\033[0m" << std::endl;

    whatever = 0;
    nhp.getParam("com_terminal_weight", whatever);
    std::cout << "com_terminal_weight: " << whatever << std::endl;
    if (whatever > 0)
      terminal_cost_model->addCost("com_track", com_cost, whatever);
    else
      std::cout << "\033[31m"
                << "com_track terminal cost not added, weight is 0"
                << "\033[0m" << std::endl;

    whatever = 0;
    nhp.getParam("root_rotation_terminal_weight", whatever);
    std::cout << "root_rotation_terminal_weight: " << whatever << std::endl;
    if (whatever > 0)
      terminal_cost_model->addCost("root_rotation", root_rotation_cost, whatever);
    else
      std::cout << "\033[31m"
                << "root_rotation terminal cost not added, weight is 0"
                << "\033[0m" << std::endl;

    whatever = 0;
    nhp.getParam("x_terminal_reg_weight", whatever);
    std::cout << "x_terminal_reg_weight: " << whatever << std::endl;
    if (whatever > 0)
      terminal_cost_model->addCost("x_reg", x_reg_cost, whatever);
    else
      std::cout << "\033[31m"
                << "x_reg terminal cost not added, weight is 0"
                << "\033[0m" << std::endl;

    whatever = 0;
    nhp.getParam("x_terminal_bounds_weight", whatever);
    std::cout << "x_terminal_bounds_weight: " << whatever << std::endl;
    if (whatever > 0)
      terminal_cost_model->addCost("x_bounds", x_bounds_cost, whatever);
    else
      std::cout << "\033[31m"
                << "x_bounds terminal cost not added, weight is 0"
                << "\033[0m" << std::endl;
  }

  double horizon, dt;
  nhp.getParam("horizon", horizon);
  nhp.getParam("dt", dt);
  std::cout << "horizon: " << horizon << " , dt: " << dt << std::endl;

  int N = horizon / dt;

  std::shared_ptr<crocoddyl::DifferentialActionModelAbstract> dmodel_running;
  std::shared_ptr<crocoddyl::DifferentialActionModelAbstract> dmodel_terminal;
  if (fwddyn)
  {
    dmodel_running =
        std::make_shared<crocoddyl::DifferentialActionModelFreeFwdDynamics>(state, actuation, running_cost_model);
    dmodel_terminal =
        std::make_shared<crocoddyl::DifferentialActionModelFreeFwdDynamics>(state, actuation, terminal_cost_model);
  }
  else
  {
    dmodel_running =
        std::make_shared<crocoddyl::DifferentialActionModelFreeInvDynamics>(state, actuation, running_cost_model);
    dmodel_terminal =
        std::make_shared<crocoddyl::DifferentialActionModelFreeInvDynamics>(state, actuation, terminal_cost_model);
  }

  std::shared_ptr<crocoddyl::ActionModelAbstract> running_model =
      std::make_shared<crocoddyl::IntegratedActionModelEuler>(dmodel_running, dt);
  std::shared_ptr<crocoddyl::ActionModelAbstract> terminal_model =
      std::make_shared<crocoddyl::IntegratedActionModelEuler>(dmodel_terminal, dt);

  Eigen::VectorXd u_lb = -Eigen::VectorXd::Ones(nu) * std::numeric_limits<double>::infinity();
  Eigen::VectorXd u_ub = Eigen::VectorXd::Ones(nu) * std::numeric_limits<double>::infinity();
  if (fwddyn)
  {
    u_lb.head(pinocchio_robot_model->getRotorNum()) = pinocchio_robot_model->getThrustLowerLimits();
    u_ub.head(pinocchio_robot_model->getRotorNum()) = pinocchio_robot_model->getThrustUpperLimits();
    u_lb.tail(nu - pinocchio_robot_model->getRotorNum()) =
        -pinocchio_robot_model->getJointTorqueLimits().tail(nu - pinocchio_robot_model->getRotorNum());
    u_ub.tail(nu - pinocchio_robot_model->getRotorNum()) =
        pinocchio_robot_model->getJointTorqueLimits().tail(nu - pinocchio_robot_model->getRotorNum());
  }
  std::cout << "u_lb: " << u_lb.transpose() << std::endl;
  std::cout << "u_ub: " << u_ub.transpose() << std::endl;
  running_model->set_u_lb(u_lb);
  running_model->set_u_ub(u_ub);
  terminal_model->set_u_lb(u_lb);
  terminal_model->set_u_ub(u_ub);

  std::vector<std::shared_ptr<crocoddyl::ActionModelAbstract> > running_models(N, running_model);

  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(pinocchio_model->nq + pinocchio_model->nv);
  {
    std::vector<double> whatever;
    nhp.getParam("x_init", whatever);
    x0 = Eigen::Map<Eigen::VectorXd>(whatever.data(), whatever.size());
    x0.segment<4>(3).normalize();
    std::cout << "x_init: " << x0.transpose() << std::endl;
  }

  std::shared_ptr<crocoddyl::ShootingProblem> problem =
      std::make_shared<crocoddyl::ShootingProblem>(x0, running_models, terminal_model);
  std::shared_ptr<crocoddyl::SolverAbstract> solver = std::make_shared<crocoddyl::SolverBoxFDDP>(problem);

  int num_threads = 1;
  int max_iter = 100;
  bool add_noise = false;
  nhp.getParam("num_threads", num_threads);
  nhp.getParam("max_iter", max_iter);
  nhp.getParam("add_noise", add_noise);
  solver->get_problem()->set_nthreads(num_threads);

  std::vector<Eigen::VectorXd> xs_init(N, x0);
  std::vector<Eigen::VectorXd> us_init = solver->get_problem()->quasiStatic_xs(xs_init);
  std::cout << "u_init: " << us_init.at(0).transpose() << std::endl;
  xs_init.push_back(x0);
  std::cout << std::endl;
  std::cout << std::endl;

  std::string robot_ns = ros::this_node::getNamespace();
  if (!robot_ns.empty() && robot_ns[0] == '/')
    robot_ns = robot_ns.substr(1);

  ros::Rate rate((int)(1.0 / dt));
  while (ros::ok())
  {
    crocoddyl::Timer timer;
    solver->solve(xs_init, us_init, max_iter);
    double time = timer.get_duration();
    std::cout << "total calculation time: " << time << "[ms]" << std::endl;
    std::cout << "Number of iterations: " << solver->get_iter() << std::endl;
    std::cout << "time per iterate: " << time / solver->get_iter() << std::endl;
    std::cout << "Total cost: " << solver->get_cost() << std::endl;
    std::cout << "Gradient norm: " << solver->get_stop() << std::endl;

    xs_init = solver->get_xs();
    us_init = solver->get_us();

    Eigen::VectorXd next_x = xs_init.at(1);
    if (add_noise)
    {
      Eigen::VectorXd noise_dq = Eigen::VectorXd::Random(pinocchio_model->nv) * 0.005;
      Eigen::VectorXd noise_v = Eigen::VectorXd::Random(pinocchio_model->nv) * 0.005;
      next_x.head(pinocchio_model->nq) =
          pinocchio::integrate(*pinocchio_model, next_x.head(pinocchio_model->nq), noise_dq);
      next_x.tail(pinocchio_model->nv) += noise_v;
    }

    problem->set_x0(next_x);

    //  std::fill(xs_init.begin(), xs_init.end(), next_x);
    //  std::fill(us_init.begin(), us_init.end(), us_init.at(0));

    std::cout << "q: " << xs_init.at(1).head(pinocchio_model->nq).transpose() << std::endl;
    std::cout << "v: " << xs_init.at(1).tail(pinocchio_model->nv).transpose() << std::endl;
    if (fwddyn)
    {
      std::cout << "thrust: " << us_init.at(0).head(pinocchio_robot_model->getRotorNum()).transpose() << std::endl;
      std::cout << "joint torque: " << us_init.at(0).tail(pinocchio_model->nv - 6).transpose() << std::endl;
    }
    else
    {
      std::cout << "root ddq: " << us_init.at(0).head(6).transpose() << std::endl;
      std::cout << "joint ddq: " << us_init.at(0).tail(nu - 6).transpose() << std::endl;
    }
    std::cout << std::endl;

    std::cout << "q final: " << xs_init.back().head(pinocchio_model->nq).transpose() << std::endl;
    std::cout << "v final: " << xs_init.back().tail(pinocchio_model->nv).transpose() << std::endl;
    if (fwddyn)
    {
      std::cout << "thrust final: " << us_init.back().head(pinocchio_robot_model->getRotorNum()).transpose()
                << std::endl;
      std::cout << "joint torque final: " << us_init.back().tail(pinocchio_model->nv - 6).transpose() << std::endl;
    }
    else
    {
      std::cout << "root ddq final: " << us_init.back().head(6).transpose() << std::endl;
      std::cout << "joint ddq final: " << us_init.back().tail(nu - 6).transpose() << std::endl;
    }
    std::cout << std::endl;
    std::cout << std::endl;

    // publish the robot state
    Eigen::VectorXd q = xs_init.at(1).head(pinocchio_model->nq);
    geometry_msgs::TransformStamped robot_base_transform;
    robot_base_transform.header.stamp = ros::Time::now();
    robot_base_transform.header.frame_id = "world";
    robot_base_transform.child_frame_id = tf::resolve(robot_ns, "root");
    robot_base_transform.transform.translation.x = q(0);
    robot_base_transform.transform.translation.y = q(1);
    robot_base_transform.transform.translation.z = q(2);
    robot_base_transform.transform.rotation.x = q(3);
    robot_base_transform.transform.rotation.y = q(4);
    robot_base_transform.transform.rotation.z = q(5);
    robot_base_transform.transform.rotation.w = q(6);

    robot_base_broadcaster.sendTransform(robot_base_transform);

    sensor_msgs::JointState joint_state_msg;
    joint_state_msg.header.stamp = ros::Time::now();
    for (pinocchio::JointIndex joint_id = 2; joint_id < (pinocchio::JointIndex)pinocchio_model->njoints;
         ++joint_id)  // universe and root
    {
      joint_state_msg.name.push_back(pinocchio_model->names[joint_id]);
      joint_state_msg.position.push_back(q[7 - 2 + joint_id]);
    }
    joint_state_pub.publish(joint_state_msg);

    rate.sleep();
  }
}
