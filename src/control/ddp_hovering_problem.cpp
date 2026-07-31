#include <dynarm/control/ddp_hovering_problem.h>

#include <aerial_robot_dynamics/robot_model.h>
#include <aerial_robot_dynamics/robot_model_ros.h>
#include <sensor_msgs/JointState.h>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

DDPHoveringProblem::DDPHoveringProblem(std::shared_ptr<pinocchio::Model> pinocchio_model,
                                       std::vector<crocoddyl::DistributedThruster> rotors, bool fwddyn,
                                       const CostWeight& cost_weight,
                                       const OptimizationParam& optimization_param = OptimizationParam())
  : pinocchio_model_(pinocchio_model)
  , rotors_(rotors)
  , fwddyn_(fwddyn)
  , cost_weight_(cost_weight)
  , optimization_param_(optimization_param)
{
  pinocchio_data_ = std::make_shared<pinocchio::Data>(*pinocchio_model_);
  state_ = std::make_shared<crocoddyl::StateMultibody>(pinocchio_model_);

  actuation_ = std::make_shared<crocoddyl::ActuationModelFloatingBaseDistributedThrusters>(state_, rotors_);

  if (fwddyn_)
    nu_ = actuation_->get_nu();
  else
    nu_ = state_->get_nv();

  std::cout << "nu: " << nu_ << std::endl;
}

std::shared_ptr<crocoddyl::ActionModelAbstract> DDPHoveringProblem::createActionModel(Eigen::VectorXd x0,
                                                                                      Eigen::VectorXd xref)
{
  x0.segment<4>(3).normalize();    // Ensure quaternion is normalized
  xref.segment<4>(3).normalize();  // Ensure quaternion is normalized

  // cost
  std::shared_ptr<crocoddyl::CostModelSum> cost_model = std::make_shared<crocoddyl::CostModelSum>(state_, nu_);

  // state bounds
  std::shared_ptr<crocoddyl::ResidualModelAbstract> x_bounds_residual =
      std::make_shared<crocoddyl::ResidualModelState>(state_, xref, nu_);
  std::shared_ptr<crocoddyl::ActivationModelAbstract> activation_x_bounds =
      std::make_shared<crocoddyl::ActivationModelQuadraticBarrier>(crocoddyl::ActivationBounds(
          state_->get_lb().tail(2 * pinocchio_model_->nv), state_->get_ub().tail(2 * pinocchio_model_->nv)));
  std::shared_ptr<crocoddyl::CostModelAbstract> x_bounds_cost =
      std::make_shared<crocoddyl::CostModelResidual>(state_, activation_x_bounds, x_bounds_residual);

  // state point
  std::shared_ptr<crocoddyl::ResidualModelState> x_residual =
      std::make_shared<crocoddyl::ResidualModelState>(state_, xref, nu_);
  std::shared_ptr<crocoddyl::CostModelAbstract> x_reg_cost = std::make_shared<crocoddyl::CostModelResidual>(
      state_, std::make_shared<crocoddyl::ActivationModelWeightedQuad>(cost_weight_.x_weights), x_residual);
  state_residuals_.push_back(x_residual);

  // control input
  Eigen::VectorXd uref = Eigen::VectorXd::Zero(nu_);
  std::shared_ptr<crocoddyl::ResidualModelAbstract> u_residual =
      std::make_shared<crocoddyl::ResidualModelControl>(state_, uref);
  std::shared_ptr<crocoddyl::CostModelAbstract> u_reg_cost = std::make_shared<crocoddyl::CostModelResidual>(
      state_, std::make_shared<crocoddyl::ActivationModelWeightedQuad>(cost_weight_.u_weights), u_residual);

  if (cost_weight_.state_weight > 0)
  {
    cost_model->addCost("x_bound", x_bounds_cost, cost_weight_.state_bound_weight);
    cost_model->addCost("x_state", x_reg_cost, cost_weight_.state_weight);
    cost_model->addCost("u_control", u_reg_cost, cost_weight_.control_weight);
  }

  std::shared_ptr<crocoddyl::DifferentialActionModelAbstract> dmodel;
  if (fwddyn_)
    dmodel = std::make_shared<crocoddyl::DifferentialActionModelFreeFwdDynamics>(state_, actuation_, cost_model);
  else
    dmodel = std::make_shared<crocoddyl::DifferentialActionModelFreeInvDynamics>(state_, actuation_, cost_model);

  std::shared_ptr<crocoddyl::ActionModelAbstract> action_model =
      std::make_shared<crocoddyl::IntegratedActionModelEuler>(dmodel, optimization_param_.dt);

  return action_model;
}

std::shared_ptr<crocoddyl::ShootingProblem> DDPHoveringProblem::createHoveringProblem(Eigen::VectorXd x0,
                                                                                      Eigen::VectorXd xref)
{
  int N = optimization_param_.horizon / optimization_param_.dt;
  std::vector<std::shared_ptr<crocoddyl::ActionModelAbstract>> action_models(0);
  state_residuals_.clear();
  for (int i = 0; i < N; i++)
  {
    action_models.push_back(createActionModel(x0, xref));
  }
  std::shared_ptr<crocoddyl::ActionModelAbstract> terminal_model = createActionModel(x0, xref);

  std::shared_ptr<crocoddyl::ShootingProblem> problem =
      std::make_shared<crocoddyl::ShootingProblem>(x0, action_models, terminal_model);

  problem->set_nthreads(optimization_param_.num_threads);

  return problem;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "crocoddyl_test");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  tf::TransformBroadcaster robot_base_broadcaster;
  ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
  std::string robot_ns = ros::this_node::getNamespace();
  if (!robot_ns.empty() && robot_ns[0] == '/')
    robot_ns = robot_ns.substr(1);

  // is_floating_base comes from dynamics/is_floating_base (default true) via
  // PinocchioRobotModelRos instead of being hard-coded here
  aerial_robot_dynamics::PinocchioRobotModelRos pinocchio_robot_model_ros(nh);
  std::shared_ptr<aerial_robot_dynamics::PinocchioRobotModel> pinocchio_robot_model =
      pinocchio_robot_model_ros.getPinocchioRobotModel();
  std::shared_ptr<pinocchio::Model> pinocchio_model = pinocchio_robot_model->getModel();

  std::cout << "effortLimit: " << pinocchio_model->effortLimit.transpose() << std::endl;

  std::vector<int> rotor_frame_indices = pinocchio_robot_model->getRotorFrameIndices();
  double m_f_rate = pinocchio_robot_model->getMFRate();

  std::vector<crocoddyl::DistributedThruster> distributed_thrusters;
  for (int i = 0; i < rotor_frame_indices.size(); i++)
  {
    int rotor_direction = pinocchio_robot_model->getRotorDirection(i);
    double thrust_lower_limit = pinocchio_robot_model->getThrustLowerLimits()(i);
    double thrust_upper_limit = pinocchio_robot_model->getThrustUpperLimits()(i);

    distributed_thrusters.emplace_back(rotor_frame_indices.at(i), pinocchio::SE3::Identity(),
                                       (float)(std::abs(m_f_rate)),
                                       ((rotor_direction == 1) ? crocoddyl::DT_CCW : crocoddyl::DT_CW),
                                       (float)thrust_lower_limit, (float)thrust_upper_limit);
  }

  bool fwddyn;
  {
    nhp.getParam("fwddyn", fwddyn);
    std::cout << "fwddyn: " << fwddyn << std::endl;
  }

  DDPHoveringProblem::CostWeight cost_weight;
  {
    double whatever_double;
    nhp.getParam("state_weight", whatever_double);
    cost_weight.state_weight = whatever_double;

    nhp.getParam("state_bound_weight", whatever_double);
    cost_weight.state_bound_weight = whatever_double;

    nhp.getParam("control_weight", whatever_double);
    cost_weight.control_weight = whatever_double;

    std::vector<double> whatever_vector;
    nhp.getParam("x_state_weights", whatever_vector);
    cost_weight.x_weights = Eigen::Map<Eigen::VectorXd>(whatever_vector.data(), whatever_vector.size());

    if (fwddyn)
      nhp.getParam("u_weight_fwddyn", whatever_vector);
    else
      nhp.getParam("u_weight_invdyn", whatever_vector);
    cost_weight.u_weights = Eigen::Map<Eigen::VectorXd>(whatever_vector.data(), whatever_vector.size());

    std::cout << "state_weight: " << cost_weight.state_weight << std::endl;
    std::cout << "state_bound_weight: " << cost_weight.state_bound_weight << std::endl;
    std::cout << "control_weight: " << cost_weight.control_weight << std::endl;
    std::cout << "x_weights: " << cost_weight.x_weights.transpose() << std::endl;
    std::cout << "u_weights: " << cost_weight.u_weights.transpose() << std::endl;
  }
  DDPHoveringProblem::OptimizationParam optimization_param;
  {
    nhp.param("horizon", optimization_param.horizon, optimization_param.horizon);
    nhp.param("dt", optimization_param.dt, optimization_param.dt);
    nhp.param("max_iter", optimization_param.max_iter, optimization_param.max_iter);
    nhp.param("num_threads", optimization_param.num_threads, optimization_param.num_threads);
    std::cout << "horizon: " << optimization_param.horizon << std::endl;
    std::cout << "dt: " << optimization_param.dt << std::endl;
    std::cout << "max_iter: " << optimization_param.max_iter << std::endl;
    std::cout << "num_threads: " << optimization_param.num_threads << std::endl;
  }

  DDPHoveringProblem hovering(pinocchio_model, distributed_thrusters, fwddyn, cost_weight, optimization_param);

  // reference state
  Eigen::VectorXd xref = Eigen::VectorXd::Zero(pinocchio_model->nq + pinocchio_model->nv);
  {
    std::vector<double> whatever(pinocchio_model->nq + pinocchio_model->nv);
    nhp.getParam("x_ref", whatever);
    xref = Eigen::Map<Eigen::VectorXd>(whatever.data(), whatever.size());
    xref.segment<4>(3).normalize();
  }

  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(pinocchio_model->nq + pinocchio_model->nv);
  {
    std::vector<double> whatever;
    nhp.getParam("x_init", whatever);
    x0 = Eigen::Map<Eigen::VectorXd>(whatever.data(), whatever.size());
    x0.segment<4>(3).normalize();
  }

  std::shared_ptr<crocoddyl::ShootingProblem> problem = hovering.createHoveringProblem(x0, xref);
  std::shared_ptr<crocoddyl::SolverAbstract> solver = std::make_shared<crocoddyl::SolverBoxFDDP>(problem);

  int N = hovering.optimization_param_.horizon / hovering.optimization_param_.dt;
  std::vector<Eigen::VectorXd> xs_init(N, x0);
  std::vector<Eigen::VectorXd> us_init = solver->get_problem()->quasiStatic_xs(xs_init);
  xs_init.push_back(x0);

  double noise = 0;
  nhp.getParam("noise", noise);

  ros::Rate rate((int)(1.0 / hovering.optimization_param_.dt));
  while (ros::ok())
  {
    crocoddyl::Timer timer;
    solver->solve(xs_init, us_init, hovering.optimization_param_.max_iter);
    double time = timer.get_duration();
    std::cout << "total calculation time: " << time << "[ms]" << std::endl;
    std::cout << "Number of iterations: " << solver->get_iter() << std::endl;
    std::cout << "time per iterate: " << time / solver->get_iter() << std::endl;
    std::cout << "Total cost: " << solver->get_cost() << std::endl;
    std::cout << "Gradient norm: " << solver->get_stop() << std::endl;

    xs_init = solver->get_xs();
    us_init = solver->get_us();

    Eigen::VectorXd next_x = xs_init.at(1);
    if (noise > 0)
    {
      Eigen::VectorXd noise_dq = Eigen::VectorXd::Random(pinocchio_model->nv) * noise;
      Eigen::VectorXd noise_v = Eigen::VectorXd::Random(pinocchio_model->nv) * noise;
      next_x.head(pinocchio_model->nq) =
          pinocchio::integrate(*pinocchio_model, next_x.head(pinocchio_model->nq), noise_dq);
      next_x.tail(pinocchio_model->nv) += noise_v;
    }
    problem->set_x0(next_x);

    std::cout << "q: " << xs_init.at(1).head(pinocchio_model->nq).transpose() << std::endl;
    std::cout << "v: " << xs_init.at(1).tail(pinocchio_model->nv).transpose() << std::endl;
    if (fwddyn)
    {
      std::cout << "thrust: " << us_init.at(0).head(pinocchio_robot_model->getRotorNum()).transpose() << std::endl;
      std::cout << "joint torque: "
                << us_init.at(0).tail(us_init.at(0).size() - pinocchio_robot_model->getRotorNum()).transpose()
                << std::endl;
    }
    else
    {
      std::cout << "root ddq: " << us_init.at(0).head(6).transpose() << std::endl;
      std::cout << "joint ddq: " << us_init.at(0).tail(us_init.at(0).size() - 6).transpose() << std::endl;
    }
    std::cout << std::endl;

    std::cout << "q final: " << xs_init.back().head(pinocchio_model->nq).transpose() << std::endl;
    std::cout << "v final: " << xs_init.back().tail(pinocchio_model->nv).transpose() << std::endl;
    if (fwddyn)
    {
      std::cout << "thrust final: " << us_init.back().head(pinocchio_robot_model->getRotorNum()).transpose()
                << std::endl;
      std::cout << "joint torque final: "
                << us_init.back().tail(us_init.back().size() - pinocchio_robot_model->getRotorNum()).transpose()
                << std::endl;
    }
    else
    {
      std::cout << "root ddq final: " << us_init.back().head(6).transpose() << std::endl;
      std::cout << "joint ddq final: " << us_init.back().tail(us_init.back().size() - 6).transpose() << std::endl;
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

    // publish optimized trajectory
    for (int i = 0; i < xs_init.size(); i++)
    {
      robot_base_transform.header.frame_id = "world";
      robot_base_transform.child_frame_id = tf::resolve(robot_ns, "root") + "_trajectory_" + std::to_string(i);
      robot_base_transform.transform.translation.x = xs_init[i](0);
      robot_base_transform.transform.translation.y = xs_init[i](1);
      robot_base_transform.transform.translation.z = xs_init[i](2);
      robot_base_transform.transform.rotation.x = xs_init[i](3);
      robot_base_transform.transform.rotation.y = xs_init[i](4);
      robot_base_transform.transform.rotation.z = xs_init[i](5);
      robot_base_transform.transform.rotation.w = xs_init[i](6);

      robot_base_broadcaster.sendTransform(robot_base_transform);
    }

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
