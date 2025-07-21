#include <dynarm/control/joint_trajectory_generator.h>

void aerial_robot_control::jointTrajectoryGenerator::publishDummyJointState()
{
  // for debug: publish current target joint angle
  sensor_msgs::JointState joint_state_msg;
  joint_state_msg.header.stamp = ros::Time::now();
  for (int i = ((pinocchio_robot_model_->getIsFloatingBase()) ? 2 : 1); i < pinocchio_model_->njoints; i++)
  {
    int joint_index_q = pinocchio_model_->joints[pinocchio_model_->getJointId(pinocchio_model_->names[i])].idx_q();
    int joint_index_v = pinocchio_model_->joints[pinocchio_model_->getJointId(pinocchio_model_->names[i])].idx_v();
    joint_state_msg.name.push_back(pinocchio_model_->names[i]);
    joint_state_msg.position.push_back(curr_target_q_(joint_index_q));
    joint_state_msg.effort.push_back(curr_target_tau_(joint_index_v));
  }
  dummy_joint_state_pub_.publish(joint_state_msg);
}

void aerial_robot_control::jointTrajectoryGenerator::update(const ros::TimerEvent& event)
{
  generateEndEffectorTrajectory();
  generateJointTrajectory();

  if (!nonlinear_mode_)
    getGimbalNominalAngles();

  solveInverseDynamics();

  if (nonlinear_mode_)
    updateTargetGimbalAngle();

  stateTransition();
  publish();
  publishDummyJointState();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joint_trajectory_test");
  ros::NodeHandle nh;
  std::shared_ptr<aerial_robot_dynamics::PinocchioRobotModel> pinocchio_robot_model;
  pinocchio_robot_model = std::make_shared<aerial_robot_dynamics::PinocchioRobotModel>(false);  // not floating base
  aerial_robot_control::jointTrajectoryGenerator* joint_trajectory_generator =
      new aerial_robot_control::jointTrajectoryGenerator(nh, pinocchio_robot_model);

  ros::Timer timer =
      nh.createTimer(ros::Duration(0.01), boost::bind(&aerial_robot_control::jointTrajectoryGenerator::update,
                                                      joint_trajectory_generator, _1));
  ros::spin();
  return 0;
}
