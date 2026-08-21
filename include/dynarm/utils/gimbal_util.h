// -*- mode: c++ -*-
// Ported from dynleg_kinematics/gimbal_util.hpp of jsk_dynleg_robot.
#pragma once

#include <pinocchio/fwd.hpp>  // should be included before any other pinocchio headers
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody.hpp>

#include <Eigen/Core>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <iostream>
#include <limits>
#include <map>
#include <string>
#include <vector>

namespace dynarm
{
template <typename Scalar>
struct GimbalGroupTpl
{
  int id = -1;
  pinocchio::FrameIndex rotor_frame_id = 0;
  std::string rotor_frame_name;
  std::vector<std::string> joint_names;
  std::vector<int> idx_qs;
  std::vector<int> idx_vs;
  std::vector<Scalar> lower_limits;
  std::vector<Scalar> upper_limits;
};

template <typename Scalar>
struct AlignUpwardOptionsTpl
{
  Scalar seed_interval = Scalar(M_PI / 2.0);
  std::vector<Scalar> extra_seeds;
  int max_iterations = 200;
  int max_damping_steps = 40;
  Scalar initial_damping = Scalar(1e-3);
  Scalar min_damping = Scalar(1e-9);
  Scalar max_damping = Scalar(1e9);
  Scalar improvement_tolerance = Scalar(10) * Eigen::NumTraits<Scalar>::epsilon();
  int max_seed_combinations = 256;
  bool verbose = true;
};

inline int parseIndexAfterKeyword(const std::string& name, const std::string& keyword)
{
  const std::size_t keyword_pos = name.find(keyword);
  if (keyword_pos == std::string::npos)
    return -1;

  std::size_t digits_begin = keyword_pos + keyword.size();
  std::size_t digits_end = digits_begin;
  while (digits_end < name.size() && std::isdigit(static_cast<unsigned char>(name[digits_end])))
    digits_end++;
  if (digits_end == digits_begin)
    return -1;

  return std::stoi(name.substr(digits_begin, digits_end - digits_begin));
}

template <typename Scalar>
std::map<int, pinocchio::FrameIndex> findRotorFrames(const pinocchio::ModelTpl<Scalar>& model)
{
  std::map<int, pinocchio::FrameIndex> rotor_frames;
  for (pinocchio::FrameIndex i = 0; i < static_cast<pinocchio::FrameIndex>(model.nframes); i++)
  {
    const auto& frame = model.frames[i];
    if (frame.type != pinocchio::JOINT && frame.type != pinocchio::FIXED_JOINT)
      continue;

    const int id = parseIndexAfterKeyword(frame.name, "rotor");
    if (id < 0)
      continue;

    if (!rotor_frames.emplace(id, i).second)
      std::cerr << "gimbal init: frame " << frame.name << " duplicates rotor index " << id << ", ignored" << std::endl;
  }
  return rotor_frames;
}

// Every rotor in the model, each with the gimbal joints that steer it (possibly none).
// This is the rotor-driven traversal: a fixed thruster is a rotor with an empty joint list,
// which is what lets the layout describe designs where the gimbal itself is a design variable.
// findGimbalGroups is the same traversal restricted to the rotors that do have a gimbal.
template <typename Scalar>
std::vector<GimbalGroupTpl<Scalar>> findRotorGroups(const pinocchio::ModelTpl<Scalar>& model, bool verbose = false)
{
  const std::map<int, pinocchio::FrameIndex> rotor_frames = findRotorFrames(model);

  // one group per rotor up front, so a rotor with no gimbal joint still appears
  std::vector<GimbalGroupTpl<Scalar>> groups;
  groups.reserve(rotor_frames.size());
  for (const auto& rotor_frame : rotor_frames)
  {
    GimbalGroupTpl<Scalar> group;
    group.id = rotor_frame.first;
    group.rotor_frame_id = rotor_frame.second;
    group.rotor_frame_name = model.frames[rotor_frame.second].name;
    groups.push_back(group);
  }

  for (pinocchio::JointIndex i = 1; i < static_cast<pinocchio::JointIndex>(model.njoints); i++)
  {
    const std::string& joint_name = model.names[i];
    const int id = parseIndexAfterKeyword(joint_name, "gimbal");
    if (id < 0)
    {
      if (joint_name.find("gimbal") != std::string::npos)
        std::cerr << "gimbal init: joint " << joint_name << " has no number after \"gimbal\", skipped" << std::endl;
      continue;
    }

    const auto rotor_frame = rotor_frames.find(id);
    if (rotor_frame == rotor_frames.end())
    {
      std::cerr << "gimbal init: joint " << joint_name << " has no matching rotor" << id << " frame, skipped"
                << std::endl;
      continue;
    }

    const auto& joint = model.joints[i];
    if (joint.nq() != 1 || joint.nv() != 1)
    {
      std::cerr << "gimbal init: joint " << joint_name << " is " << model.joints[i].shortname() << " (nq " << joint.nq()
                << "), only single-axis gimbals are supported, skipped" << std::endl;
      continue;
    }

    const pinocchio::JointIndex rotor_parent_joint = model.frames[rotor_frame->second].parentJoint;
    const auto& support = model.supports[rotor_parent_joint];
    if (std::find(support.begin(), support.end(), i) == support.end())
    {
      std::cerr << "gimbal init: joint " << joint_name << " is not an ancestor of "
                << model.frames[rotor_frame->second].name << ", skipped" << std::endl;
      continue;
    }

    // the group exists: rotor_frames.find(id) above already succeeded
    auto group =
        std::find_if(groups.begin(), groups.end(), [id](const GimbalGroupTpl<Scalar>& g) { return g.id == id; });

    group->joint_names.push_back(joint_name);
    group->idx_qs.push_back(joint.idx_q());
    group->idx_vs.push_back(joint.idx_v());
    group->lower_limits.push_back(model.lowerPositionLimit(joint.idx_q()));
    group->upper_limits.push_back(model.upperPositionLimit(joint.idx_q()));
  }

  std::sort(groups.begin(), groups.end(),
            [](const GimbalGroupTpl<Scalar>& a, const GimbalGroupTpl<Scalar>& b) { return a.id < b.id; });

  if (verbose)
  {
    for (const auto& group : groups)
    {
      std::cout << "rotor group " << group.id << " -> " << group.rotor_frame_name << ":";
      if (group.joint_names.empty())
        std::cout << " (no gimbal)";
      for (std::size_t j = 0; j < group.joint_names.size(); j++)
        std::cout << " " << group.joint_names[j] << "(q " << group.idx_qs[j] << ")";
      std::cout << std::endl;
    }
  }

  return groups;
}

// The rotors that are actually steerable. Kept as a separate view because the alignment
// helpers below have nothing to do on a rotor without a gimbal joint.
template <typename Scalar>
std::vector<GimbalGroupTpl<Scalar>> findGimbalGroups(const pinocchio::ModelTpl<Scalar>& model, bool verbose = true)
{
  std::vector<GimbalGroupTpl<Scalar>> groups = findRotorGroups(model, false);
  groups.erase(std::remove_if(groups.begin(), groups.end(),
                              [](const GimbalGroupTpl<Scalar>& g) { return g.joint_names.empty(); }),
               groups.end());

  if (verbose)
  {
    for (const auto& group : groups)
    {
      std::cout << "gimbal group " << group.id << " -> " << group.rotor_frame_name << ":";
      for (std::size_t j = 0; j < group.joint_names.size(); j++)
        std::cout << " " << group.joint_names[j] << "(q " << group.idx_qs[j] << ")";
      std::cout << std::endl;
    }
  }

  return groups;
}

template <typename Scalar>
Scalar rotorUpwardAlignment(const pinocchio::ModelTpl<Scalar>& model, pinocchio::DataTpl<Scalar>& data,
                            const typename pinocchio::ModelTpl<Scalar>::ConfigVectorType& q,
                            pinocchio::FrameIndex rotor_frame_id)
{
  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacement(model, data, rotor_frame_id);
  return data.oMf[rotor_frame_id].rotation()(2, 2);
}

template <typename Scalar>
Scalar alignRotorUpward(const pinocchio::ModelTpl<Scalar>& model, pinocchio::DataTpl<Scalar>& data,
                        typename pinocchio::ModelTpl<Scalar>::ConfigVectorType& q, const GimbalGroupTpl<Scalar>& group,
                        const AlignUpwardOptionsTpl<Scalar>& options)
{
  typedef typename pinocchio::ModelTpl<Scalar>::ConfigVectorType ConfigVectorType;
  typedef Eigen::Matrix<Scalar, 3, 1> Vector3s;
  typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1> VectorXs;
  typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> MatrixXs;
  typedef Eigen::Matrix<Scalar, 3, Eigen::Dynamic> Matrix3xs;
  typedef Eigen::Matrix<Scalar, 6, Eigen::Dynamic> Matrix6xs;

  const int dofs = static_cast<int>(group.idx_qs.size());
  const auto clampToLimits = [&group, dofs](const VectorXs& angles) {
    VectorXs clamped(dofs);
    for (int j = 0; j < dofs; j++)
      clamped(j) = std::min(std::max(angles(j), group.lower_limits[j]), group.upper_limits[j]);
    return clamped;
  };
  const auto write = [&group, dofs](ConfigVectorType& target, const VectorXs& angles) {
    for (int j = 0; j < dofs; j++)
      target(group.idx_qs[j]) = angles(j);
  };

  std::vector<std::vector<Scalar>> seeds(dofs);
  const int grid_size = std::max(1, static_cast<int>(std::lround(Scalar(2 * M_PI) / options.seed_interval)));
  for (int j = 0; j < dofs; j++)
  {
    std::vector<Scalar> candidates = options.extra_seeds;
    candidates.push_back(q(group.idx_qs[j]));
    for (int k = 0; k < grid_size; k++)
    {
      Scalar angle = Scalar(k) * options.seed_interval;
      if (angle > Scalar(M_PI))
        angle -= Scalar(2 * M_PI);  // keep the grid in (-pi, pi], the shortest way to each direction
      candidates.push_back(angle);
    }

    for (const Scalar candidate : candidates)
    {
      if (candidate < group.lower_limits[j] || candidate > group.upper_limits[j])
        continue;
      if (std::find_if(seeds[j].begin(), seeds[j].end(), [&](Scalar s) {
            return std::abs(s - candidate) < options.improvement_tolerance;
          }) == seeds[j].end())
        seeds[j].push_back(candidate);
    }
    if (seeds[j].empty())  // every seed is out of range, so start from the closest reachable angle
      seeds[j].push_back(std::min(std::max(Scalar(0), group.lower_limits[j]), group.upper_limits[j]));
  }

  long combinations = 1;
  for (int j = 0; j < dofs; j++)
    combinations *= static_cast<long>(seeds[j].size());
  if (combinations > options.max_seed_combinations)
  {
    if (options.verbose)
      std::cerr << "gimbal init: " << group.rotor_frame_name << " has " << combinations
                << " seed combinations, only the first " << options.max_seed_combinations << " are tried" << std::endl;
    combinations = options.max_seed_combinations;
  }

  ConfigVectorType q_work = q;
  Matrix6xs jacobian(6, model.nv);
  VectorXs best_angles(dofs);
  Scalar best_alignment = -std::numeric_limits<Scalar>::infinity();

  for (long combination = 0; combination < combinations; combination++)
  {
    // Odometer over the seed lists, last joint fastest, so combination 0 is the first seed of every joint.
    VectorXs angles(dofs);
    long remainder = combination;
    for (int j = dofs - 1; j >= 0; j--)
    {
      angles(j) = seeds[j][remainder % static_cast<long>(seeds[j].size())];
      remainder /= static_cast<long>(seeds[j].size());
    }

    write(q_work, angles);
    Scalar alignment = rotorUpwardAlignment(model, data, q_work, group.rotor_frame_id);
    Scalar damping = options.initial_damping;

    for (int iteration = 0; iteration < options.max_iterations; iteration++)
    {
      // The rotor z axis has to reach the world vertical, so the residual to drive to zero is e_z - R e_z, whose
      // squared norm is 2 (1 - alignment): descending it is exactly maximizing the alignment.
      // d(R e_z)/dq_j = omega_j x (R e_z), with omega_j the world angular velocity the joint generates.
      jacobian.setZero();
      pinocchio::computeFrameJacobian(model, data, q_work, group.rotor_frame_id, pinocchio::LOCAL_WORLD_ALIGNED,
                                      jacobian);
      pinocchio::updateFramePlacement(model, data, group.rotor_frame_id);
      const Vector3s rotor_z = data.oMf[group.rotor_frame_id].rotation().col(2);
      const Vector3s residual = Vector3s::UnitZ() - rotor_z;

      Matrix3xs residual_jacobian(3, dofs);
      for (int j = 0; j < dofs; j++)
      {
        const Vector3s omega = jacobian.template bottomRows<3>().col(group.idx_vs[j]);
        residual_jacobian.col(j) = -omega.cross(rotor_z);
      }
      const VectorXs gradient = residual_jacobian.transpose() * residual;
      if (gradient.norm() < options.improvement_tolerance)
        break;

      const MatrixXs hessian = residual_jacobian.transpose() * residual_jacobian;
      bool improved = false;
      for (int damping_step = 0; damping_step < options.max_damping_steps; damping_step++)
      {
        const MatrixXs damped = hessian + damping * MatrixXs::Identity(dofs, dofs);
        const VectorXs candidate = clampToLimits(angles - damped.ldlt().solve(gradient));
        write(q_work, candidate);
        const Scalar candidate_alignment = rotorUpwardAlignment(model, data, q_work, group.rotor_frame_id);
        if (candidate_alignment > alignment + options.improvement_tolerance)
        {
          angles = candidate;
          alignment = candidate_alignment;
          damping = std::max(options.min_damping, damping * Scalar(0.5));
          improved = true;
          break;
        }
        damping = std::min(options.max_damping, damping * Scalar(10));
      }
      if (!improved)  // no amount of damping improves on this any more: the local maximum is reached
      {
        write(q_work, angles);
        break;
      }
    }

    if (alignment > best_alignment + options.improvement_tolerance)
    {
      best_alignment = alignment;
      best_angles = angles;
    }
  }

  for (int j = 0; j < dofs; j++)
  {
    Scalar normalized = best_angles(j);
    while (normalized > Scalar(M_PI))
      normalized -= Scalar(2 * M_PI);
    while (normalized < -Scalar(M_PI))
      normalized += Scalar(2 * M_PI);
    if (normalized >= group.lower_limits[j] && normalized <= group.upper_limits[j])
      best_angles(j) = normalized;
  }

  write(q, best_angles);
  if (options.verbose)
  {
    std::cout << "gimbal init: " << group.rotor_frame_name << " best alignment " << best_alignment << " ("
              << std::acos(std::min(std::max(best_alignment, Scalar(-1)), Scalar(1))) * Scalar(180.0 / M_PI)
              << " deg from vertical) at";
    for (int j = 0; j < dofs; j++)
      std::cout << " " << group.joint_names[j] << "=" << best_angles(j);
    std::cout << std::endl;
  }
  return best_alignment;
}

template <typename Scalar>
void alignGimbalsUpward(const pinocchio::ModelTpl<Scalar>& model, pinocchio::DataTpl<Scalar>& data,
                        typename pinocchio::ModelTpl<Scalar>::ConfigVectorType& q,
                        const AlignUpwardOptionsTpl<Scalar>& options = AlignUpwardOptionsTpl<Scalar>())
{
  const std::vector<GimbalGroupTpl<Scalar>> groups = findGimbalGroups(model, options.verbose);
  if (groups.empty())
  {
    if (options.verbose)
      std::cout << "gimbal init: no gimbal joint found, initial configuration left as is" << std::endl;
    return;
  }

  for (const auto& group : groups)
    alignRotorUpward(model, data, q, group, options);
}

template <typename Scalar>
void alignGimbalsUpward(const pinocchio::ModelTpl<Scalar>& model,
                        typename pinocchio::ModelTpl<Scalar>::ConfigVectorType& q,
                        const AlignUpwardOptionsTpl<Scalar>& options = AlignUpwardOptionsTpl<Scalar>())
{
  pinocchio::DataTpl<Scalar> data(model);
  alignGimbalsUpward(model, data, q, options);
}

}  // namespace dynarm
