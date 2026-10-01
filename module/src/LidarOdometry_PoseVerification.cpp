/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola
 Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: GPL-3.0
 See LICENSE for full license information.
 Closed-source licenses available upon request, for this odometry package
 alone or in combination with the complete SLAM system.
*/
/**
 * @file   LidarOdometry_PoseVerification.cpp
 * @brief  Standstill verification of the accepted pose against nearby registrations, off the scan worker
 */

// This module:
#include <mola_lidar_odometry/LidarOdometry.h>

#include <string>

namespace mola
{

void LidarOdometry::doPoseVerification(
  const mp2p_icp::metric_map_t::ConstPtr & observation, const mp2p_icp::Parameters & icpParams,
  double restingQuality, const mrpt::Clock::time_point & stamp)
{
  const auto & o = params_.pose_verification;
  auto & pv = state_.pose_verification;
  const double t = mrpt::Clock::toDouble(stamp);

  if (!o.enabled || !pv.scheduler.due(state_.last_lidar_pose.mean, t, o)) {
    return;
  }
  if (pose_verification_running_) {
    return;
  }

  PoseVerificationJob job;
  job.observation = observation;
  job.local_map = state_.local_map;
  job.icp_params = icpParams;
  {
    auto lckImu = mrpt::lockHelper(imu_state_mtx_);
    job.icp_variables = state_.parameter_source.getVariableValues();
  }
  job.resting = state_.last_lidar_pose.mean;
  job.resting_quality = restingQuality;
  job.stamp = stamp;
  job.generation = pv.generation;

  pv.scheduler.done(t);
  pose_verification_running_ = true;
  (void)worker_pose_verification_.enqueue([this, job]() {
    try {
      runPoseVerification(job);
    } catch (const std::exception & e) {
      MRPT_LOG_ERROR_FMT("Pose verification failed: %s", e.what());
    }
    pose_verification_running_ = false;
  });
}

void LidarOdometry::runPoseVerification(const PoseVerificationJob & job)
{
  const ProfilerEntry tle(profiler_, "pose_verification");
  const auto & o = params_.pose_verification;

  for (const auto & [name, value] : job.icp_variables) {
    pose_verification_parameter_source_.updateVariable(name, value);
  }
  pose_verification_parameter_source_.realize();

  std::vector<PoseVerificationCandidate> candidates;
  {
    // The registration reads the local map while worker_lidar_ may be
    // updating it; the map content mutex is what excludes the two.
    auto lckMapContents = mrpt::lockHelper(local_map_content_mtx_);
    for (const auto & guess : poseVerificationGuesses(job.resting, o)) {
      mp2p_icp::Results r;
      pose_verification_icp_->align(
        *job.observation, *job.local_map, guess.asTPose(), job.icp_params, r);
      candidates.push_back({r.optimal_tf.mean, r.quality, static_cast<uint32_t>(r.nIterations)});
    }
  }

  const auto verdict = evaluatePoseVerification(
    job.resting, job.resting_quality, params_.min_icp_goodness, candidates, o);

  {
    auto lck = mrpt::lockHelper(state_mtx_);
    auto & pv = state_.pose_verification;
    if (pv.generation != job.generation) {
      MRPT_LOG_INFO("Pose verification: discarded, the vehicle was relocalized meanwhile");
      return;
    }
    pv.last_verdict = verdict;
    pv.last_stamp = job.stamp;
    pv.last_resting_quality = job.resting_quality;
    pv.checks++;
    if (verdict.better_pose_found) {
      pv.better_found++;
    }
  }

  if (!verdict.better_pose_found) {
    std::string elsewhere;
    if (verdict.best_distinct) {
      elsewhere = mrpt::format(", best of those %.3f", verdict.best_distinct->quality);
    }
    MRPT_LOG_INFO_FMT(
      "Pose verification: confirmed at %s (quality %.3f, %zu of %zu guesses settled elsewhere%s)",
      job.resting.asString().c_str(), job.resting_quality, verdict.distinct, candidates.size(),
      elsewhere.c_str());
    return;
  }

  const auto & best = *verdict.best_distinct;
  MRPT_LOG_WARN_FMT(
    "Pose verification: a better registration exists %.2f m from the accepted pose: %s "
    "(quality %.3f in %u iterations, versus %.3f here). Publishing quality 0 until a later "
    "verification passes or a relocalization arrives.",
    (best.pose - job.resting).translation().norm(), best.pose.asString().c_str(), best.quality,
    best.iterations, job.resting_quality);
}

}  // namespace mola
