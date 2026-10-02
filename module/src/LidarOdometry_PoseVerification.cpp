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
  job.icp = pose_verification_icp_;
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
  job.generation = pose_verification_generation_;

  pv.scheduler.done(t);
  pose_verification_running_ = true;
  (void)worker_pose_verification_.enqueue([this, job]() {
    // Cleared however the run ends: shutdown waits on this flag.
    struct ClearOnExit
    {
      std::atomic<bool> & flag;
      ~ClearOnExit() { flag = false; }
    };
    const ClearOnExit cleared{pose_verification_running_};
    try {
      runPoseVerification(job);
    } catch (const std::exception & e) {
      MRPT_LOG_ERROR_FMT("Pose verification failed: %s", e.what());
    } catch (...) {
      MRPT_LOG_ERROR("Pose verification failed with an unknown exception");
    }
  });
}

void LidarOdometry::runPoseVerification(const PoseVerificationJob & job)
{
  const ProfilerEntry tle(profiler_, "pose_verification");
  const auto & o = params_.pose_verification;
  const double startTime = mrpt::Clock::nowDouble();

  for (const auto & [name, value] : job.icp_variables) {
    job.icp->parameter_source.updateVariable(name, value);
  }
  job.icp->parameter_source.realize();

  std::vector<PoseVerificationCandidate> candidates;
  for (const auto & guess : poseVerificationGuesses(job.resting, o)) {
    // Excludes a change to the map's contents during a registration, and is
    // released between guesses so whoever makes one waits for a guess, not the
    // ring. The scan worker registers against the same map meanwhile, and the
    // map keeps one ICP search submap, rebuilt for the keyframes nearest the
    // initial guess of whichever registration asked last. That is sound only
    // while both select the same keyframes, which a loaded localization map,
    // one keyframe, guarantees; the verification is off while mapping.
    auto lckMapContents = mrpt::lockHelper(local_map_content_mtx_);
    mp2p_icp::Results r;
    job.icp->icp->align(*job.observation, *job.local_map, guess.asTPose(), job.icp_params, r);
    candidates.push_back({r.optimal_tf.mean, r.quality, static_cast<uint32_t>(r.nIterations)});
  }

  const auto verdict = evaluatePoseVerification(
    job.resting, job.resting_quality, params_.min_icp_goodness, candidates, o);

  {
    auto lck = mrpt::lockHelper(state_mtx_);
    auto & pv = state_.pose_verification;
    if (pose_verification_generation_ != job.generation) {
      MRPT_LOG_INFO("Pose verification: discarded, the pose it judged was replaced meanwhile");
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

  const std::string cost = mrpt::format("took %.2f s", mrpt::Clock::nowDouble() - startTime);

  if (!verdict.better_pose_found) {
    std::string elsewhere;
    if (verdict.best_distinct) {
      elsewhere = mrpt::format(", best of those %.3f", verdict.best_distinct->quality);
    }
    MRPT_LOG_INFO_FMT(
      "Pose verification: confirmed at %s (quality %.3f, %zu of %zu guesses settled elsewhere%s; "
      "%s)",
      job.resting.asString().c_str(), job.resting_quality, verdict.distinct, candidates.size(),
      elsewhere.c_str(), cost.c_str());
    return;
  }

  const auto & best = *verdict.best_distinct;
  MRPT_LOG_WARN_FMT(
    "Pose verification: a better registration exists %.2f m from the accepted pose: %s "
    "(quality %.3f in %u iterations, versus %.3f here; %s). Publishing quality 0 until a later "
    "verification passes or a relocalization arrives.",
    (best.pose - job.resting).translation().norm(), best.pose.asString().c_str(), best.quality,
    best.iterations, job.resting_quality, cost.c_str());
}

}  // namespace mola
