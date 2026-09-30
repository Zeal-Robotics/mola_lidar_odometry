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
 * @brief  Standstill verification of the accepted pose against nearby registrations
 */

// This module:
#include <mola_lidar_odometry/LidarOdometry.h>

#include <string>

namespace mola
{

void LidarOdometry::doPoseVerification(
  const mp2p_icp::metric_map_t & observation, mp2p_icp::ICP & icp,
  const mp2p_icp::Parameters & icpParams, double restingQuality,
  const mrpt::Clock::time_point & stamp)
{
  const auto & o = params_.pose_verification;
  auto & pv = state_.pose_verification;
  const double t = mrpt::Clock::toDouble(stamp);

  if (!o.enabled || !pv.scheduler.due(state_.last_lidar_pose.mean, t, o)) {
    return;
  }

  const ProfilerEntry tle(profiler_, "onLidar.3b.pose_verification");

  const auto & resting = state_.last_lidar_pose.mean;

  std::vector<PoseVerificationCandidate> candidates;
  for (const auto & guess : poseVerificationGuesses(resting, o)) {
    mp2p_icp::Results r;
    icp.align(observation, *state_.local_map, guess.asTPose(), icpParams, r);
    candidates.push_back({r.optimal_tf.mean, r.quality, static_cast<uint32_t>(r.nIterations)});
  }

  pv.last_verdict =
    evaluatePoseVerification(resting, restingQuality, params_.min_icp_goodness, candidates, o);
  pv.last_stamp = stamp;
  pv.last_resting_quality = restingQuality;
  pv.checks++;
  pv.scheduler.done(t);

  if (!pv.last_verdict.better_pose_found) {
    std::string elsewhere;
    if (pv.last_verdict.best_distinct) {
      elsewhere = mrpt::format(", best of those %.3f", pv.last_verdict.best_distinct->quality);
    }
    MRPT_LOG_INFO_FMT(
      "Pose verification: confirmed at %s (quality %.3f, %zu of %zu guesses settled elsewhere%s)",
      resting.asString().c_str(), restingQuality, pv.last_verdict.distinct, candidates.size(),
      elsewhere.c_str());
    return;
  }

  pv.better_found++;
  const auto & best = *pv.last_verdict.best_distinct;
  MRPT_LOG_WARN_FMT(
    "Pose verification: a better registration exists %.2f m from the accepted pose: %s "
    "(quality %.3f in %u iterations, versus %.3f here). Publishing quality 0 until a later "
    "verification passes or a relocalization arrives.",
    (best.pose - resting).translation().norm(), best.pose.asString().c_str(), best.quality,
    best.iterations, restingQuality);
}

}  // namespace mola
