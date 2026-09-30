/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola
 Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: GPL-3.0
 See LICENSE for full license information.
*/
/**
 * @file   PoseVerification.cpp
 * @brief  Standstill check that the accepted pose is the best registration nearby
 */

#include <mola_lidar_odometry/PoseVerification.h>
#include <mola_yaml/yaml_helpers.h>
#include <mrpt/core/bits_math.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/math/wrap2pi.h>

#include <cmath>

namespace mola
{

namespace
{
bool withinBounds(
  const mrpt::poses::CPose3D & a, const mrpt::poses::CPose3D & b, double translation,
  double rotation_deg)
{
  const auto delta = b - a;
  return delta.translation().norm() <= translation &&
         std::abs(mrpt::math::wrapToPi(delta.yaw())) <= mrpt::DEG2RAD(rotation_deg);
}
}  // namespace

void PoseVerificationOptions::initialize(const mrpt::containers::yaml & cfg)
{
  YAML_LOAD_OPT(enabled, bool);
  YAML_LOAD_OPT(still_translation, double);
  YAML_LOAD_OPT(still_rotation_deg, double);
  YAML_LOAD_OPT(still_seconds, double);
  YAML_LOAD_OPT(period_seconds, double);
  YAML_LOAD_OPT(ring_radius, double);
  YAML_LOAD_OPT(ring_directions, uint32_t);
  YAML_LOAD_OPT(distinct_translation, double);
  YAML_LOAD_OPT(distinct_rotation_deg, double);
  YAML_LOAD_OPT(quality_margin, double);

  if (cfg.has("yaw_offsets_deg")) {
    yaw_offsets_deg.clear();
    const auto offsets = cfg["yaw_offsets_deg"];
    for (const auto & v : offsets.asSequence()) {
      yaw_offsets_deg.push_back(v.as<double>());
    }
  }

  ASSERTMSG_(ring_directions >= 1, "pose_verification: ring_directions must be >= 1");
  ASSERTMSG_(!yaw_offsets_deg.empty(), "pose_verification: yaw_offsets_deg must not be empty");
  ASSERTMSG_(
    ring_radius > distinct_translation,
    "pose_verification: ring_radius must exceed distinct_translation, or a guess that never "
    "moved would count as a different registration");
}

bool PoseVerificationScheduler::due(
  const mrpt::poses::CPose3D & pose, double t, const PoseVerificationOptions & o)
{
  if (!anchor_ || !withinBounds(*anchor_, pose, o.still_translation, o.still_rotation_deg)) {
    anchor_ = pose;
    anchor_since_ = t;
    last_done_.reset();
    return false;
  }
  if (t - anchor_since_ < o.still_seconds) {
    return false;
  }
  return !last_done_ || t - *last_done_ >= o.period_seconds;
}

void PoseVerificationScheduler::done(double t) { last_done_ = t; }

void PoseVerificationScheduler::reset()
{
  anchor_.reset();
  last_done_.reset();
}

std::vector<mrpt::poses::CPose3D> poseVerificationGuesses(
  const mrpt::poses::CPose3D & resting, const PoseVerificationOptions & o)
{
  std::vector<mrpt::poses::CPose3D> guesses;
  guesses.reserve(o.ring_directions * o.yaw_offsets_deg.size());
  for (uint32_t i = 0; i < o.ring_directions; i++) {
    const double heading = 2.0 * M_PI * i / o.ring_directions;
    for (const double yaw_deg : o.yaw_offsets_deg) {
      const auto offset = mrpt::poses::CPose3D::FromXYZYawPitchRoll(
        o.ring_radius * std::cos(heading), o.ring_radius * std::sin(heading), 0,
        mrpt::DEG2RAD(yaw_deg), 0, 0);
      guesses.push_back(resting + offset);
    }
  }
  return guesses;
}

PoseVerificationVerdict evaluatePoseVerification(
  const mrpt::poses::CPose3D & resting, double resting_quality, double min_quality,
  const std::vector<PoseVerificationCandidate> & candidates, const PoseVerificationOptions & o)
{
  PoseVerificationVerdict v;
  for (const auto & c : candidates) {
    if (withinBounds(resting, c.pose, o.distinct_translation, o.distinct_rotation_deg)) {
      continue;
    }
    v.distinct++;
    if (!v.best_distinct || c.quality > v.best_distinct->quality) {
      v.best_distinct = c;
    }
  }
  v.better_pose_found = v.best_distinct && v.best_distinct->quality >= min_quality &&
                        v.best_distinct->quality >= resting_quality + o.quality_margin;
  return v;
}

}  // namespace mola
