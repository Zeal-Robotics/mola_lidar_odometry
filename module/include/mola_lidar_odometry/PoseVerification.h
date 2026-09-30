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
 * @file   PoseVerification.h
 * @brief  Standstill check that the accepted pose is the best registration nearby
 */
#pragma once

#include <mrpt/containers/yaml.h>
#include <mrpt/poses/CPose3D.h>

#include <cstdint>
#include <optional>
#include <vector>

namespace mola
{
/** A registration can be self-consistent and still be wrong: on a map with
 *  repeated structure, ICP settles onto a nearby surface that is not the one
 *  the vehicle is on, with a quality that reads as fine, and every scan after
 *  that confirms it because the search radius never reaches the true pose.
 *  The one test that tells the two apart is to start ICP again from guesses
 *  a search radius away and see whether one of them settles somewhere else
 *  with a clearly better quality. That costs several registrations, so it
 *  runs only while the vehicle rests.
 *
 * \ingroup mola_lidar_odometry_grp
 */
struct PoseVerificationOptions
{
  bool enabled = false;

  /** The vehicle rests when the accepted pose stays within these bounds of
   *  where it first came to rest for `still_seconds`. */
  double still_translation = 0.30;  //!< [m]
  double still_rotation_deg = 3.0;
  double still_seconds = 2.0;

  /** Each rest is verified once when it starts, then again this often. */
  double period_seconds = 30.0;

  /** Guesses are placed on a ring around the accepted pose, in the vehicle
   *  frame, at each of the yaw offsets. The radius must reach past the basin
   *  the accepted pose sits in, which is of the order of the matcher's search
   *  radius, or every guess falls straight back into it. */
  double ring_radius = 1.0;  //!< [m]
  uint32_t ring_directions = 8;
  std::vector<double> yaw_offsets_deg = {0.0};

  /** A guess that settles farther than this from the accepted pose found a
   *  different registration. */
  double distinct_translation = 0.5;  //!< [m]
  double distinct_rotation_deg = 5.0;

  /** A different registration is reported as better only when its quality
   *  exceeds the accepted one by this margin, since equal-quality minima on
   *  repeated structure cannot be told apart from a single scan. */
  double quality_margin = 0.05;

  void initialize(const mrpt::containers::yaml & c);
};

/** Decides when a verification is due: once the pose has rested for
 *  `still_seconds`, and again every `period_seconds` while it keeps resting.
 *  Motion beyond the rest bounds starts a new rest. */
class PoseVerificationScheduler
{
public:
  /** Feeds the accepted pose at time `t` [s] and returns true when a
   *  verification is due now. */
  bool due(const mrpt::poses::CPose3D & pose, double t, const PoseVerificationOptions & o);

  /** Records that a verification ran at time `t`. */
  void done(double t);

  void reset();

private:
  std::optional<mrpt::poses::CPose3D> anchor_;
  double anchor_since_ = 0;
  std::optional<double> last_done_;
};

/** Where one guess settled. */
struct PoseVerificationCandidate
{
  mrpt::poses::CPose3D pose;
  double quality = 0;
  uint32_t iterations = 0;
};

struct PoseVerificationVerdict
{
  /** A different registration beat the accepted pose by the margin. */
  bool better_pose_found = false;

  /** Guesses that settled somewhere other than the accepted pose. */
  size_t distinct = 0;

  /** The best of those, if any, whatever its quality. */
  std::optional<PoseVerificationCandidate> best_distinct;
};

/** The initial guesses to verify `resting` from. */
std::vector<mrpt::poses::CPose3D> poseVerificationGuesses(
  const mrpt::poses::CPose3D & resting, const PoseVerificationOptions & o);

/** Judges the guesses' outcomes against the accepted pose and its quality.
 *  `min_quality` is the acceptance threshold every registration must meet. */
PoseVerificationVerdict evaluatePoseVerification(
  const mrpt::poses::CPose3D & resting, double resting_quality, double min_quality,
  const std::vector<PoseVerificationCandidate> & candidates, const PoseVerificationOptions & o);

}  // namespace mola
