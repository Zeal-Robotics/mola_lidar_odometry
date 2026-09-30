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
 * @file   test_pose_verification.cpp
 * @brief  Unit tests for the standstill pose verification policy
 */

#include <gtest/gtest.h>
#include <mola_lidar_odometry/PoseVerification.h>
#include <mrpt/core/exceptions.h>

#include <cmath>

namespace
{
mrpt::poses::CPose3D at(double x, double y = 0, double yaw_deg = 0)
{
  return mrpt::poses::CPose3D::FromXYZYawPitchRoll(x, y, 0, mrpt::DEG2RAD(yaw_deg), 0, 0);
}

mola::PoseVerificationOptions options()
{
  mola::PoseVerificationOptions o;
  o.enabled = true;
  o.still_translation = 0.30;
  o.still_rotation_deg = 3.0;
  o.still_seconds = 2.0;
  o.period_seconds = 30.0;
  o.ring_radius = 1.0;
  o.ring_directions = 8;
  o.distinct_translation = 0.5;
  o.distinct_rotation_deg = 5.0;
  o.quality_margin = 0.05;
  return o;
}

mola::PoseVerificationCandidate settled(const mrpt::poses::CPose3D & p, double quality)
{
  mola::PoseVerificationCandidate c;
  c.pose = p;
  c.quality = quality;
  return c;
}
}  // namespace

TEST(PoseVerificationScheduler, DueOnceRestedThenEveryPeriod)
{
  const auto o = options();
  mola::PoseVerificationScheduler s;

  EXPECT_FALSE(s.due(at(0), 0.0, o));
  EXPECT_FALSE(s.due(at(0.1), 1.0, o));
  EXPECT_TRUE(s.due(at(0.2), 2.0, o));
  s.done(2.0);
  EXPECT_FALSE(s.due(at(0.1), 10.0, o));
  EXPECT_TRUE(s.due(at(0.1), 32.0, o));
}

TEST(PoseVerificationScheduler, MotionStartsANewRest)
{
  const auto o = options();
  mola::PoseVerificationScheduler s;

  EXPECT_FALSE(s.due(at(0), 0.0, o));
  s.done(2.0);
  EXPECT_FALSE(s.due(at(5.0), 3.0, o));
  EXPECT_FALSE(s.due(at(5.1), 4.0, o));
  EXPECT_TRUE(s.due(at(5.1), 5.0, o));

  EXPECT_FALSE(s.due(at(5.1, 0, 10.0), 6.0, o));
  EXPECT_FALSE(s.due(at(5.1, 0, 10.0), 7.0, o));
  EXPECT_TRUE(s.due(at(5.1, 0, 10.0), 8.0, o));
}

TEST(PoseVerificationScheduler, ResetForgetsTheRest)
{
  const auto o = options();
  mola::PoseVerificationScheduler s;

  EXPECT_FALSE(s.due(at(0), 0.0, o));
  s.reset();
  EXPECT_FALSE(s.due(at(0), 5.0, o));
  EXPECT_FALSE(s.due(at(0), 6.0, o));
  EXPECT_TRUE(s.due(at(0), 7.0, o));
}

TEST(PoseVerificationGuesses, RingAroundTheRestingPoseInItsFrame)
{
  auto o = options();
  o.yaw_offsets_deg = {0.0, 20.0};
  const auto resting = at(10, 20, 90);

  const auto guesses = mola::poseVerificationGuesses(resting, o);
  ASSERT_EQ(guesses.size(), 16u);
  for (const auto & g : guesses) {
    const auto delta = g - resting;
    EXPECT_NEAR(delta.translation().norm(), 1.0, 1e-9);
    EXPECT_NEAR(delta.z(), 0.0, 1e-9);
  }
  EXPECT_NEAR(guesses[0].x(), 10.0, 1e-9);
  EXPECT_NEAR(guesses[0].y(), 21.0, 1e-9);
  EXPECT_NEAR(mrpt::RAD2DEG(guesses[1].yaw()), 110.0, 1e-9);
}

TEST(PoseVerificationVerdict, GuessesFallingBackConfirmThePose)
{
  const auto o = options();
  const auto resting = at(0);
  const std::vector<mola::PoseVerificationCandidate> back = {
    settled(at(0.1), 0.9), settled(at(-0.2, 0.1), 0.95), settled(at(0, 0, 2.0), 0.9)};

  const auto v = mola::evaluatePoseVerification(resting, 0.9, 0.5, back, o);
  EXPECT_FALSE(v.better_pose_found);
  EXPECT_EQ(v.distinct, 0u);
  EXPECT_FALSE(v.best_distinct.has_value());
}

TEST(PoseVerificationVerdict, ADistinctlyBetterRegistrationIsReported)
{
  const auto o = options();
  const auto resting = at(0);
  const std::vector<mola::PoseVerificationCandidate> found = {
    settled(at(0.1), 0.86), settled(at(0, -1.7), 0.99), settled(at(0, -1.6), 0.98)};

  const auto v = mola::evaluatePoseVerification(resting, 0.86, 0.5, found, o);
  EXPECT_TRUE(v.better_pose_found);
  EXPECT_EQ(v.distinct, 2u);
  ASSERT_TRUE(v.best_distinct.has_value());
  EXPECT_NEAR(v.best_distinct->pose.y(), -1.7, 1e-9);
  EXPECT_NEAR(v.best_distinct->quality, 0.99, 1e-9);
}

TEST(PoseVerificationVerdict, EqualMinimaAndPoorOnesAreNotBetter)
{
  const auto o = options();
  const auto resting = at(0);

  const std::vector<mola::PoseVerificationCandidate> equal = {settled(at(0, 2.0), 0.92)};
  const auto v1 = mola::evaluatePoseVerification(resting, 0.90, 0.5, equal, o);
  EXPECT_FALSE(v1.better_pose_found);
  EXPECT_EQ(v1.distinct, 1u);
  ASSERT_TRUE(v1.best_distinct.has_value());

  const std::vector<mola::PoseVerificationCandidate> poor = {settled(at(0, 2.0), 0.45)};
  const auto v2 = mola::evaluatePoseVerification(resting, 0.30, 0.5, poor, o);
  EXPECT_FALSE(v2.better_pose_found);

  const std::vector<mola::PoseVerificationCandidate> turned = {settled(at(0, 0, 8.0), 0.99)};
  const auto v3 = mola::evaluatePoseVerification(resting, 0.90, 0.5, turned, o);
  EXPECT_TRUE(v3.better_pose_found);
}

TEST(PoseVerificationOptions, LoadsListsAndRefusesARingInsideTheDistinctRadius)
{
  mrpt::containers::yaml c = mrpt::containers::yaml::FromText(R"(
enabled: true
ring_radius: 1.5
yaw_offsets_deg: [0, -15, 15]
)");
  mola::PoseVerificationOptions o;
  o.initialize(c);
  EXPECT_TRUE(o.enabled);
  EXPECT_DOUBLE_EQ(o.ring_radius, 1.5);
  ASSERT_EQ(o.yaw_offsets_deg.size(), 3u);
  EXPECT_DOUBLE_EQ(o.yaw_offsets_deg[1], -15.0);

  mrpt::containers::yaml bad = mrpt::containers::yaml::FromText(R"(
ring_radius: 0.4
distinct_translation: 0.5
)");
  mola::PoseVerificationOptions o2;
  EXPECT_THROW(o2.initialize(bad), std::exception);
}
