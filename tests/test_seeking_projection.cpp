// test_seeking_projection.cpp
//
// Direct coverage of the directional seeking projection (Algorithm 1),
// shared by the batch simulator and the ROS 2 / Gazebo node through
// adaptive::project_seeking_velocity (Math.hpp): the clip must engage only
// against the current excitation half-period's push direction, at exactly
// the allowance b = A e^{-lambda T_bar} / pi, leave x and aligned seeking
// untouched, alternate sign with the half-period index, and loosen as the
// decay slows. Guards the two call sites against silent divergence.

#include <cmath>

#include "adaptive_localization/Math.hpp"
#include "test_framework.hpp"

using namespace adaptive;

namespace {

// Implemented closed-loop constants; the expected allowance is evaluated
// the same way the helper evaluates it.
constexpr double kAmplitude = 0.25;
constexpr double kDecay = 2.0;
constexpr double kSamplePeriod = 0.08;
constexpr double kFrequency = 0.45;

double allowance() {
    return kAmplitude * std::exp(-kDecay * kSamplePeriod) / kPi;
}

double half_period() {
    return kPi / kFrequency;
}

}  // namespace

ADAPTIVE_TEST(projection_clips_opposing_seeking_in_even_half_period) {
    // At t = 0 the push direction is +e_y: only seeking that opposes it
    // (negative y) is clipped, exactly at -b, and x passes through.
    const Vec2 clipped = project_seeking_velocity(
        {0.3, -5.0}, kAmplitude, kDecay, kSamplePeriod, kFrequency, 0.0);
    EXPECT_NEAR(clipped.x, 0.3, 0.0);
    EXPECT_NEAR(clipped.y, -allowance(), 1e-15);

    const Vec2 inside = project_seeking_velocity(
        {0.3, -0.5 * allowance()}, kAmplitude, kDecay, kSamplePeriod, kFrequency, 0.0);
    EXPECT_NEAR(inside.y, -0.5 * allowance(), 0.0);

    const Vec2 aligned = project_seeking_velocity(
        {0.3, 2.0}, kAmplitude, kDecay, kSamplePeriod, kFrequency, 0.0);
    EXPECT_NEAR(aligned.y, 2.0, 0.0);
}

ADAPTIVE_TEST(projection_flips_sign_in_odd_half_period) {
    // In the second half-period the push direction is -e_y: positive y is
    // clipped at +b and negative y passes through.
    const Vec2 clipped = project_seeking_velocity(
        {-0.4, 5.0}, kAmplitude, kDecay, kSamplePeriod, kFrequency, 1.5 * half_period());
    EXPECT_NEAR(clipped.x, -0.4, 0.0);
    EXPECT_NEAR(clipped.y, allowance(), 1e-15);

    const Vec2 aligned = project_seeking_velocity(
        {-0.4, -5.0}, kAmplitude, kDecay, kSamplePeriod, kFrequency, 1.5 * half_period());
    EXPECT_NEAR(aligned.y, -5.0, 0.0);
}

ADAPTIVE_TEST(projection_sign_alternates_with_half_period_index) {
    // Just before the first boundary the even sign still applies, and it
    // returns in the third half-period.
    const Vec2 pre_boundary = project_seeking_velocity(
        {0.0, -5.0}, kAmplitude, kDecay, kSamplePeriod, kFrequency,
        half_period() - 1e-6);
    EXPECT_NEAR(pre_boundary.y, -allowance(), 1e-15);

    const Vec2 third = project_seeking_velocity(
        {0.0, -5.0}, kAmplitude, kDecay, kSamplePeriod, kFrequency,
        2.5 * half_period());
    EXPECT_NEAR(third.y, -allowance(), 1e-15);
}

ADAPTIVE_TEST(projection_allowance_grows_as_decay_slows) {
    const Vec2 slow = project_seeking_velocity(
        {0.0, -5.0}, kAmplitude, 0.0, kSamplePeriod, kFrequency, 0.0);
    CHECK(slow.y < -allowance());
    EXPECT_NEAR(slow.y, -kAmplitude / kPi, 1e-15);
}
