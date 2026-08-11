// Math.hpp
//
// Minimal 2D vector/geometry primitives shared by every other module in this
// codebase: a lightweight Vec2 type with the usual arithmetic operators, plus
// free functions for norms, angle wrapping, and the rotate/bearing operations
// used to convert between global (world) coordinates and a beacon's local
// (possibly unknown-yaw) frame throughout the estimation and simulation code.

#pragma once

#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

namespace adaptive {

// pi, used for angle wrapping and for building periodic test trajectories
// (e.g. circles, figure-eights) elsewhere in the codebase.
constexpr double kPi = 3.141592653589793238462643383279502884;

/**
 * A plain 2D Euclidean vector (or point), used throughout the codebase to
 * represent robot/target/beacon positions and displacement vectors.
 * Supports the usual vector-space operations: addition, subtraction, and
 * scaling (multiply/divide by a scalar). There is intentionally no notion of
 * "point" vs "vector" distinction -- callers use Vec2 for both.
 */
struct Vec2 {
    double x = 0.0;
    double y = 0.0;

    Vec2() = default;
    Vec2(double x_, double y_) : x(x_), y(y_) {}

    Vec2 operator+(const Vec2& other) const { return {x + other.x, y + other.y}; }
    Vec2 operator-(const Vec2& other) const { return {x - other.x, y - other.y}; }
    Vec2 operator*(double scale) const { return {x * scale, y * scale}; }
    Vec2 operator/(double scale) const { return {x / scale, y / scale}; }
};

/// Euclidean dot product of two 2D vectors.
inline double dot(const Vec2& a, const Vec2& b) {
    return a.x * b.x + a.y * b.y;
}

/// Euclidean (L2) length of a 2D vector, i.e. sqrt(dot(v, v)).
inline double norm(const Vec2& v) {
    return std::sqrt(dot(v, v));
}

/**
 * Wraps an angle (radians) into the canonical range (-pi, pi].
 *
 * Used everywhere a bearing or yaw residual is computed, since raw
 * differences of angles (e.g. predicted bearing minus measured bearing) can
 * land outside (-pi, pi] even though the underlying angular quantity is
 * periodic; without wrapping, a residual near +-2*pi would be mistaken by the
 * solver for a huge error instead of a near-zero one.
 */
inline double wrap_angle(double angle) {
    while (angle > kPi) {
        angle -= 2.0 * kPi;
    }
    while (angle < -kPi) {
        angle += 2.0 * kPi;
    }
    return angle;
}

/// Returns the unit vector (cos(angle), sin(angle)) pointing along `angle`
/// (radians, measured counter-clockwise from the +x axis).
inline Vec2 unit_from_angle(double angle) {
    return {std::cos(angle), std::sin(angle)};
}

/**
 * Rotates vector `v` counter-clockwise by `yaw` radians (a standard 2D
 * rotation matrix application). Used to convert a vector between a beacon's
 * local/body frame and the global frame: rotating a local-frame vector by the
 * beacon's yaw maps it into the global frame, and rotating by -yaw maps a
 * global-frame vector into the beacon's local frame.
 */
inline Vec2 rotate(const Vec2& v, double yaw) {
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    return {c * v.x - s * v.y, s * v.x + c * v.y};
}

/// Global-frame bearing (radians, atan2 convention) from point `from` to
/// point `to`. Note this is independent of any beacon's local yaw offset --
/// callers subtract a beacon's yaw separately to express the bearing in that
/// beacon's local frame.
inline double bearing(const Vec2& from, const Vec2& to) {
    return std::atan2(to.y - from.y, to.x - from.x);
}

/**
 * Draws one zero-mean Gaussian noise sample with standard deviation `sigma`,
 * or exactly 0.0 (no distribution constructed) when `sigma <= 0.0`, which is
 * how the rest of the codebase represents a noiseless channel. Shared by
 * every measurement-noise call site (see the "Results-to-paper map" note in
 * README.md on why `std::normal_distribution`'s sample sequence -- and
 * hence bit-for-bit reproduction of the committed results -- is tied to a
 * specific standard library implementation).
 */
inline double sample_noise(double sigma, std::mt19937& rng) {
    if (sigma <= 0.0) {
        return 0.0;
    }
    std::normal_distribution<double> distribution(0.0, sigma);
    return distribution(rng);
}

// Directional seeking projection of Algorithm 1 (closed-loop paper): while
// the stored window is underexcited, the seeking-velocity component opposing
// the current excitation half-period's push direction
// n(t) = (-1)^floor(omega t / pi) e_y is clipped at the allowance
// b = A e^{-lambda T_bar} / pi, so the finite-acquisition proposition's
// hypothesis (ii) holds for the implemented controller by construction; the
// x component and any seeking aligned with n(t) pass through untouched.
// Callers gate on supervised mode, an underexcited (retriggered) window, and
// amplitude > 0 -- with no excitation there is no half-period structure to
// protect. Both the batch simulator and the ROS 2 / Gazebo node share this
// implementation.
inline Vec2 project_seeking_velocity(const Vec2& seeking, double amplitude,
                                     double decay, double sample_period,
                                     double frequency, double time) {
    const double allowance =
        amplitude * std::exp(-decay * sample_period) / kPi;
    const long long half_period =
        static_cast<long long>(std::floor(frequency * time / kPi));
    Vec2 projected = seeking;
    if (half_period % 2 == 0) {
        projected.y = std::max(projected.y, -allowance);
    } else {
        projected.y = std::min(projected.y, allowance);
    }
    return projected;
}

/**
 * Trajectory spread S_v = sum_k ||q_k - q_bar||^2 computed directly from the
 * known vehicle poses at which measurements were taken. This is exactly the
 * certificate the theory defines (in the noiseless model S_v over local
 * vectors equals the spread of the global poses, and the vehicle path is
 * known by assumption), so it is 0 at a stationary pose regardless of
 * measurement noise. The excitation supervisor triggers on THIS quantity --
 * both in the batch simulator and in the ROS 2 / Gazebo closed-loop node,
 * which share this implementation.
 */
inline double path_spread(const std::vector<Vec2>& path) {
    if (path.empty()) {
        return 0.0;
    }
    Vec2 sum;
    for (const auto& pose : path) {
        sum = sum + pose;
    }
    const Vec2 mean = sum / static_cast<double>(path.size());
    double spread = 0.0;
    for (const auto& pose : path) {
        const Vec2 centered = pose - mean;
        spread += dot(centered, centered);
    }
    return spread;
}

}  // namespace adaptive
