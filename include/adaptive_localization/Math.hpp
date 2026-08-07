#pragma once

#include <cmath>

namespace adaptive {

constexpr double kPi = 3.141592653589793238462643383279502884;

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

inline double dot(const Vec2& a, const Vec2& b) {
    return a.x * b.x + a.y * b.y;
}

inline double norm(const Vec2& v) {
    return std::sqrt(dot(v, v));
}

inline double wrap_angle(double angle) {
    while (angle > kPi) {
        angle -= 2.0 * kPi;
    }
    while (angle < -kPi) {
        angle += 2.0 * kPi;
    }
    return angle;
}

inline Vec2 unit_from_angle(double angle) {
    return {std::cos(angle), std::sin(angle)};
}

inline Vec2 rotate(const Vec2& v, double yaw) {
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    return {c * v.x - s * v.y, s * v.x + c * v.y};
}

inline double bearing(const Vec2& from, const Vec2& to) {
    return std::atan2(to.y - from.y, to.x - from.x);
}

}  // namespace adaptive
