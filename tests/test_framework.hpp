// test_framework.hpp
//
// Minimal, dependency-free unit-test harness: a handful of assertion macros
// plus static self-registration, so each ADAPTIVE_TEST(...) block anywhere in
// the tests/ directory is automatically picked up by test_main.cpp's runner
// without any central list to keep in sync. Deliberately not a general
// testing framework -- just enough machinery to keep this repo's stated
// "dependency-free C++17" property while still having real unit tests on the
// hand-rolled numerical primitives (see the top-level README's Build section).

#pragma once

#include <cmath>
#include <cstdio>
#include <exception>
#include <string>
#include <vector>

namespace adaptive_test {

/// One registered test: a human-readable name plus the function to run.
struct TestCase {
    std::string name;
    void (*fn)();
};

/// The global list of registered tests, populated by static Registrar
/// instances (one per ADAPTIVE_TEST(...) block) before main() runs.
inline std::vector<TestCase>& registry() {
    static std::vector<TestCase> tests;
    return tests;
}

/// Constructing one of these (as a file-scope static) appends its test to
/// the registry; this is what lets ADAPTIVE_TEST(...) self-register.
struct Registrar {
    Registrar(const char* name, void (*fn)()) { registry().push_back({name, fn}); }
};

/// Thrown by CHECK/EXPECT_NEAR on failure; caught by test_main.cpp's runner
/// so one failing assertion reports as a failed test rather than aborting
/// the whole suite.
struct TestFailure : std::exception {
    std::string message;
    explicit TestFailure(std::string msg) : message(std::move(msg)) {}
    const char* what() const noexcept override { return message.c_str(); }
};

}  // namespace adaptive_test

/// Defines a self-registering test function. Usage:
///     ADAPTIVE_TEST(some_descriptive_name) {
///         CHECK(...);
///     }
#define ADAPTIVE_TEST(name)                                                \
    static void name();                                                   \
    static const ::adaptive_test::Registrar registrar_##name(#name, &name); \
    static void name()

/// Fails the current test (with file:line and the failing expression) if
/// `condition` is false.
#define CHECK(condition)                                                \
    do {                                                                \
        if (!(condition)) {                                             \
            throw ::adaptive_test::TestFailure(                         \
                std::string(__FILE__) + ":" + std::to_string(__LINE__) + \
                ": CHECK failed: " #condition);                         \
        }                                                                \
    } while (false)

/// Fails the current test if `actual` is not within `tolerance` of
/// `expected` (absolute difference).
#define EXPECT_NEAR(actual, expected, tolerance)                             \
    do {                                                                     \
        const double adaptive_test_actual = (actual);                       \
        const double adaptive_test_expected = (expected);                   \
        const double adaptive_test_tolerance = (tolerance);                 \
        if (!(std::abs(adaptive_test_actual - adaptive_test_expected) <=    \
              adaptive_test_tolerance)) {                                   \
            throw ::adaptive_test::TestFailure(                            \
                std::string(__FILE__) + ":" + std::to_string(__LINE__) +   \
                ": EXPECT_NEAR failed: " #actual " = " +                   \
                std::to_string(adaptive_test_actual) + ", " #expected " = " + \
                std::to_string(adaptive_test_expected) + ", tolerance = " + \
                std::to_string(adaptive_test_tolerance));                   \
        }                                                                    \
    } while (false)
