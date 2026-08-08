// Matrix.hpp
//
// Small dense-matrix helpers, represented as std::vector<std::vector<double>>
// for compatibility with the rest of the codebase (which builds these
// matrices row-by-row without a fixed compile-time size). This module exists
// to give a single home to the handful of matrix operations that were
// previously duplicated as raw nested loops in both Solver.cpp (the
// Gauss-Newton normal-equation solve) and Simulation.cpp (the EKF baseline's
// innovation covariance, Kalman gain, Joseph-form covariance update, and the
// closed-form local-information covariance). Kept intentionally minimal:
// only the operations actually needed by those call sites, not a
// general-purpose linear-algebra library.
//
// Every function here preserves the exact summation order of the code it
// replaces (row-major, ascending index order), so switching a call site to
// use these helpers does not change any floating-point result bit-for-bit.

#pragma once

#include <vector>

namespace adaptive {

using Matrix = std::vector<std::vector<double>>;

/// How solve_dense_linear_system behaves when a pivot column is numerically
/// singular (best available pivot magnitude below 1e-12).
enum class SingularPivotPolicy {
    /// Nudge the diagonal entry by a small jitter (1e-8) and keep going --
    /// used by the Gauss-Newton normal equations, where the Levenberg
    /// damping term already biases the problem toward well-posedness, so a
    /// slightly perturbed step is preferable to no step at all.
    RegularizeDiagonal,
    /// Return an all-zero solution vector -- used by the EKF/covariance
    /// solves, where a singular system should read as "no information"
    /// rather than one biased by synthetically injected damping.
    ReturnZero,
};

/**
 * Solves the dense linear system a*x = b via Gauss-Jordan elimination with
 * partial pivoting (row-reduces `a` to the identity while carrying `b`
 * along), returning the solution vector in place of `b`. `a` and `b` are
 * taken by value since both are mutated during elimination.
 *
 * @param a n x n coefficient matrix.
 * @param b right-hand side vector of length n.
 * @param policy what to do if a pivot column is numerically singular (best
 *     available pivot magnitude below 1e-12); see SingularPivotPolicy.
 * @return the solution vector, or (under ReturnZero) an all-zero vector if
 *     `a` was singular to working precision.
 */
std::vector<double> solve_dense_linear_system(
    std::vector<std::vector<double>> a,
    std::vector<double> b,
    SingularPivotPolicy policy);

/// Builds an n x n matrix that is `scale` on the diagonal and zero
/// elsewhere (identity when `scale == 1.0`).
Matrix identity_matrix(int n, double scale = 1.0);

/// Transpose of `a` (an m x n matrix becomes n x m).
Matrix transpose(const Matrix& a);

/// Matrix product a * b (a is p x q, b is q x r; result is p x r).
Matrix matmul(const Matrix& a, const Matrix& b);

/// The Gram matrix a^T * a of an m x n matrix `a` (result is n x n), i.e.
/// the sum over rows of a of the outer product of that row with itself.
/// Used to form Gauss-Newton/EKF normal-equation information matrices from
/// a stacked residual Jacobian.
Matrix gram_matrix(const Matrix& a);

/// The sandwich product a * b * a^T (a is p x q, b is q x q; result is
/// p x p). Used for the EKF innovation covariance H P H^T (the caller adds
/// the measurement-noise covariance R on the diagonal separately).
Matrix sandwich(const Matrix& a, const Matrix& b);

}  // namespace adaptive
