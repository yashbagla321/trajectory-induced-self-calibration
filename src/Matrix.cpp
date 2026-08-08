// Matrix.cpp
//
// Implementation of the dense-matrix helpers declared in Matrix.hpp. See
// that header for the rationale (de-duplicating what used to be repeated
// raw nested loops in Solver.cpp and Simulation.cpp) and the bit-for-bit
// reproducibility guarantee each function preserves.

#include "adaptive_localization/Matrix.hpp"

#include <cmath>
#include <cstddef>

namespace adaptive {

std::vector<double> solve_dense_linear_system(
    std::vector<std::vector<double>> a,
    std::vector<double> b,
    SingularPivotPolicy policy) {
    const std::size_t n = b.size();
    for (std::size_t col = 0; col < n; ++col) {
        // Partial pivoting: swap in the row with the largest magnitude entry
        // in this column to improve numerical stability.
        std::size_t pivot = col;
        double best = std::abs(a[col][col]);
        for (std::size_t row = col + 1; row < n; ++row) {
            const double value = std::abs(a[row][col]);
            if (value > best) {
                best = value;
                pivot = row;
            }
        }
        if (best < 1e-12) {
            if (policy == SingularPivotPolicy::ReturnZero) {
                return std::vector<double>(n, 0.0);
            }
            // RegularizeDiagonal: nudge the diagonal rather than dividing by
            // (near-)zero below.
            a[col][col] += 1e-8;
            pivot = col;
        }
        if (pivot != col) {
            std::swap(a[pivot], a[col]);
            std::swap(b[pivot], b[col]);
        }

        // Normalize the pivot row so a[col][col] becomes 1.
        const double diag = a[col][col];
        for (std::size_t j = col; j < n; ++j) {
            a[col][j] /= diag;
        }
        b[col] /= diag;

        // Eliminate this column from every other row.
        for (std::size_t row = 0; row < n; ++row) {
            if (row == col) {
                continue;
            }
            const double factor = a[row][col];
            for (std::size_t j = col; j < n; ++j) {
                a[row][j] -= factor * a[col][j];
            }
            b[row] -= factor * b[col];
        }
    }
    return b;
}

Matrix identity_matrix(int n, double scale) {
    Matrix matrix(static_cast<std::size_t>(n), std::vector<double>(static_cast<std::size_t>(n), 0.0));
    for (int i = 0; i < n; ++i) {
        matrix[static_cast<std::size_t>(i)][static_cast<std::size_t>(i)] = scale;
    }
    return matrix;
}

Matrix transpose(const Matrix& a) {
    if (a.empty()) {
        return {};
    }
    const std::size_t rows = a.size();
    const std::size_t cols = a[0].size();
    Matrix result(cols, std::vector<double>(rows, 0.0));
    for (std::size_t i = 0; i < rows; ++i) {
        for (std::size_t j = 0; j < cols; ++j) {
            result[j][i] = a[i][j];
        }
    }
    return result;
}

Matrix matmul(const Matrix& a, const Matrix& b) {
    if (a.empty() || b.empty()) {
        return {};
    }
    const std::size_t p = a.size();
    const std::size_t q = b.size();
    const std::size_t r = b[0].size();
    Matrix result(p, std::vector<double>(r, 0.0));
    for (std::size_t i = 0; i < p; ++i) {
        for (std::size_t j = 0; j < r; ++j) {
            double value = 0.0;
            for (std::size_t k = 0; k < q; ++k) {
                value += a[i][k] * b[k][j];
            }
            result[i][j] = value;
        }
    }
    return result;
}

Matrix gram_matrix(const Matrix& a) {
    // Result is n x n where n is the column count of `a` (an m x n matrix);
    // accumulated as the sum, over each row of `a`, of that row's outer
    // product with itself -- i.e. the same row-at-a-time order used by the
    // call site this replaces, so the accumulation order (and hence the
    // floating-point result) is unchanged.
    if (a.empty()) {
        return {};
    }
    const std::size_t n = a[0].size();
    Matrix result(n, std::vector<double>(n, 0.0));
    for (const auto& row : a) {
        for (std::size_t i = 0; i < n; ++i) {
            for (std::size_t j = 0; j < n; ++j) {
                result[i][j] += row[i] * row[j];
            }
        }
    }
    return result;
}

Matrix sandwich(const Matrix& a, const Matrix& b) {
    // Result is p x p where p is the row count of `a` (a p x q matrix) and
    // `b` is q x q. Each entry is accumulated as sum_i sum_j a[row][i] *
    // b[i][j] * a[col][j], matching the call site this replaces term for
    // term and in the same order.
    const std::size_t p = a.size();
    if (p == 0) {
        return {};
    }
    const std::size_t q = b.size();
    Matrix result(p, std::vector<double>(p, 0.0));
    for (std::size_t row = 0; row < p; ++row) {
        for (std::size_t col = 0; col < p; ++col) {
            double value = 0.0;
            for (std::size_t i = 0; i < q; ++i) {
                for (std::size_t j = 0; j < q; ++j) {
                    value += a[row][i] * b[i][j] * a[col][j];
                }
            }
            result[row][col] = value;
        }
    }
    return result;
}

}  // namespace adaptive
