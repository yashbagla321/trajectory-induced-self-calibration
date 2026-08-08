// test_jacobi_eigenvalues.cpp
//
// Checks Simulation.hpp's jacobi_eigenvalues (the hand-rolled cyclic Jacobi
// eigensolver used to turn the local-observability normal matrix into
// singular values) against symmetric 5x5 matrices whose eigenvalues are
// known by hand, rather than only by the indirect Monte Carlo sweeps that
// are its production caller.

#include "adaptive_localization/Simulation.hpp"
#include "test_framework.hpp"

using namespace adaptive;

namespace {

std::size_t index(int row, int col) {
    return static_cast<std::size_t>(row * 5 + col);
}

}  // namespace

ADAPTIVE_TEST(jacobi_eigenvalues_diagonal_matrix_returns_diagonal_sorted) {
    // An already-diagonal matrix exercises the early-exit path (the largest
    // off-diagonal entry is 0 on the very first sweep).
    std::array<double, 25> a{};
    const std::array<double, 5> diagonal = {5.0, 1.0, 4.0, 2.0, 3.0};
    for (int i = 0; i < 5; ++i) {
        a[index(i, i)] = diagonal[static_cast<std::size_t>(i)];
    }

    const auto eigenvalues = jacobi_eigenvalues(a);

    const std::array<double, 5> expected = {1.0, 2.0, 3.0, 4.0, 5.0};
    for (std::size_t i = 0; i < 5; ++i) {
        EXPECT_NEAR(eigenvalues[i], expected[i], 1e-9);
    }
}

ADAPTIVE_TEST(jacobi_eigenvalues_block_matrix_matches_hand_computed_values) {
    // Block-diagonal: a 2x2 symmetric block [[3,1],[1,3]] (eigenvalues of
    // [[a,b],[b,a]] are a+b, a-b, i.e. 4 and 2 here) plus three decoupled
    // diagonal entries {5, 6, 7}. This forces at least one real Jacobi
    // rotation (on the (0,1) pair) while remaining exactly hand-verifiable.
    std::array<double, 25> a{};
    a[index(0, 0)] = 3.0;
    a[index(0, 1)] = 1.0;
    a[index(1, 0)] = 1.0;
    a[index(1, 1)] = 3.0;
    a[index(2, 2)] = 5.0;
    a[index(3, 3)] = 6.0;
    a[index(4, 4)] = 7.0;

    const auto eigenvalues = jacobi_eigenvalues(a);

    const std::array<double, 5> expected = {2.0, 4.0, 5.0, 6.0, 7.0};
    for (std::size_t i = 0; i < 5; ++i) {
        EXPECT_NEAR(eigenvalues[i], expected[i], 1e-9);
    }
}
