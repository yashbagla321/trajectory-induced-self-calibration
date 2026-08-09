cmake_minimum_required(VERSION 3.16)

project(AdaptiveLocalizationSim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Off by default; enabled by the "sanitize-debug" CMake preset (see
# CMakePresets.json) to catch out-of-bounds/undefined-behavior bugs in the
# hand-rolled pointer/array indexing used throughout the EKF and Jacobi
# eigensolver. MSVC only supports AddressSanitizer, not UBSan.
option(ENABLE_SANITIZERS "Build with AddressSanitizer (and UBSan on non-MSVC compilers)" OFF)

add_library(adaptive_localization_core STATIC
    src/Config.cpp
    src/Estimators.cpp
    src/Matrix.cpp
    src/Measurements.cpp
    src/Output.cpp
    src/Simulation.cpp
    src/Solver.cpp
    src/World.cpp
)

target_include_directories(adaptive_localization_core PUBLIC include)

if(MSVC)
    target_compile_options(adaptive_localization_core PRIVATE /W4)
else()
    target_compile_options(adaptive_localization_core PRIVATE -Wall -Wextra -pedantic)
endif()

# Applies AddressSanitizer (+ UndefinedBehaviorSanitizer where supported) to
# `target`. See the ENABLE_SANITIZERS option above.
function(apply_sanitizers_if_enabled target)
    if(NOT ENABLE_SANITIZERS)
        return()
    endif()
    if(MSVC)
        target_compile_options(${target} PRIVATE /fsanitize=address)
    else()
        target_compile_options(${target} PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
        target_link_options(${target} PRIVATE -fsanitize=address,undefined)
    endif()
endfunction()

apply_sanitizers_if_enabled(adaptive_localization_core)

if(CMAKE_SOURCE_DIR STREQUAL PROJECT_SOURCE_DIR)
    add_executable(adaptive_localization_sim
        src/main.cpp
    )

    target_link_libraries(adaptive_localization_sim PRIVATE adaptive_localization_core)

    if(MSVC)
        target_compile_options(adaptive_localization_sim PRIVATE /W4)
    else()
        target_compile_options(adaptive_localization_sim PRIVATE -Wall -Wextra -pedantic)
    endif()

    apply_sanitizers_if_enabled(adaptive_localization_sim)

    # Zero-dependency unit tests (tests/test_framework.hpp) on the hand-rolled
    # numerical primitives: the analytic Jacobian (vs. finite differences),
    # the Gauss-Newton solver, the Jacobi eigensolver, and the dense linear
    # solver. Run via `ctest` after building, or the test binary directly.
    enable_testing()

    add_executable(adaptive_localization_tests
        tests/test_main.cpp
        tests/test_matrix.cpp
        tests/test_jacobian.cpp
        tests/test_gauss_newton.cpp
        tests/test_jacobi_eigenvalues.cpp
        tests/test_closed_loop_gate.cpp
    )

    target_link_libraries(adaptive_localization_tests PRIVATE adaptive_localization_core)

    if(MSVC)
        target_compile_options(adaptive_localization_tests PRIVATE /W4)
    else()
        target_compile_options(adaptive_localization_tests PRIVATE -Wall -Wextra -pedantic)
    endif()

    apply_sanitizers_if_enabled(adaptive_localization_tests)

    add_test(NAME adaptive_localization_tests COMMAND adaptive_localization_tests)
endif()
