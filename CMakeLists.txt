cmake_minimum_required(VERSION 3.16)

project(AdaptiveLocalizationSim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

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
endif()
