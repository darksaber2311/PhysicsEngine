cmake_minimum_required(VERSION 3.25)
project(PhysicsEngine2D VERSION 0.1.0 LANGUAGES C CXX)

# ─────────────────────────────────────────────────────────────────────────────
# Global settings
# ─────────────────────────────────────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)          # -std=c++20, not -std=gnu++20

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)  # For clangd / IDE tooling

# Predictable output locations inside the build tree
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# ─────────────────────────────────────────────────────────────────────────────
# Dependencies (fetched automatically — no manual install required)
# ─────────────────────────────────────────────────────────────────────────────
include(FetchContent)

# ── GLFW ──────────────────────────────────────────────────────────────────────
set(GLFW_BUILD_DOCS     OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS    OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_INSTALL        OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
    glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG        3.4
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(glfw)

# ── GLAD2 (OpenGL 3.3 Core function loader) ───────────────────────────────
# GLAD2's CMake interface lives in its cmake/ subdirectory.
# glad_add_library() generates a ready-to-link static loader target.
FetchContent_Declare(
    glad
    GIT_REPOSITORY https://github.com/Dav1dde/glad.git
    GIT_TAG        v2.0.8
    GIT_SHALLOW    TRUE
    SOURCE_SUBDIR  cmake
)
FetchContent_MakeAvailable(glad)

# Generate a loader for OpenGL 3.3 Core, no extensions needed at this stage
glad_add_library(glad_gl_core_33
    STATIC
    REPRODUCIBLE
    API gl:core=3.3
)

# ── Catch2 v3 (unit testing) ───────────────────────────────────────────────
FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG        v3.7.1
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(Catch2)
list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
include(Catch)   # Provides catch_discover_tests()

# ── OpenGL & Threads ───────────────────────────────────────────────────────
find_package(OpenGL REQUIRED)
find_package(Threads REQUIRED)

# ─────────────────────────────────────────────────────────────────────────────
# Compiler warning helper
# Applied per first-party target; third-party targets are never affected.
# ─────────────────────────────────────────────────────────────────────────────
function(target_set_warnings target)
    if(MSVC)
        target_compile_options(${target} PRIVATE
            /W4
            /wd4996   # CRT deprecation (noisy, not safety-relevant here)
        )
    else()
        target_compile_options(${target} PRIVATE
            -Wall
            -Wextra
            -Wpedantic
            -Wno-unused-parameter   # Expected in early stubs; re-enable later
        )
    endif()
endfunction()

# ─────────────────────────────────────────────────────────────────────────────
# PhysicsEngine — reusable static library
# Physics, math, spatial acceleration: no rendering dependency.
# Usable in headless contexts (tests, benchmarks) without an OpenGL context.
# ─────────────────────────────────────────────────────────────────────────────
add_library(PhysicsEngine STATIC
    src/Concurrency/WorkerPool.cpp
    src/Physics/Engine.cpp
    src/Physics/Integration.cpp
    src/Physics/SimulationController.cpp
    src/Physics/Collision.cpp
    src/Physics/CollisionSolver.cpp
    src/Physics/SpatialGrid.cpp
)

target_link_libraries(PhysicsEngine PUBLIC
    Threads::Threads
)

target_include_directories(PhysicsEngine
    PUBLIC  include   # Consumers use include/PhysicsEngine/...
    PRIVATE src       # Internal implementation details
)

target_set_warnings(PhysicsEngine)

# ─────────────────────────────────────────────────────────────────────────────
# PhysicsEngineApp — windowed application (GLFW + OpenGL 3.3 Core)
# The rendering layer (src/Rendering/) is compiled only into this target.
# ─────────────────────────────────────────────────────────────────────────────
add_executable(PhysicsEngineApp
    src/main.cpp
    src/Rendering/Shader.cpp
    src/Rendering/Camera2D.cpp
    src/Rendering/Renderer2D.cpp
    src/Rendering/UI.cpp
)

target_link_libraries(PhysicsEngineApp PRIVATE
    PhysicsEngine
    glfw
    glad_gl_core_33
    OpenGL::GL
)

target_include_directories(PhysicsEngineApp PRIVATE src)
target_set_warnings(PhysicsEngineApp)

# ─────────────────────────────────────────────────────────────────────────────
# Tests — headless, no OpenGL context required
# ─────────────────────────────────────────────────────────────────────────────
enable_testing()
add_subdirectory(tests)

# ─────────────────────────────────────────────────────────────────────────────
# Benchmarks — headless, no OpenGL context required
# ─────────────────────────────────────────────────────────────────────────────
add_executable(PhysicsEngineBenchmarks
    benchmarks/main.cpp
)

target_link_libraries(PhysicsEngineBenchmarks PRIVATE
    PhysicsEngine
)

target_include_directories(PhysicsEngineBenchmarks PRIVATE src)
target_set_warnings(PhysicsEngineBenchmarks)
