# tests/CMakeLists.txt
# Defines the PhysicsEngineTests executable and registers it with CTest.
#
# Design decisions:
#   - Tests link ONLY against PhysicsEngine (the static library) and Catch2.
#   - No GLFW, no OpenGL, no glad: tests must run headlessly.
#   - catch_discover_tests() asks the test binary to enumerate its own test
#     cases at configure time, keeping CTest's test list accurate without
#     manual registration.

add_executable(PhysicsEngineTests
    test_vec2.cpp
    test_engine.cpp
    test_integration.cpp
    test_controller.cpp
    test_collision.cpp
    test_collision_solver.cpp
    test_stability.cpp
    test_spatial_grid.cpp
    test_worker_pool.cpp
    test_parallel_broadphase.cpp
    test_sandbox_controls.cpp
    test_audit_milestone10.cpp
)

target_link_libraries(PhysicsEngineTests PRIVATE
    PhysicsEngine
    Catch2::Catch2WithMain   # Provides main(); no need to write our own
)

# Apply the same warnings as production code
if(MSVC)
    target_compile_options(PhysicsEngineTests PRIVATE /W4 /wd4996)
else()
    target_compile_options(PhysicsEngineTests PRIVATE
        -Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

# Register all Catch2 TEST_CASEs with CTest automatically
catch_discover_tests(PhysicsEngineTests)
