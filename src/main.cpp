// main.cpp -- PhysicsEngine2D interactive sandbox & live visualization.
//
// Milestone 10: Physics Sandbox Reliability Audit and Release Readiness
//   - Zero external GUI dependencies (custom immediate-mode 2D GUI via UIContext).
//   - Pixel-perfect DPI coordinate scaling across window resizes and multi-monitor setups.
//   - Disambiguated input: Click vs Drag distinction guarantees releasing a pan gesture
//     never unexpectedly spawns a body.
//   - UI pointer capture inhibits viewport clicks, camera panning, and keyboard shortcuts.
//   - Scrollable UI panel with scrollbar prevents clipping on low vertical resolutions.
//   - Robust body spawning with boundary clamping, strict capacity enforcement, and feedback.
//   - Immediate synchronization between UI display and underlying Engine / Controller state.

#include <glad/gl.h>
#include <GLFW/glfw3.h>

#include "PhysicsEngine/Concurrency/WorkerPool.h"
#include "PhysicsEngine/Math/Vec2.h"
#include "PhysicsEngine/Physics/Body.h"
#include "PhysicsEngine/Physics/Collision.h"
#include "PhysicsEngine/Physics/Engine.h"
#include "PhysicsEngine/Physics/SimulationController.h"
#include "PhysicsEngine/Physics/WorldConfig.h"
#include "Rendering/Camera2D.h"
#include "Rendering/Renderer2D.h"
#include "Rendering/UI.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

using namespace PhysicsEngine;

namespace {

constexpr int   kInitialWindowWidth  = 1280;
constexpr int   kInitialWindowHeight = 720;
constexpr char  kWindowTitle[]       = "PhysicsEngine2D - Interactive Physics Sandbox";

// Dark neutral slate-charcoal background
constexpr float kClearR = 0.09f;
constexpr float kClearG = 0.09f;
constexpr float kClearB = 0.11f;

struct AppState {
    Camera2D camera{Vec2{0.f, 0.f}, 22.0f};

    // Framebuffer-space mouse positions (DPI-scaled)
    double currentMouseX        = 0.0;
    double currentMouseY        = 0.0;
    double lastPanX             = 0.0;
    double lastPanY             = 0.0;

    // Mouse button & gesture states
    bool   leftMouseDown        = false;
    bool   isPotentialClick     = false;
    double pressX               = 0.0;
    double pressY               = 0.0;
    bool   hasDraggedPastThresh = false;
    bool   isPanning            = false; // Right or Middle drag OR Left drag past threshold

    // Viewport spawn click event
    bool   spawnRequested       = false;
    double spawnScreenX         = 0.0;
    double spawnScreenY         = 0.0;

    // UI Input state
    bool   uiLeftClicked        = false;
    bool   uiLeftReleased       = false;
    float  scrollDeltaY         = 0.f;
    bool   isUiCaptured         = false;

    // Hotkey requested actions
    bool   resetRequested          = false;
    bool   singleStepRequested     = false;
    bool   togglePauseRequested    = false;
    bool   toggleContactsRequested = false;
    bool   toggleGridRequested     = false;
    bool   toggleSpatialGridReq    = false;
};

// ── GLFW Callbacks ────────────────────────────────────────────────────────────

void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    glViewport(0, 0, width, height);
    auto* state = static_cast<AppState*>(glfwGetWindowUserPointer(window));
    if (state && width > 0 && height > 0) {
        state->camera.setViewport(width, height);
    }
}

void mouse_button_callback(GLFWwindow* window, int button, int action, int /*mods*/)
{
    auto* state = static_cast<AppState*>(glfwGetWindowUserPointer(window));
    if (!state) return;

    if (button == GLFW_MOUSE_BUTTON_LEFT) {
        if (action == GLFW_PRESS) {
            state->leftMouseDown        = true;
            state->uiLeftClicked        = true;
            state->pressX               = state->currentMouseX;
            state->pressY               = state->currentMouseY;
            state->lastPanX             = state->currentMouseX;
            state->lastPanY             = state->currentMouseY;
            state->isPotentialClick     = true;
            state->hasDraggedPastThresh = false;
        } else if (action == GLFW_RELEASE) {
            state->leftMouseDown        = false;
            state->uiLeftReleased       = true;
            // Spawn only on intentional click: no drag past threshold and not captured by UI
            if (state->isPotentialClick && !state->hasDraggedPastThresh && !state->isUiCaptured) {
                state->spawnRequested   = true;
                state->spawnScreenX     = state->currentMouseX;
                state->spawnScreenY     = state->currentMouseY;
            }
            state->isPotentialClick     = false;
            state->hasDraggedPastThresh = false;
            state->isPanning            = false;
        }
    } else if (button == GLFW_MOUSE_BUTTON_RIGHT || button == GLFW_MOUSE_BUTTON_MIDDLE) {
        if (action == GLFW_PRESS) {
            if (!state->isUiCaptured) {
                state->isPanning = true;
                state->lastPanX  = state->currentMouseX;
                state->lastPanY  = state->currentMouseY;
            }
        } else if (action == GLFW_RELEASE) {
            state->isPanning = false;
        }
    }
}

void cursor_pos_callback(GLFWwindow* window, double xpos, double ypos)
{
    auto* state = static_cast<AppState*>(glfwGetWindowUserPointer(window));
    if (!state) return;

    // Convert screen coordinates to framebuffer pixels for DPI-aware rendering & hit testing
    int winW = 1, winH = 1, fbW = 1, fbH = 1;
    glfwGetWindowSize(window, &winW, &winH);
    glfwGetFramebufferSize(window, &fbW, &fbH);
    const double scaleX = (winW > 0) ? (static_cast<double>(fbW) / winW) : 1.0;
    const double scaleY = (winH > 0) ? (static_cast<double>(fbH) / winH) : 1.0;

    const double fbMouseX = xpos * scaleX;
    const double fbMouseY = ypos * scaleY;

    state->currentMouseX = fbMouseX;
    state->currentMouseY = fbMouseY;

    if (state->isPanning) {
        const double deltaX = fbMouseX - state->lastPanX;
        const double deltaY = fbMouseY - state->lastPanY;
        state->lastPanX = fbMouseX;
        state->lastPanY = fbMouseY;
        state->camera.panPixels(deltaX, deltaY);
    } else if (state->leftMouseDown && !state->isUiCaptured) {
        const double dist = std::hypot(fbMouseX - state->pressX, fbMouseY - state->pressY);
        if (dist > 6.0) { // Drag threshold: initiates left-click camera pan
            state->hasDraggedPastThresh = true;
            state->isPotentialClick     = false;
            const double deltaX = fbMouseX - state->lastPanX;
            const double deltaY = fbMouseY - state->lastPanY;
            state->lastPanX = fbMouseX;
            state->lastPanY = fbMouseY;
            state->camera.panPixels(deltaX, deltaY);
        }
    }
}

void scroll_callback(GLFWwindow* window, double /*xoffset*/, double yoffset)
{
    auto* state = static_cast<AppState*>(glfwGetWindowUserPointer(window));
    if (!state) return;

    if (state->isUiCaptured) {
        // Feed scroll into UI panel (inhibits camera zoom)
        state->scrollDeltaY += static_cast<float>(yoffset);
    } else {
        // Zoom camera in viewport toward mouse world position
        const Vec2 mouseWorld = state->camera.screenToWorld(state->currentMouseX, state->currentMouseY);
        const float factor = (yoffset > 0.0) ? 0.88f : 1.136f;
        state->camera.zoom(factor, mouseWorld);
    }
}

void key_callback(GLFWwindow* window, int key, int /*scancode*/, int action, int /*mods*/)
{
    if (action != GLFW_PRESS) return;

    auto* state = static_cast<AppState*>(glfwGetWindowUserPointer(window));
    if (!state) return;

    if (key == GLFW_KEY_ESCAPE) {
        glfwSetWindowShouldClose(window, GLFW_TRUE);
        return;
    }

    // Inhibit simulation hotkeys when cursor is over or interacting with UI controls
    if (state->isUiCaptured) {
        return;
    }

    if (key == GLFW_KEY_SPACE) {
        state->togglePauseRequested = true;
    } else if (key == GLFW_KEY_S) {
        state->singleStepRequested = true;
    } else if (key == GLFW_KEY_R) {
        state->resetRequested = true;
    } else if (key == GLFW_KEY_C) {
        state->toggleContactsRequested = true;
    } else if (key == GLFW_KEY_G) {
        state->toggleGridRequested = true;
    } else if (key == GLFW_KEY_B) {
        state->toggleSpatialGridReq = true;
    }
}

void glfw_error_callback(int error_code, const char* description)
{
    std::fprintf(stderr, "[GLFW error %d] %s\n", error_code, description);
}

// ── Initial Sandbox Scene Setup ──────────────────────────────────────────────

void setupInitialScene(Engine& engine)
{
    engine.clear();

    // ── Static Environmental Geometry ─────────────────────────────────────────
    // 1. Static ground floor (AABB)
    BodyDesc floor;
    floor.position    = Vec2{0.f, -7.5f};
    floor.mass        = 0.f; // static (infinite mass)
    floor.shapeType   = ShapeType::AABB;
    floor.radius      = 17.0f; // halfWidth
    floor.halfHeight  = 0.8f;  // halfHeight
    floor.restitution = 0.45f;
    (void)engine.addBody(floor);

    // 2. Left containment barrier
    BodyDesc leftWall;
    leftWall.position    = Vec2{-16.2f, 1.0f};
    leftWall.mass        = 0.f;
    leftWall.shapeType   = ShapeType::AABB;
    leftWall.radius      = 0.8f;
    leftWall.halfHeight  = 8.0f;
    leftWall.restitution = 0.3f;
    (void)engine.addBody(leftWall);

    // 3. Right containment barrier
    BodyDesc rightWall;
    rightWall.position    = Vec2{16.2f, 1.0f};
    rightWall.mass        = 0.f;
    rightWall.shapeType   = ShapeType::AABB;
    rightWall.radius      = 0.8f;
    rightWall.halfHeight  = 8.0f;
    rightWall.restitution = 0.3f;
    (void)engine.addBody(rightWall);

    // 4. Elevated static platform
    BodyDesc platform;
    platform.position    = Vec2{0.f, -3.5f};
    platform.mass        = 0.f;
    platform.shapeType   = ShapeType::AABB;
    platform.radius      = 3.8f;
    platform.halfHeight  = 0.4f;
    platform.restitution = 0.5f;
    (void)engine.addBody(platform);

    // 5. Static circular deflector peg
    BodyDesc peg;
    peg.position    = Vec2{0.f, 2.5f};
    peg.mass        = 0.f;
    peg.shapeType   = ShapeType::Circle;
    peg.radius      = 0.95f;
    peg.restitution = 0.65f;
    (void)engine.addBody(peg);

    // ── Dynamic Falling & Bouncing Bodies ─────────────────────────────────────
    // Dynamic circles of various sizes, masses, and restitution
    const struct CircleSpec {
        Vec2  pos;
        float radius;
        float mass;
        float restitution;
    } circles[] = {
        { {-1.2f, 5.0f},  0.65f, 1.0f, 0.60f },
        { { 1.3f, 5.8f},  0.75f, 1.3f, 0.65f },
        { {-3.2f, 6.5f},  0.55f, 0.8f, 0.70f },
        { { 3.4f, 7.2f},  0.85f, 1.6f, 0.55f },
        { {-6.0f, 8.0f},  0.60f, 1.0f, 0.60f },
        { { 6.2f, 8.5f},  0.95f, 2.0f, 0.50f },
        { {-0.6f, 9.8f},  0.70f, 1.2f, 0.65f },
        { { 0.7f, 11.2f}, 0.50f, 0.7f, 0.75f },
        { {-8.2f, 4.2f},  0.80f, 1.4f, 0.55f },
        { { 8.5f, 5.0f},  0.70f, 1.1f, 0.60f },
    };

    for (const auto& c : circles) {
        BodyDesc b;
        b.position    = c.pos;
        b.mass        = c.mass;
        b.shapeType   = ShapeType::Circle;
        b.radius      = c.radius;
        b.halfHeight  = 0.f;
        b.restitution = c.restitution;
        (void)engine.addBody(b);
    }

    // Dynamic AABBs (boxes) of various dimensions
    const struct BoxSpec {
        Vec2  pos;
        float halfW;
        float halfH;
        float mass;
        float restitution;
    } boxes[] = {
        { { 0.0f, -1.8f}, 1.1f, 0.8f, 2.0f, 0.35f }, // resting on platform
        { {-3.5f, -5.8f}, 1.0f, 0.9f, 2.0f, 0.40f },
        { { 3.5f, -5.8f}, 1.0f, 0.9f, 2.0f, 0.40f },
        { {-2.2f, 8.8f},  0.8f, 0.6f, 1.5f, 0.45f },
        { { 2.4f, 9.2f},  0.7f, 0.9f, 1.7f, 0.40f },
        { {-4.8f, 10.5f}, 0.9f, 0.7f, 1.8f, 0.35f },
        { { 4.6f, 11.5f}, 0.6f, 0.8f, 1.3f, 0.45f },
        { { 0.0f, 13.0f}, 1.2f, 0.7f, 2.2f, 0.30f },
    };

    for (const auto& bx : boxes) {
        BodyDesc b;
        b.position    = bx.pos;
        b.mass        = bx.mass;
        b.shapeType   = ShapeType::AABB;
        b.radius      = bx.halfW;
        b.halfHeight  = bx.halfH;
        b.restitution = bx.restitution;
        (void)engine.addBody(b);
    }
}

// ── Sandbox Configuration & State ─────────────────────────────────────────────

struct SandboxSettings {
    // Gravity editing
    float gravityX = 0.0f;
    float gravityY = -9.81f;

    // Broad-Phase selection: 0: Brute Force, 1: Spatial Grid (Seq), 2: Spatial Grid (Par)
    int   broadPhaseIdx = 1;

    // Debug visual toggles
    bool  showContacts    = false;
    bool  showGrid        = true;
    bool  showSpatialGrid = false;

    // Spawn parameters
    int   spawnShape       = 0;     // 0: Circle, 1: AABB
    int   spawnType        = 0;     // 0: Dynamic, 1: Static
    float spawnRadius      = 0.75f; // Circle radius
    float spawnHalfW       = 0.80f; // AABB halfWidth
    float spawnHalfH       = 0.70f; // AABB halfHeight
    float spawnMass        = 1.0f;  // Dynamic body mass (kg)
    float spawnRestitution = 0.60f; // Restitution [0, 1]
    float spawnVx          = 0.0f;  // Initial Vx (m/s)
    float spawnVy          = 0.0f;  // Initial Vy (m/s)

    // Status / User Feedback
    char  feedbackMsg[128] = "Click viewport to spawn bodies";
    Color feedbackColor    = Color::fromRgb(0x88C0D0);
    float feedbackTimer    = 4.0f;
};

} // anonymous namespace

// ── Application Entry Point ───────────────────────────────────────────────────

int main(int argc, char* argv[])
{
    int maxFrames = -1;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--frames") == 0 && i + 1 < argc) {
            maxFrames = std::atoi(argv[++i]);
        }
    }

    glfwSetErrorCallback(glfw_error_callback);

    if (!glfwInit()) {
        std::fprintf(stderr, "Fatal: glfwInit() failed.\n");
        return EXIT_FAILURE;
    }

    // OpenGL 3.3 Core profile
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    GLFWwindow* window = glfwCreateWindow(
        kInitialWindowWidth, kInitialWindowHeight, kWindowTitle,
        nullptr, nullptr
    );

    if (!window) {
        std::fprintf(stderr, "Fatal: glfwCreateWindow() failed.\n");
        glfwTerminate();
        return EXIT_FAILURE;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable VSync

    AppState appState;
    glfwSetWindowUserPointer(window, &appState);

    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    glfwSetMouseButtonCallback(window, mouse_button_callback);
    glfwSetCursorPosCallback(window, cursor_pos_callback);
    glfwSetScrollCallback(window, scroll_callback);
    glfwSetKeyCallback(window, key_callback);

    if (!gladLoadGL(glfwGetProcAddress)) {
        std::fprintf(stderr, "Fatal: gladLoadGL() failed.\n");
        glfwDestroyWindow(window);
        glfwTerminate();
        return EXIT_FAILURE;
    }

    std::fprintf(stdout, "OpenGL %s | GLSL %s | Renderer: %s\n",
                 glGetString(GL_VERSION),
                 glGetString(GL_SHADING_LANGUAGE_VERSION),
                 glGetString(GL_RENDERER));
    std::fflush(stdout);

    // Initialize viewport and camera
    int fbW, fbH;
    glfwGetFramebufferSize(window, &fbW, &fbH);
    glViewport(0, 0, fbW, fbH);
    appState.camera.setViewport(fbW, fbH);

    // Initialize Physics Simulation
    WorldConfig config;
    config.maxBodies = 1024;
    config.gravity   = Vec2{0.f, -9.81f};
    config.fixedDt   = 1.0f / 120.0f;
    Engine engine{config};

    // Initialize persistent worker pool for parallel broad-phase
    WorkerPool workerPool;

    SimulationController controller;
    controller.setWorkerPool(&workerPool);
    controller.setBroadPhaseType(BroadPhaseType::SpatialGrid);

    setupInitialScene(engine);

    // Sandbox state
    SandboxSettings sandbox;
    UIContext ui;

    // RAII Scope for OpenGL rendering resources
    {
        Renderer2D renderer;
        if (!renderer.init()) {
            std::fprintf(stderr, "Fatal: Renderer2D::init() failed.\n");
            glfwDestroyWindow(window);
            glfwTerminate();
            return EXIT_FAILURE;
        }

        StatusInfo statusInfo;
        statusInfo.broadPhaseName = "Grid (Seq)";
        statusInfo.showContacts   = false;
        statusInfo.showGrid       = true;

        double lastTime    = glfwGetTime();
        double lastFpsTime = lastTime;
        int    frameCount  = 0;
        int    totalFrames = 0;
        double currentFps  = 60.0;

        // ── Main Render & Simulation Loop ─────────────────────────────────────
        while (!glfwWindowShouldClose(window)) {
            const double now = glfwGetTime();
            float frameTime = static_cast<float>(now - lastTime);
            lastTime = now;

            // Clamp frameTime to avoid spiral of death on window drag / modal pause
            if (frameTime > 0.1f) {
                frameTime = 0.1f;
            }

            // FPS calculation every 0.25 seconds
            ++frameCount;
            ++totalFrames;
            if (now - lastFpsTime >= 0.25) {
                currentFps  = static_cast<double>(frameCount) / (now - lastFpsTime);
                frameCount  = 0;
                lastFpsTime = now;
            }

            // Update feedback message timer
            if (sandbox.feedbackTimer > 0.0f) {
                sandbox.feedbackTimer -= frameTime;
                if (sandbox.feedbackTimer <= 0.0f) {
                    std::snprintf(sandbox.feedbackMsg, sizeof(sandbox.feedbackMsg),
                                  "Click viewport to spawn bodies");
                    sandbox.feedbackColor = Color::fromRgb(0x88C0D0);
                }
            }

            // 1. Process Hotkey Actions
            if (appState.resetRequested) {
                setupInitialScene(engine);
                sandbox.gravityX = 0.0f;
                sandbox.gravityY = -9.81f;
                engine.setGravity(Vec2{sandbox.gravityX, sandbox.gravityY});
                controller.resetAccumulator();
                controller.clearContacts();
                appState.resetRequested = false;
                std::snprintf(sandbox.feedbackMsg, sizeof(sandbox.feedbackMsg), "Scene reset to defaults");
                sandbox.feedbackColor = Color::fromRgb(0x88C0D0);
                sandbox.feedbackTimer = 2.5f;
            }
            if (appState.togglePauseRequested) {
                controller.setPaused(!controller.isPaused());
                appState.togglePauseRequested = false;
            }
            if (appState.toggleContactsRequested) {
                sandbox.showContacts = !sandbox.showContacts;
                appState.toggleContactsRequested = false;
            }
            if (appState.toggleGridRequested) {
                sandbox.showGrid = !sandbox.showGrid;
                appState.toggleGridRequested = false;
            }
            if (appState.toggleSpatialGridReq) {
                sandbox.showSpatialGrid = !sandbox.showSpatialGrid;
                appState.toggleSpatialGridReq = false;
            }

            // 2. Prepare UI Input
            UIInput uiInput;
            uiInput.mouseX        = static_cast<float>(appState.currentMouseX);
            uiInput.mouseY        = static_cast<float>(appState.currentMouseY);
            uiInput.mouseDown     = appState.leftMouseDown;
            uiInput.mouseClicked  = appState.uiLeftClicked;
            uiInput.mouseReleased = appState.uiLeftReleased;
            uiInput.scrollDeltaY  = appState.scrollDeltaY;

            appState.uiLeftClicked  = false;
            appState.uiLeftReleased = false;
            appState.scrollDeltaY   = 0.f;

            // 3. Build UI Panel
            ui.begin(uiInput);
            glfwGetFramebufferSize(window, &fbW, &fbH);
            const float panelW = 320.0f;
            const float panelH = std::min(static_cast<float>(fbH) - 28.0f, 720.0f);
            ui.beginPanel("Sandbox Controls", 14.0f, 14.0f, panelW, panelH);

            // ── Section 1: Simulation Controls ────────────────────────────────
            ui.sectionHeader("Simulation");
            if (ui.button(controller.isPaused() ? "Resume Simulation [Space]" : "Pause Simulation [Space]")) {
                controller.setPaused(!controller.isPaused());
            }
            if (ui.button("Single-Step [S]")) {
                appState.singleStepRequested = true;
            }
            if (ui.button("Reset Scene [R]")) {
                appState.resetRequested = true;
            }

            // ── Section 2: Gravity Controls ───────────────────────────────────
            ui.sectionHeader("Gravity");
            sandbox.gravityX = engine.config().gravity.x;
            sandbox.gravityY = engine.config().gravity.y;
            if (ui.stepper("Gravity X", &sandbox.gravityX, 1.0f, -50.0f, 50.0f, "%.1f m/s²")) {
                engine.setGravity(Vec2{sandbox.gravityX, sandbox.gravityY});
            }
            if (ui.stepper("Gravity Y", &sandbox.gravityY, 1.0f, -50.0f, 50.0f, "%.1f m/s²")) {
                engine.setGravity(Vec2{sandbox.gravityX, sandbox.gravityY});
            }

            const char* gravPresets[] = {"Earth", "Moon", "Zero-G", "Invert"};
            int gravPresetIdx = -1;
            if (sandbox.gravityX == 0.0f && std::abs(sandbox.gravityY - (-9.81f)) < 0.05f) {
                gravPresetIdx = 0;
            } else if (sandbox.gravityX == 0.0f && std::abs(sandbox.gravityY - (-1.62f)) < 0.05f) {
                gravPresetIdx = 1;
            } else if (sandbox.gravityX == 0.0f && std::abs(sandbox.gravityY) < 0.05f) {
                gravPresetIdx = 2;
            } else if (sandbox.gravityX == 0.0f && std::abs(sandbox.gravityY - 9.81f) < 0.05f) {
                gravPresetIdx = 3;
            }

            if (ui.choice("Presets", gravPresets, 4, &gravPresetIdx)) {
                switch (gravPresetIdx) {
                    case 0: sandbox.gravityX = 0.0f; sandbox.gravityY = -9.81f; break; // Earth
                    case 1: sandbox.gravityX = 0.0f; sandbox.gravityY = -1.62f; break; // Moon
                    case 2: sandbox.gravityX = 0.0f; sandbox.gravityY =  0.00f; break; // Zero-G
                    case 3: sandbox.gravityX = 0.0f; sandbox.gravityY =  9.81f; break; // Inverted
                    default: break;
                }
                engine.setGravity(Vec2{sandbox.gravityX, sandbox.gravityY});
            }

            // ── Section 3: Broad-Phase Selection ──────────────────────────────
            ui.sectionHeader("Broad-Phase");
            const char* bpNames[] = {"Brute Force", "Grid (Seq)", "Grid (Par)"};
            sandbox.broadPhaseIdx = static_cast<int>(controller.broadPhaseType());
            int chosenBp = sandbox.broadPhaseIdx;
            if (ui.choice("Mode", bpNames, 3, &chosenBp) && chosenBp != sandbox.broadPhaseIdx) {
                sandbox.broadPhaseIdx = chosenBp;
                switch (sandbox.broadPhaseIdx) {
                    case 0:
                        controller.setBroadPhaseType(BroadPhaseType::BruteForce);
                        statusInfo.broadPhaseName = "Brute Force";
                        break;
                    case 1:
                        controller.setBroadPhaseType(BroadPhaseType::SpatialGrid);
                        statusInfo.broadPhaseName = "Grid (Seq)";
                        break;
                    case 2:
                        controller.setBroadPhaseType(BroadPhaseType::SpatialGridParallel);
                        statusInfo.broadPhaseName = "Grid (Par)";
                        break;
                    default:
                        break;
                }
            }

            // ── Section 4: Spawn Body Configuration ───────────────────────────
            ui.sectionHeader("Spawn Body");
            const char* shapes[] = {"Circle", "AABB Box"};
            ui.choice("Shape", shapes, 2, &sandbox.spawnShape);

            const char* bodyTypes[] = {"Dynamic", "Static"};
            ui.choice("Type", bodyTypes, 2, &sandbox.spawnType);

            if (sandbox.spawnShape == 0) {
                ui.stepper("Radius", &sandbox.spawnRadius, 0.1f, 0.2f, 5.0f, "%.2f m");
            } else {
                ui.stepper("Half-Width", &sandbox.spawnHalfW, 0.1f, 0.2f, 5.0f, "%.2f m");
                ui.stepper("Half-Height", &sandbox.spawnHalfH, 0.1f, 0.2f, 5.0f, "%.2f m");
            }

            if (sandbox.spawnType == 0) {
                ui.stepper("Mass", &sandbox.spawnMass, 0.5f, 0.1f, 100.0f, "%.1f kg");
                ui.stepper("Restitution", &sandbox.spawnRestitution, 0.05f, 0.0f, 1.0f, "%.2f");
                ui.stepper("Velocity X", &sandbox.spawnVx, 1.0f, -50.0f, 50.0f, "%.1f m/s");
                ui.stepper("Velocity Y", &sandbox.spawnVy, 1.0f, -50.0f, 50.0f, "%.1f m/s");
            } else {
                ui.stepper("Restitution", &sandbox.spawnRestitution, 0.05f, 0.0f, 1.0f, "%.2f");
            }

            // ── Section 5: Debug Visualization ────────────────────────────────
            ui.sectionHeader("Debug Visuals");
            ui.toggle("Show Contacts [C]", &sandbox.showContacts);
            ui.toggle("Show World Grid [G]", &sandbox.showGrid);
            ui.toggle("Show Grid Cells [B]", &sandbox.showSpatialGrid);

            // ── Status & Feedback Banner ──────────────────────────────────────
            ui.separator();
            ui.label(sandbox.feedbackMsg, sandbox.feedbackColor);

            ui.endPanel();

            // Record pointer capture state: blocks world clicking & camera gestures
            appState.isUiCaptured = ui.isMouseCaptured();

            // 4. Viewport Spawning on confirmed intentional click
            if (appState.spawnRequested && !appState.isUiCaptured) {
                appState.spawnRequested = false;
                const Vec2 worldPos = appState.camera.screenToWorld(appState.spawnScreenX, appState.spawnScreenY);

                if (engine.bodyCount() >= config.maxBodies) {
                    std::snprintf(sandbox.feedbackMsg, sizeof(sandbox.feedbackMsg),
                                  "Capacity full! Max %u bodies.", config.maxBodies);
                    sandbox.feedbackColor = Color::fromRgb(0xBF616A); // Warning red
                    sandbox.feedbackTimer = 3.5f;
                } else {
                    BodyDesc d;
                    d.position    = worldPos;
                    d.shapeType   = (sandbox.spawnShape == 0) ? ShapeType::Circle : ShapeType::AABB;
                    d.radius      = (sandbox.spawnShape == 0) ? std::max(0.1f, sandbox.spawnRadius)
                                                              : std::max(0.1f, sandbox.spawnHalfW);
                    d.halfHeight  = (sandbox.spawnShape == 0) ? 0.0f
                                                              : std::max(0.1f, sandbox.spawnHalfH);
                    d.mass        = (sandbox.spawnType == 0)  ? std::max(0.01f, sandbox.spawnMass)
                                                              : 0.0f;
                    d.restitution = std::clamp(sandbox.spawnRestitution, 0.0f, 1.0f);
                    d.velocity    = (sandbox.spawnType == 0)  ? Vec2{sandbox.spawnVx, sandbox.spawnVy}
                                                              : Vec2{0.f, 0.f};

                    BodyHandle h = engine.addBody(d);
                    if (engine.isValidHandle(h)) {
                        std::snprintf(sandbox.feedbackMsg, sizeof(sandbox.feedbackMsg),
                                      "Spawned %s %s at (%.1f, %.1f)",
                                      (sandbox.spawnType == 0 ? "Dyn" : "Stat"),
                                      (sandbox.spawnShape == 0 ? "Circle" : "Box"),
                                      worldPos.x, worldPos.y);
                        sandbox.feedbackColor = Color::fromRgb(0xA3BE8C); // Green
                        sandbox.feedbackTimer = 2.5f;
                    }
                }
            } else {
                appState.spawnRequested = false;
            }

            // 5. Advance Physics Simulation
            double stepDurationMs = 0.0;
            if (appState.singleStepRequested) {
                controller.pause();
                const auto tPhysStart = std::chrono::high_resolution_clock::now();
                controller.singleStep(engine);
                const auto tPhysEnd   = std::chrono::high_resolution_clock::now();
                stepDurationMs = std::chrono::duration<double, std::milli>(tPhysEnd - tPhysStart).count();
                appState.singleStepRequested = false;

                std::snprintf(sandbox.feedbackMsg, sizeof(sandbox.feedbackMsg), "Stepped 1 fixedDt step");
                sandbox.feedbackColor = Color::fromRgb(0x88C0D0);
                sandbox.feedbackTimer = 2.0f;
            } else if (!controller.isPaused()) {
                const auto tPhysStart = std::chrono::high_resolution_clock::now();
                const uint32_t stepsTaken = controller.advance(engine, frameTime);
                const auto tPhysEnd   = std::chrono::high_resolution_clock::now();

                stepDurationMs = (stepsTaken > 0)
                    ? std::chrono::duration<double, std::milli>(tPhysEnd - tPhysStart).count() / stepsTaken
                    : 0.0;
            }

            // 6. Render Frame
            glClearColor(kClearR, kClearG, kClearB, 1.0f);
            glClear(GL_COLOR_BUFFER_BIT);

            renderer.begin();

            if (sandbox.showGrid) {
                renderer.drawGrid(appState.camera, 1.0f, 5.0f);
            }

            if (sandbox.showSpatialGrid) {
                renderer.drawSpatialGrid(appState.camera,
                                         controller.spatialGrid().cellSize(),
                                         Color::fromRgb(0x4C566A, 0.45f));
            }

            renderer.drawPhysicsScene(engine, controller.lastContacts(), sandbox.showContacts);

            // Update status HUD info
            statusInfo.totalBodies = engine.bodyCount();
            uint32_t dynCount = 0;
            const float* invM = engine.inverseMassesData();
            for (uint32_t i = 0; i < statusInfo.totalBodies; ++i) {
                if (invM[i] > 0.f) ++dynCount;
            }
            statusInfo.dynamicBodies  = dynCount;
            statusInfo.staticBodies   = statusInfo.totalBodies - dynCount;
            statusInfo.contactCount   = controller.lastContacts().size();
            statusInfo.physicsStepMs  = stepDurationMs;
            statusInfo.frameTimeMs    = frameTime * 1000.0f;
            statusInfo.fps            = currentFps;
            statusInfo.isPaused       = controller.isPaused();
            statusInfo.showContacts   = sandbox.showContacts;
            statusInfo.showGrid       = sandbox.showGrid;

            // Draw HUD overlay in top-right
            renderer.drawStatusOverlay(statusInfo, fbW, fbH);

            // Draw immediate-mode UI panel on left side
            ui.render(renderer);

            // Flush all world and screen draw calls
            renderer.end(appState.camera);

            glfwSwapBuffers(window);
            glfwPollEvents();

            GLenum err = glGetError();
            if (err != GL_NO_ERROR) {
                std::fprintf(stderr, "[OpenGL Error]: %u\n", err);
            }

            if (maxFrames > 0 && totalFrames >= maxFrames) {
                std::fprintf(stdout, "Completed %d verification frames cleanly (Bodies: %u, Contacts: %zu, FPS: %.1f).\n",
                             maxFrames, engine.bodyCount(), controller.lastContacts().size(), currentFps);
                std::fflush(stdout);
                glfwSetWindowShouldClose(window, GLFW_TRUE);
            }
        }

        // renderer and shaders cleaned up cleanly here before window destruction
    }

    glfwDestroyWindow(window);
    glfwTerminate();

    return EXIT_SUCCESS;
}
