#pragma once

// Renderer2D.h -- 2D batch renderer for circles, AABBs, lines, world grid,
// and HUD overlay using OpenGL 3.3 Core profile.

#include "PhysicsEngine/Math/Vec2.h"
#include "PhysicsEngine/Physics/Contact.h"
#include "PhysicsEngine/Physics/Engine.h"
#include "Rendering/Camera2D.h"
#include "Rendering/Shader.h"

#include <cstdint>
#include <string>
#include <vector>

namespace PhysicsEngine {

struct Color {
    float r = 1.f;
    float g = 1.f;
    float b = 1.f;
    float a = 1.f;

    static constexpr Color fromRgb(uint32_t hex, float alpha = 1.0f) noexcept {
        return {
            static_cast<float>((hex >> 16) & 0xFF) / 255.0f,
            static_cast<float>((hex >> 8) & 0xFF) / 255.0f,
            static_cast<float>(hex & 0xFF) / 255.0f,
            alpha
        };
    }
};

struct StatusInfo {
    uint32_t    totalBodies      = 0;
    uint32_t    dynamicBodies    = 0;
    uint32_t    staticBodies     = 0;
    size_t      contactCount     = 0;
    double      physicsStepMs    = 0.0;
    double      frameTimeMs      = 0.0;
    double      fps              = 0.0;
    const char* broadPhaseName   = "Spatial Grid";
    bool        isPaused         = false;
    bool        showContacts     = false;
    bool        showGrid         = true;
};

class Renderer2D {
public:
    Renderer2D();
    ~Renderer2D();

    Renderer2D(const Renderer2D&) = delete;
    Renderer2D& operator=(const Renderer2D&) = delete;
    Renderer2D(Renderer2D&&) noexcept;
    Renderer2D& operator=(Renderer2D&&) noexcept;

    /// Initializes shaders, VAO, and dynamic VBOs.
    /// Must be called with an active OpenGL 3.3 Core context.
    bool init();

    /// Releases all GPU resources.
    void destroy();

    /// Begins a new frame batch.
    void begin();

    // ── World Geometry Drawing ───────────────────────────────────────────────

    void drawGrid(const Camera2D& camera, float minorStep = 1.0f, float majorStep = 5.0f);

    void drawLine(Vec2 start, Vec2 end, Color color);

    void drawCircle(Vec2 center, float radius, Color fillColor, Color outlineColor, int segments = 32);

    void drawAABB(Vec2 center, Vec2 halfExtents, Color fillColor, Color outlineColor);

    void drawContact(Vec2 point, Vec2 normal, float penetration, Color color = Color::fromRgb(0xEBCB8B));

    void drawPhysicsScene(const Engine& engine,
                          const std::vector<Contact>& contacts,
                          bool showContacts = true);

    // ── Screen-space Drawing & HUD ───────────────────────────────────────────

    void drawScreenQuad(float x, float y, float w, float h, Color color);
    void drawScreenLine(float x1, float y1, float x2, float y2, Color color);
    void drawScreenText(const char* text, float startX, float startY, float scale, Color color, bool bold = false);

    /// Computes the pixel width of the given text at the specified scale.
    static float measureTextWidth(const char* text, float scale) noexcept;

    void drawSpatialGrid(const Camera2D& camera, float cellSize, Color color);

    void drawStatusOverlay(const StatusInfo& info, int windowW, int windowH);

    /// Flushes all batched geometry to the screen.
    void end(const Camera2D& camera);

private:
    struct Vertex {
        float x, y;
        float r, g, b, a;
    };

    void flushWorld(const Camera2D& camera);
    void flushScreen(const Camera2D& camera);

    Shader               shader_;
    uint32_t             vao_ = 0;
    uint32_t             vbo_ = 0;

    std::vector<Vertex>  worldTriangles_;
    std::vector<Vertex>  worldLines_;

    std::vector<Vertex>  screenTriangles_;
    std::vector<Vertex>  screenLines_;
};

} // namespace PhysicsEngine
