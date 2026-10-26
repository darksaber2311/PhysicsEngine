#pragma once

// Camera2D.h -- 2D orthographic camera with aspect-ratio preservation,
// panning, zoom, and world-screen coordinate conversions.

#include "PhysicsEngine/Math/Vec2.h"

namespace PhysicsEngine {

class Camera2D {
public:
    explicit Camera2D(Vec2 center = {0.f, 0.f}, float viewHeight = 22.f) noexcept;

    // Viewport dimensions (pixels)
    void setViewport(int width, int height) noexcept;
    [[nodiscard]] int viewportWidth() const noexcept { return viewportWidth_; }
    [[nodiscard]] int viewportHeight() const noexcept { return viewportHeight_; }
    [[nodiscard]] float aspectRatio() const noexcept { return aspectRatio_; }

    // Position & Zoom
    [[nodiscard]] Vec2 center() const noexcept { return center_; }
    void setCenter(Vec2 center) noexcept { center_ = center; }

    [[nodiscard]] float viewHeight() const noexcept { return viewHeight_; }
    void setViewHeight(float height) noexcept;

    [[nodiscard]] float viewWidth() const noexcept { return viewHeight_ * aspectRatio_; }

    // Navigation
    void pan(Vec2 deltaWorld) noexcept;
    void panPixels(double deltaPixelX, double deltaPixelY) noexcept;

    /// Zooms the camera by a multiplicative factor.
    /// \param factor          Scale factor (< 1 zooms in, > 1 zooms out).
    /// \param focusWorldPos   Optional world coordinate to keep stationary under zoom.
    void zoom(float factor, Vec2 focusWorldPos) noexcept;
    void zoom(float factor) noexcept;

    // Visible World Bounds
    [[nodiscard]] float left() const noexcept { return center_.x - viewWidth() * 0.5f; }
    [[nodiscard]] float right() const noexcept { return center_.x + viewWidth() * 0.5f; }
    [[nodiscard]] float bottom() const noexcept { return center_.y - viewHeight_ * 0.5f; }
    [[nodiscard]] float top() const noexcept { return center_.y + viewHeight_ * 0.5f; }

    // Coordinate conversions
    [[nodiscard]] Vec2 screenToWorld(double screenX, double screenY) const noexcept;
    [[nodiscard]] Vec2 worldToScreen(Vec2 worldPos) const noexcept;

    // Projection Matrix (column-major 4x4 for OpenGL)
    void getProjectionMatrix(float outMat16[16]) const noexcept;

    // Screen-space overlay projection (pixels [0, width] x [0, height], +Y down)
    void getScreenProjectionMatrix(float outMat16[16]) const noexcept;

private:
    Vec2  center_{0.f, 0.f};
    float viewHeight_     = 22.f; // Visible vertical span in world meters
    float aspectRatio_    = 1.3333f;
    int   viewportWidth_  = 800;
    int   viewportHeight_ = 600;

    static constexpr float kMinViewHeight = 2.0f;
    static constexpr float kMaxViewHeight = 250.0f;
};

} // namespace PhysicsEngine
