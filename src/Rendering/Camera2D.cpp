// Camera2D.cpp -- 2D camera implementation.

#include "Rendering/Camera2D.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace PhysicsEngine {

Camera2D::Camera2D(Vec2 center, float viewHeight) noexcept
    : center_(center)
{
    setViewHeight(viewHeight);
}

void Camera2D::setViewport(int width, int height) noexcept
{
    viewportWidth_  = std::max(1, width);
    viewportHeight_ = std::max(1, height);
    aspectRatio_    = static_cast<float>(viewportWidth_) / static_cast<float>(viewportHeight_);
}

void Camera2D::setViewHeight(float height) noexcept
{
    if (std::isfinite(height)) {
        viewHeight_ = std::clamp(height, kMinViewHeight, kMaxViewHeight);
    }
}

void Camera2D::pan(Vec2 deltaWorld) noexcept
{
    center_ += deltaWorld;
}

void Camera2D::panPixels(double deltaPixelX, double deltaPixelY) noexcept
{
    // Screen +X is right, +Y is down
    // World +X is right, +Y is up
    const float unitsPerPixelX = viewWidth() / static_cast<float>(viewportWidth_);
    const float unitsPerPixelY = viewHeight_ / static_cast<float>(viewportHeight_);

    center_.x -= static_cast<float>(deltaPixelX) * unitsPerPixelX;
    center_.y += static_cast<float>(deltaPixelY) * unitsPerPixelY;
}

void Camera2D::zoom(float factor, Vec2 focusWorldPos) noexcept
{
    if (factor <= 0.f || !std::isfinite(factor)) {
        return;
    }

    const float oldHeight = viewHeight_;
    setViewHeight(viewHeight_ * factor);
    const float actualScale = viewHeight_ / oldHeight;

    // Zoom centered around focus point:
    // (focus - newCenter) = (focus - oldCenter) * actualScale
    center_ = focusWorldPos + (center_ - focusWorldPos) * actualScale;
}

void Camera2D::zoom(float factor) noexcept
{
    zoom(factor, center_);
}

Vec2 Camera2D::screenToWorld(double screenX, double screenY) const noexcept
{
    const float ndcX = (2.0f * static_cast<float>(screenX) / static_cast<float>(viewportWidth_)) - 1.0f;
    const float ndcY = 1.0f - (2.0f * static_cast<float>(screenY) / static_cast<float>(viewportHeight_));

    const float halfW = viewWidth() * 0.5f;
    const float halfH = viewHeight_ * 0.5f;

    return Vec2{center_.x + ndcX * halfW, center_.y + ndcY * halfH};
}

Vec2 Camera2D::worldToScreen(Vec2 worldPos) const noexcept
{
    const float halfW = viewWidth() * 0.5f;
    const float halfH = viewHeight_ * 0.5f;

    const float ndcX = (worldPos.x - center_.x) / halfW;
    const float ndcY = (worldPos.y - center_.y) / halfH;

    const float screenX = (ndcX + 1.0f) * 0.5f * static_cast<float>(viewportWidth_);
    const float screenY = (1.0f - ndcY) * 0.5f * static_cast<float>(viewportHeight_);

    return Vec2{screenX, screenY};
}

void Camera2D::getProjectionMatrix(float outMat16[16]) const noexcept
{
    std::memset(outMat16, 0, 16 * sizeof(float));

    const float l = left();
    const float r = right();
    const float b = bottom();
    const float t = top();

    const float invW = 1.0f / (r - l);
    const float invH = 1.0f / (t - b);

    // Column-major orthographic projection matrix:
    outMat16[0]  = 2.0f * invW;
    outMat16[5]  = 2.0f * invH;
    outMat16[10] = -1.0f;
    outMat16[12] = -(r + l) * invW;
    outMat16[13] = -(t + b) * invH;
    outMat16[15] = 1.0f;
}

void Camera2D::getScreenProjectionMatrix(float outMat16[16]) const noexcept
{
    std::memset(outMat16, 0, 16 * sizeof(float));

    // Screen space: [0, viewportWidth_] x [0, viewportHeight_], origin top-left, +Y down
    const float w = static_cast<float>(viewportWidth_);
    const float h = static_cast<float>(viewportHeight_);

    outMat16[0]  = 2.0f / w;
    outMat16[5]  = -2.0f / h;
    outMat16[10] = -1.0f;
    outMat16[12] = -1.0f;
    outMat16[13] = 1.0f;
    outMat16[15] = 1.0f;
}

} // namespace PhysicsEngine
