// Renderer2D.cpp -- 2D batch renderer implementation.

#include "Rendering/Renderer2D.h"

#include <glad/gl.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <utility>

namespace PhysicsEngine {

namespace {

const char* kVertexShaderSource = R"(#version 330 core
layout (location = 0) in vec2 aPos;
layout (location = 1) in vec4 aColor;

uniform mat4 uProjection;

out vec4 vColor;

void main() {
    vColor = aColor;
    gl_Position = uProjection * vec4(aPos, 0.0, 1.0);
}
)";

const char* kFragmentShaderSource = R"(#version 330 core
in vec4 vColor;
out vec4 FragColor;

void main() {
    FragColor = vColor;
}
)";

constexpr float kPi = 3.14159265358979323846f;

} // anonymous namespace

Renderer2D::Renderer2D() = default;

Renderer2D::~Renderer2D()
{
    destroy();
}

Renderer2D::Renderer2D(Renderer2D&& other) noexcept
    : shader_(std::move(other.shader_))
    , vao_(other.vao_)
    , vbo_(other.vbo_)
    , worldTriangles_(std::move(other.worldTriangles_))
    , worldLines_(std::move(other.worldLines_))
    , screenTriangles_(std::move(other.screenTriangles_))
    , screenLines_(std::move(other.screenLines_))
{
    other.vao_ = 0;
    other.vbo_ = 0;
}

Renderer2D& Renderer2D::operator=(Renderer2D&& other) noexcept
{
    if (this != &other) {
        destroy();
        shader_          = std::move(other.shader_);
        vao_             = other.vao_;
        vbo_             = other.vbo_;
        worldTriangles_  = std::move(other.worldTriangles_);
        worldLines_      = std::move(other.worldLines_);
        screenTriangles_ = std::move(other.screenTriangles_);
        screenLines_     = std::move(other.screenLines_);

        other.vao_ = 0;
        other.vbo_ = 0;
    }
    return *this;
}

bool Renderer2D::init()
{
    destroy();

    if (!shader_.loadFromSource(kVertexShaderSource, kFragmentShaderSource)) {
        std::fprintf(stderr, "Renderer2D: Failed to compile shaders.\n");
        return false;
    }

    glGenVertexArrays(1, &vao_);
    glGenBuffers(1, &vbo_);

    glBindVertexArray(vao_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);

    // Vertex layout: position (vec2) + color (vec4)
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(0));
    glEnableVertexAttribArray(0);

    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                          reinterpret_cast<void*>(2 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    // Reserve initial capacity for smooth 60fps batching
    worldTriangles_.reserve(4096);
    worldLines_.reserve(4096);
    screenTriangles_.reserve(1024);
    screenLines_.reserve(2048);

    return true;
}

void Renderer2D::destroy()
{
    if (vbo_ != 0) {
        glDeleteBuffers(1, &vbo_);
        vbo_ = 0;
    }
    if (vao_ != 0) {
        glDeleteVertexArrays(1, &vao_);
        vao_ = 0;
    }
    shader_.destroy();

    worldTriangles_.clear();
    worldLines_.clear();
    screenTriangles_.clear();
    screenLines_.clear();
}

void Renderer2D::begin()
{
    worldTriangles_.clear();
    worldLines_.clear();
    screenTriangles_.clear();
    screenLines_.clear();
}

void Renderer2D::drawLine(Vec2 start, Vec2 end, Color color)
{
    worldLines_.push_back({start.x, start.y, color.r, color.g, color.b, color.a});
    worldLines_.push_back({end.x,   end.y,   color.r, color.g, color.b, color.a});
}

void Renderer2D::drawCircle(Vec2 center, float radius, Color fillColor, Color outlineColor, int segments)
{
    if (radius <= 0.f || segments < 3) return;

    const float angleStep = (2.0f * kPi) / static_cast<float>(segments);

    // Filled circle (triangle fan converted to triangles)
    for (int i = 0; i < segments; ++i) {
        const float a1 = static_cast<float>(i) * angleStep;
        const float a2 = static_cast<float>(i + 1) * angleStep;

        const float x1 = center.x + radius * std::cos(a1);
        const float y1 = center.y + radius * std::sin(a1);
        const float x2 = center.x + radius * std::cos(a2);
        const float y2 = center.y + radius * std::sin(a2);

        worldTriangles_.push_back({center.x, center.y, fillColor.r, fillColor.g, fillColor.b, fillColor.a});
        worldTriangles_.push_back({x1,       y1,       fillColor.r, fillColor.g, fillColor.b, fillColor.a});
        worldTriangles_.push_back({x2,       y2,       fillColor.r, fillColor.g, fillColor.b, fillColor.a});

        // Outline line
        worldLines_.push_back({x1, y1, outlineColor.r, outlineColor.g, outlineColor.b, outlineColor.a});
        worldLines_.push_back({x2, y2, outlineColor.r, outlineColor.g, outlineColor.b, outlineColor.a});
    }
}

void Renderer2D::drawAABB(Vec2 center, Vec2 halfExtents, Color fillColor, Color outlineColor)
{
    if (halfExtents.x <= 0.f || halfExtents.y <= 0.f) return;

    const float x0 = center.x - halfExtents.x;
    const float x1 = center.x + halfExtents.x;
    const float y0 = center.y - halfExtents.y;
    const float y1 = center.y + halfExtents.y;

    // Filled Quad (2 triangles)
    worldTriangles_.push_back({x0, y0, fillColor.r, fillColor.g, fillColor.b, fillColor.a});
    worldTriangles_.push_back({x1, y0, fillColor.r, fillColor.g, fillColor.b, fillColor.a});
    worldTriangles_.push_back({x1, y1, fillColor.r, fillColor.g, fillColor.b, fillColor.a});

    worldTriangles_.push_back({x0, y0, fillColor.r, fillColor.g, fillColor.b, fillColor.a});
    worldTriangles_.push_back({x1, y1, fillColor.r, fillColor.g, fillColor.b, fillColor.a});
    worldTriangles_.push_back({x0, y1, fillColor.r, fillColor.g, fillColor.b, fillColor.a});

    // Outline
    worldLines_.push_back({x0, y0, outlineColor.r, outlineColor.g, outlineColor.b, outlineColor.a});
    worldLines_.push_back({x1, y0, outlineColor.r, outlineColor.g, outlineColor.b, outlineColor.a});

    worldLines_.push_back({x1, y0, outlineColor.r, outlineColor.g, outlineColor.b, outlineColor.a});
    worldLines_.push_back({x1, y1, outlineColor.r, outlineColor.g, outlineColor.b, outlineColor.a});

    worldLines_.push_back({x1, y1, outlineColor.r, outlineColor.g, outlineColor.b, outlineColor.a});
    worldLines_.push_back({x0, y1, outlineColor.r, outlineColor.g, outlineColor.b, outlineColor.a});

    worldLines_.push_back({x0, y1, outlineColor.r, outlineColor.g, outlineColor.b, outlineColor.a});
    worldLines_.push_back({x0, y0, outlineColor.r, outlineColor.g, outlineColor.b, outlineColor.a});
}

void Renderer2D::drawContact(Vec2 point, Vec2 normal, float penetration, Color color)
{
    // Draw contact point crosshair
    const float r = 0.12f;
    drawLine({point.x - r, point.y}, {point.x + r, point.y}, color);
    drawLine({point.x, point.y - r}, {point.x, point.y + r}, color);

    // Draw normal indicator arrow (scaled by penetration + base length)
    const float arrowLen = 0.35f + std::min(penetration * 2.0f, 0.5f);
    const Vec2 tip = point + normal * arrowLen;
    drawLine(point, tip, color);
}

void Renderer2D::drawGrid(const Camera2D& camera, float minorStep, float majorStep)
{
    const float l = camera.left();
    const float r = camera.right();
    const float b = camera.bottom();
    const float t = camera.top();

    const Color minorColor = Color::fromRgb(0x23262F, 0.45f);
    const Color majorColor = Color::fromRgb(0x353945, 0.70f);
    const Color axisXColor = Color::fromRgb(0x8C3B3B, 0.85f);
    const Color axisYColor = Color::fromRgb(0x3B8C4A, 0.85f);

    // Vertical grid lines
    const float startX = std::floor(l / minorStep) * minorStep;
    for (float x = startX; x <= r; x += minorStep) {
        if (std::abs(x) < 1e-4f) continue;
        const bool isMajor = (std::fmod(std::abs(x), majorStep) < 1e-4f);
        drawLine({x, b}, {x, t}, isMajor ? majorColor : minorColor);
    }

    // Horizontal grid lines
    const float startY = std::floor(b / minorStep) * minorStep;
    for (float y = startY; y <= t; y += minorStep) {
        if (std::abs(y) < 1e-4f) continue;
        const bool isMajor = (std::fmod(std::abs(y), majorStep) < 1e-4f);
        drawLine({l, y}, {r, y}, isMajor ? majorColor : minorColor);
    }

    // World axes
    if (b <= 0.f && t >= 0.f) {
        drawLine({l, 0.f}, {r, 0.f}, axisXColor);
    }
    if (l <= 0.f && r >= 0.f) {
        drawLine({0.f, b}, {0.f, t}, axisYColor);
    }
}

void Renderer2D::drawSpatialGrid(const Camera2D& camera, float cellSize, Color color)
{
    if (cellSize <= 0.f || !std::isfinite(cellSize)) return;

    const float l = camera.left();
    const float r = camera.right();
    const float b = camera.bottom();
    const float t = camera.top();

    const float startX = std::floor(l / cellSize) * cellSize;
    for (float x = startX; x <= r; x += cellSize) {
        drawLine({x, b}, {x, t}, color);
    }

    const float startY = std::floor(b / cellSize) * cellSize;
    for (float y = startY; y <= t; y += cellSize) {
        drawLine({l, y}, {r, y}, color);
    }
}

void Renderer2D::drawPhysicsScene(const Engine& engine,
                                  const std::vector<Contact>& contacts,
                                  bool showContacts)
{
    const uint32_t count = engine.bodyCount();
    if (count == 0) return;

    const Vec2*      positions     = engine.positionsData();
    const float*     radii         = engine.radiiData();
    const float*     halfHeights   = engine.halfHeightsData();
    const ShapeType* shapeTypes    = engine.shapeTypesData();
    const float*     inverseMasses = engine.inverseMassesData();

    // Distinct palettes for clear contrast
    const Color staticFill   = Color::fromRgb(0x2E3440, 0.95f);
    const Color staticBorder = Color::fromRgb(0x4C566A, 1.00f);

    const Color circlePaletteFill[] = {
        Color::fromRgb(0xBF616A, 0.90f), // Coral Red
        Color::fromRgb(0x5E81AC, 0.90f), // Frost Blue
        Color::fromRgb(0xB48EAD, 0.90f), // Purple Orchid
        Color::fromRgb(0x88C0D0, 0.90f), // Arctic Cyan
    };
    const Color circlePaletteBorder[] = {
        Color::fromRgb(0xE06C75, 1.0f),
        Color::fromRgb(0x81A1C1, 1.0f),
        Color::fromRgb(0xD08770, 1.0f),
        Color::fromRgb(0x8FBCBB, 1.0f),
    };

    const Color aabbPaletteFill[] = {
        Color::fromRgb(0xA3BE8C, 0.90f), // Sage Green
        Color::fromRgb(0xEBCB8B, 0.90f), // Golden Amber
        Color::fromRgb(0xD08770, 0.90f), // Warm Terracotta
        Color::fromRgb(0x81A1C1, 0.90f), // Steel Blue
    };
    const Color aabbPaletteBorder[] = {
        Color::fromRgb(0xB5D19C, 1.0f),
        Color::fromRgb(0xF0D69E, 1.0f),
        Color::fromRgb(0xE59882, 1.0f),
        Color::fromRgb(0x88C0D0, 1.0f),
    };

    for (uint32_t i = 0; i < count; ++i) {
        const Vec2 pos = positions[i];
        const bool isStatic = (inverseMasses[i] == 0.f);
        const ShapeType type = shapeTypes[i];

        if (type == ShapeType::Circle) {
            const float r = radii[i];
            Color fill = isStatic ? staticFill : circlePaletteFill[i % 4];
            Color border = isStatic ? staticBorder : circlePaletteBorder[i % 4];
            drawCircle(pos, r, fill, border, 32);

            // Radius spoke line to show orientation
            const Vec2 spoke = pos + Vec2{r * 0.7071f, r * 0.7071f};
            drawLine(pos, spoke, Color::fromRgb(0x1E222A, 0.65f));
        } else {
            const float hw = radii[i];
            const float hh = halfHeights[i];
            Color fill = isStatic ? staticFill : aabbPaletteFill[i % 4];
            Color border = isStatic ? staticBorder : aabbPaletteBorder[i % 4];
            drawAABB(pos, {hw, hh}, fill, border);
        }
    }

    if (showContacts) {
        for (const auto& c : contacts) {
            drawContact(c.point, c.normal, c.penetration);
        }
    }
}

// ── Screen-Space HUD & Vector Font ───────────────────────────────────────────

void Renderer2D::drawScreenQuad(float x, float y, float w, float h, Color color)
{
    screenTriangles_.push_back({x,     y,     color.r, color.g, color.b, color.a});
    screenTriangles_.push_back({x + w, y,     color.r, color.g, color.b, color.a});
    screenTriangles_.push_back({x + w, y + h, color.r, color.g, color.b, color.a});

    screenTriangles_.push_back({x,     y,     color.r, color.g, color.b, color.a});
    screenTriangles_.push_back({x + w, y + h, color.r, color.g, color.b, color.a});
    screenTriangles_.push_back({x,     y + h, color.r, color.g, color.b, color.a});
}

void Renderer2D::drawScreenLine(float x1, float y1, float x2, float y2, Color color)
{
    screenLines_.push_back({x1, y1, color.r, color.g, color.b, color.a});
    screenLines_.push_back({x2, y2, color.r, color.g, color.b, color.a});
}

namespace {

float getCharWidth(char c) noexcept
{
    switch (c) {
        case ' ': return 3.2f;
        case 'i': case 'l': case '!': case '|': case '\'': case '.': case ':': return 1.6f;
        case 'j': case ';': case ',': return 1.8f;
        case 'I': return 3.0f;
        case 't': case 'f': case 'r': case '[': case ']': case '(': case ')': return 2.8f;
        case 'M': case 'W': case 'm': case 'w': return 5.8f;
        case 'c': case 'k': case 's': case 'v': case 'x': case 'y': case 'z': return 3.6f;
        case 'e': case 'h': case 'n': case 'o': case 'u': return 3.8f;
        case 'a': case 'b': case 'd': case 'g': case 'p': case 'q': return 4.0f;
        case 'E': case 'F': case 'L': case 'Z': return 4.0f;
        case 'B': case 'H': case 'K': case 'R': case 'S': case 'T': case 'U': case 'X': case 'Y': return 4.5f;
        case 'A': case 'C': case 'D': case 'G': case 'N': case 'O': case 'P': case 'Q': case 'V': return 4.8f;
        case '0': case '1': case '2': case '3': case '4': case '5': case '6': case '7': case '8': case '9': return 4.4f;
        case '+': case '-': case '*': case '/': case '=': case '<': case '>': return 4.2f;
        default: return 4.0f;
    }
}

} // anonymous namespace

float Renderer2D::measureTextWidth(const char* text, float scale) noexcept
{
    if (!text || *text == '\0') return 0.f;
    const float spacing = 1.6f;
    float totalW = 0.f;
    for (const char* p = text; *p != '\0'; ++p) {
        if (static_cast<unsigned char>(*p) == 0xC2 && static_cast<unsigned char>(*(p + 1)) == 0xB2) {
            totalW += (2.8f + spacing) * scale;
            ++p;
            continue;
        }
        totalW += (getCharWidth(*p) + spacing) * scale;
    }
    if (totalW > 0.f) {
        totalW -= spacing * scale;
    }
    return totalW;
}

void Renderer2D::drawScreenText(const char* text, float startX, float startY, float scale, Color color, bool bold)
{
    if (!text) return;

    const float spacing = 1.6f * scale;
    float curX = startX;

    for (const char* p = text; *p != '\0'; ++p) {
        // Support UTF-8 superscript 2 ('²' = 0xC2 0xB2)
        if (static_cast<unsigned char>(*p) == 0xC2 && static_cast<unsigned char>(*(p + 1)) == 0xB2) {
            auto line = [&](float x1, float y1, float x2, float y2) {
                drawScreenLine(curX + x1 * scale, startY + y1 * scale,
                               curX + x2 * scale, startY + y2 * scale, color);
                if (bold) {
                    const float b = 0.8f;
                    drawScreenLine(curX + x1 * scale + b, startY + y1 * scale,
                                   curX + x2 * scale + b, startY + y2 * scale, color);
                    drawScreenLine(curX + x1 * scale, startY + y1 * scale + b,
                                   curX + x2 * scale, startY + y2 * scale + b, color);
                }
            };
            line(0.0f, 0.8f, 0.8f, 0.0f);
            line(0.8f, 0.0f, 2.0f, 0.0f);
            line(2.0f, 0.0f, 2.8f, 0.8f);
            line(2.8f, 0.8f, 2.8f, 1.8f);
            line(2.8f, 1.8f, 0.0f, 3.8f);
            line(0.0f, 3.8f, 2.8f, 3.8f);
            curX += (2.8f * scale) + spacing;
            ++p;
            continue;
        }

        const char c = *p;
        const float charW = getCharWidth(c);

        auto line = [&](float x1, float y1, float x2, float y2) {
            drawScreenLine(curX + x1 * scale, startY + y1 * scale,
                           curX + x2 * scale, startY + y2 * scale, color);
            if (bold) {
                const float b = 0.8f;
                drawScreenLine(curX + x1 * scale + b, startY + y1 * scale,
                               curX + x2 * scale + b, startY + y2 * scale, color);
                drawScreenLine(curX + x1 * scale, startY + y1 * scale + b,
                               curX + x2 * scale, startY + y2 * scale + b, color);
            }
        };

        switch (c) {
            // Digits
            case '0':
                line(1.2f, 0.0f, 3.2f, 0.0f); line(3.2f, 0.0f, 4.4f, 1.6f);
                line(4.4f, 1.6f, 4.4f, 5.4f); line(4.4f, 5.4f, 3.2f, 7.0f);
                line(3.2f, 7.0f, 1.2f, 7.0f); line(1.2f, 7.0f, 0.0f, 5.4f);
                line(0.0f, 5.4f, 0.0f, 1.6f); line(0.0f, 1.6f, 1.2f, 0.0f);
                line(0.9f, 5.6f, 3.5f, 1.4f);
                break;
            case '1':
                line(0.8f, 1.8f, 2.2f, 0.0f); line(2.2f, 0.0f, 2.2f, 7.0f);
                line(0.5f, 7.0f, 3.8f, 7.0f);
                break;
            case '2':
                line(0.0f, 1.6f, 1.2f, 0.0f); line(1.2f, 0.0f, 3.2f, 0.0f);
                line(3.2f, 0.0f, 4.4f, 1.6f); line(4.4f, 1.6f, 4.4f, 3.2f);
                line(4.4f, 3.2f, 0.0f, 7.0f); line(0.0f, 7.0f, 4.4f, 7.0f);
                break;
            case '3':
                line(0.0f, 0.0f, 4.4f, 0.0f); line(4.4f, 0.0f, 2.2f, 3.2f);
                line(2.2f, 3.2f, 3.5f, 3.4f); line(3.5f, 3.4f, 4.4f, 4.5f);
                line(4.4f, 4.5f, 4.4f, 5.6f); line(4.4f, 5.6f, 3.2f, 7.0f);
                line(3.2f, 7.0f, 1.0f, 7.0f); line(1.0f, 7.0f, 0.0f, 6.0f);
                break;
            case '4':
                line(3.2f, 7.0f, 3.2f, 0.0f); line(3.2f, 0.0f, 0.0f, 4.5f);
                line(0.0f, 4.5f, 4.4f, 4.5f);
                break;
            case '5':
                line(4.4f, 0.0f, 0.0f, 0.0f); line(0.0f, 0.0f, 0.0f, 3.2f);
                line(0.0f, 3.2f, 3.2f, 3.2f); line(3.2f, 3.2f, 4.4f, 4.4f);
                line(4.4f, 4.4f, 4.4f, 5.6f); line(4.4f, 5.6f, 3.2f, 7.0f);
                line(3.2f, 7.0f, 1.0f, 7.0f); line(1.0f, 7.0f, 0.0f, 6.0f);
                break;
            case '6':
                line(3.4f, 0.0f, 1.4f, 0.0f); line(1.4f, 0.0f, 0.0f, 1.8f);
                line(0.0f, 1.8f, 0.0f, 5.4f); line(0.0f, 5.4f, 1.4f, 7.0f);
                line(1.4f, 7.0f, 3.2f, 7.0f); line(3.2f, 7.0f, 4.4f, 5.6f);
                line(4.4f, 5.6f, 4.4f, 4.0f); line(4.4f, 4.0f, 3.2f, 3.2f);
                line(3.2f, 3.2f, 0.0f, 3.5f);
                break;
            case '7':
                line(0.0f, 0.0f, 4.4f, 0.0f); line(4.4f, 0.0f, 1.8f, 7.0f);
                break;
            case '8':
                line(1.2f, 0.0f, 3.2f, 0.0f); line(3.2f, 0.0f, 4.4f, 1.5f);
                line(4.4f, 1.5f, 3.2f, 3.4f); line(3.2f, 3.4f, 4.4f, 5.4f);
                line(4.4f, 5.4f, 3.2f, 7.0f); line(3.2f, 7.0f, 1.2f, 7.0f);
                line(1.2f, 7.0f, 0.0f, 5.4f); line(0.0f, 5.4f, 1.2f, 3.4f);
                line(1.2f, 3.4f, 0.0f, 1.5f); line(0.0f, 1.5f, 1.2f, 0.0f);
                line(1.2f, 3.4f, 3.2f, 3.4f);
                break;
            case '9':
                line(4.4f, 3.5f, 1.2f, 3.8f); line(1.2f, 3.8f, 0.0f, 3.0f);
                line(0.0f, 3.0f, 0.0f, 1.6f); line(0.0f, 1.6f, 1.2f, 0.0f);
                line(1.2f, 0.0f, 3.2f, 0.0f); line(3.2f, 0.0f, 4.4f, 1.6f);
                line(4.4f, 1.6f, 4.4f, 5.4f); line(4.4f, 5.4f, 3.0f, 7.0f);
                line(3.0f, 7.0f, 1.0f, 7.0f);
                break;

            // Uppercase
            case 'A':
                line(0.0f, 7.0f, 2.4f, 0.0f); line(2.4f, 0.0f, 4.8f, 7.0f);
                line(1.1f, 4.2f, 3.7f, 4.2f);
                break;
            case 'B':
                line(0.0f, 0.0f, 0.0f, 7.0f); line(0.0f, 0.0f, 3.0f, 0.0f);
                line(3.0f, 0.0f, 4.5f, 1.7f); line(4.5f, 1.7f, 3.0f, 3.5f);
                line(3.0f, 3.5f, 0.0f, 3.5f); line(3.0f, 3.5f, 4.5f, 5.2f);
                line(4.5f, 5.2f, 3.0f, 7.0f); line(3.0f, 7.0f, 0.0f, 7.0f);
                break;
            case 'C':
                line(4.8f, 1.8f, 3.2f, 0.0f); line(3.2f, 0.0f, 1.6f, 0.0f);
                line(1.6f, 0.0f, 0.0f, 1.8f); line(0.0f, 1.8f, 0.0f, 5.2f);
                line(0.0f, 5.2f, 1.6f, 7.0f); line(1.6f, 7.0f, 3.2f, 7.0f);
                line(3.2f, 7.0f, 4.8f, 5.2f);
                break;
            case 'D':
                line(0.0f, 0.0f, 0.0f, 7.0f); line(0.0f, 0.0f, 2.8f, 0.0f);
                line(2.8f, 0.0f, 4.8f, 2.0f); line(4.8f, 2.0f, 4.8f, 5.0f);
                line(4.8f, 5.0f, 2.8f, 7.0f); line(2.8f, 7.0f, 0.0f, 7.0f);
                break;
            case 'E':
                line(0.0f, 0.0f, 0.0f, 7.0f); line(0.0f, 0.0f, 4.0f, 0.0f);
                line(0.0f, 3.5f, 3.0f, 3.5f); line(0.0f, 7.0f, 4.0f, 7.0f);
                break;
            case 'F':
                line(0.0f, 0.0f, 0.0f, 7.0f); line(0.0f, 0.0f, 4.0f, 0.0f);
                line(0.0f, 3.5f, 3.0f, 3.5f);
                break;
            case 'G':
                line(4.8f, 1.8f, 3.2f, 0.0f); line(3.2f, 0.0f, 1.6f, 0.0f);
                line(1.6f, 0.0f, 0.0f, 1.8f); line(0.0f, 1.8f, 0.0f, 5.2f);
                line(0.0f, 5.2f, 1.6f, 7.0f); line(1.6f, 7.0f, 3.4f, 7.0f);
                line(3.4f, 7.0f, 4.8f, 5.2f); line(4.8f, 5.2f, 4.8f, 3.5f);
                line(4.8f, 3.5f, 2.4f, 3.5f);
                break;
            case 'H':
                line(0.0f, 0.0f, 0.0f, 7.0f); line(4.5f, 0.0f, 4.5f, 7.0f);
                line(0.0f, 3.5f, 4.5f, 3.5f);
                break;
            case 'I':
                line(0.0f, 0.0f, 3.0f, 0.0f); line(1.5f, 0.0f, 1.5f, 7.0f);
                line(0.0f, 7.0f, 3.0f, 7.0f);
                break;
            case 'J':
                line(1.0f, 0.0f, 3.5f, 0.0f); line(2.5f, 0.0f, 2.5f, 5.4f);
                line(2.5f, 5.4f, 1.4f, 7.0f); line(1.4f, 7.0f, 0.0f, 5.8f);
                break;
            case 'K':
                line(0.0f, 0.0f, 0.0f, 7.0f); line(4.5f, 0.0f, 0.0f, 4.0f);
                line(1.6f, 2.6f, 4.5f, 7.0f);
                break;
            case 'L':
                line(0.0f, 0.0f, 0.0f, 7.0f); line(0.0f, 7.0f, 4.0f, 7.0f);
                break;
            case 'M':
                line(0.0f, 7.0f, 0.0f, 0.0f); line(0.0f, 0.0f, 2.9f, 4.5f);
                line(2.9f, 4.5f, 5.8f, 0.0f); line(5.8f, 0.0f, 5.8f, 7.0f);
                break;
            case 'N':
                line(0.0f, 7.0f, 0.0f, 0.0f); line(0.0f, 0.0f, 4.8f, 7.0f);
                line(4.8f, 7.0f, 4.8f, 0.0f);
                break;
            case 'O':
                line(1.6f, 0.0f, 3.2f, 0.0f); line(3.2f, 0.0f, 4.8f, 1.8f);
                line(4.8f, 1.8f, 4.8f, 5.2f); line(4.8f, 5.2f, 3.2f, 7.0f);
                line(3.2f, 7.0f, 1.6f, 7.0f); line(1.6f, 7.0f, 0.0f, 5.2f);
                line(0.0f, 5.2f, 0.0f, 1.8f); line(0.0f, 1.8f, 1.6f, 0.0f);
                break;
            case 'P':
                line(0.0f, 7.0f, 0.0f, 0.0f); line(0.0f, 0.0f, 3.2f, 0.0f);
                line(3.2f, 0.0f, 4.8f, 1.8f); line(4.8f, 1.8f, 3.2f, 3.6f);
                line(3.2f, 3.6f, 0.0f, 3.6f);
                break;
            case 'Q':
                line(1.6f, 0.0f, 3.2f, 0.0f); line(3.2f, 0.0f, 4.8f, 1.8f);
                line(4.8f, 1.8f, 4.8f, 5.2f); line(4.8f, 5.2f, 3.2f, 7.0f);
                line(3.2f, 7.0f, 1.6f, 7.0f); line(1.6f, 7.0f, 0.0f, 5.2f);
                line(0.0f, 5.2f, 0.0f, 1.8f); line(0.0f, 1.8f, 1.6f, 0.0f);
                line(2.8f, 5.2f, 5.2f, 8.2f);
                break;
            case 'R':
                line(0.0f, 7.0f, 0.0f, 0.0f); line(0.0f, 0.0f, 3.0f, 0.0f);
                line(3.0f, 0.0f, 4.5f, 1.8f); line(4.5f, 1.8f, 3.0f, 3.6f);
                line(3.0f, 3.6f, 0.0f, 3.6f); line(2.2f, 3.6f, 4.5f, 7.0f);
                break;
            case 'S':
                line(4.5f, 1.8f, 3.2f, 0.0f); line(3.2f, 0.0f, 1.4f, 0.0f);
                line(1.4f, 0.0f, 0.0f, 1.4f); line(0.0f, 1.4f, 0.0f, 2.8f);
                line(0.0f, 2.8f, 1.4f, 3.5f); line(1.4f, 3.5f, 3.2f, 3.8f);
                line(3.2f, 3.8f, 4.5f, 4.8f); line(4.5f, 4.8f, 4.5f, 5.6f);
                line(4.5f, 5.6f, 3.2f, 7.0f); line(3.2f, 7.0f, 1.4f, 7.0f);
                line(1.4f, 7.0f, 0.0f, 5.4f);
                break;
            case 'T':
                line(0.0f, 0.0f, 4.5f, 0.0f); line(2.25f, 0.0f, 2.25f, 7.0f);
                break;
            case 'U':
                line(0.0f, 0.0f, 0.0f, 5.4f); line(0.0f, 5.4f, 1.4f, 7.0f);
                line(1.4f, 7.0f, 3.1f, 7.0f); line(3.1f, 7.0f, 4.5f, 5.4f);
                line(4.5f, 5.4f, 4.5f, 0.0f);
                break;
            case 'V':
                line(0.0f, 0.0f, 2.4f, 7.0f); line(2.4f, 7.0f, 4.8f, 0.0f);
                break;
            case 'W':
                line(0.0f, 0.0f, 1.4f, 7.0f); line(1.4f, 7.0f, 2.9f, 2.5f);
                line(2.9f, 2.5f, 4.4f, 7.0f); line(4.4f, 7.0f, 5.8f, 0.0f);
                break;
            case 'X':
                line(0.0f, 0.0f, 4.5f, 7.0f); line(4.5f, 0.0f, 0.0f, 7.0f);
                break;
            case 'Y':
                line(0.0f, 0.0f, 2.25f, 3.5f); line(4.5f, 0.0f, 2.25f, 3.5f);
                line(2.25f, 3.5f, 2.25f, 7.0f);
                break;
            case 'Z':
                line(0.0f, 0.0f, 4.0f, 0.0f); line(4.0f, 0.0f, 0.0f, 7.0f);
                line(0.0f, 7.0f, 4.0f, 7.0f);
                break;

            // Lowercase
            case 'a':
                line(0.8f, 2.5f, 3.2f, 2.5f); line(3.2f, 2.5f, 4.0f, 3.3f);
                line(4.0f, 3.3f, 4.0f, 7.0f); line(4.0f, 4.5f, 1.0f, 4.5f);
                line(1.0f, 4.5f, 0.0f, 5.6f); line(0.0f, 5.6f, 1.2f, 7.0f);
                line(1.2f, 7.0f, 4.0f, 7.0f);
                break;
            case 'b':
                line(0.0f, 0.0f, 0.0f, 7.0f); line(0.0f, 3.8f, 1.2f, 2.5f);
                line(1.2f, 2.5f, 3.0f, 2.5f); line(3.0f, 2.5f, 4.0f, 3.8f);
                line(4.0f, 3.8f, 4.0f, 5.7f); line(4.0f, 5.7f, 3.0f, 7.0f);
                line(3.0f, 7.0f, 0.0f, 7.0f);
                break;
            case 'c':
                line(3.6f, 3.5f, 2.5f, 2.5f); line(2.5f, 2.5f, 1.2f, 2.5f);
                line(1.2f, 2.5f, 0.0f, 3.8f); line(0.0f, 3.8f, 0.0f, 5.7f);
                line(0.0f, 5.7f, 1.2f, 7.0f); line(1.2f, 7.0f, 2.5f, 7.0f);
                line(2.5f, 7.0f, 3.6f, 6.0f);
                break;
            case 'd':
                line(4.0f, 0.0f, 4.0f, 7.0f); line(4.0f, 5.7f, 3.0f, 7.0f);
                line(3.0f, 7.0f, 1.2f, 7.0f); line(1.2f, 7.0f, 0.0f, 5.7f);
                line(0.0f, 5.7f, 0.0f, 3.8f); line(0.0f, 3.8f, 1.2f, 2.5f);
                line(1.2f, 2.5f, 4.0f, 2.5f);
                break;
            case 'e':
                line(0.0f, 4.8f, 3.8f, 4.8f); line(3.8f, 4.8f, 3.8f, 3.6f);
                line(3.8f, 3.6f, 2.5f, 2.5f); line(2.5f, 2.5f, 1.2f, 2.5f);
                line(1.2f, 2.5f, 0.0f, 3.8f); line(0.0f, 3.8f, 0.0f, 5.7f);
                line(0.0f, 5.7f, 1.2f, 7.0f); line(1.2f, 7.0f, 2.6f, 7.0f);
                line(2.6f, 7.0f, 3.8f, 6.0f);
                break;
            case 'f':
                line(2.8f, 1.0f, 1.8f, 0.0f); line(1.8f, 0.0f, 1.0f, 0.0f);
                line(1.0f, 0.0f, 0.8f, 1.0f); line(0.8f, 1.0f, 0.8f, 7.0f);
                line(0.0f, 2.5f, 2.8f, 2.5f);
                break;
            case 'g':
                line(4.0f, 2.5f, 4.0f, 8.2f); line(4.0f, 8.2f, 2.8f, 9.2f);
                line(2.8f, 9.2f, 1.0f, 9.2f); line(1.0f, 9.2f, 0.0f, 8.2f);
                line(4.0f, 3.8f, 3.0f, 2.5f); line(3.0f, 2.5f, 1.2f, 2.5f);
                line(1.2f, 2.5f, 0.0f, 3.8f); line(0.0f, 3.8f, 0.0f, 5.7f);
                line(0.0f, 5.7f, 1.2f, 7.0f); line(1.2f, 7.0f, 3.0f, 7.0f);
                line(3.0f, 7.0f, 4.0f, 5.7f);
                break;
            case 'h':
                line(0.0f, 0.0f, 0.0f, 7.0f); line(0.0f, 3.8f, 1.2f, 2.5f);
                line(1.2f, 2.5f, 2.8f, 2.5f); line(2.8f, 2.5f, 3.8f, 3.8f);
                line(3.8f, 3.8f, 3.8f, 7.0f);
                break;
            case 'i':
                line(0.8f, 0.6f, 0.8f, 1.2f); line(0.8f, 2.5f, 0.8f, 7.0f);
                break;
            case 'j':
                line(1.2f, 0.6f, 1.2f, 1.2f); line(1.2f, 2.5f, 1.2f, 8.2f);
                line(1.2f, 8.2f, 0.0f, 9.2f);
                break;
            case 'k':
                line(0.0f, 0.0f, 0.0f, 7.0f); line(3.6f, 2.5f, 0.0f, 5.0f);
                line(1.4f, 4.0f, 3.6f, 7.0f);
                break;
            case 'l':
                line(0.8f, 0.0f, 0.8f, 6.4f); line(0.8f, 6.4f, 1.6f, 7.0f);
                break;
            case 'm':
                line(0.0f, 2.5f, 0.0f, 7.0f); line(0.0f, 3.8f, 1.0f, 2.5f);
                line(1.0f, 2.5f, 2.2f, 2.5f); line(2.2f, 2.5f, 2.9f, 3.8f);
                line(2.9f, 3.8f, 2.9f, 7.0f); line(2.9f, 3.8f, 3.9f, 2.5f);
                line(3.9f, 2.5f, 5.0f, 2.5f); line(5.0f, 2.5f, 5.8f, 3.8f);
                line(5.8f, 3.8f, 5.8f, 7.0f);
                break;
            case 'n':
                line(0.0f, 2.5f, 0.0f, 7.0f); line(0.0f, 3.8f, 1.2f, 2.5f);
                line(1.2f, 2.5f, 2.8f, 2.5f); line(2.8f, 2.5f, 3.8f, 3.8f);
                line(3.8f, 3.8f, 3.8f, 7.0f);
                break;
            case 'o':
                line(1.2f, 2.5f, 2.6f, 2.5f); line(2.6f, 2.5f, 3.8f, 3.8f);
                line(3.8f, 3.8f, 3.8f, 5.7f); line(3.8f, 5.7f, 2.6f, 7.0f);
                line(2.6f, 7.0f, 1.2f, 7.0f); line(1.2f, 7.0f, 0.0f, 5.7f);
                line(0.0f, 5.7f, 0.0f, 3.8f); line(0.0f, 3.8f, 1.2f, 2.5f);
                break;
            case 'p':
                line(0.0f, 2.5f, 0.0f, 9.2f); line(0.0f, 3.8f, 1.2f, 2.5f);
                line(1.2f, 2.5f, 3.0f, 2.5f); line(3.0f, 2.5f, 4.0f, 3.8f);
                line(4.0f, 3.8f, 4.0f, 5.7f); line(4.0f, 5.7f, 3.0f, 7.0f);
                line(3.0f, 7.0f, 0.0f, 7.0f);
                break;
            case 'q':
                line(4.0f, 2.5f, 4.0f, 9.2f); line(4.0f, 5.7f, 3.0f, 7.0f);
                line(3.0f, 7.0f, 1.2f, 7.0f); line(1.2f, 7.0f, 0.0f, 5.7f);
                line(0.0f, 5.7f, 0.0f, 3.8f); line(0.0f, 3.8f, 1.2f, 2.5f);
                line(1.2f, 2.5f, 4.0f, 2.5f);
                break;
            case 'r':
                line(0.0f, 2.5f, 0.0f, 7.0f); line(0.0f, 4.0f, 1.0f, 2.7f);
                line(1.0f, 2.7f, 2.2f, 2.5f); line(2.2f, 2.5f, 2.8f, 3.2f);
                break;
            case 's':
                line(3.4f, 3.5f, 2.4f, 2.5f); line(2.4f, 2.5f, 1.0f, 2.5f);
                line(1.0f, 2.5f, 0.0f, 3.5f); line(0.0f, 3.5f, 1.0f, 4.8f);
                line(1.0f, 4.8f, 2.4f, 4.8f); line(2.4f, 4.8f, 3.4f, 5.8f);
                line(3.4f, 5.8f, 2.4f, 7.0f); line(2.4f, 7.0f, 1.0f, 7.0f);
                line(1.0f, 7.0f, 0.0f, 6.0f);
                break;
            case 't':
                line(1.0f, 0.8f, 1.0f, 6.2f); line(1.0f, 6.2f, 2.0f, 7.0f);
                line(2.0f, 7.0f, 2.8f, 6.8f); line(0.0f, 2.5f, 2.8f, 2.5f);
                break;
            case 'u':
                line(0.0f, 2.5f, 0.0f, 5.7f); line(0.0f, 5.7f, 1.2f, 7.0f);
                line(1.2f, 7.0f, 2.8f, 7.0f); line(2.8f, 7.0f, 3.8f, 5.7f);
                line(3.8f, 2.5f, 3.8f, 7.0f);
                break;
            case 'v':
                line(0.0f, 2.5f, 1.8f, 7.0f); line(1.8f, 7.0f, 3.6f, 2.5f);
                break;
            case 'w':
                line(0.0f, 2.5f, 1.4f, 7.0f); line(1.4f, 7.0f, 2.9f, 4.0f);
                line(2.9f, 4.0f, 4.4f, 7.0f); line(4.4f, 7.0f, 5.8f, 2.5f);
                break;
            case 'x':
                line(0.0f, 2.5f, 3.6f, 7.0f); line(3.6f, 2.5f, 0.0f, 7.0f);
                break;
            case 'y':
                line(0.0f, 2.5f, 1.8f, 7.0f); line(3.6f, 2.5f, 1.8f, 7.0f);
                line(1.8f, 7.0f, 0.8f, 9.2f); line(0.8f, 9.2f, 0.0f, 8.8f);
                break;
            case 'z':
                line(0.0f, 2.5f, 3.4f, 2.5f); line(3.4f, 2.5f, 0.0f, 7.0f);
                line(0.0f, 7.0f, 3.4f, 7.0f);
                break;

            // Symbols & Punctuation
            case '+':
                line(0.4f, 3.5f, 3.8f, 3.5f); line(2.1f, 1.8f, 2.1f, 5.2f);
                break;
            case '-':
                line(0.5f, 3.5f, 3.7f, 3.5f);
                break;
            case '.':
                line(0.8f, 6.0f, 0.8f, 7.0f);
                break;
            case ',':
                line(0.8f, 5.6f, 0.8f, 6.8f); line(0.8f, 6.8f, 0.2f, 8.0f);
                break;
            case ':':
                line(0.8f, 2.0f, 0.8f, 2.8f); line(0.8f, 5.2f, 0.8f, 6.0f);
                break;
            case ';':
                line(0.8f, 2.0f, 0.8f, 2.8f); line(0.8f, 5.2f, 0.8f, 6.2f);
                line(0.8f, 6.2f, 0.2f, 7.4f);
                break;
            case '[':
                line(2.4f, 0.0f, 0.6f, 0.0f); line(0.6f, 0.0f, 0.6f, 7.0f);
                line(0.6f, 7.0f, 2.4f, 7.0f);
                break;
            case ']':
                line(0.4f, 0.0f, 2.2f, 0.0f); line(2.2f, 0.0f, 2.2f, 7.0f);
                line(2.2f, 7.0f, 0.4f, 7.0f);
                break;
            case '(':
                line(2.2f, 0.0f, 0.8f, 2.2f); line(0.8f, 2.2f, 0.8f, 4.8f);
                line(0.8f, 4.8f, 2.2f, 7.0f);
                break;
            case ')':
                line(0.6f, 0.0f, 2.0f, 2.2f); line(2.0f, 2.2f, 2.0f, 4.8f);
                line(2.0f, 4.8f, 0.6f, 7.0f);
                break;
            case '<':
                line(3.6f, 1.2f, 0.6f, 3.5f); line(0.6f, 3.5f, 3.6f, 5.8f);
                break;
            case '>':
                line(0.6f, 1.2f, 3.6f, 3.5f); line(3.6f, 3.5f, 0.6f, 5.8f);
                break;
            case '/':
                line(0.6f, 7.0f, 3.6f, 0.0f);
                break;
            case '\\':
                line(0.6f, 0.0f, 3.6f, 7.0f);
                break;
            case '=':
                line(0.5f, 2.6f, 3.7f, 2.6f); line(0.5f, 4.4f, 3.7f, 4.4f);
                break;
            case '!':
                line(0.8f, 0.0f, 0.8f, 4.6f); line(0.8f, 6.0f, 0.8f, 7.0f);
                break;
            case '?':
                line(0.4f, 1.6f, 1.4f, 0.0f); line(1.4f, 0.0f, 2.8f, 0.0f);
                line(2.8f, 0.0f, 3.8f, 1.4f); line(3.8f, 1.4f, 3.8f, 2.6f);
                line(3.8f, 2.6f, 2.0f, 3.8f); line(2.0f, 3.8f, 2.0f, 4.8f);
                line(2.0f, 6.0f, 2.0f, 7.0f);
                break;
            case '%':
                line(0.6f, 7.0f, 4.2f, 0.0f);
                line(0.8f, 0.8f, 1.8f, 0.8f); line(1.8f, 0.8f, 1.8f, 2.2f);
                line(1.8f, 2.2f, 0.8f, 2.2f); line(0.8f, 2.2f, 0.8f, 0.8f);
                line(3.0f, 4.8f, 4.0f, 4.8f); line(4.0f, 4.8f, 4.0f, 6.2f);
                line(4.0f, 6.2f, 3.0f, 6.2f); line(3.0f, 6.2f, 3.0f, 4.8f);
                break;
            case '_':
                line(0.0f, 7.2f, 4.0f, 7.2f);
                break;
            case '|':
                line(0.8f, 0.0f, 0.8f, 7.0f);
                break;
            case '^':
                line(0.6f, 3.2f, 2.1f, 0.5f); line(2.1f, 0.5f, 3.6f, 3.2f);
                break;
            case '~':
                line(0.4f, 3.8f, 1.2f, 2.8f); line(1.2f, 2.8f, 2.0f, 3.8f);
                line(2.0f, 3.8f, 2.8f, 2.8f); line(2.8f, 2.8f, 3.6f, 3.8f);
                break;
            case '*':
                line(0.6f, 3.5f, 3.6f, 3.5f); line(2.1f, 2.0f, 2.1f, 5.0f);
                line(1.0f, 2.3f, 3.2f, 4.7f); line(1.0f, 4.7f, 3.2f, 2.3f);
                break;
            case '\'':
                line(0.8f, 0.0f, 0.8f, 2.2f);
                break;
            case '"':
                line(0.8f, 0.0f, 0.8f, 2.2f); line(2.0f, 0.0f, 2.0f, 2.2f);
                break;
            case ' ':
            default:
                break;
        }

        curX += (charW * scale) + spacing;
    }
}

void Renderer2D::drawStatusOverlay(const StatusInfo& info, int windowW, int windowH)
{
    // HUD Card Background (anchored to top-right corner)
    const float cardW = 300.0f;
    const float cardH = 152.0f;
    const float cardX = std::max(14.0f, static_cast<float>(windowW) - cardW - 14.0f);
    const float cardY = 14.0f;

    const Color bgFill   = Color::fromRgb(0x1B1E26, 0.90f);
    const Color bgBorder = Color::fromRgb(0x434C5E, 0.95f);
    const Color textCol  = Color::fromRgb(0xD8DEE9, 1.00f);
    const Color titleCol = Color::fromRgb(0x88C0D0, 1.00f);
    const Color valCol   = Color::fromRgb(0xEBCB8B, 1.00f);
    const Color dimCol   = Color::fromRgb(0x7B88A1, 1.00f);

    drawScreenQuad(cardX, cardY, cardW, cardH, bgFill);

    // Card border lines
    drawScreenLine(cardX, cardY, cardX + cardW, cardY, bgBorder);
    drawScreenLine(cardX + cardW, cardY, cardX + cardW, cardY + cardH, bgBorder);
    drawScreenLine(cardX + cardW, cardY + cardH, cardX, cardY + cardH, bgBorder);
    drawScreenLine(cardX, cardY + cardH, cardX, cardY, bgBorder);

    char buf[128];
    float lineY = cardY + 14.0f;
    const float lineSpacing = 19.0f;

    // Header Title (Bold)
    std::snprintf(buf, sizeof(buf), "Physics Engine 2D");
    drawScreenText(buf, cardX + 14.0f, lineY, 1.55f, titleCol, true);
    lineY += lineSpacing + 2.0f;

    // Bodies
    std::snprintf(buf, sizeof(buf), "Bodies: %u  (Dyn: %u, Stat: %u)",
                  info.totalBodies, info.dynamicBodies, info.staticBodies);
    drawScreenText(buf, cardX + 14.0f, lineY, 1.25f, textCol, false);
    lineY += lineSpacing;

    // Contacts
    std::snprintf(buf, sizeof(buf), "Contacts: %zu  %s",
                  info.contactCount, info.showContacts ? "[Contacts: On]" : "[Contacts: Off]");
    drawScreenText(buf, cardX + 14.0f, lineY, 1.25f, valCol, false);
    lineY += lineSpacing;

    // Timing & FPS
    std::snprintf(buf, sizeof(buf), "Step: %.2f ms | FPS: %.1f",
                  info.physicsStepMs, info.fps);
    drawScreenText(buf, cardX + 14.0f, lineY, 1.25f, textCol, false);
    lineY += lineSpacing;

    // Broad-Phase & State
    std::snprintf(buf, sizeof(buf), "Broad-Phase: %s  %s",
                  info.broadPhaseName, info.isPaused ? "[Paused]" : "[Running]");
    drawScreenText(buf, cardX + 14.0f, lineY, 1.25f, titleCol, false);
    lineY += lineSpacing;

    // Controls hint
    std::snprintf(buf, sizeof(buf), "Pan: Drag | Zoom: Scroll | Reset: [R]");
    drawScreenText(buf, cardX + 14.0f, lineY, 1.15f, dimCol, false);
}

void Renderer2D::flushWorld(const Camera2D& camera)
{
    float proj[16];
    camera.getProjectionMatrix(proj);
    shader_.setMat4("uProjection", proj);

    glBindVertexArray(vao_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);

    // Draw filled world triangles
    if (!worldTriangles_.empty()) {
        glBufferData(GL_ARRAY_BUFFER,
                     static_cast<GLsizeiptr>(worldTriangles_.size() * sizeof(Vertex)),
                     worldTriangles_.data(), GL_DYNAMIC_DRAW);
        glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(worldTriangles_.size()));
    }

    // Draw world lines
    if (!worldLines_.empty()) {
        glBufferData(GL_ARRAY_BUFFER,
                     static_cast<GLsizeiptr>(worldLines_.size() * sizeof(Vertex)),
                     worldLines_.data(), GL_DYNAMIC_DRAW);
        glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(worldLines_.size()));
    }
}

void Renderer2D::flushScreen(const Camera2D& camera)
{
    if (screenTriangles_.empty() && screenLines_.empty()) {
        return;
    }

    float proj[16];
    camera.getScreenProjectionMatrix(proj);
    shader_.setMat4("uProjection", proj);

    glBindVertexArray(vao_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);

    // Draw screen quads
    if (!screenTriangles_.empty()) {
        glBufferData(GL_ARRAY_BUFFER,
                     static_cast<GLsizeiptr>(screenTriangles_.size() * sizeof(Vertex)),
                     screenTriangles_.data(), GL_DYNAMIC_DRAW);
        glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(screenTriangles_.size()));
    }

    // Draw screen text lines
    if (!screenLines_.empty()) {
        glBufferData(GL_ARRAY_BUFFER,
                     static_cast<GLsizeiptr>(screenLines_.size() * sizeof(Vertex)),
                     screenLines_.data(), GL_DYNAMIC_DRAW);
        glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(screenLines_.size()));
    }
}

void Renderer2D::end(const Camera2D& camera)
{
    shader_.use();

    // Enable standard alpha blending
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDisable(GL_DEPTH_TEST);

    flushWorld(camera);
    flushScreen(camera);

    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

} // namespace PhysicsEngine
