#pragma once

// UI.h -- Lightweight immediate-mode 2D GUI for PhysicsEngine2D Sandbox.
//
// Features:
//   - Zero external GUI dependencies (uses Renderer2D primitives and vector font).
//   - Full hit-testing and pointer capture: ensures world clicks and camera gestures
//     never trigger when interacting with UI panels.
//   - Reusable widgets: Buttons, Toggles, Radio choices, Number steppers, Section headers.
//   - Clean state retention and hover/active visual feedback.

#include "Rendering/Renderer2D.h"

#include <cstdint>
#include <string>
#include <vector>

namespace PhysicsEngine {

struct UIInput {
    float mouseX          = 0.f;
    float mouseY          = 0.f;
    bool  mouseDown       = false; // Left mouse currently held down
    bool  mouseClicked    = false; // Left mouse pressed this exact frame
    bool  mouseReleased   = false; // Left mouse released this frame
    float scrollDeltaY    = 0.f;   // Scroll wheel delta this frame
};

class UIContext {
public:
    UIContext() = default;

    /// Begins a new UI frame. Call before drawing any widgets.
    void begin(const UIInput& input);

    /// Ends the UI frame and builds draw lists into Renderer2D.
    void render(Renderer2D& renderer);

    /// Returns true if the pointer is currently hovering over or interacting with any UI element.
    [[nodiscard]] bool isMouseCaptured() const noexcept { return isCaptured_; }

    // ── Layout & Containers ──────────────────────────────────────────────────

    void beginPanel(const char* title, float x, float y, float w, float h);
    void endPanel();

    void sectionHeader(const char* title);
    void separator();
    void label(const char* text, Color color = Color::fromRgb(0xD8DEE9));

    // ── Interactive Widgets ──────────────────────────────────────────────────

    /// Interactive push button. Returns true on click release.
    bool button(const char* label, bool isSelected = false, float customWidth = 0.f);

    /// Checkbox toggle. Modifies *value and returns true if changed.
    bool toggle(const char* label, bool* value);

    /// Horizontal segmented radio selector. Modifies *selectedIdx and returns true if changed.
    bool choice(const char* label, const char* options[], int count, int* selectedIdx);

    /// Floating-point value stepper with [-] and [+] decrement/increment buttons.
    bool stepper(const char* label, float* value, float step, float minVal, float maxVal, const char* format = "%.2f");

    /// Integer value stepper with [-] and [+] buttons.
    bool intStepper(const char* label, int* value, int step, int minVal, int maxVal);

private:
    struct Panel {
        float x = 0.f;
        float y = 0.f;
        float w = 0.f;
        float h = 0.f;
        float cursorX = 0.f;
        float cursorY = 0.f;
        std::string title;
    };

    struct DrawCmd {
        enum class Type { Quad, Line, Text };
        Type        type;
        float       x1, y1, x2, y2;
        Color       color;
        std::string text;
        float       scale;
        bool        bold = false;
    };

    bool isHovered(float x, float y, float w, float h) const noexcept;

    UIInput              input_;
    bool                 isCaptured_   = false;
    float                panelScrollY_ = 0.f;
    float                maxScroll_    = 0.f;
    std::vector<Panel>   panelStack_;
    std::vector<DrawCmd> drawCommands_;

    static constexpr float kDefaultItemHeight = 26.f;
    static constexpr float kItemSpacing       = 7.f;
};

} // namespace PhysicsEngine
