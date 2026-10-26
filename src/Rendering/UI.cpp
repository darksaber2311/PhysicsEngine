// UI.cpp -- Immediate-mode 2D GUI implementation for PhysicsEngine2D.

#include "Rendering/UI.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace PhysicsEngine {

namespace {

const Color kPanelBg        = Color::fromRgb(0x181A20, 0.94f);
const Color kPanelBorder    = Color::fromRgb(0x353B45, 0.95f);
const Color kBtnNormal      = Color::fromRgb(0x2E3440, 0.90f);
const Color kBtnHover       = Color::fromRgb(0x3B4252, 0.95f);
const Color kBtnActive      = Color::fromRgb(0x4C6A92, 0.95f);
const Color kBtnBorder      = Color::fromRgb(0x434C5E, 0.90f);
const Color kBtnBorderAct   = Color::fromRgb(0x88C0D0, 1.00f);
const Color kTextWhite      = Color::fromRgb(0xE5E9F0, 1.00f);
const Color kTextDim        = Color::fromRgb(0x828D9F, 1.00f);
const Color kTextCyan       = Color::fromRgb(0x88C0D0, 1.00f);
const Color kTextGold       = Color::fromRgb(0xEBCB8B, 1.00f);
const Color kTextGreen      = Color::fromRgb(0xA3BE8C, 1.00f);

} // anonymous namespace

void UIContext::begin(const UIInput& input)
{
    input_ = input;
    isCaptured_ = false;
    panelStack_.clear();
    drawCommands_.clear();
}

bool UIContext::isHovered(float x, float y, float w, float h) const noexcept
{
    return (input_.mouseX >= x && input_.mouseX <= x + w &&
            input_.mouseY >= y && input_.mouseY <= y + h);
}

void UIContext::beginPanel(const char* title, float x, float y, float w, float h)
{
    const bool panelHovered = isHovered(x, y, w, h);
    if (panelHovered) {
        isCaptured_ = true;
        if (input_.scrollDeltaY != 0.f) {
            panelScrollY_ -= input_.scrollDeltaY * 28.0f;
        }
    }
    panelScrollY_ = std::clamp(panelScrollY_, 0.0f, maxScroll_);

    Panel p;
    p.x = x;
    p.y = y;
    p.w = w;
    p.h = h;
    p.cursorX = x + 14.0f;
    p.cursorY = y + 44.0f;
    p.title   = title ? title : "";
    panelStack_.push_back(p);

    // Panel background & outer border
    drawCommands_.push_back({DrawCmd::Type::Quad, x, y, x + w, y + h, kPanelBg, "", 0.f, false});
    drawCommands_.push_back({DrawCmd::Type::Line, x, y, x + w, y, kPanelBorder, "", 0.f, false});
    drawCommands_.push_back({DrawCmd::Type::Line, x + w, y, x + w, y + h, kPanelBorder, "", 0.f, false});
    drawCommands_.push_back({DrawCmd::Type::Line, x + w, y + h, x, y + h, kPanelBorder, "", 0.f, false});
    drawCommands_.push_back({DrawCmd::Type::Line, x, y + h, x, y, kPanelBorder, "", 0.f, false});
}

void UIContext::endPanel()
{
    if (panelStack_.empty()) return;
    const Panel p = panelStack_.back();
    panelStack_.pop_back();

    // 1. Title banner quad and separator (rendered on top of scroll content)
    drawCommands_.push_back({DrawCmd::Type::Quad, p.x + 1.0f, p.y + 1.0f, p.x + p.w - 1.0f, p.y + 36.0f, kPanelBg, "", 0.f, false});
    if (!p.title.empty()) {
        drawCommands_.push_back({DrawCmd::Type::Text, p.x + 14.0f, p.y + 11.0f, 0.f, 0.f, kTextCyan, p.title, 1.45f, true});
        drawCommands_.push_back({DrawCmd::Type::Line, p.x + 10.0f, p.y + 36.0f, p.x + p.w - 10.0f, p.y + 36.0f, kPanelBorder, "", 0.f, false});
    }

    // 2. Compute content height and scroll range
    const float contentHeight = p.cursorY - (p.y + 44.0f);
    maxScroll_ = std::max(0.0f, contentHeight - (p.h - 52.0f));

    // 3. Scrollbar track & thumb when content exceeds panel view
    if (maxScroll_ > 0.0f) {
        const float trackTop = p.y + 40.0f;
        const float trackH   = p.h - 48.0f;
        const float thumbH   = std::max(20.0f, trackH * (trackH / (contentHeight + 20.0f)));
        const float thumbY   = trackTop + (panelScrollY_ / maxScroll_) * (trackH - thumbH);

        drawCommands_.push_back({DrawCmd::Type::Quad, p.x + p.w - 7.0f, trackTop, p.x + p.w - 3.0f, trackTop + trackH, kBtnNormal, "", 0.f, false});
        drawCommands_.push_back({DrawCmd::Type::Quad, p.x + p.w - 7.0f, thumbY, p.x + p.w - 3.0f, thumbY + thumbH, kBtnBorderAct, "", 0.f, false});
    }
}

void UIContext::sectionHeader(const char* title)
{
    if (panelStack_.empty()) return;
    auto& p = panelStack_.back();

    p.cursorY += 8.0f;
    const float scrY = p.cursorY - panelScrollY_;
    if (scrY >= p.y + 36.0f && scrY <= p.y + p.h - 6.0f) {
        drawCommands_.push_back({DrawCmd::Type::Line, p.x + 10.0f, scrY, p.x + p.w - 10.0f, scrY, Color::fromRgb(0x3B4252), "", 0.f, false});
    }
    p.cursorY += 8.0f;

    if (title) {
        const float textY = p.cursorY - panelScrollY_;
        if (textY >= p.y + 36.0f && textY <= p.y + p.h - 10.0f) {
            drawCommands_.push_back({DrawCmd::Type::Text, p.cursorX, textY, 0.f, 0.f, kTextGold, title, 1.35f, true});
        }
        p.cursorY += 18.0f;
    }
}

void UIContext::separator()
{
    if (panelStack_.empty()) return;
    auto& p = panelStack_.back();

    p.cursorY += 6.0f;
    const float scrY = p.cursorY - panelScrollY_;
    if (scrY >= p.y + 36.0f && scrY <= p.y + p.h - 4.0f) {
        drawCommands_.push_back({DrawCmd::Type::Line, p.x + 10.0f, scrY, p.x + p.w - 10.0f, scrY, kPanelBorder, "", 0.f, false});
    }
    p.cursorY += 8.0f;
}

void UIContext::label(const char* text, Color color)
{
    if (panelStack_.empty() || !text) return;
    auto& p = panelStack_.back();

    const float scrY = p.cursorY - panelScrollY_;
    if (scrY >= p.y + 36.0f && scrY <= p.y + p.h - 6.0f) {
        drawCommands_.push_back({DrawCmd::Type::Text, p.cursorX, scrY, 0.f, 0.f, color, text, 1.25f, false});
    }
    p.cursorY += 18.0f;
}

bool UIContext::button(const char* labelText, bool isSelected, float customWidth)
{
    if (panelStack_.empty()) return false;
    auto& p = panelStack_.back();

    const float btnX = p.cursorX;
    const float btnY = p.cursorY - panelScrollY_;
    const float btnW = (customWidth > 0.f) ? customWidth : (p.w - 28.0f);
    const float btnH = kDefaultItemHeight;

    const bool isVisible = (btnY + btnH >= p.y + 36.0f && btnY <= p.y + p.h - 2.0f);
    const bool hovered   = isVisible && isHovered(btnX, btnY, btnW, btnH);
    const bool active    = hovered && input_.mouseDown;
    if (hovered) {
        isCaptured_ = true;
    }

    const bool clicked = hovered && input_.mouseClicked;

    if (isVisible) {
        Color bg     = isSelected ? kBtnActive : (active ? kBtnActive : (hovered ? kBtnHover : kBtnNormal));
        Color border = isSelected ? kBtnBorderAct : (active ? kBtnBorderAct : (hovered ? kTextWhite : kBtnBorder));
        Color textCol= isSelected ? kTextWhite : (hovered ? kTextWhite : Color::fromRgb(0xD8DEE9));

        drawCommands_.push_back({DrawCmd::Type::Quad, btnX, btnY, btnX + btnW, btnY + btnH, bg, "", 0.f, false});
        drawCommands_.push_back({DrawCmd::Type::Line, btnX, btnY, btnX + btnW, btnY, border, "", 0.f, false});
        drawCommands_.push_back({DrawCmd::Type::Line, btnX + btnW, btnY, btnX + btnW, btnY + btnH, border, "", 0.f, false});
        drawCommands_.push_back({DrawCmd::Type::Line, btnX + btnW, btnY + btnH, btnX, btnY + btnH, border, "", 0.f, false});
        drawCommands_.push_back({DrawCmd::Type::Line, btnX, btnY + btnH, btnX, btnY, border, "", 0.f, false});

        if (labelText) {
            const float labelW = Renderer2D::measureTextWidth(labelText, 1.25f);
            const float textX  = btnX + std::max(8.0f, (btnW - labelW) * 0.5f);
            const float textY  = btnY + (btnH - 9.0f) * 0.5f;
            drawCommands_.push_back({DrawCmd::Type::Text, textX, textY, 0.f, 0.f, textCol, labelText, 1.25f, true});
        }
    }

    p.cursorY += btnH + kItemSpacing;
    return clicked;
}

bool UIContext::toggle(const char* labelText, bool* value)
{
    if (panelStack_.empty() || !value) return false;
    auto& p = panelStack_.back();

    const float rowX = p.cursorX;
    const float rowY = p.cursorY - panelScrollY_;
    const float rowW = p.w - 28.0f;
    const float rowH = 24.0f;
    const float boxSize = 18.0f;
    const float boxY = rowY + (rowH - boxSize) * 0.5f;

    const bool isVisible = (rowY + rowH >= p.y + 36.0f && rowY <= p.y + p.h - 2.0f);
    const bool hovered   = isVisible && isHovered(rowX, rowY, rowW, rowH);
    if (hovered) {
        isCaptured_ = true;
    }

    bool changed = false;
    if (hovered && input_.mouseClicked) {
        *value = !(*value);
        changed = true;
    }

    if (isVisible) {
        Color boxBg  = *value ? kBtnActive : (hovered ? kBtnHover : kBtnNormal);
        Color border = *value ? kBtnBorderAct : (hovered ? kTextWhite : kBtnBorder);

        drawCommands_.push_back({DrawCmd::Type::Quad, rowX, boxY, rowX + boxSize, boxY + boxSize, boxBg, "", 0.f, false});
        drawCommands_.push_back({DrawCmd::Type::Line, rowX, boxY, rowX + boxSize, boxY, border, "", 0.f, false});
        drawCommands_.push_back({DrawCmd::Type::Line, rowX + boxSize, boxY, rowX + boxSize, boxY + boxSize, border, "", 0.f, false});
        drawCommands_.push_back({DrawCmd::Type::Line, rowX + boxSize, boxY + boxSize, rowX, boxY + boxSize, border, "", 0.f, false});
        drawCommands_.push_back({DrawCmd::Type::Line, rowX, boxY + boxSize, rowX, boxY, border, "", 0.f, false});

        if (*value) {
            // Crisp, bold green checkmark
            drawCommands_.push_back({DrawCmd::Type::Line, rowX + 3.5f, boxY + 9.0f, rowX + 7.5f, boxY + 14.0f, kTextGreen, "", 0.f, false});
            drawCommands_.push_back({DrawCmd::Type::Line, rowX + 3.5f, boxY + 10.0f, rowX + 7.5f, boxY + 15.0f, kTextGreen, "", 0.f, false});
            drawCommands_.push_back({DrawCmd::Type::Line, rowX + 7.5f, boxY + 14.0f, rowX + 14.5f, boxY + 4.0f, kTextGreen, "", 0.f, false});
            drawCommands_.push_back({DrawCmd::Type::Line, rowX + 7.5f, boxY + 15.0f, rowX + 14.5f, boxY + 5.0f, kTextGreen, "", 0.f, false});
        }

        if (labelText) {
            const float textX = rowX + boxSize + 10.0f;
            const float textY = rowY + (rowH - 9.0f) * 0.5f;
            drawCommands_.push_back({DrawCmd::Type::Text, textX, textY, 0.f, 0.f,
                                     hovered ? kTextWhite : Color::fromRgb(0xD8DEE9), labelText, 1.25f, false});
        }
    }

    p.cursorY += rowH + kItemSpacing;
    return changed;
}

bool UIContext::choice(const char* labelText, const char* options[], int count, int* selectedIdx)
{
    if (panelStack_.empty() || !options || count <= 0 || !selectedIdx) return false;
    auto& p = panelStack_.back();

    if (labelText && labelText[0] != '\0') {
        const float textY = p.cursorY - panelScrollY_;
        if (textY >= p.y + 36.0f && textY <= p.y + p.h - 6.0f) {
            drawCommands_.push_back({DrawCmd::Type::Text, p.cursorX, textY, 0.f, 0.f, kTextDim, labelText, 1.20f, false});
        }
        p.cursorY += 16.0f;
    }

    const float totalW   = p.w - 28.0f;
    const float gap      = 5.0f;
    const float optW     = (totalW - (count - 1) * gap) / static_cast<float>(count);
    const float optH     = 24.0f;
    bool changed         = false;

    for (int i = 0; i < count; ++i) {
        const float optX = p.cursorX + i * (optW + gap);
        const float optY = p.cursorY - panelScrollY_;

        const bool isVisible = (optY + optH >= p.y + 36.0f && optY <= p.y + p.h - 2.0f);
        const bool isSelected = (*selectedIdx == i);
        const bool hovered    = isVisible && isHovered(optX, optY, optW, optH);
        const bool active     = hovered && input_.mouseDown;
        if (hovered) {
            isCaptured_ = true;
        }

        if (hovered && input_.mouseClicked) {
            *selectedIdx = i;
            changed = true;
        }

        if (isVisible) {
            Color bg     = isSelected ? kBtnActive : (active ? kBtnActive : (hovered ? kBtnHover : kBtnNormal));
            Color border = isSelected ? kBtnBorderAct : (active ? kBtnBorderAct : (hovered ? kTextWhite : kBtnBorder));
            Color textC  = isSelected ? kTextWhite : (hovered ? kTextWhite : kTextDim);

            drawCommands_.push_back({DrawCmd::Type::Quad, optX, optY, optX + optW, optY + optH, bg, "", 0.f, false});
            drawCommands_.push_back({DrawCmd::Type::Line, optX, optY, optX + optW, optY, border, "", 0.f, false});
            drawCommands_.push_back({DrawCmd::Type::Line, optX + optW, optY, optX + optW, optY + optH, border, "", 0.f, false});
            drawCommands_.push_back({DrawCmd::Type::Line, optX + optW, optY + optH, optX, optY + optH, border, "", 0.f, false});
            drawCommands_.push_back({DrawCmd::Type::Line, optX, optY + optH, optX, optY, border, "", 0.f, false});

            const float tw = Renderer2D::measureTextWidth(options[i], 1.20f);
            const float tx = optX + std::max(3.0f, (optW - tw) * 0.5f);
            const float ty = optY + (optH - 9.0f) * 0.5f;
            drawCommands_.push_back({DrawCmd::Type::Text, tx, ty, 0.f, 0.f, textC, options[i], 1.20f, isSelected});
        }
    }

    p.cursorY += optH + kItemSpacing;
    return changed;
}

bool UIContext::stepper(const char* labelText, float* value, float step, float minVal, float maxVal, const char* format)
{
    if (panelStack_.empty() || !value) return false;
    auto& p = panelStack_.back();

    const float rowX  = p.cursorX;
    const float rowY  = p.cursorY - panelScrollY_;
    const float rowW  = p.w - 28.0f;
    const float rowH  = 26.0f;
    const float btnW  = 28.0f;
    const float btnH  = 24.0f;
    const float btnGap = 5.0f;
    const float btnY  = rowY + (rowH - btnH) * 0.5f;
    bool changed      = false;

    const bool isVisible = (rowY + rowH >= p.y + 36.0f && rowY <= p.y + p.h - 2.0f);

    // [-] and [+] button positions on the far right
    const float plusX  = rowX + rowW - btnW;
    const float minusX = plusX - btnGap - btnW;

    const bool minusHover  = isVisible && isHovered(minusX, btnY, btnW, btnH);
    const bool minusActive = minusHover && input_.mouseDown;
    if (minusHover) isCaptured_ = true;
    if (minusHover && input_.mouseClicked) {
        *value = std::clamp(*value - step, minVal, maxVal);
        changed = true;
    }

    const bool plusHover  = isVisible && isHovered(plusX, btnY, btnW, btnH);
    const bool plusActive = plusHover && input_.mouseDown;
    if (plusHover) isCaptured_ = true;
    if (plusHover && input_.mouseClicked) {
        *value = std::clamp(*value + step, minVal, maxVal);
        changed = true;
    }

    if (isVisible) {
        const float textY = rowY + (rowH - 9.0f) * 0.5f;

        // 1. Parameter name on the left (medium readable neutral)
        if (labelText) {
            drawCommands_.push_back({DrawCmd::Type::Text, rowX, textY, 0.f, 0.f, Color::fromRgb(0xD8DEE9), labelText, 1.25f, false});
        }

        // 2. Formatted numeric value and unit (bold cyan accent, right-aligned before [-] button)
        char buf[64];
        std::snprintf(buf, sizeof(buf), format ? format : "%.2f", *value);
        const float valW = Renderer2D::measureTextWidth(buf, 1.25f);
        const float valX = minusX - 10.0f - valW;
        drawCommands_.push_back({DrawCmd::Type::Text, valX, textY, 0.f, 0.f, kTextCyan, buf, 1.25f, true});

        // 3. [-] button box & centered bold vector minus bar
        Color minusBg      = minusActive ? kBtnActive : (minusHover ? kBtnHover : kBtnNormal);
        Color minusBorder  = minusActive ? kBtnBorderAct : (minusHover ? kTextWhite : kBtnBorder);
        Color minusIconCol = minusActive ? kTextWhite : (minusHover ? kTextWhite : kTextCyan);

        drawCommands_.push_back({DrawCmd::Type::Quad, minusX, btnY, minusX + btnW, btnY + btnH, minusBg, "", 0.f, false});
        drawCommands_.push_back({DrawCmd::Type::Line, minusX, btnY, minusX + btnW, btnY, minusBorder, "", 0.f, false});
        drawCommands_.push_back({DrawCmd::Type::Line, minusX + btnW, btnY, minusX + btnW, btnY + btnH, minusBorder, "", 0.f, false});
        drawCommands_.push_back({DrawCmd::Type::Line, minusX + btnW, btnY + btnH, minusX, btnY + btnH, minusBorder, "", 0.f, false});
        drawCommands_.push_back({DrawCmd::Type::Line, minusX, btnY + btnH, minusX, btnY, minusBorder, "", 0.f, false});

        const float mcx = minusX + btnW * 0.5f;
        const float mcy = btnY + btnH * 0.5f;
        drawCommands_.push_back({DrawCmd::Type::Line, mcx - 5.0f, mcy, mcx + 5.0f, mcy, minusIconCol, "", 0.f, false});
        drawCommands_.push_back({DrawCmd::Type::Line, mcx - 5.0f, mcy + 1.0f, mcx + 5.0f, mcy + 1.0f, minusIconCol, "", 0.f, false});

        // 4. [+] button box & centered bold vector plus cross
        Color plusBg      = plusActive ? kBtnActive : (plusHover ? kBtnHover : kBtnNormal);
        Color plusBorder  = plusActive ? kBtnBorderAct : (plusHover ? kTextWhite : kBtnBorder);
        Color plusIconCol = plusActive ? kTextWhite : (plusHover ? kTextWhite : kTextCyan);

        drawCommands_.push_back({DrawCmd::Type::Quad, plusX, btnY, plusX + btnW, btnY + btnH, plusBg, "", 0.f, false});
        drawCommands_.push_back({DrawCmd::Type::Line, plusX, btnY, plusX + btnW, btnY, plusBorder, "", 0.f, false});
        drawCommands_.push_back({DrawCmd::Type::Line, plusX + btnW, btnY, plusX + btnW, btnY + btnH, plusBorder, "", 0.f, false});
        drawCommands_.push_back({DrawCmd::Type::Line, plusX + btnW, btnY + btnH, plusX, btnY + btnH, plusBorder, "", 0.f, false});
        drawCommands_.push_back({DrawCmd::Type::Line, plusX, btnY + btnH, plusX, btnY, plusBorder, "", 0.f, false});

        const float pcx = plusX + btnW * 0.5f;
        const float pcy = btnY + btnH * 0.5f;
        // Horizontal bar
        drawCommands_.push_back({DrawCmd::Type::Line, pcx - 5.0f, pcy, pcx + 5.0f, pcy, plusIconCol, "", 0.f, false});
        drawCommands_.push_back({DrawCmd::Type::Line, pcx - 5.0f, pcy + 1.0f, pcx + 5.0f, pcy + 1.0f, plusIconCol, "", 0.f, false});
        // Vertical bar
        drawCommands_.push_back({DrawCmd::Type::Line, pcx, pcy - 5.0f, pcx, pcy + 5.0f, plusIconCol, "", 0.f, false});
        drawCommands_.push_back({DrawCmd::Type::Line, pcx + 1.0f, pcy - 5.0f, pcx + 1.0f, pcy + 5.0f, plusIconCol, "", 0.f, false});
    }

    p.cursorY += rowH + kItemSpacing;
    return changed;
}

bool UIContext::intStepper(const char* labelText, int* value, int step, int minVal, int maxVal)
{
    float valF = static_cast<float>(*value);
    if (stepper(labelText, &valF, static_cast<float>(step), static_cast<float>(minVal), static_cast<float>(maxVal), "%d")) {
        *value = static_cast<int>(std::round(valF));
        return true;
    }
    return false;
}

void UIContext::render(Renderer2D& renderer)
{
    for (const auto& cmd : drawCommands_) {
        switch (cmd.type) {
            case DrawCmd::Type::Quad:
                renderer.drawScreenQuad(cmd.x1, cmd.y1, cmd.x2 - cmd.x1, cmd.y2 - cmd.y1, cmd.color);
                break;
            case DrawCmd::Type::Line:
                renderer.drawScreenLine(cmd.x1, cmd.y1, cmd.x2, cmd.y2, cmd.color);
                break;
            case DrawCmd::Type::Text:
                renderer.drawScreenText(cmd.text.c_str(), cmd.x1, cmd.y1, cmd.scale, cmd.color, cmd.bold);
                break;
        }
    }
}

} // namespace PhysicsEngine
