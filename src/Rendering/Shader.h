#pragma once

// Shader.h -- OpenGL 3.3 Core shader program abstraction.
//
// Handles shader compilation, program linking, error diagnostics,
// uniform binding, and RAII resource destruction before context teardown.

#include <cstdint>

namespace PhysicsEngine {

class Shader {
public:
    Shader() = default;
    ~Shader();

    Shader(const Shader&) = delete;
    Shader& operator=(const Shader&) = delete;
    Shader(Shader&& other) noexcept;
    Shader& operator=(Shader&& other) noexcept;

    /// Compiles vertex and fragment shaders from source strings and links program.
    /// Logs detailed diagnostics to stderr if compilation or linking fails.
    /// \return True on success, false on error.
    bool loadFromSource(const char* vertexSource, const char* fragmentSource);

    /// Binds this shader program for subsequent rendering commands.
    void use() const;

    /// Explicitly frees the OpenGL program resource.
    void destroy();

    [[nodiscard]] uint32_t id() const noexcept { return programId_; }
    [[nodiscard]] bool isValid() const noexcept { return programId_ != 0; }

    // Uniform setters
    void setMat4(const char* name, const float* mat16) const;
    void setVec4(const char* name, float x, float y, float z, float w) const;
    void setVec2(const char* name, float x, float y) const;
    void setFloat(const char* name, float val) const;
    void setInt(const char* name, int val) const;

private:
    uint32_t programId_ = 0;
};

} // namespace PhysicsEngine
