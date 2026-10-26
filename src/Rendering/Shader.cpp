// Shader.cpp -- OpenGL 3.3 Core shader program implementation.

#include "Rendering/Shader.h"

#include <glad/gl.h>

#include <cstdio>
#include <utility>
#include <vector>

namespace PhysicsEngine {

namespace {

GLuint compileShader(GLenum type, const char* source)
{
    GLuint shader = glCreateShader(type);
    glShaderSource(shader, 1, &source, nullptr);
    glCompileShader(shader);

    GLint success = 0;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        GLint logLength = 0;
        glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &logLength);
        std::vector<char> infoLog(static_cast<size_t>(logLength > 0 ? logLength : 1024));
        glGetShaderInfoLog(shader, static_cast<GLsizei>(infoLog.size()), nullptr, infoLog.data());
        std::fprintf(stderr, "[OpenGL Shader Error] (%s): %s\n",
                     (type == GL_VERTEX_SHADER ? "VERTEX" : "FRAGMENT"),
                     infoLog.data());
        glDeleteShader(shader);
        return 0;
    }

    return shader;
}

} // anonymous namespace

Shader::~Shader()
{
    destroy();
}

Shader::Shader(Shader&& other) noexcept
    : programId_(other.programId_)
{
    other.programId_ = 0;
}

Shader& Shader::operator=(Shader&& other) noexcept
{
    if (this != &other) {
        destroy();
        programId_ = other.programId_;
        other.programId_ = 0;
    }
    return *this;
}

void Shader::destroy()
{
    if (programId_ != 0) {
        glDeleteProgram(programId_);
        programId_ = 0;
    }
}

bool Shader::loadFromSource(const char* vertexSource, const char* fragmentSource)
{
    destroy();

    GLuint vertShader = compileShader(GL_VERTEX_SHADER, vertexSource);
    if (vertShader == 0) {
        return false;
    }

    GLuint fragShader = compileShader(GL_FRAGMENT_SHADER, fragmentSource);
    if (fragShader == 0) {
        glDeleteShader(vertShader);
        return false;
    }

    GLuint program = glCreateProgram();
    glAttachShader(program, vertShader);
    glAttachShader(program, fragShader);
    glLinkProgram(program);

    // Flag shaders for deletion once detached from linked program
    glDeleteShader(vertShader);
    glDeleteShader(fragShader);

    GLint linked = 0;
    glGetProgramiv(program, GL_LINK_STATUS, &linked);
    if (!linked) {
        GLint logLength = 0;
        glGetProgramiv(program, GL_INFO_LOG_LENGTH, &logLength);
        std::vector<char> infoLog(static_cast<size_t>(logLength > 0 ? logLength : 1024));
        glGetProgramInfoLog(program, static_cast<GLsizei>(infoLog.size()), nullptr, infoLog.data());
        std::fprintf(stderr, "[OpenGL Program Link Error]: %s\n", infoLog.data());
        glDeleteProgram(program);
        return false;
    }

    programId_ = program;
    return true;
}

void Shader::use() const
{
    if (programId_ != 0) {
        glUseProgram(programId_);
    }
}

void Shader::setMat4(const char* name, const float* mat16) const
{
    GLint loc = glGetUniformLocation(programId_, name);
    if (loc != -1) {
        glUniformMatrix4fv(loc, 1, GL_FALSE, mat16);
    }
}

void Shader::setVec4(const char* name, float x, float y, float z, float w) const
{
    GLint loc = glGetUniformLocation(programId_, name);
    if (loc != -1) {
        glUniform4f(loc, x, y, z, w);
    }
}

void Shader::setVec2(const char* name, float x, float y) const
{
    GLint loc = glGetUniformLocation(programId_, name);
    if (loc != -1) {
        glUniform2f(loc, x, y);
    }
}

void Shader::setFloat(const char* name, float val) const
{
    GLint loc = glGetUniformLocation(programId_, name);
    if (loc != -1) {
        glUniform1f(loc, val);
    }
}

void Shader::setInt(const char* name, int val) const
{
    GLint loc = glGetUniformLocation(programId_, name);
    if (loc != -1) {
        glUniform1i(loc, val);
    }
}

} // namespace PhysicsEngine
