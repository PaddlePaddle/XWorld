#ifndef RENDER_ENGINE_SHADER_H_
#define RENDER_ENGINE_SHADER_H_

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include "glm/glm.hpp"

#define INCLUDE_GL_CONTEXT_HEADERS
#include "gl_header.h"

namespace xrobot {
namespace render_engine {

class Shader {
public:
    Shader() : id_(0) {};

    Shader(const std::string& vertexPath,
           const std::string& fragmentPath, 
           const std::string& controlPath = "",
           const std::string& evaluationPath = "");

    Shader(const std::string& vertexPath,
           const std::string& fragmentPath,
           const std::string& geometryPath,
           const int count = -1,
           const char **varyings = nullptr,
           GLenum buffermode = GL_INTERLEAVED_ATTRIBS);

    unsigned int id() const { return id_; }
  
    void use() {
        glUseProgram(id_);
    }

    void setBool(const std::string &name, bool value) const {
        glUniform1i(glGetUniformLocation(id_, name.c_str()), (int)value);
    }

    void setInt(const std::string &name, int value) const {
        glUniform1i(glGetUniformLocation(id_, name.c_str()), value);
    }

    void setFloat(const std::string &name, float value) const {
        glUniform1f(glGetUniformLocation(id_, name.c_str()), value);
    }

    void setVec2(const std::string &name, const glm::vec2 &value) const {
        glUniform2fv(glGetUniformLocation(id_, name.c_str()), 1, &value[0]);
    }

    void setVec2(const std::string &name, float x, float y) const {
        glUniform2f(glGetUniformLocation(id_, name.c_str()), x, y);
    }

    void setVec3(const std::string &name, const glm::vec3 &value) const {
        glUniform3fv(glGetUniformLocation(id_, name.c_str()), 1, &value[0]);
    }

    void setVec3(const std::string &name, float x, float y, float z) const {
        glUniform3f(glGetUniformLocation(id_, name.c_str()), x, y, z);
    }

    void setVec4(const std::string &name, const glm::vec4 &value) const {
        glUniform4fv(glGetUniformLocation(id_, name.c_str()), 1, &value[0]);
    }

    void setVec4(const std::string &name, float x, float y, float z, float w) {
        glUniform4f(glGetUniformLocation(id_, name.c_str()), x, y, z, w);
    }

    void setMat2(const std::string &name, const glm::mat2 &mat) const {
        GLint loc = glGetUniformLocation(id_, name.c_str());
        glUniformMatrix2fv(loc, 1, GL_FALSE, &mat[0][0]);
    }

    void setMat3(const std::string &name, const glm::mat3 &mat) const {
        GLint loc = glGetUniformLocation(id_, name.c_str());
        glUniformMatrix3fv(loc, 1, GL_FALSE, &mat[0][0]);
    }

    void setMat4(const std::string &name, const glm::mat4 &mat) const {
        GLint loc = glGetUniformLocation(id_, name.c_str());
        glUniformMatrix4fv(loc, 1, GL_FALSE, &mat[0][0]);
    }
    
private:
    unsigned int id_;

    void checkCompileErrors(const GLuint shader, const std::string& type);
};

} } // xrobot::render_engine

#endif // RENDER_ENGINE_SHADER_H_
