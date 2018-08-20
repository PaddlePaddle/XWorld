#include "shader.h"

namespace xrobot {
namespace render_engine {

Shader::Shader(const std::string& computePath) {

    std::string computCode;
    std::ifstream cShaderFile;
    cShaderFile.exceptions (std::ifstream::failbit | std::ifstream::badbit);
    try
    {
        cShaderFile.open(computePath);
        std::stringstream cShaderStream;
        cShaderStream << cShaderFile.rdbuf();
        cShaderFile.close();
        computCode = cShaderStream.str();

        if (!computePath.empty())
        {
            cShaderFile.open(computePath);
            std::stringstream cShaderStream;
            cShaderStream << cShaderFile.rdbuf();
            cShaderFile.close();
            computCode = cShaderStream.str();
        }
    }

    catch (std::ifstream::failure e)
    {
        std::cout << "ERROR::SHADER::FILE_NOT_SUCCESFULLY_READ" << std::endl;
    }

    const char* cShaderCode = computCode.c_str();
    unsigned int comput;
    int success;
    char infoLog[512];
    comput = glCreateShader(GL_COMPUTE_SHADER);
    glShaderSource(comput, 1, &cShaderCode, NULL);
    glCompileShader(comput);
    checkCompileErrors(comput, "COMPUTE");
    id_ = glCreateProgram();
    glAttachShader(id_, comput);
    glLinkProgram(id_);
    checkCompileErrors(id_, "PROGRAM");
    glDeleteShader(comput);
}

Shader::Shader(const std::string& vertexPath,
               const std::string& fragmentPath, 
               const std::string& controlPath,
               const std::string& evaluationPath) {

    std::string vertexCode;
    std::string fragmentCode;
    std::string controlCode;
    std::string evaluationCode;

    std::ifstream vShaderFile;
    std::ifstream fShaderFile;
    std::ifstream cShaderFile;
    std::ifstream eShaderFile;

    vShaderFile.exceptions (std::ifstream::failbit | std::ifstream::badbit);
    fShaderFile.exceptions (std::ifstream::failbit | std::ifstream::badbit);
    cShaderFile.exceptions (std::ifstream::failbit | std::ifstream::badbit);
    eShaderFile.exceptions (std::ifstream::failbit | std::ifstream::badbit);

    try {
        vShaderFile.open(vertexPath.c_str());
        fShaderFile.open(fragmentPath.c_str());
        std::stringstream vShaderStream, fShaderStream;
        vShaderStream << vShaderFile.rdbuf();
        fShaderStream << fShaderFile.rdbuf();
        vShaderFile.close();
        fShaderFile.close();
        vertexCode = vShaderStream.str();
        fragmentCode = fShaderStream.str();

        if (!controlPath.empty() && !evaluationPath.empty()) {
            cShaderFile.open(controlPath.c_str());
            eShaderFile.open(evaluationPath.c_str());
            std::stringstream cShaderStream, eShaderStream;
            cShaderStream << cShaderFile.rdbuf();
            cShaderFile.close();
            eShaderStream << eShaderFile.rdbuf();
            eShaderFile.close();
            controlCode = cShaderStream.str();
            evaluationCode = eShaderStream.str();
        }
    }
    catch (std::ifstream::failure e) {
        std::cout << "ERROR::SHADER::FILE_NOT_SUCCESFULLY_READ" << std::endl;
    }

    const char* vShaderCode = vertexCode.c_str();
    const char* fShaderCode = fragmentCode.c_str();

    unsigned int vertex, fragment;
    int success;
    char infoLog[512];

    vertex = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertex, 1, &vShaderCode, NULL);
    glCompileShader(vertex);
    checkCompileErrors(vertex, "VERTEX");

    fragment = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragment, 1, &fShaderCode, NULL);
    glCompileShader(fragment);
    checkCompileErrors(fragment, "FRAGMENT");

    unsigned int control, evaluation;
    if (!controlPath.empty() && !evaluationPath.empty()) {
        const char * cShaderCode = controlCode.c_str();
        control = glCreateShader(GL_TESS_CONTROL_SHADER);
        glShaderSource(control, 1, &cShaderCode, NULL);
        glCompileShader(control);
        checkCompileErrors(control, "CONTROL");
        
        const char * eShaderCode = evaluationCode.c_str();
        evaluation = glCreateShader(GL_TESS_EVALUATION_SHADER);
        glShaderSource(evaluation, 1, &eShaderCode, NULL);
        glCompileShader(evaluation);
        checkCompileErrors(evaluation, "EVALUATION");
    }

    id_ = glCreateProgram();
    glAttachShader(id_, vertex);
    glAttachShader(id_, fragment);
    if (!controlPath.empty() && !evaluationPath.empty()) {
        glAttachShader(id_, control);
        glAttachShader(id_, evaluation);
    }

    glLinkProgram(id_);
    checkCompileErrors(id_, "PROGRAM");
    glDeleteShader(vertex);
    glDeleteShader(fragment);
    if (!controlPath.empty() && !evaluationPath.empty()) {
        glDeleteShader(control);
        glDeleteShader(evaluation);
    }
}


Shader::Shader(const std::string& vertexPath,
               const std::string& fragmentPath,
               const std::string& geometryPath,
               const int count,
               const char **varyings,
               GLenum buffermode) {
    std::string vertexCode;
    std::string fragmentCode;
    std::string geometryCode;

    std::ifstream vShaderFile;
    std::ifstream fShaderFile;
    std::ifstream gShaderFile;

    vShaderFile.exceptions (std::ifstream::failbit | std::ifstream::badbit);
    fShaderFile.exceptions (std::ifstream::failbit | std::ifstream::badbit);
    gShaderFile.exceptions (std::ifstream::failbit | std::ifstream::badbit);

    try {
        vShaderFile.open(vertexPath.c_str());
        fShaderFile.open(fragmentPath.c_str());
        std::stringstream vShaderStream, fShaderStream;

        vShaderStream << vShaderFile.rdbuf();
        fShaderStream << fShaderFile.rdbuf();

        vShaderFile.close();
        fShaderFile.close();

        vertexCode = vShaderStream.str();
        fragmentCode = fShaderStream.str();

        if (!geometryPath.empty()) {
            gShaderFile.open(geometryPath.c_str());
            std::stringstream gShaderStream;
            gShaderStream << gShaderFile.rdbuf();
            gShaderFile.close();
            geometryCode = gShaderStream.str();
        }
    }
    catch (std::ifstream::failure e) {
        std::cout << "ERROR::SHADER::FILE_NOT_SUCCESFULLY_READ" << std::endl;
    }

    const char* vShaderCode = vertexCode.c_str();
    const char * fShaderCode = fragmentCode.c_str();
    unsigned int vertex, fragment;
    int success;
    char infoLog[512];

    vertex = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertex, 1, &vShaderCode, NULL);
    glCompileShader(vertex);
    checkCompileErrors(vertex, "VERTEX");

    fragment = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragment, 1, &fShaderCode, NULL);
    glCompileShader(fragment);
    checkCompileErrors(fragment, "FRAGMENT");

    unsigned int geometry;
    if (!geometryPath.empty()) {
        const char * gShaderCode = geometryCode.c_str();
        geometry = glCreateShader(GL_GEOMETRY_SHADER);
        glShaderSource(geometry, 1, &gShaderCode, NULL);
        glCompileShader(geometry);
        checkCompileErrors(geometry, "GEOMETRY");
    }

    id_ = glCreateProgram();
    glAttachShader(id_, vertex);
    glAttachShader(id_, fragment);
    if (!geometryPath.empty()) {
        glAttachShader(id_, geometry);
    }

    if (count > 0) {
        glTransformFeedbackVaryings(id_, count, varyings, buffermode);
    }

    glLinkProgram(id_);
    checkCompileErrors(id_, "PROGRAM");
    glDeleteShader(vertex);
    glDeleteShader(fragment);
    if (!geometryPath.empty()) {
        glDeleteShader(geometry);
    }
}

void Shader::checkCompileErrors(const GLuint shader, const std::string& type) {
    GLint success;
    GLchar infoLog[1024];
    if (type != "PROGRAM") {
        glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
        if (!success) {
            glGetShaderInfoLog(shader, 1024, NULL, infoLog);
            std::cout << "ERROR::SHADER_COMPILATION_ERROR of type: "
                      << type << "\n" << infoLog << "\n"
                      << "---------------------------------------------------"
                      << std::endl;
        }
    } else {
        glGetProgramiv(shader, GL_LINK_STATUS, &success);
        if (!success) {
            glGetProgramInfoLog(shader, 1024, NULL, infoLog);
            std::cout << "ERROR::PROGRAM_LINKING_ERROR of type: "
                      << type << "\n" << infoLog << "\n"
                      << "---------------------------------------------------" 
                      << std::endl;
        }
    }
}

} } // xrobot::render_engine
