#ifndef RENDER_ENGINE_RENDER_UTILS_H
#define RENDER_ENGINE_RENDER_UTILS_H

#include <string>
#include <vector>

#define INCLUDE_GL_CONTEXT_HEADERS
#include "gl_header.h"

namespace xrobot {
namespace render_engine {

GLuint LoadCubemap(const std::vector<std::string>& faces);

unsigned int TextureFromFile(
        const char *path,
        const std::string &directory,
        bool gamma = false);

} } // xrobot::render_engine

#endif
