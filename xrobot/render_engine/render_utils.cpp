#include "render_utils.h"

#include <iostream>

#define STB_IMAGE_IMPLEMENTATION
#include "../vendor/stb_image.h"

namespace xrobot {
namespace render_engine {

GLuint LoadCubemap(const std::vector<std::string>& faces) {
    GLuint textureID;
    glGenTextures(1, &textureID);
    glBindTexture(GL_TEXTURE_CUBE_MAP, textureID);

    int width, height, nrComponents;
    for (size_t i = 0; i < faces.size(); i++) {
        unsigned char *data = stbi_load(
                faces[i].c_str(), &width, &height, &nrComponents, 0);
        if (data) {
            glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i,
                         0,
                         GL_RGB,
                         width,
                         height,
                         0,
                         GL_RGB,
                         GL_UNSIGNED_BYTE,
                         data);
            stbi_image_free(data);
        } else {
            std::cout << "[Renderer] Failed to load cubemap." << std::endl;
            stbi_image_free(data);
            exit(-1);
        }
    }
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);

    return textureID;
}

unsigned int HDRTextureFromFile(const char* path){

    unsigned int hdrTexture;
    stbi_set_flip_vertically_on_load(true);
    int width, height, nrComponents;
    float *data = stbi_loadf(path, &width, &height, &nrComponents, 0);

    if (data)
    {
        glGenTextures(1, &hdrTexture);
        glBindTexture(GL_TEXTURE_2D, hdrTexture);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16F, width, height, 0, GL_RGB, GL_FLOAT, data);
        
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        
        stbi_image_free(data);
    }
    else
    {
        std::cout << "[Renderer] Failed to load HDR image." << std::endl;
        stbi_image_free(data);
        exit(-1);
    }
    stbi_set_flip_vertically_on_load(false);

    return hdrTexture;
}

unsigned int TextureFromFile(const std::string &path, const int fail) {
    unsigned int textureID;
    glGenTextures(1, &textureID);

    int width, height, nrComponents;
    unsigned char *data = stbi_load(
            path.c_str(), &width, &height, &nrComponents, 0);
    if (data) {
        GLenum format;
        if (nrComponents == 1) {
            format = GL_RED;
        } else if (nrComponents == 3) {
            format = GL_RGB;
        } else if (nrComponents == 4) {
            format = GL_RGBA;
        }

        glBindTexture(GL_TEXTURE_2D, textureID);
        glTexImage2D(GL_TEXTURE_2D,
                     0,
                     format,
                     width,
                     height,
                     0,
                     format,
                     GL_UNSIGNED_BYTE,
                     data);
        glGenerateMipmap(GL_TEXTURE_2D);

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D,
                        GL_TEXTURE_MIN_FILTER,
                        GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        stbi_image_free(data);
    } else {

        if(fail == 1) {
            std::cout << "[Renderer] Texture data corrupted!" << std::endl;
            std::cout << "[Renderer] Cannot load error.png!" << std::endl;
            exit(-1);
        }

        std::cout << "[Renderer] Failed to load texture at path: " << path << std::endl;
        stbi_image_free(data);

        std::size_t ext = path.find("data");
        std::string icon_path = path.substr(0, ext) + "data/error.png";

        std::cout << "[Renderer] Load default texture from: " << icon_path << std::endl;

        return TextureFromFile(icon_path, 1);
    }

    return textureID;
}

unsigned int TextureFromFile(
        const char *path,
        const std::string &directory,
        bool gamma) {
    std::string filename = directory + "/" + path;

    // printf("file: %s\n", filename.c_str());

    unsigned int textureID;
    glGenTextures(1, &textureID);

    int width, height, nrComponents;
    unsigned char *data = stbi_load(
            filename.c_str(), &width, &height, &nrComponents, 0);
    if (data) {
        GLenum format;
        if (nrComponents == 1) {
            format = GL_RED;
        } else if (nrComponents == 3) {
            format = GL_RGB;
        } else if (nrComponents == 4) {
            format = GL_RGBA;
        }

        glBindTexture(GL_TEXTURE_2D, textureID);
        glTexImage2D(GL_TEXTURE_2D,
                     0,
                     format,
                     width,
                     height,
                     0,
                     format,
                     GL_UNSIGNED_BYTE,
                     data);
        glGenerateMipmap(GL_TEXTURE_2D);

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D,
                        GL_TEXTURE_MIN_FILTER,
                        GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        stbi_image_free(data);
    } else {
        std::cout << "[Renderer] Failid to load texture at path: " << path << std::endl;
        stbi_image_free(data);
        exit(-1);
    }

    return textureID;
}

} } // xrobot::render_engine
