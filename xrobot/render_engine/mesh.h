#ifndef RENDER_ENGINE_MESH_H_
#define RENDER_ENGINE_MESH_H_

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "assimp/Importer.hpp"
#include "assimp/postprocess.h"
#include "assimp/scene.h"
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"

#include "shader.h"

namespace xrobot {
namespace render_engine {

struct Vertex {
    glm::vec3 position;
    glm::vec3 normal;
    glm::vec2 texcoords;
    glm::vec3 tangent;
    glm::vec3 bitangent;
};


struct Texture {
    unsigned int id;
    std::string type;
    std::string path;
};

// TEST
// Skinning
struct VertexBoneData {
    unsigned int ids[4];
    float weights[4];
};

struct BoneInfo {
    aiMatrix4x4 offset_matrix;
};

class Mesh {
public:
    std::vector<Vertex> vertices_;
    std::vector<unsigned int> indices_;
    std::vector<Texture> textures_;
    unsigned int VAO_;
    unsigned int VBO_, EBO_;

    // Mtl
    glm::vec3 kA_;
    glm::vec3 kD_;
    glm::vec3 kS_;
    float d_;
    float Ns_;
    bool diffuse_;
    bool bump_;
    bool specular_;
    bool height_;
    bool ao_;

    // Skinning
    int num_bones_;
    std::vector<BoneInfo> bone_info_list_;
    std::vector<VertexBoneData> bones_;

    Mesh() {}

    Mesh(const std::vector<Vertex>& vertices,
         const std::vector<unsigned int>& indices,
         const std::vector<Texture>& textures);
    
    Mesh(const std::vector<Vertex>& vertices,
         const std::vector<unsigned int>& indices,
         const std::vector<Texture>& textures,
         const glm::vec3& kAmbient,
         const glm::vec3& KDiffuse,
         const glm::vec3& kSpecular,
         float opacity,
         float shininess,
         bool diffuseMap,
         bool normalMap,
         bool specularMap,
         bool heightMap,
         bool aoMap);

    void Draw(const Shader& shader);

private:
    void SetupMesh();
};

} } // xrobot::render_engine

#endif // RENDER_ENGINE_MESH_H_
