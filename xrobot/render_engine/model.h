#ifndef RENDER_ENGINE_MODEL_H_
#define RENDER_ENGINE_MODEL_H_

#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "assimp/Importer.hpp"
#include "assimp/postprocess.h"
#include "assimp/scene.h"
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"

#include "mesh.h"
#include "shader.h"

namespace xrobot {
namespace render_engine {

struct Cylinder {
    Cylinder(float r, float l) : radius(r), length(l) {}
    float radius;
    float length;
};

struct Sphere {
    Sphere(float r) : radius(r) {}
    float radius;
};

struct Box {
    Box(float x, float y, float z) : sx(x), sy(y), sz(z) {}
    float sx;
    float sy;
    float sz;
};

struct OriginTransformation {
    OriginTransformation() {}
    bool highlight = false;
    float scale = 1.0f;
    float flip = -1.0f;
    glm::vec3 local_scale = glm::vec3(1);
    glm::mat4 origin = glm::mat4(1);
};

enum GeometryTypes {
    kMesh,
    kStaticMesh,
    kCylinder,
    kSphere,
    kCapsule,
    kBox
};

class ModelData {
public:
    ModelData();

    ~ModelData();
    
    void BoneTransform(float time, float duration,
            std::vector<aiMatrix4x4>& transforms);


    void Draw(const Shader& shader);

    void Reset();

public: 
    std::vector<Texture> textures_loaded_;
    std::vector<Mesh> meshes_;
    std::string directory_;
    bool gamma_correction_;
    int primitive_type_;
    Cylinder *cylinder_;
    Sphere *sphere_;
    Box *box_;
    bool converted_to_mesh_;

private:
    void LoadModel(const std::string& path);

    void ProcessNode(const aiNode *node, const aiScene *scene);

    Mesh ProcessPrim();

    Mesh ProcessMesh(const aiMesh *mesh, const aiScene *scene);

    std::vector<Texture> LoadMaterialTextures(const aiMaterial *mat,
                                              aiTextureType type,
                                              const std::string& typeName);
};

} } // xrobot::render_engine

#endif // RENDER_ENGINE_MODEL_H_
