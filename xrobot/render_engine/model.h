#ifndef RENDER_ENGINE_MODEL_H_
#define RENDER_ENGINE_MODEL_H_

#include <fstream>
#include <map>
#include <memory>
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
    glm::vec4 color = glm::vec4(0);
};

enum GeometryTypes {
    kSphere = 2,
    kBox = 3,
    kCylinder = 4,
    kMesh = 5,
    kCapsule = 7,
};

class ModelData {
public:
    ModelData();

    ~ModelData();
    
    void Draw(const Shader& shader);

    void Reset();

    void Reset(
            int geometry_type,
            bool create_new,
            const glm::vec3& scale,
            std::shared_ptr<OriginTransformation>& T);
public: 
    std::vector<Texture> textures_loaded_;
    std::vector<Mesh> meshes_;
    std::string directory_;
    bool gamma_correction_;
    int primitive_type_;
    std::shared_ptr<Cylinder> cylinder_;
    std::shared_ptr<Sphere> sphere_;
    std::shared_ptr<Box> box_;
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

typedef std::shared_ptr<ModelData> ModelDataSPtr;

} } // xrobot::render_engine

#endif // RENDER_ENGINE_MODEL_H_
