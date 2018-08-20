#ifndef MODEL_H_
#define MODEL_H_

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <map>
#include <vector>

#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "assimp/Importer.hpp"
#include "assimp/scene.h"
#include "assimp/postprocess.h"

#include "mesh.h"
#include "shader.h"

using namespace std;

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


GLuint TextureFromFile(const char *path, const string &directory, bool gamma = false);
GLuint LoadCubemap(const std::vector<std::string>& faces);

enum GeometryTypes
{
    kMesh,
    kStaticMesh,
    kCylinder,
    kSphere,
    kCapsule,
    kBox
};

class ModelData 
{
public:
    ModelData();
    ~ModelData();
    
    void Draw(Shader shader);
    void Reset();
    
    vector<Texture> textures_loaded_;
    vector<Mesh> meshes_;
    string directory_;
    bool gamma_correction_;
    int primitive_type_;
    Cylinder *cylinder_;
    Sphere *sphere_;
    Box *box_;
    bool converted_to_mesh_;

private:
    void LoadModel(string const &path);
    void ProcessNode(aiNode *node, const aiScene *scene);
    Mesh ProcessPrim();
    Mesh ProcessMesh(aiMesh *mesh, const aiScene *scene);
    vector<Texture> LoadMaterialTextures(aiMaterial *mat, aiTextureType type, string typeName);
};

#endif // MODEL_H_