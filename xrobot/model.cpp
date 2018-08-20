#include "model.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

ModelData::ModelData() : 
textures_loaded_(0), meshes_(0),
directory_(), gamma_correction_(false),
primitive_type_(kMesh),
cylinder_(nullptr), sphere_(nullptr), box_(nullptr),
converted_to_mesh_(false) {}

ModelData::~ModelData()
{
    for (unsigned int i = 0; i < textures_loaded_.size(); ++i)
    {
        Texture texture = textures_loaded_[i];
        glDeleteTextures(1, &texture.id);
    }

    for (unsigned int i = 0; i < meshes_.size(); ++i)
    {
        Mesh mesh = meshes_[i];
        glDeleteVertexArrays(1, &mesh.VAO_);
        glDeleteBuffers(1, &mesh.EBO_);
        glDeleteBuffers(1, &mesh.VBO_);
    }

    if(cylinder_) 
    {
        delete cylinder_;
        cylinder_ = nullptr;
    }

    if(sphere_)
    {
        delete sphere_;
        sphere_ = nullptr;
    }

    if(box_)
    {
        delete box_;
        box_ = nullptr;
    }
}

void ModelData::Draw(Shader shader)
{
    for(unsigned int i = 0; i < meshes_.size(); ++i)
        meshes_[i].Draw(shader);
}

void ModelData::Reset()
{
    if(primitive_type_ == kMesh)
    {
        LoadModel(directory_);
    }
    else {
        meshes_.push_back(ProcessPrim());
    }
}

void ModelData::LoadModel(string const &path)
{
    Assimp::Importer importer;
    aiMatrix4x4 root_trans;
    importer.SetPropertyInteger(AI_CONFIG_PP_PTV_ADD_ROOT_TRANSFORMATION, 1);
    importer.SetPropertyMatrix(AI_CONFIG_PP_PTV_ROOT_TRANSFORMATION, root_trans); 

    const aiScene* scene = importer.ReadFile(path, 
        aiProcess_JoinIdenticalVertices | aiProcess_GenNormals | 
        aiProcess_ImproveCacheLocality | aiProcess_PreTransformVertices | 
        aiProcess_Triangulate | aiProcess_FlipUVs);


    if(!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode)
    {
        cout << "ERROR::ASSIMP:: " << importer.GetErrorString() << endl;
        return;
    }

    directory_ = path.substr(0, path.find_last_of('/'));
    ProcessNode(scene->mRootNode, scene);
}

void ModelData::ProcessNode(aiNode *node, const aiScene *scene)
{
    for(unsigned int i = 0; i < node->mNumMeshes; i++)
    {
        aiMesh* mesh = scene->mMeshes[node->mMeshes[i]];
        meshes_.push_back(ProcessMesh(mesh, scene));
    }

    for(unsigned int i = 0; i < node->mNumChildren; i++)
    {
        ProcessNode(node->mChildren[i], scene);
    }

}

Mesh ModelData::ProcessPrim()
{
    if(primitive_type_ == kSphere || primitive_type_ == kCapsule)
    {
        vector<Vertex> vertices;
        vector<unsigned int> indices;
        vector<Texture> textures;

        std::vector<aiVector3D> v(12, aiVector3D());

        double theta = 26.56505117707799 * M_PI / 180.0;

        v[0] = aiVector3D(0,0,-1);

        double phi = M_PI / 5.0;

        for (int i = 0; i < 6; ++i)
        {
            v[i] = aiVector3D(cos(theta)*cos(phi), cos(theta)*sin(phi), -sin(theta));
            phi += 2 * M_PI / 5.0;
        }
        phi = 0.0;
        for (int i=6; i<11; ++i) {
            v[i] = aiVector3D(cos(theta)*cos(phi), cos(theta)*sin(phi), sin(theta));
            phi += 2*M_PI / 5;
        }

        v[11] = aiVector3D(0, 0, 1);
        int idx[] = {
            0,2,1,
            0,3,2,
            0,4,3,
            0,5,4,
            0,1,5,
            1,2,7,
            2,3,8,
            3,4,9,
            4,5,10,
            5,1,6,
            1,7,6,
            2,8,7,
            3,9,8,
            4,10,9,
            5,6,10,
            6,7,11,
            7,8,11,
            8,9,11,
            9,10,11,
            10,6,11
        };

        std::vector<float> vt;

        for (int i = 0; i < 20 * 3; ++i)
        {
            vt.push_back(v[idx[i]].x);
            vt.push_back(v[idx[i]].y);
            vt.push_back(v[idx[i]].z);
        }

        const int BEST_DETAILS = 2;
        for (int c = 0; c < BEST_DETAILS; ++c)
        {
            std::vector<float> v;
            v.swap(vt);
            for (int i = 0; i < (int)v.size(); i+=9)
            {
                aiVector3D e0(v[i+0], v[i+1], v[i+2]);
                aiVector3D e1(v[i+3], v[i+4], v[i+5]);
                aiVector3D e2(v[i+6], v[i+7], v[i+8]);
                aiVector3D mid01(e0.x+e1.x, e0.y+e1.y, e0.z+e1.z);
                aiVector3D mid12(e1.x+e2.x, e1.y+e2.y, e1.z+e2.z);
                aiVector3D mid20(e2.x+e0.x, e2.y+e0.y, e2.z+e0.z);
                mid01.Normalize();
                mid12.Normalize();
                mid20.Normalize();

                vt.push_back(mid01.x); vt.push_back(mid01.y); vt.push_back(mid01.z);
                vt.push_back(mid12.x); vt.push_back(mid12.y); vt.push_back(mid12.z);
                vt.push_back(mid20.x); vt.push_back(mid20.y); vt.push_back(mid20.z);
                vt.push_back(e0.x); vt.push_back(e0.y); vt.push_back(e0.z);

                vt.push_back(mid01.x); vt.push_back(mid01.y); vt.push_back(mid01.z);
                vt.push_back(mid20.x); vt.push_back(mid20.y); vt.push_back(mid20.z);
                vt.push_back(e1.x); vt.push_back(e1.y); vt.push_back(e1.z);

                vt.push_back(mid12.x); vt.push_back(mid12.y); vt.push_back(mid12.z);
                vt.push_back(mid01.x); vt.push_back(mid01.y); vt.push_back(mid01.z);
                vt.push_back(e2.x); vt.push_back(e2.y); vt.push_back(e2.z);

                vt.push_back(mid20.x); vt.push_back(mid20.y); vt.push_back(mid20.z);
                vt.push_back(mid12.x); vt.push_back(mid12.y); vt.push_back(mid12.z);
            }
        }

        bool capsule = primitive_type_ == kCapsule;
        float rad = capsule ? cylinder_->radius : sphere_->radius;
        float len = capsule ? cylinder_->length / 2 : 0;

        for (int i = 0; i < (int)vt.size() / 3; ++i)
        {
            Vertex vertex;

            vertex.position = glm::vec3(
                vt[3 * i + 0] * rad,
                vt[3 * i + 1] * rad,
                vt[3 * i + 2] * rad
             );

            vertex.normal = glm::vec3(
                vt[3 * i + 0],
                vt[3 * i + 1],
                vt[3 * i + 2]
            );

            vertex.texcoords = glm::vec2(0.0f, 0.0f);
            vertex.tangent = glm::vec3(0.0f, 0.0f, 0.0f);
            vertex.bitangent = glm::vec3(0.0f, 0.0f, 0.0f);

            if(capsule)
            {
                if (vertex.position[2] > 0)
                {
                    vertex.position[2] += len;
                } 
                else if (vertex.position[2] < 0)
                {
                    vertex.position[2] -= len;
                }
            }

            vertices.push_back(vertex);
        }
        return Mesh(vertices, indices, textures);

    }
    else if(primitive_type_ == kCylinder)
    {
        vector<Vertex> vertices;
        vector<unsigned int> indices;
        vector<Texture> textures;

        const int BEST_DETAILS = 16; // 8 // 3
        int side_faces = BEST_DETAILS;

        float l = cylinder_->length;
        float r = cylinder_->radius;

        for (int c = 0; c < side_faces; ++c)
        {
            float angle1 = float(c) / side_faces * 2 * M_PI;
            float angle2 = float(c+1) / side_faces * 2 * M_PI;
            float n1[3], n2[3];
            n1[0] = cos(angle1); n1[1] = sin(angle1); n1[2] = 0;
            n2[0] = cos(angle2); n2[1] = sin(angle2); n2[2] = 0;

            Vertex vertex;
            vertex.texcoords = glm::vec2(0.0f, 0.0f);
            vertex.tangent = glm::vec3(0.0f, 0.0f, 0.0f);
            vertex.bitangent = glm::vec3(0.0f, 0.0f, 0.0f);

            vertex.normal = glm::vec3(0.0f, 0.0f, 1.0f);
            vertex.position = glm::vec3(n1[0] * r, n1[1] * r, l * 0.5f);
            vertices.push_back(vertex);
            vertex.position = glm::vec3(n2[0] * r, n2[1] * r, l * 0.5f);
            vertices.push_back(vertex);
            vertex.position = glm::vec3(0.0f, 0.0f, l * 0.5f);
            vertices.push_back(vertex);

            vertex.normal = glm::vec3(0.0f, 0.0f, -1.0f);
            vertex.position = glm::vec3(n1[0] * r, n1[1] * r, -l * 0.5f);
            vertices.push_back(vertex);
            vertex.position = glm::vec3(0.0f, 0.0f, -l * 0.5f);
            vertices.push_back(vertex);
            vertex.position = glm::vec3(n2[0] * r, n2[1] * r, -l * 0.5f);
            vertices.push_back(vertex);
        }

        for (int c = 0; c < side_faces; ++c)
        {
            float angle1 = float(c)   / side_faces * 2 * M_PI;
            float angle2 = float(c+1) / side_faces * 2 * M_PI;
            float n1[3], n2[3];
            n1[0] = cos(angle1); n1[1] = sin(angle1); n1[2] = 0;
            n2[0] = cos(angle2); n2[1] = sin(angle2); n2[2] = 0;

            Vertex vertex;
            vertex.texcoords = glm::vec2(0.0f, 0.0f);
            vertex.tangent = glm::vec3(0.0f, 0.0f, 0.0f);
            vertex.bitangent = glm::vec3(0.0f, 0.0f, 0.0f);

            vertex.normal = glm::vec3(n1[0], n1[1], n1[2]);
            vertex.position = glm::vec3(n1[0] * r, n1[1] * r, -l * 0.5f);
            vertices.push_back(vertex);

            vertex.normal = glm::vec3(n2[0], n2[1], n2[2]);
            vertex.position = glm::vec3(n2[0] * r, n2[1] * r, -l * 0.5f);
            vertices.push_back(vertex);

            vertex.normal = glm::vec3(n2[0], n2[1], n2[2]);
            vertex.position = glm::vec3(n2[0] * r, n2[1] * r, +l * 0.5f);
            vertices.push_back(vertex);


            vertex.normal = glm::vec3(n1[0], n1[1], n1[2]);
            vertex.position = glm::vec3(n1[0] * r, n1[1] * r, -l * 0.5f);
            vertices.push_back(vertex);

            vertex.normal = glm::vec3(n2[0], n2[1], n2[2]);
            vertex.position = glm::vec3(n2[0] * r, n2[1] * r, +l * 0.5f);
            vertices.push_back(vertex);

            vertex.normal = glm::vec3(n1[0], n1[1], n1[2]);
            vertex.position = glm::vec3(n1[0] * r, n1[1] * r, +l * 0.5f);
            vertices.push_back(vertex);
        }

        return Mesh(vertices, indices, textures);

    }
    else if(primitive_type_ == kBox)
    {
        vector<Vertex> vertices;
        vector<unsigned int> indices;
        vector<Texture> textures;

        double n[] = {
            +1, 0, 0,
            -1, 0, 0,
            0, +1, 0,
            0, -1, 0,
            0, 0, +1,
            0, 0, -1 
        };

        for (int f = 0; f < 6; ++f)
        {
            double side[] = {
                +1, +1,
                -1, +1,
                -1, -1,
                +1, -1
            };

            int zero1 = n[3*f + 0]==0 ? 0 : 1;
            int zero2 = n[3*f + 2]==0 ? 2 : 1;
            int sign = n[3*f + 0] + n[3*f + 1] + n[3*f + 2];
            if (f==2 || f==3) sign *= -1;
            int ind_reloc[] = { 0,1,3, 3,1,2 };

            for (int i = 0; i < 6; ++i)
            {
                int idx = ind_reloc[i];

                Vertex vertex;

                vertex.normal   = glm::vec3(n[3*f+0], n[3*f+1], n[3*f+2]);

                float v[3];
                v[0] = n[3*f+0];
                v[1] = n[3*f+1];
                v[2] = n[3*f+2];
                if (sign > 0) {
                    v[zero1] = side[2*idx + 0];
                    v[zero2] = side[2*idx + 1];
                } else {
                    v[zero1] = side[6 - 2*idx];
                    v[zero2] = side[7 - 2*idx];
                }

                vertex.position = glm::vec3(
                    v[0] * 0.5f * box_->sx,
                    v[1] * 0.5f * box_->sy,
                    v[2] * 0.5f * box_->sz
                );

                vertex.texcoords = glm::vec2(0.0f, 0.0f);
                vertex.tangent = glm::vec3(0.0f, 0.0f, 0.0f);
                vertex.bitangent = glm::vec3(0.0f, 0.0f, 0.0f);
                vertices.push_back(vertex);
            }
        }
        return Mesh(vertices, indices, textures);
    }
}


Mesh ModelData::ProcessMesh(aiMesh *mesh, const aiScene *scene)
{
    vector<Vertex> vertices;
    vector<unsigned int> indices;
    vector<Texture> textures;

    for(unsigned int i = 0; i < mesh->mNumVertices; i++)
    {

        Vertex vertex;
        glm::vec3 vector; 
        vector.x = mesh->mVertices[i].x ;
        vector.y = mesh->mVertices[i].y ;
        vector.z = mesh->mVertices[i].z ;
        vertex.position = vector;

        vector.x = mesh->mNormals[i].x;
        vector.y = mesh->mNormals[i].y;
        vector.z = mesh->mNormals[i].z;
        vertex.normal = vector;

        if(mesh->mTextureCoords[0]) 
        {
            glm::vec2 vec;
            vec.x = mesh->mTextureCoords[0][i].x; 
            vec.y = mesh->mTextureCoords[0][i].y;
            vertex.texcoords = vec;
        }
        else
            vertex.texcoords = glm::vec2(0.0f, 0.0f);

    #ifdef USE_TANGENT_BITANGENT
        // tangent
        if(mesh->HasTangentsAndBitangents()) {
            vector.x = mesh->mTangents[i].x;
            vector.y = mesh->mTangents[i].y;
            vector.z = mesh->mTangents[i].z;
            vertex.tangent = vector;
        }
        else
            vertex.tangent = glm::vec3(0.0f, 0.0f, 0.0f);

        // bitangent
        if(mesh->HasTangentsAndBitangents()) {
            vector.x = mesh->mBitangents[i].x;
            vector.y = mesh->mBitangents[i].y;
            vector.z = mesh->mBitangents[i].z;
            vertex.bitangent = vector;
        }
        else
            vertex.bitangent = glm::vec3(0.0f, 0.0f, 0.0f);
    #else
        vertex.tangent = glm::vec3(0.0f, 0.0f, 0.0f);
        vertex.bitangent = glm::vec3(0.0f, 0.0f, 0.0f);
    #endif

        vertices.push_back(vertex);
    }
    
    for(unsigned int i = 0; i < mesh->mNumFaces; i++)
    {
        aiFace face = mesh->mFaces[i];
        for(unsigned int j = 0; j < face.mNumIndices; j++)
            indices.push_back(face.mIndices[j]);
    }

    aiMaterial* material = scene->mMaterials[mesh->mMaterialIndex];    

    glm::vec3 kA;
    aiColor4D ambient;
    if(aiGetMaterialColor(material, AI_MATKEY_COLOR_AMBIENT, &ambient) == AI_SUCCESS)
    {
        kA = glm::vec3(ambient[0], ambient[1], ambient[2]);
    }

    glm::vec3 kD;
    aiColor4D diffuse;
    if(aiGetMaterialColor(material, AI_MATKEY_COLOR_DIFFUSE, &diffuse) == AI_SUCCESS)
    {
        kD = glm::vec3(diffuse[0], diffuse[1], diffuse[2]);
    }

    glm::vec3 kS;
    aiColor4D specular;
    if(aiGetMaterialColor(material, AI_MATKEY_COLOR_SPECULAR, &specular) == AI_SUCCESS)
    {
        kS = glm::vec3(specular[0], specular[1], specular[2]);
    }

    float d;
    float opacity;
    if(aiGetMaterialFloat(material, AI_MATKEY_OPACITY, &opacity) == AI_SUCCESS)
    {
        d = opacity;
    }

    float Ns;
    float shininess;
    if(aiGetMaterialFloat(material, AI_MATKEY_SHININESS, &shininess) == AI_SUCCESS)
    {
        Ns = shininess;
    }


    // 1. diffuse maps
    vector<Texture> diffuseMaps = LoadMaterialTextures(material, aiTextureType_DIFFUSE, "texture_diffuse");
    textures.insert(textures.end(), diffuseMaps.begin(), diffuseMaps.end());
    // 2. specular maps
    vector<Texture> specularMaps = LoadMaterialTextures(material, aiTextureType_SPECULAR, "texture_specular");
    textures.insert(textures.end(), specularMaps.begin(), specularMaps.end());
    // 3. normal maps
    std::vector<Texture> normalMaps = LoadMaterialTextures(material, aiTextureType_HEIGHT, "texture_normal");
    textures.insert(textures.end(), normalMaps.begin(), normalMaps.end());
    // 4. height maps
    std::vector<Texture> heightMaps = LoadMaterialTextures(material, aiTextureType_AMBIENT, "texture_height");
    textures.insert(textures.end(), heightMaps.begin(), heightMaps.end());

    return Mesh(vertices, indices, textures, kA, kD, kS, d, Ns);
}

vector<Texture> ModelData::LoadMaterialTextures(aiMaterial *mat, aiTextureType type, string typeName)
{
    vector<Texture> textures;
    for(unsigned int i = 0; i < mat->GetTextureCount(type); i++)
    {
        aiString str;
        mat->GetTexture(type, i, &str);

        bool skip = false;
        for(unsigned int j = 0; j < textures_loaded_.size(); j++)
        {
            if(std::strcmp(textures_loaded_[j].path.data(), str.C_Str()) == 0)
            {
                textures.push_back(textures_loaded_[j]);
                skip = true; 
                break;
            }
        }
        if(!skip)
        {
            Texture texture;
            texture.id = TextureFromFile(str.C_Str(), this->directory_);
            texture.type = typeName;
            texture.path = str.C_Str();
            textures.push_back(texture);
            textures_loaded_.push_back(texture);
        }
    }
    return textures;
}

unsigned int TextureFromFile(const char *path, const string &directory, bool gamma)
{
    string filename = string(path);
    filename = directory + '/' + filename;

    unsigned int textureID;
    glGenTextures(1, &textureID);

    int width, height, nrComponents;
    unsigned char *data = stbi_load(filename.c_str(), &width, &height, &nrComponents, 0);
    if (data)
    {
        GLenum format;
        if (nrComponents == 1)
            format = GL_RED;
        else if (nrComponents == 3)
            format = GL_RGB;
        else if (nrComponents == 4)
            format = GL_RGBA;

        glBindTexture(GL_TEXTURE_2D, textureID);
        glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format, GL_UNSIGNED_BYTE, data);
        glGenerateMipmap(GL_TEXTURE_2D);

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        stbi_image_free(data);
    }
    else
    {
        std::cout << "Texture failed to load at path: " << path << std::endl;
        stbi_image_free(data);
    }

    return textureID;
}

GLuint LoadCubemap(const std::vector<std::string>& faces)
{
    GLuint textureID;
    glGenTextures(1, &textureID);
    glBindTexture(GL_TEXTURE_CUBE_MAP, textureID);

    int width, height, nrComponents;
    for (unsigned int i = 0; i < faces.size(); i++)
    {
        unsigned char *data = stbi_load(faces[i].c_str(), &width, &height, &nrComponents, 0);
        if (data)
        {
            glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
            stbi_image_free(data);
        }
        else
        {
            std::cout << "Cubemap texture failed!" << faces[i] << std::endl;
            stbi_image_free(data);
        }
    }
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);

    return textureID;
}