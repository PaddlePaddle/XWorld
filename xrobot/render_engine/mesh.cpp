#include "mesh.h"

namespace xrobot {
namespace render_engine {

Mesh::Mesh(const std::vector<Vertex>& vertices,
           const std::vector<unsigned int>& indices,
           const std::vector<Texture>& textures) {
    vertices_ = vertices;
    indices_ = indices;
    textures_ = textures;
    kA_ = glm::vec3(0.3f);
    kD_ = glm::vec3(0.5);
    kS_ = glm::vec3(0.5);
    d_  = 1.0f;
    Ns_ = 1.0f;
    diffuse_ = false;
    bump_ = false;
    specular_ = false;
    height_ = false;
    ao_ = false;

    SetupMesh();
}

Mesh::Mesh(const std::vector<Vertex>& vertices,
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
           bool aoMap) {
    vertices_ = vertices;
    indices_ = indices;
    textures_ = textures;

    kA_ = kAmbient;
    kD_ = KDiffuse;
    kS_ = kSpecular;
    d_  = opacity;
    Ns_ = shininess;
    diffuse_ = diffuseMap;
    bump_ = normalMap;
    specular_ = specularMap;
    height_ = heightMap;
    ao_ = aoMap;

    SetupMesh();
}

void Mesh::Draw(const Shader& shader) {
    unsigned int diffuse_num  = 0;
    unsigned int specular_num = 0;
    unsigned int normal_num   = 0;
    unsigned int height_num   = 0;
    unsigned int ao_num       = 0;

    for (size_t i = 0; i < textures_.size(); i++) {
        glActiveTexture(GL_TEXTURE4 + i);

        std::string number;
        std::string name = textures_[i].type;
        if(name == "texture_diffuse")
            number = std::to_string(diffuse_num++);
        else if(name == "texture_specular")
            number = std::to_string(specular_num++);
        else if(name == "texture_normal")
            number = std::to_string(normal_num++);
        else if(name == "texture_height")
            number = std::to_string(height_num++);
        else if(name == "texture_ao")
            number = std::to_string(ao_num++);

        glUniform1i(glGetUniformLocation(shader.id(), (name+number).c_str()), 4 + i);
        glBindTexture(GL_TEXTURE_2D, textures_[i].id);
    }

    // bind mtls
    glUniform3fv(glGetUniformLocation(shader.id(), "kA"), 1, &kA_[0]);
    glUniform3fv(glGetUniformLocation(shader.id(), "kD"), 1, &kD_[0]);
    glUniform3fv(glGetUniformLocation(shader.id(), "kS"), 1, &kS_[0]);
    glUniform1f(glGetUniformLocation(shader.id(), "d"), d_);
    glUniform1f(glGetUniformLocation(shader.id(), "Ns"), Ns_);
    glUniform1i(glGetUniformLocation(shader.id(), "bump"), bump_);
    glUniform1i(glGetUniformLocation(shader.id(), "displacement"), height_);
    glUniform1i(glGetUniformLocation(shader.id(), "diffuseMap"), diffuse_);
    glUniform1i(glGetUniformLocation(shader.id(), "specularMap"), specular_);
    glUniform1i(glGetUniformLocation(shader.id(), "aoMap"), ao_);

    // draw mesh
    glBindVertexArray(VAO_);
    if(indices_.size() != 0) 
        glDrawElements(GL_TRIANGLES, indices_.size(), GL_UNSIGNED_INT, 0);
    else 
        glDrawArrays(GL_TRIANGLES, 0, vertices_.size());
    glBindVertexArray(0);

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, 0);
}

void Mesh::SetupMesh() {
    glGenVertexArrays(1, &VAO_);
    glGenBuffers(1, &VBO_);
    glGenBuffers(1, &EBO_);

    glBindVertexArray(VAO_);
    glBindBuffer(GL_ARRAY_BUFFER, VBO_);
    glBufferData(GL_ARRAY_BUFFER,
                 vertices_.size() * sizeof(Vertex),
                 &vertices_[0],
                 GL_STATIC_DRAW);
                   
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                 indices_.size() * sizeof(unsigned int),
                 &indices_[0],
                 GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);	
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)0);
    glEnableVertexAttribArray(1);	
    glVertexAttribPointer(1,
                          3,
                          GL_FLOAT,
                          GL_FALSE,
                          sizeof(Vertex),
                          (void*)offsetof(Vertex, normal));
    glEnableVertexAttribArray(2);	
    glVertexAttribPointer(2,
                          2,
                          GL_FLOAT,
                          GL_FALSE,
                          sizeof(Vertex),
                          (void*)offsetof(Vertex, texcoords));
    glEnableVertexAttribArray(3);
    glVertexAttribPointer(3,
                          3,
                          GL_FLOAT,
                          GL_FALSE,
                          sizeof(Vertex),
                          (void*)offsetof(Vertex, tangent));
    glEnableVertexAttribArray(4);
    glVertexAttribPointer(4,
                          3,
                          GL_FLOAT,
                          GL_FALSE,
                          sizeof(Vertex),
                          (void*)offsetof(Vertex, bitangent));

    glBindVertexArray(0);
}

} } // xrobot::render_engine
