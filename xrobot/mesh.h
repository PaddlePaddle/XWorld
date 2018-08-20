#ifndef MESH_H_
#define MESH_H_

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>

#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"

#include "shader.h"

using namespace std;

struct Vertex {
    glm::vec3 position;
    glm::vec3 normal;
    glm::vec2 texcoords;
    glm::vec3 tangent;
    glm::vec3 bitangent;
};


struct Texture {
    unsigned int id;
    string type;
    string path;
};

class Mesh {
public:
    vector<Vertex> vertices_;
    vector<unsigned int> indices_;
    vector<Texture> textures_;
    unsigned int VAO_;
    unsigned int VBO_, EBO_;

    // Mtl
    glm::vec3 kA_;
    glm::vec3 kD_;
    glm::vec3 kS_;
    float d_;
    float Ns_;


    Mesh() {}

    Mesh(vector<Vertex> vertices, vector<unsigned int> indices, vector<Texture> textures)
    {
        vertices_ = vertices;
        indices_ = indices;
        textures_ = textures;
        kA_ = glm::vec3(0.3f);
        kD_ = glm::vec3(0.5);
        kS_ = glm::vec3(0.5);
        d_  = 1.0f;
        Ns_ = 1.0f;

        SetupMesh();
    }

    Mesh(vector<Vertex> vertices, vector<unsigned int> indices, vector<Texture> textures,
        glm::vec3 kAmbient, glm::vec3 KDiffuse, glm::vec3 kSpecular, float opacity, float shininess)
    {
        vertices_ = vertices;
        indices_ = indices;
        textures_ = textures;

        kA_ = kAmbient;
        kD_ = KDiffuse;
        kS_ = kSpecular;
        d_  = opacity;
        Ns_ = shininess;

        SetupMesh();
    }


    void Draw(Shader shader) 
    {
        unsigned int diffuse_num  = 1;
        unsigned int specular_num = 1;
        unsigned int normal_num   = 1;
        unsigned int height_num   = 1;
        for(unsigned int i = 0; i < textures_.size(); i++)
        {
            glActiveTexture(GL_TEXTURE0 + i);

            string number;
            string name = textures_[i].type;
            if(name == "texture_diffuse")
				number = std::to_string(diffuse_num++);
			else if(name == "texture_specular")
				number = std::to_string(specular_num++);
            else if(name == "texture_normal")
				number = std::to_string(normal_num++);
             else if(name == "texture_height")
			    number = std::to_string(height_num++);
	
            glUniform1i(glGetUniformLocation(shader.id_, (name + number).c_str()), i);
            glBindTexture(GL_TEXTURE_2D, textures_[i].id);
        }

        // bind mtls
        glUniform3fv(glGetUniformLocation(shader.id_, "kA"), 1, &kA_[0]);
        glUniform3fv(glGetUniformLocation(shader.id_, "kD"), 1, &kD_[0]);
        glUniform3fv(glGetUniformLocation(shader.id_, "kS"), 1, &kS_[0]);
        glUniform1f(glGetUniformLocation(shader.id_, "d"), d_);
        glUniform1f(glGetUniformLocation(shader.id_, "Ns"), Ns_);

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

private:
    void SetupMesh()
    {
        glGenVertexArrays(1, &VAO_);
        glGenBuffers(1, &VBO_);
        glGenBuffers(1, &EBO_);

        glBindVertexArray(VAO_);
        glBindBuffer(GL_ARRAY_BUFFER, VBO_);
        glBufferData(GL_ARRAY_BUFFER, vertices_.size() * sizeof(Vertex), &vertices_[0], GL_STATIC_DRAW);  
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO_);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices_.size() * sizeof(unsigned int), &indices_[0], GL_STATIC_DRAW);

        glEnableVertexAttribArray(0);	
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)0);
        glEnableVertexAttribArray(1);	
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, normal));
        glEnableVertexAttribArray(2);	
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, texcoords));
        glEnableVertexAttribArray(3);
        glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, tangent));
        glEnableVertexAttribArray(4);
        glVertexAttribPointer(4, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, bitangent));

        glBindVertexArray(0);
    }
};

#endif // MESH_H_