#ifndef RENDER_ENGINE_CAMERA_H
#define RENDER_ENGINE_CAMERA_H

#include <vector>

#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"

#define INCLUDE_GL_CONTEXT_HEADERS
#include "gl_header.h"

namespace xrobot {
namespace render_engine {

const float YAW         =  0.0f;
const float PITCH       =  0.0f;
const float SPEED       =  10.0f;
const float SENSITIVITY =  0.5f;
const float ZOOM        =  40.0f;
const float NEAR        =  0.01f;
const float FAR         =  100.0f;

class Camera {
public:
    enum Camera_Movement {
        FORWARD,
        BACKWARD,
        LEFT,
        RIGHT,
        UP,
        DOWN,
    };

public:
    // Camera Attributes
    float Pre_Pitch;
    glm::vec3 Offset;
    glm::vec3 Position;
    glm::vec3 Front;
    glm::vec3 Up;
    glm::vec3 Right;
    glm::vec3 WorldUp;
    // Euler Angles
    float Yaw;
    float Pitch;
    // Camera options
    float MovementSpeed;
    float MouseSensitivity;
    float Zoom;
    
    float Near;
    float Far;
    float Aspect;
    
    // Constructor with vectors
    Camera(glm::vec3 position = glm::vec3(0.0f, 0.0f, 0.0f),
           glm::vec3 offset = glm::vec3(0.0f, 0.0f, 0.0f),
           glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f),
           float yaw = YAW,
           float pitch = PITCH);

    // Constructor with scalar values
    Camera(float posX, float posY, float posZ,
           float upX, float upY, float upZ,
           float yaw = YAW,
           float pitch = PITCH);
    
    // Returns the view matrix calculated using Euler Angles and the LookAt
    // Matrix
    glm::mat4 GetViewMatrix() {
        return glm::lookAt(Position, Position + Front, Up);
    }
    
    glm::mat4 GetProjectionMatrix() {
        return glm::perspective(glm::radians(Zoom), Aspect, Near, Far);
    }
    
    // Processes input received from any keyboard-like input system. Accepts
    // input parameter in the form of camera defined ENUM (to abstract it from 
    // windowing systems)
    void ProcessKeyboard(Camera_Movement direction, float deltaTime);
    
    // Processes input received from a mouse input system. Expects the offset
    // value in both the x and y direction.
    void ProcessMouseMovement(float xoffset, float yoffset,
                              GLboolean constrainPitch = true);
    
    // Processes input received from a mouse scroll-wheel event. Only requires 
    // input on the vertical wheel-axis
    void ProcessMouseScroll(float yoffset);    

    // Calculates the front vector from the Camera's (updated) Euler Angles
    void updateCameraVectors();
};

} } // xrobot::render_engine

#endif
