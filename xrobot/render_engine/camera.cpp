#include "camera.h"

namespace xrobot {
namespace render_engine {

Camera::Camera(glm::vec3 position,
               glm::vec3 offset,
               glm::vec3 up,
               float yaw,
               float pitch) : Front(glm::vec3(0.0f, 0.0f, -1.0f)),
                              MovementSpeed(SPEED),
                              MouseSensitivity(SENSITIVITY),
                              Zoom(ZOOM) {
    Offset = offset;
    Position = position;
    WorldUp = up;
    Yaw = yaw;
    Pitch = pitch;
    Pre_Pitch = 0;
    updateCameraVectors();
}

Camera::Camera(float posX, float posY, float posZ,
               float upX, float upY, float upZ,
               float yaw, float pitch) : Front(glm::vec3(0.0f, 0.0f, -1.0f)),
                                         MovementSpeed(SPEED),
                                         MouseSensitivity(SENSITIVITY),
                                         Zoom(ZOOM) {
    Position = glm::vec3(posX, posY, posZ);
    WorldUp = glm::vec3(upX, upY, upZ);
    Yaw = yaw;
    Pitch = pitch;
    Pre_Pitch = 0;
    updateCameraVectors();
}

void Camera::ProcessKeyboard(Camera_Movement direction, float deltaTime) {
    float velocity = MovementSpeed * deltaTime;
    switch (direction) {
        case FORWARD: Position += Front * velocity;
                      break;
        case BACKWARD: Position -= Front * velocity;
                       break;
        case LEFT: Position -= Right * velocity;
                   break;
        case RIGHT: Position += Right * velocity;
                    break;
        case UP: Position += Up * velocity;
                 break;
        case DOWN: Position -= Up * velocity;
                   break;
    }
}

void Camera::ProcessMouseMovement(float xoffset,
                                  float yoffset,
                                  GLboolean constrainPitch) {
    xoffset *= MouseSensitivity;
    yoffset *= MouseSensitivity;
    
    Yaw   += xoffset;
    Pitch += yoffset;
    
    // Make sure that when pitch is out of bounds, screen doesn't get
    // flipped
    if (constrainPitch) {
        if (Pitch > 89.0f)
            Pitch = 89.0f;
        if (Pitch < -89.0f)
            Pitch = -89.0f;
    }
    
    // Update Front, Right and Up Vectors using the updated Euler angles
    updateCameraVectors();
}

void Camera::ProcessMouseScroll(float yoffset) {
    if (Zoom >= 1.0f && Zoom <= 45.0f) {
        Zoom -= yoffset;
    }
    Zoom = std::min(std::max(1.0f, Zoom), 45.0f);
}

void Camera::updateCameraVectors() {
    // Calculate the new Front vector
    glm::vec3 front;
    front.x = cos(glm::radians(Yaw)) * cos(glm::radians(Pitch));
    front.y = sin(glm::radians(Pitch));
    front.z = sin(glm::radians(Yaw)) * cos(glm::radians(Pitch));
    Front = glm::normalize(front);
    // Also re-calculate the Right and Up vector
    // Normalize the vectors, because their length gets closer to 0 the more
    // you look up or down which results in slower movement.
    Right = glm::normalize(glm::cross(Front, WorldUp));  
    Up    = glm::normalize(glm::cross(Right, Front));
}

} } // xrobot::render_engine
