#include "camera.h"

namespace xrobot {
namespace render_engine {

Camera::Camera(glm::vec3 position,
			   glm::vec3 offset,
			   glm::vec3 up,
			   float yaw,
			   float pitch) : front_(glm::vec3(0.0f, 0.0f, -1.0f)),
							  move_speed_(kMovementSpeed),
							  sensitivity_(kMouseSensitivity),
							  fov_(kFOV),
							  offset_(offset),
							  position_(position),
							  world_up_(up),
							  yaw_(yaw),
							  pitch_(pitch),
							  pre_pitch_(0) { Update(); }

void Camera::ProcessKeyboard(const Movement direction, float delta_time) {
	float velocity = move_speed_ * delta_time;
	switch (direction) {
		case kForward: position_ += front_ * velocity; break;
		case kBackward: position_ -= front_ * velocity; break;
		case kLeft: position_ -= right_ * velocity; break;
		case kRight: position_ += right_ * velocity; break;
		case kUp: position_ += up_ * velocity; break;
		case kDown: position_ -= up_ * velocity; break;
	}

	Update();
}

void Camera::ProcessMouseMovement(const float xoffset,
								  const float yoffset,
								  const bool lock) {
	float xoffset_s = xoffset * sensitivity_;
	float yoffset_s = yoffset * sensitivity_;
	yaw_   += xoffset;
	pitch_ += yoffset;
	
	if (lock) {
		pitch_ = glm::clamp(pitch_, -89.0f, 89.0f);
	}
	
	Update();
}

void Camera::Update() {
	glm::vec3 front;
	front.x = cos(glm::radians(yaw_)) * cos(glm::radians(pitch_));
	front.y = sin(glm::radians(pitch_));
	front.z = sin(glm::radians(yaw_)) * cos(glm::radians(pitch_));
	front_  = glm::normalize(front);
	right_  = glm::normalize(glm::cross(front_, world_up_));  
	up_     = glm::normalize(glm::cross(right_, front_));
}

} } // xrobot::render_engine
