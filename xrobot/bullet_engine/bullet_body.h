#ifndef BULLET_BODY_H_
#define BULLET_BODY_H_

#include "bullet_joint.h"
#include "bullet_object.h"
#include "common.h"

namespace xrobot {
namespace bullet_engine {

struct BulletBodyData {
	//btVector3 scale = btVector3(1.0f, 1.0f, 1.0f);
	btVector3 scale = {1.0f, 1.0f, 1.0f};
    bool fixed = false;
    bool concave = false;
    double mass = 0.0f; // Only for .Obj
    std::string label = "unlabeled";
    bool pickable = false;
    std::string urdf_name = "";
    std::string path = "";
    int body_uid = -1;
    int attach_to_id = -2; // -1 is used by root
    btTransform attach_transform;
    btTransform attach_orientation; 
};

class BulletBody {
public:
    BulletBody() : 
            body_data_(BulletBodyData()),
            orientation_(btVector3(0,1,0),0),
            base_orientation_(),
            angle_(0),
            first_move_(true) {}

    virtual ~BulletBody() {}

// protected:
    bool load_urdf(
            const ClientHandle client,
            const std::string& filename,
            const glm::vec3& pos,
            const glm::vec4& quat,
            const double scale,
            const bool fixed_base,
            const bool self_collision,
            const bool use_multibody,
            const bool concave);

    void load_root_part(const ClientHandle client, BulletObject* root_part);

    bool load_part(
            const ClientHandle client,
            const int part_id,
            BulletJoint* joint,
            BulletObject* part);

    int get_visual_shape_info(const ClientHandle client);

    int get_visual_shape(
            int i,
            glm::vec4& color,
            glm::vec3& scale,
            glm::vec3& pos,
            glm::vec4& quat,
            std::string& filename,
            int& geometry_type);

    bool load_obj(
            const ClientHandle client,
            BulletObject* root_part,
            const std::string& filename,
            const glm::vec3& pos,
            const glm::vec4& quat,
            const glm::vec3& scale,
            const double mass,
            const bool concave);

    void remove_from_bullet(const ClientHandle client, const int id);

    void update_joints(const ClientHandle client);

    void query_pose(const ClientHandle client,
                    const double** room_iner_frame,
                    const double** q,
                    const double** q_dot);

    void query_link(const ClientHandle client,
                    const int id,
                    b3LinkState& state);

    void reset_move() { first_move_ = true; }

    void move(
            const double move,
            const double rot,
            BulletObject* root_part,
            double* p,
            double* q,
            double* prev_q,
            double* prev_o);

    void attach(BulletObject* root_part, const BulletObject* target_root,
            const float pitch, const glm::vec3& offset);

    void attach_camera(
            const BulletObject* part,
            const glm::vec3& offset,
            const float pitch,
            glm::vec3& loc,
            glm::vec3& front,
            glm::vec3& right,
            glm::vec3& up);

    void inverse_kinematics(
            const ClientHandle client,
            const int id,
            const int end_index,
            const glm::vec3& target_pos,
            const glm::vec4& target_quat,
            const double* joint_damping,
            double* output_joint_pos,
            int& num_poses);

    void get_closest_points(const ClientHandle client, 
            std::vector<ContactPoint>& points);

    void get_contact_points(const ClientHandle client, 
            std::vector<ContactPoint>& points);


public:
    BulletBodyData body_data_;
    b3VisualShapeInformation visual_shape_info_;
    btQuaternion orientation_;
    btQuaternion base_orientation_;
    double angle_;
    bool first_move_;
};


}} // namespace xrobot::bullet_engine
#endif
