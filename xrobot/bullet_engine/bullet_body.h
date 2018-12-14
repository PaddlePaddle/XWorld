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
    xScalar mass = 0.0f; // Only for .Obj
    std::string label = "unlabeled";
    bool pickable = false;
    std::string urdf_name = "";
    std::string path = "";
    int body_uid = -1;
    int attach_to_id = -2; // -1 is used by root
};

class BulletBody {
public:
    BulletBody() : 
            body_data_(BulletBodyData()),
            orientation_(btVector3(0,1,0),0),
            angle_(0) {}

    virtual ~BulletBody() {}

// protected:
    bool load_urdf(
            const ClientHandle client,
            const std::string& filename,
            const glm::vec3& pos,
            const glm::vec4& quat,
            const xScalar scale,
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
            const glm::vec3& color,
            const glm::vec3& scale,
            const glm::vec3& pos,
            const glm::vec4& quat,
            std::string& filename,
            int& geometry_type);

    bool load_obj(
            const ClientHandle client,
            BulletObject* root_part,
            const std::string& filename,
            const glm::vec3& pos,
            const glm::vec4& quat,
            const glm::vec3& scale,
            const xScalar mass,
            const bool concave);

    void remove_from_bullet(const ClientHandle client, const int id);

    void update_joints(const ClientHandle client);

    void query_pose(const ClientHandle client,
                    const xScalar** room_iner_frame,
                    const xScalar** q,
                    const xScalar** q_dot);

    void query_link(const ClientHandle client,
                    const int id,
                    b3LinkState& state);

    void move(
            const xScalar move,
            const xScalar rot,
            BulletObject* root_part,
            xScalar* p,
            xScalar* q,
            xScalar* prev_q);

    void attach(BulletObject* part, const BulletObject* target_root);

    // void detach(BulletObject* root);

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
            const xScalar* target_pos,
            const xScalar* target_quat,
            const xScalar* joint_damping,
            xScalar* output_joint_pos,
            int& num_poses);

    void get_closest_points(const ClientHandle client, 
            std::vector<ContactPoint>& points);

    void get_contact_points(const ClientHandle client, 
            std::vector<ContactPoint>& points);


public:
    BulletBodyData body_data_;
    b3VisualShapeInformation visual_shape_info_;
    btQuaternion orientation_;
    xScalar angle_;
};


}} // namespace xrobot::bullet_engine
#endif
