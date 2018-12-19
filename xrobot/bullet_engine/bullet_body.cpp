#include "bullet_body.h"

namespace xrobot {
namespace bullet_engine {

bool BulletBody::load_urdf(
        const ClientHandle client,
        const std::string& filename,
        const glm::vec3& pos,
        const glm::vec4& quat,
        const xScalar scale,
        const bool fixed_base,
        const bool self_collision,
        const bool use_multibody,
        const bool concave) {
 
    CommandHandle cmd_handle =
            b3LoadUrdfCommandInit(client, filename.c_str());
    
    b3LoadUrdfCommandSetStartPosition(
            cmd_handle, pos[0], pos[1], pos[2]);
    b3LoadUrdfCommandSetStartOrientation(
            cmd_handle, quat[0], quat[1], quat[2], quat[3]);
    b3LoadUrdfCommandSetUseFixedBase(cmd_handle, fixed_base);
    b3LoadUrdfCommandSetGlobalScaling(cmd_handle, scale);
    b3LoadUrdfCommandSetUseMultiBody(cmd_handle, use_multibody);

    if (self_collision) {
        b3LoadUrdfCommandSetFlags(
                cmd_handle,
                kURDFSelfCollision | kURDFSelfCollisionExParents);
    } else {
        b3LoadUrdfCommandSetFlags(cmd_handle, 0); // kURDFEnableSleeping
    }

    StatusHandle status_handle =
            b3SubmitClientCommandAndWaitStatus(client, cmd_handle);
    int status_type = b3GetStatusType(status_handle);
    if (status_type != CMD_URDF_LOADING_COMPLETED) {
        fprintf(stderr, "Cannot load URDF file '%s'.\n", filename.c_str());
        return false;
    }

    body_data_.scale = btVector3(scale, scale, scale);
    body_data_.fixed = fixed_base;
    body_data_.concave = concave;
    body_data_.mass = 0.0f;
    body_data_.urdf_name = filename;
    body_data_.path = filename;
    body_data_.body_uid = b3GetStatusBodyIndex(status_handle);

    return true;
}

void BulletBody::load_root_part(
        const ClientHandle client, BulletObject* root_part) {
    b3BodyInfo info;
    b3GetBodyInfo(client, body_data_.body_uid, &info);
    root_part->bullet_link_id_ = -1;
    root_part->object_name_ = info.m_baseName;
}

bool BulletBody::load_part(
        const ClientHandle client,
        const int part_id,
        BulletJoint* joint,
        BulletObject* part) {
    
    bool keep_joint = false;
    struct b3JointInfo info;
    b3GetJointInfo(client, body_data_.body_uid, part_id, &info);
    if (info.m_jointType == kRevolute || info.m_jointType == kPrismatic) {
        joint->bullet_joint_id_ = part_id;
        joint->joint_name_ = info.m_jointName;
        joint->joint_type_ =
                info.m_jointType == kRevolute ? kRotationMotor : kLinearMotor;
        joint->bullet_q_index_ = info.m_qIndex;
        joint->bullet_u_index_ = info.m_uIndex;
        joint->joint_has_limits_ = info.m_jointLowerLimit < info.m_jointUpperLimit;
        joint->joint_limit_1_ = info.m_jointLowerLimit;
        joint->joint_limit_2_ = info.m_jointUpperLimit;
        joint->joint_max_force_ = info.m_jointMaxForce;
        joint->joint_max_velocity_ = info.m_jointMaxVelocity;
        keep_joint = true;
    }

    part->bullet_link_id_ = part_id;
    part->object_name_ = info.m_linkName;

    return keep_joint;
}

int BulletBody::get_visual_shape_info(const ClientHandle client) {
    CommandHandle cmd_handle =
            b3InitRequestVisualShapeInformation(client, body_data_.body_uid);
    StatusHandle status_handle =
            b3SubmitClientCommandAndWaitStatus(client, cmd_handle);
    int status_type = b3GetStatusType(status_handle);
    if (status_type != CMD_VISUAL_SHAPE_INFO_COMPLETED) {
        fprintf(stderr, "Cannot load visual shape infomation\n");
        return -1;
    }

    b3GetVisualShapeInformation(client, &visual_shape_info_);

    return visual_shape_info_.m_numVisualShapes;
}

int BulletBody::get_visual_shape(
        int i,
        glm::vec4& color,
        glm::vec3& scale,
        glm::vec3& pos,
        glm::vec4& quat,
        std::string& filename,
        int& geometry_type) {

    int link_id = visual_shape_info_.m_visualShapeData[i].m_linkIndex;
    assert(link_id >= -1);

    filename = visual_shape_info_.m_visualShapeData[i].m_meshAssetFileName;
    geometry_type = visual_shape_info_.m_visualShapeData[i].m_visualGeometryType;

    if(filename == "") {
        filename = std::to_string(geometry_type);
    }

    color[0] = visual_shape_info_.m_visualShapeData[i].m_rgbaColor[0];
    color[1] = visual_shape_info_.m_visualShapeData[i].m_rgbaColor[1];
    color[2] = visual_shape_info_.m_visualShapeData[i].m_rgbaColor[2];
    color[3] = visual_shape_info_.m_visualShapeData[i].m_rgbaColor[3];
            
    scale[0] = visual_shape_info_.m_visualShapeData[i].m_dimensions[0];
    scale[1] = visual_shape_info_.m_visualShapeData[i].m_dimensions[1];
    scale[2] = visual_shape_info_.m_visualShapeData[i].m_dimensions[2];

    pos[0] = visual_shape_info_.m_visualShapeData[i].m_localVisualFrame[0];
    pos[1] = visual_shape_info_.m_visualShapeData[i].m_localVisualFrame[1];
    pos[2] = visual_shape_info_.m_visualShapeData[i].m_localVisualFrame[2];
            
    quat[0] = visual_shape_info_.m_visualShapeData[i].m_localVisualFrame[3];
    quat[1] = visual_shape_info_.m_visualShapeData[i].m_localVisualFrame[4];
    quat[2] = visual_shape_info_.m_visualShapeData[i].m_localVisualFrame[5];
    quat[3] = visual_shape_info_.m_visualShapeData[i].m_localVisualFrame[6];

    return link_id;
}

bool BulletBody::load_obj(
        const ClientHandle client,
        BulletObject* root_part,
        const std::string& filename,
        const glm::vec3& pos,
        const glm::vec4& quat,
        const glm::vec3& scale,
        const xScalar mass,
        const bool concave) {
    CommandHandle cmd_handle = b3LoadObjCommandInit(client, filename.c_str());

    if (concave) {
        b3LoadObjCommandSetFlags(cmd_handle, kOBJConcave);
    }

    b3LoadObjCommandSetStartPosition(cmd_handle, pos[0], pos[1], pos[2]);
    b3LoadObjCommandSetStartOrientation(
            cmd_handle, quat[0], quat[1], quat[2], quat[3]);
    b3LoadObjCommandSetStartScale(cmd_handle, scale[0], scale[1], scale[2]);
    b3LoadObjCommandSetMass(cmd_handle, mass);
    StatusHandle status_handle = 
        b3SubmitClientCommandAndWaitStatus(client, cmd_handle);

    int status_type = b3GetStatusType(status_handle);
    if (status_type != CMD_OBJ_LOADING_COMPLETED) {
        fprintf(stderr, "Cannot load OBJ file '%s'.\n", filename.c_str());
        return false;
    }

    b3BodyInfo info;
    b3GetBodyInfo(client, body_data_.body_uid, &info);

    std::string filename_with_scale = 
            filename + ":" + 
            std::to_string(scale[0]) + ":" + 
            std::to_string(scale[1]) + ":" + 
            std::to_string(scale[2]);

    body_data_.scale = btVector3(scale[0], scale[1], scale[2]);
    body_data_.fixed = true;
    body_data_.concave = concave;
    body_data_.mass = mass;
    body_data_.urdf_name = filename_with_scale;
    body_data_.path = filename;
    body_data_.body_uid = b3GetStatusBodyIndex(status_handle);

    root_part->object_name_ = info.m_baseName;
    root_part->bullet_link_id_ = -1;
    
    return true;
}

void BulletBody::remove_from_bullet(const ClientHandle client, const int id) {
    b3SubmitClientCommandAndWaitStatus(
            client, b3InitRemoveBodyCommand(client, id));
}

void BulletBody::update_joints(const ClientHandle client) {
    CommandHandle cmd_handle =
            b3JointControlCommandInit2(client, body_data_.body_uid, kVelocity);
    b3SubmitClientCommandAndWaitStatus(client, cmd_handle);
}

void BulletBody::query_pose(const ClientHandle client,
                            const xScalar** room_iner_frame,
                            const xScalar** q,
                            const xScalar** q_dot) {
    CommandHandle cmd_handle =
            b3RequestActualStateCommandInit(client, body_data_.body_uid);
    StatusHandle status_handle = 
            b3SubmitClientCommandAndWaitStatus(client, cmd_handle);

    b3GetStatusActualState(
        status_handle,
        0,
        0,
        0,
        room_iner_frame,
        q,
        q_dot,
        0
    );
}

void BulletBody::query_link(const ClientHandle client,
                            const int id,
                            b3LinkState& state) {
    CommandHandle cmd_handle =
            b3RequestActualStateCommandInit(client, body_data_.body_uid);
    b3RequestActualStateCommandComputeLinkVelocity(cmd_handle, kComputeVelocity);
    StatusHandle status_handle = 
            b3SubmitClientCommandAndWaitStatus(client, cmd_handle);

    b3GetLinkState(client, status_handle, id, &state);   
}

void BulletBody::move(
        const xScalar move,
        const xScalar rot,
        BulletObject* root_part,
        xScalar* p,
        xScalar* q,
        xScalar* prev_q,
        xScalar* prev_o) {

    btVector3 pos;
    btQuaternion quat;
    root_part->pose(pos, quat);

    if (first_move_) {
        angle_ = 0;
        base_orientation_ = quat;
        first_move_ = false;
        orientation_ = btQuaternion(btVector3(0, 1, 0), 0);
    }

    prev_o[0] = orientation_[0];
    prev_o[1] = orientation_[1];
    prev_o[2] = orientation_[2];
    prev_o[3] = orientation_[3];

    orientation_ = orientation_ * btQuaternion(btVector3(0, 1, 0), rot);
    angle_ += rot;

    btVector3 pos_new = pos + btTransform(orientation_) * btVector3(move, 0, 0);
    btQuaternion orn_new = btQuaternion(btVector3(0, 1, 0), angle_) * base_orientation_;
    
    p[0] = pos_new[0];
    p[1] = pos_new[1];
    p[2] = pos_new[2];
    q[0] = orn_new[0];
    q[1] = orn_new[1];
    q[2] = orn_new[2];
    q[3] = orn_new[3];
    prev_q[0] = quat[0];
    prev_q[1] = quat[1];
    prev_q[2] = quat[2];
    prev_q[3] = quat[3];
}

void BulletBody::attach(BulletObject* part, const BulletObject* target_root) {
    btTransform T_target = target_root->object_position_;
    btTransform T = part->object_position_.inverse() * T_target;
    part->attach_transform_ = T;
}

// void BulletBody::detach(BulletObject* root) {
//     if(body_data_.attach_to_id < -1) {
//         printf("Nothing to detach!\n");
//         return;
//     }

//     if(body_data_.attach_to_id == -1) {
//         root->detach();
//     } else {
//         //body_data_.other_parts_[attach_to_id_]->attach_object_ = object;
//         // parts_[body_data_.attach_to_id]->detach();
//     }
//     body_data_.attach_to_id = -2;
// }

void BulletBody::attach_camera(
        const BulletObject* part,
        const glm::vec3& offset,
        const float pitch,
        glm::vec3& loc,
        glm::vec3& front,
        glm::vec3& right,
        glm::vec3& up) {
    btTransform pose = part->object_position_;
    auto base_orientation = pose.getBasis();
    btVector3 base_front = base_orientation * btVector3(1, 0, 0); 

    // TODO: this part of codes are just used to compute the offset of camera
    // relative to the body, i.e., camera_aim.
    glm::vec3 f = glm::normalize(
            glm::vec3(base_front[0], base_front[1], base_front[2]));
    glm::vec3 r = glm::normalize(glm::cross(f, glm::vec3(0,-1,0)));
    glm::vec3 u = glm::normalize(glm::cross(f, r));
    glm::vec3 camera_aim(f*offset.x + u*offset.y + r*offset.z);

    btMatrix3x3 pre_orientation;
    pre_orientation.setIdentity();
    pre_orientation.setEulerYPR(0, glm::radians(pitch), 0);
    base_front = base_orientation * pre_orientation * btVector3(1, 0, 0); 
    front = glm::normalize(glm::vec3(base_front[0], base_front[1], base_front[2]));
    right = glm::normalize(glm::cross(front, glm::vec3(0,-1,0)));
    up = glm::normalize(glm::cross(front, right));

    auto base_position = pose.getOrigin();
    loc = glm::vec3(base_position[0], base_position[1], base_position[2]) 
          + camera_aim;
}

void BulletBody::inverse_kinematics(
        const ClientHandle client,
        const int id,
        const int end_index,
        const glm::vec3& target_pos,
        const glm::vec4& target_quat,
        const xScalar* joint_damping,
        xScalar* output_joint_pos,
        int& num_poses) {
    const int solver = 0;
    const int num_joints = b3GetNumJoints(client, id);
    const int dof = b3ComputeDofCount(client, id);

    xScalar p[3] = {target_pos[0], target_pos[1], target_pos[2]};
    xScalar q[4] =
            {target_quat[0], target_quat[1], target_quat[2], target_quat[3]};

    CommandHandle cmd_handle =
            b3CalculateInverseKinematicsCommandInit(client, id);
    b3CalculateInverseKinematicsSelectSolver(cmd_handle, solver);

    b3CalculateInverseKinematicsAddTargetPositionWithOrientation(
            cmd_handle, end_index, p, q);
    b3CalculateInverseKinematicsSetJointDamping(cmd_handle, dof, joint_damping);
    StatusHandle status_handle =
            b3SubmitClientCommandAndWaitStatus(client, cmd_handle);

    int result_body_index;
    int result = b3GetStatusInverseKinematicsJointPositions(
            status_handle, &result_body_index, &num_poses, 0);

    if (result && num_poses) {
        result = b3GetStatusInverseKinematicsJointPositions(
                status_handle, &result_body_index, &num_poses, output_joint_pos);
    }
}

void BulletBody::get_closest_points(const ClientHandle client, 
        std::vector<ContactPoint>& points) {
    struct b3ContactInformation contact_point_data;
    CommandHandle cmd_handle = b3InitClosestDistanceQuery(client);

    b3SetClosestDistanceFilterBodyA(cmd_handle, body_data_.body_uid);
    b3SetClosestDistanceThreshold(cmd_handle, 0.f);
    b3SubmitClientCommandAndWaitStatus(client, cmd_handle);
    
    b3GetContactPointInformation(client, &contact_point_data);

    points.clear();
    for (int i = 0; i < contact_point_data.m_numContactPoints; ++i)
    {
        ContactPoint point;

        float normal_x, normal_y, normal_z;
        normal_x = contact_point_data.m_contactPointData[i].m_contactNormalOnBInWS[0];
        normal_y = contact_point_data.m_contactPointData[i].m_contactNormalOnBInWS[1];
        normal_z = contact_point_data.m_contactPointData[i].m_contactNormalOnBInWS[2];

        float pos_a_x, pos_a_y, pos_a_z;
        pos_a_x = contact_point_data.m_contactPointData[i].m_positionOnAInWS[0];
        pos_a_y = contact_point_data.m_contactPointData[i].m_positionOnAInWS[1];
        pos_a_z = contact_point_data.m_contactPointData[i].m_positionOnAInWS[2];

        float pos_b_x, pos_b_y, pos_b_z;
        pos_b_x = contact_point_data.m_contactPointData[i].m_positionOnBInWS[0];
        pos_b_y = contact_point_data.m_contactPointData[i].m_positionOnBInWS[1];
        pos_b_z = contact_point_data.m_contactPointData[i].m_positionOnBInWS[2];

        point.contact_normal = glm::vec3(normal_x, normal_y, normal_z);
        point.contact_force = 
                contact_point_data.m_contactPointData[i].m_normalForce;
        point.contact_distance = 
                contact_point_data.m_contactPointData[i].m_contactDistance;
        point.bullet_id_a = 
                contact_point_data.m_contactPointData[i].m_bodyUniqueIdA;
        point.bullet_id_b = 
                contact_point_data.m_contactPointData[i].m_bodyUniqueIdB;
        point.contact_position_a = glm::vec3(pos_a_x, pos_a_y, pos_a_z);
        point.contact_position_b = glm::vec3(pos_b_x, pos_b_y, pos_b_z);

        points.push_back(point);
    }
}

void BulletBody::get_contact_points(const ClientHandle client,
        std::vector<ContactPoint>& points) {
    struct b3ContactInformation contact_point_data;
    CommandHandle cmd_handle = b3InitRequestContactPointInformation(client);

    b3SetContactFilterBodyA(cmd_handle, body_data_.body_uid);
    b3SubmitClientCommandAndWaitStatus(client, cmd_handle);

    b3GetContactPointInformation(client, &contact_point_data);
    points.clear();

    for (int i = 0; i < contact_point_data.m_numContactPoints; ++i)
    {
        ContactPoint point;

        float normal_x, normal_y, normal_z;
        normal_x = contact_point_data.m_contactPointData[i].m_contactNormalOnBInWS[0];
        normal_y = contact_point_data.m_contactPointData[i].m_contactNormalOnBInWS[1];
        normal_z = contact_point_data.m_contactPointData[i].m_contactNormalOnBInWS[2];

        float pos_a_x, pos_a_y, pos_a_z;
        pos_a_x = contact_point_data.m_contactPointData[i].m_positionOnAInWS[0];
        pos_a_y = contact_point_data.m_contactPointData[i].m_positionOnAInWS[1];
        pos_a_z = contact_point_data.m_contactPointData[i].m_positionOnAInWS[2];

        float pos_b_x, pos_b_y, pos_b_z;
        pos_b_x = contact_point_data.m_contactPointData[i].m_positionOnBInWS[0];
        pos_b_y = contact_point_data.m_contactPointData[i].m_positionOnBInWS[1];
        pos_b_z = contact_point_data.m_contactPointData[i].m_positionOnBInWS[2];

        point.contact_normal = glm::vec3(normal_x, normal_y, normal_z);
        point.contact_force = contact_point_data.m_contactPointData[i].m_normalForce;
        point.contact_distance = 
                contact_point_data.m_contactPointData[i].m_contactDistance;
        point.bullet_id_a = contact_point_data.m_contactPointData[i].m_bodyUniqueIdA;
        point.bullet_id_b = contact_point_data.m_contactPointData[i].m_bodyUniqueIdB;
        point.contact_position_a = glm::vec3(pos_a_x, pos_a_y, pos_a_z);
        point.contact_position_b = glm::vec3(pos_b_x, pos_b_y, pos_b_z);
        points.push_back(point);
    }
}

}} // namespace xrobot::bullet_engine
