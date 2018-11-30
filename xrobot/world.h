#ifndef WORLD_H_
#define WORLD_H_

#include <map>
#include <string>
#include <functional>
#include <vector>
#include <unordered_map>
#include <queue>
#include <unistd.h>
#include <errno.h>
#include <utility>
#include <chrono>
#include <thread>

#include "bullet_engine/common.h"
#include "bullet_engine/bullet_joint.h"
#include "bullet_engine/bullet_object.h"
#include "render_engine/model.h"
#include "render_engine/render_world.h"

#include "inventory.h"

namespace xrobot {

class Joint : public bullet_engine::BulletJoint {
public:
    Joint();
    
    ~Joint();

    void EnableJointSensor(const bool enable);
    
    void GetJointMotorState(glm::vec3& force, glm::vec3& torque);

    void ResetJointState(const float pos, const float vel);

    void SetJointMotorControlTorque(const float torque);

    void SetJointMotorControlVelocity(
            const float speed, const float k_d, const float max_force);

    void SetJointMotorControlPosition(
            const float target,
            const float k_p,
            const float k_d,
            const float max_force);

    std::weak_ptr<World> bullet_world_;    
    std::weak_ptr<RobotBase> bullet_robot_;
};

class Object : public render_engine::RenderPart,
               public bullet_engine::BulletObject {
public:
    Object();

    ~Object() {};

    void Sleep();

    void EnableSleeping();
    
    void DisableSleeping();

    void Wake();

    void GetMass(float& mass);

    void SetStatic();
    
    void RecoverFromStatic();

    void ChangeLinearDamping(const float damping);

    void ChangeAngularDamping(const float damping);

    void ChangeLateralFriction(const float friction);

    void ChangeSpinningFriction(const float friction);

    void ChangeRollingFriction(const float friction);

    void ApplyForce(const float x,
                    const float y,
                    const float z,
                    const int flags = EF_LINK_FRAME);

    void ApplyTorque(const float x,
                     const float y,
                     const float z, 
                     const int flags = EF_LINK_FRAME);
public:
    std::weak_ptr<World> bullet_world_;

    int body_uid_; // id of Robot this Object belongs to

    std::string object_name_;

    std::weak_ptr<RobotBase> attach_object_;

private:
    float object_mass_original_;

// xrobot::render_engine::RenderPart
public:
    int id() const override { return body_uid_; }

    void set_id(int id) override { body_uid_ = id; }

    void GetAABB(glm::vec3& aabb_min, glm::vec3& aabb_max) override;

    glm::mat4 translation_matrix() const override;

    glm::mat4 local_inertial_frame() const override;
};

struct RobotData {
	RobotData();

	RobotData(btVector3 scale, bool fixed, float mass,
			std::string label, std::string urdf_name,
			std::string path, int bullet_handle,
			std::shared_ptr<Object> root_part,
			std::vector<std::shared_ptr<Object>> other_parts,
			std::vector<std::shared_ptr<Joint>> joints_list);

	btVector3 scale_;
    bool fixed_;
    bool concave_;
    float mass_; // Only for .Obj
    std::string label_;
    bool pickable_;
    std::string urdf_name_;
    std::string path_;
    int bullet_handle_;
    int attach_to_id_;
    std::shared_ptr<Object> root_part_;
    std::vector<std::shared_ptr<Object>> other_parts_;
    std::vector<std::shared_ptr<Joint>> joints_list_;
};

class RobotBase : public render_engine::RenderBody,
                  public std::enable_shared_from_this<RobotBase> {
    using RenderPart = render_engine::RenderPart;
public:

    RobotBase(std::weak_ptr<World> bullet_world);

    void UpdatePickable(const bool pick) { robot_data_.pickable_ = pick; }
    void UpdateTag(const std::string& tag) { robot_data_.label_ = tag; }

    virtual ~RobotBase();

    virtual bool InteractWith(const std::string& tag);
    virtual bool TakeAction(const int act_id);
    virtual std::vector<std::string> GetActions() const;

    virtual void LoadConvertedObject(
        const std::string& filename,
        const btVector3 position,
        const btQuaternion rotation,
        const float scale = 1.0f,
        const std::string& label = "",
        const bool concave = false
    );
    virtual void LoadAnimatedObject(
        const std::string& filename,
        const btVector3 position,
        const btQuaternion rotation,
        const float scale = 1.0f,
        const std::string& label = "",
        const bool concave = false
    );

    void LoadOBJ(
        const std::string& filename,
        const btVector3 position,
        const btQuaternion rotation,
        const btVector3 scale,
        const std::string& label = "unlabeled",
        const float mass = 0,
        const bool flip = false,
        const bool concave = false
    );

    void LoadURDF(
        const std::string& filename,
        const btVector3 position,
        const btQuaternion rotation,
        const float scale = 1.0f,
        const std::string& label = "unlabeled",
        const bool fixed_base = false
    );

    void LoadURDFFile(
        const std::string& filename,
        const btVector3 position,
        const btQuaternion rotation,
        const float scale = 1.0f,
        const std::string& label = "unlabeled",
        const bool fixed_base = false,
        const bool self_collision = false,
        const bool use_multibody = true,
        const bool concave = false
    );


    void LoadOBJFile(
        const std::string& filename,
        const btVector3 position,
        const btQuaternion rotation,
        const btVector3 scale,
        const std::string& label = "unlabeled",
        const float mass = 0,
        const bool flip = false,
        const bool concave = false
    );

    void LoadRobotShape(const float scale);
    void LoadRobotJoint(const std::string &filename);

    void DisableSleeping();

    void Wake();

    void Sleep();

    virtual void recycle() override;

    void reuse() override {
        recycle_ = false;
        hide_ = false;
    }

    void RemoveRobotFromBullet();

    void SetJointVelocity(const int joint_id, const float speed,
                          const float k_d, const float max_force);

    void SetJointPosition(const int joint_id, const float target,
                          const float k_p, const float k_d,
                          const float max_force);

    void ResetJointState(const int joint_id, const float pos,
                         const float vel);

    virtual void Move(const float move, const float rotate, const bool remit = false);

    virtual void Teleport2(const glm::vec3 walk_move,
                  const glm::vec3 walk_rotate,
                  const float speed, const bool remit = false); 

    virtual void Teleport(const glm::vec3 walk_move,
                  const glm::vec3 walk_rotate,
                  const float speed, const bool remit = false);

    virtual void UnFreeze();
    virtual void Freeze(const bool remit = false);
    virtual void MoveForward(const float speed);
    virtual void MoveBackward(const float speed);
    virtual void TurnLeft(const float speed);
    virtual void TurnRight(const float speed);
    virtual void PickUp(std::shared_ptr<Inventory> inventory,
            const glm::vec3 from, const glm::vec3 to);
    virtual void PutDown(std::shared_ptr<Inventory> inventory, 
            const glm::vec3 from, const glm::vec3 to);
    virtual void RotateObject(const glm::vec3 rotate_angle,
            const glm::vec3 from, const glm::vec3 to);
    virtual void AttachObject(std::weak_ptr<RobotBase> object, const int id = -1);
    virtual void DetachObject();

    // RenderBody
    virtual const RenderPart* render_root_ptr() const override;

    virtual RenderPart* render_root_ptr() override;

    virtual const RenderPart* render_part_ptr(const size_t i) const override;

    virtual RenderPart* render_part_ptr(const size_t i) override;

    size_t size() const override { return robot_data_.other_parts_.size(); }

    void attach_camera(const glm::vec3& offset,
                       const float pitch,
                       glm::vec3& loc,
                       glm::vec3& front,
                       glm::vec3& right,
                       glm::vec3& up) override;

    void hide(const bool hide) override;

    int bullet_handle() { return robot_data_.bullet_handle_; }

    std::weak_ptr<World> bullet_world_;
    RobotData robot_data_;

protected:
    void do_recycle(const std::string& key);
};

class Robot : public xrobot::RobotBase {
public:
	Robot(std::weak_ptr<World> bullet_world);
	~Robot();

    void CalculateInverseKinematics(
            const int end_index, 
            const btVector3 target_position,
            const btQuaternion target_orientation,
            double* joint_damping,
            double* ik_output_joint_pos,
            int &num_poses);
};

class RobotWithConvertion : public xrobot::RobotBase {
public:
    RobotWithConvertion(std::weak_ptr<World> bullet_world);
    ~RobotWithConvertion();

    void LoadConvertedObject(
        const std::string& filename,
        const btVector3 position,
        const btQuaternion rotation,
        const float scale = 1.0f,
        const std::string& label = "",
        const bool concave = false
    );

    bool TakeAction(const int act_id);
    void SetCycle(const bool cycle) { cycle_ = cycle; }
    void SetStatus(const int status) { status_ = status; }
    void recycle() override;

    std::vector<std::string> GetActions() const { return object_name_list_; }

    float scale_;
    std::string label_;
    std::string path_;
    std::vector<std::string> object_path_list_;
    std::vector<std::string> object_name_list_;

private:
    void Remove();

    int status_;
    bool cycle_;
};

class RobotWithAnimation : public xrobot::RobotBase {
public:
    RobotWithAnimation(std::weak_ptr<World> bullet_world);
    ~RobotWithAnimation();

    void LoadAnimatedObject(
        const std::string& filename,
        const btVector3 position,
        const btQuaternion rotation,
        const float scale = 1.0f,
        const std::string& label = "",
        const bool concave = false
    );

    bool InteractWith(const std::string& tag);
    bool TakeAction(const int act_id);
    void SetStatus(const int status) { status_ = status; }
    void SetJoint(const int joint) { joint_ = joint; }
    void SetLock(const bool lock) { lock_ = lock; }
    bool GetLock() const { return lock_; }
    int GetJoint() const { return joint_; }
    float GetPosition(const int id) { return positions_[id]; }
    void recycle() override;

    std::vector<std::string> GetActions() const { return object_name_list_; }

    std::string unlock_tag_;
    std::string path_;
    std::vector<std::string> object_name_list_;

private:
    bool lock_;
    int status_;
    int joint_;
    std::map<int, float> positions_;
};

struct ContactPoint {
    glm::vec3 contact_normal;
    glm::vec3 contact_position_a;
    glm::vec3 contact_position_b;
    float contact_force;
    float contact_distance;
    int bullet_id_a;
    int bullet_id_b;
};

struct Ray {
    glm::vec3 from;
    glm::vec3 to;
};

struct RayTestInfo
{
    int bullet_id;
    glm::vec3 pos;
    glm::vec3 norm;
};

struct ObjectAttributes {
    glm::vec3 aabb_min;
    glm::vec3 aabb_max;
    int bullet_id;
};

struct ObjectDirections {
    float dirs[9];
    int bullet_id;
};

class World : public render_engine::RenderWorld,
              public std::enable_shared_from_this<World> {
    using RenderBody = render_engine::RenderBody;
public:
    World();
    ~World();

    b3PhysicsClientHandle client_;
    
    std::map<std::string, std::string> tag_list_;
    std::map<std::string, bool> pickable_list_;

    std::vector<std::shared_ptr<RobotBase>> robot_list_;
    std::map<int, std::shared_ptr<RobotBase>> bullet_handle_to_robot_map_;
    std::map<std::string, std::vector<std::shared_ptr<RobotBase>>> recycle_robot_map_;
    std::map<std::string, render_engine::ModelDataPtr> model_cache_;
    std::map<std::string, std::vector<int>> object_locations_;

    float bullet_gravity_;
    float bullet_timestep_;
    float bullet_timestep_sent_;
    float bullet_skip_frames_sent_;
    double bullet_ts_;
    int reset_count_;

    void LoadMetadata(const char * filename);
    void AssignTag(const std::string& path, const std::string& tag);
    void UpdatePickableList(const std::string& tag, const bool pick);

    std::weak_ptr<RobotBase> LoadRobot(
        const std::string& filename,
        const btVector3 position,
        const btQuaternion rotation,
        const btVector3 scale = btVector3(1,1,1),
        const std::string& label = "unlabeled",
        const bool fixed_base = false,
        const float mass = 0,
        const bool flip = false,
        const bool concave = false
    );

   std::shared_ptr<RobotBase> LoadModelFromCache(
        const std::string& filename,
        const btVector3 position,
        const btQuaternion rotation
    );

    render_engine::ModelDataPtr FindInCache(
            const std::string &key,
            std::vector<render_engine::ModelDataPtr> &model_list,
            bool& reset);

    void ClearCache();
    void PrintCacheInfo();
    void BulletInit(const float gravity = 9.81f, const float timestep = 1.0/100.0);
    void BulletStep(const int skip_frames = 1);
    void QueryPositions();
    void QueryPosition(std::shared_ptr<RobotBase> robot);
    void RemoveRobot(std::weak_ptr<RobotBase> rm_robot);
    void CleanEverything();
    void CleanEverything2();
    void ResetSimulation();
    int RayTest(const glm::vec3 ray_from_position, const glm::vec3 ray_to_position);
    void BatchRayTest(const std::vector<Ray> rays, std::vector<RayTestInfo>& result,
        const int num_threads = 0);
    void SetTransformation(std::weak_ptr<RobotBase> robot_tr, const btTransform& tranform);
    void SetVelocity(std::weak_ptr<RobotBase> robot_vel, const btVector3& velocity);

    void GetRootClosestPoints(std::weak_ptr<RobotBase> robot_in, std::weak_ptr<Object> part_in,
        std::vector<ContactPoint>& contact_points);
    void GetRootContactPoints(std::weak_ptr<RobotBase> robot_in, std::weak_ptr<Object> part_in,
        std::vector<ContactPoint>& contact_points);

    int CreateFixedRootToTargetConstraint(std::weak_ptr<RobotBase> parent_robot,
        const btVector3& parent_relative_position,
        const btVector3& child_relative_position,
        const btQuaternion& parent_relative_orientation = btQuaternion(),
        const btQuaternion& child_relative_orientation = btQuaternion()
    );

    void ChangeFixedRootToTargetConstraint(const int constraint_id,
        const btVector3& child_relative_position,
        const btQuaternion& child_relative_orientation,
        const float max_force = 500.0f
    );

    void CharacterMove(std::weak_ptr<RobotBase> robot_move, const glm::vec3 walk_move,
        const glm::vec3 walk_rotate, const float speed);

    void AddObjectWithLabel(const std::string& label,
        const int id);
    void RemoveObjectWithLabel(const int id);
    void QueryObjectDirectionByLabel(const std::string& label, const glm::vec3 front,
        const glm::vec3 eye, std::vector<ObjectDirections>& result);
    void QueryObjectByLabel(const std::string& label,
        std::vector<ObjectAttributes>& result);
    
public:
    size_t size() const override { return robot_list_.size(); }

    const RenderBody* render_body_ptr(const size_t i) const override;

    RenderBody* render_body_ptr(const size_t i) override;
};

}

#endif // WORLD_H_
