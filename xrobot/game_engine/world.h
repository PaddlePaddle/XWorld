#ifndef WORLD_H_
#define WORLD_H_

#include <map>
#include <string>
#include <functional>
#include <vector>
#include <unistd.h>
#include <errno.h>
#include <utility>
#include <chrono>

#include "glm/glm.hpp"

#include "bullet_engine/bullet_joint.h"
#include "bullet_engine/bullet_object.h"
#include "bullet_engine/bullet_body.h"
#include "bullet_engine/bullet_world.h"
#include "render_engine/model.h"
#include "render_engine/render_world.h"

#include "inventory.h"

namespace xrobot {

// TODO: Right now xScalar cannot be float
#ifdef USE_DOUBLE_PRECISION
typedef double xScalar;
#else
typedef float xScalar;
#endif

class Inventory;
class RobotBase;
class World;

typedef std::shared_ptr<RobotBase> RobotBaseSPtr;
typedef std::weak_ptr<RobotBase> RobotBaseWPtr;
typedef std::shared_ptr<World> WorldSPtr;
typedef std::weak_ptr<World> WorldWPtr;

class Joint : public bullet_engine::BulletJoint {
public:
    Joint();
    
    ~Joint();

    void EnableJointSensor(const bool enable);
    
    void GetJointMotorState(glm::vec3& force, glm::vec3& torque);

    void ResetJointState(const xScalar pos, const xScalar vel);

    void SetJointMotorControlTorque(const xScalar torque);

    void SetJointMotorControlVelocity(
            const xScalar speed, const xScalar k_d, const xScalar max_force);

    void SetJointMotorControlPosition(
            const xScalar target,
            const xScalar k_p,
            const xScalar k_d,
            const xScalar max_force);

    WorldWPtr bullet_world_;    
    RobotBaseWPtr bullet_robot_;
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

    void GetMass(xScalar& mass);

    void SetMass(const xScalar mass);

    void SetStatic();
    
    void RecoverFromStatic();

    void ChangeLinearDamping(const xScalar damping);

    void ChangeAngularDamping(const xScalar damping);

    void ChangeLateralFriction(const xScalar friction);

    void ChangeSpinningFriction(const xScalar friction);

    void ChangeRollingFriction(const xScalar friction);

    void ApplyForce(const xScalar x,
                    const xScalar y,
                    const xScalar z,
                    const int flags = EF_LINK_FRAME);

    void ApplyTorque(const xScalar x,
                     const xScalar y,
                     const xScalar z, 
                     const int flags = EF_LINK_FRAME);
public:
    WorldWPtr bullet_world_;

    int body_uid_; // id of Robot this Object belongs to

    std::string object_name_;

    RobotBaseWPtr attach_object_;

private:
    xScalar object_mass_original_;

// xrobot::render_engine::RenderPart
public:
    int id() const override { return body_uid_; }

    void set_id(int id) override { body_uid_ = id; }

    void GetAABB(glm::vec3& aabb_min, glm::vec3& aabb_max) override;

    glm::mat4 translation_matrix() const override;

    glm::mat4 local_inertial_frame() const override;
};

class RobotBase : public render_engine::RenderBody,
                  public bullet_engine::BulletBody,
                  public std::enable_shared_from_this<RobotBase> {
    using RenderPart = render_engine::RenderPart;
public:
    RobotBase(WorldWPtr bullet_world);

    virtual ~RobotBase() {}

    void UpdatePickable(const bool pick) { body_data_.pickable = pick; }

    void UpdateTag(const std::string& tag) { body_data_.label = tag; }

    virtual bool InteractWith(const std::string& tag) { return false; }
    
    virtual bool TakeAction(const int act_id) { return false; }

    virtual std::vector<std::string> GetActions() const {
        return std::vector<std::string>(0);
    }

    void LoadURDFFile(
            const std::string& filename,
            const glm::vec3& pos,
            const glm::vec4& quat,
            const xScalar scale = 1.0f,
            const std::string& label = "unlabeled",
            const bool fixed_base = false,
            const bool self_collision = false,
            const bool use_multibody = true,
            const bool concave = false);

    void LoadOBJFile(
            const std::string& filename,
            const glm::vec3& pos,
            const glm::vec4& quat,
            const glm::vec3& scale,
            const std::string& label = "unlabeled",
            const xScalar mass = 0,
            const bool flip = false,
            const bool concave = false);

    void RemoveRobotFromBullet();

    virtual void recycle() override;

    void Sleep();

    void Wake();

    void DisableSleeping();

    void reuse() override {
        recycle_ = false;
        hide_ = false;
    }

    virtual void Move(const xScalar translate, const xScalar rotate);

    virtual void UnFreeze();

    virtual void Freeze();

    virtual void MoveForward(const xScalar speed);

    virtual void MoveBackward(const xScalar speed);

    virtual void TurnLeft(const xScalar speed);

    virtual void TurnRight(const xScalar speed);

    void SetJointVelocity(
            const int joint_id,
            const xScalar speed,
            const xScalar k_d,
            const xScalar max_force);

    void SetJointPosition(
            const int joint_id,
            const xScalar target,
            const xScalar k_p,
            const xScalar k_d,
            const xScalar max_force);

    void ResetJointState(
            const int joint_id,
            const xScalar pos,
            const xScalar vel);

    virtual std::string PickUp(
            std::shared_ptr<Inventory>& inventory,
            const glm::vec3& from,
            const glm::vec3& to);

    virtual std::string PutDown(
            std::shared_ptr<Inventory>& inventory, 
            const glm::vec3& from,
            const glm::vec3& to);

    virtual std::string Rotate(
            const glm::vec3& rotate_angle,
            const glm::vec3& from,
            const glm::vec3& to);

    virtual void AttachTo(RobotBaseWPtr object);

    virtual void Detach();

    // RenderBody
    virtual const RenderPart* render_root_ptr() const override;

    virtual RenderPart* render_root_ptr() override;

    virtual const RenderPart* render_part_ptr(const size_t i) const override;

    virtual RenderPart* render_part_ptr(const size_t i) override;

    size_t size() const override { return parts_.size(); }

    void attach_camera(const glm::vec3& offset,
                       const float pitch,
                       glm::vec3& loc,
                       glm::vec3& front,
                       glm::vec3& right,
                       glm::vec3& up) override;

    void hide(const bool hide) override;

    int id() { return body_data_.body_uid; }

    WorldWPtr bullet_world_;
    std::shared_ptr<Object> root_part_;
    std::vector<std::shared_ptr<Object>> parts_;
    std::vector<std::shared_ptr<Joint>> joints_;

protected:
    void load_robot_joints(const std::string& filename);

    void load_robot_shapes(const xScalar scale);

    void do_recycle(const std::string& key);

    bool occupy_test(
            WorldSPtr& world,
            RobotBaseSPtr& item,
            const glm::vec3& c,
            xScalar& low);
};

class Robot : public RobotBase {
public:
	Robot(WorldWPtr bullet_world) : RobotBase(bullet_world) {}

	~Robot() {}

    void CalculateInverseKinematics(
            const int end_index, 
            const glm::vec3& target_position,
            const glm::vec4& target_orientation,
            xScalar* joint_damping,
            xScalar* ik_output_joint_pos,
            int &num_poses);
};

class RobotWithConvertion : public xrobot::RobotBase {
public:
    RobotWithConvertion(WorldWPtr bullet_world);

    ~RobotWithConvertion() {}

    void LoadConvertedObject(
            const std::string& filename,
            const glm::vec3& position,
            const glm::vec4& rotation,
            const xScalar scale = 1.0f,
            const std::string& label = "",
            const bool concave = false);

    bool InteractWith(const std::string& tag);

    bool TakeAction(const int act_id);

    void SetCycle(const bool cycle) { cycle_ = cycle; }

    bool GetCycle() const { return cycle_; }

    void SetStatus(const int status) { status_ = status; }

    void SetLock(const bool lock) { lock_ = lock; }

    bool GetLock() const { return lock_; }

    std::string GetUnlockedTage() const { return unlock_tag_; }

    void recycle() override;

    std::vector<std::string> GetActions() const { return object_name_list_; }
 
private:
    int status_;
    std::string label_;
    std::string unlock_tag_;
    xScalar scale_;
    bool lock_;
    bool cycle_;
    std::string path_;
    std::vector<std::string> object_path_list_;
    std::vector<std::string> object_name_list_;
};

class RobotWithAnimation : public xrobot::RobotBase {
public:
    RobotWithAnimation(WorldWPtr bullet_world);
    ~RobotWithAnimation();

    void LoadAnimatedObject(
        const std::string& filename,
        const glm::vec3& pos,
        const glm::vec4& quat,
        const xScalar scale = 1.0f,
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
    xScalar GetPosition(const int id) { return positions_[id]; }
    void recycle() override;

    std::vector<std::string> GetActions() const { return object_name_list_; }

    std::string unlock_tag_;
    std::string path_;
    std::vector<std::string> object_name_list_;

private:
    bool lock_;
    int status_;
    int joint_;
    std::map<int, xScalar> positions_;
};

struct ObjectAttributes {
    glm::vec3 aabb_min;
    glm::vec3 aabb_max;
    int bullet_id;
};

class World : public render_engine::RenderWorld,
              public bullet_engine::BulletWorld,
              public std::enable_shared_from_this<World> {
    using RenderBody = render_engine::RenderBody;
public:
    World();
    ~World();
    
    std::map<std::string, std::string> tag_list_;
    std::map<std::string, bool> pickable_list_;
    std::map<int, RobotBaseSPtr> id_to_robot_;
    std::map<std::string, std::vector<RobotBaseSPtr>> recycle_robot_map_;
    std::map<std::string, render_engine::ModelDataSPtr> model_cache_;
    std::map<std::string, std::vector<int>> object_locations_;

    int reset_count_;

    void LoadMetadata(const char * filename);
    void AssignTag(const std::string& path, const std::string& tag);
    void UpdatePickableList(const std::string& tag, const bool pick);

    RobotBaseWPtr LoadRobot(
            const std::string& filename,
            const glm::vec3& pos,
            const glm::vec3& rot_axis,
            const xScalar rot_angle,
            const glm::vec3& scale = glm::vec3(1,1,1),
            const std::string& label = "unlabeled",
            const bool fixed_base = false,
            const xScalar mass = 0,
            const bool flip = false,
            const bool concave = false);

    RobotBaseWPtr LoadRobot(
            const std::string& filename,
            const glm::vec3& position,
            const glm::vec4& rotation,
            const glm::vec3& scale = glm::vec3(1,1,1),
            const std::string& label = "unlabeled",
            const bool fixed_base = false,
            const xScalar mass = 0,
            const bool flip = false,
            const bool concave = false);

    RobotBaseSPtr LoadModelFromCache(
            const std::string& filename,
            const glm::vec3& position,
            const glm::vec4& rotation);

    render_engine::ModelDataSPtr FindInCache(
            const std::string &key,
            std::vector<render_engine::ModelDataSPtr> &model_list,
            bool& reset);

    void UpdateAttachObjects(RobotBaseSPtr robot);
    void FixLockedObjects(RobotBaseSPtr robot);
    void QueryPose(RobotBaseSPtr robot);

    void QueryMovable();
    void QueryInteractable(RobotBaseSPtr robot);

    void ClearCache();
    void PrintCacheInfo();
    void BulletInit(const float gravity = 9.81f, const float timestep = 1.0/100.0);
    void BulletStep(const int skip_frames = 1);
    void RemoveRobot(RobotBaseWPtr rm_robot);
    void CleanEverything();
    void CleanEverything2();
    void ResetSimulation();

    void RayTest(const glm::vec3 from, const glm::vec3 to, RayTestInfo& result);
    void BatchRayTest(const std::vector<Ray> rays, 
                      std::vector<RayTestInfo>& result,
                      const int num_threads = 0);

    void SetTransformation(RobotBaseWPtr robot, const btTransform& tr);

    void GetRootClosestPoints(
            RobotBaseWPtr robot_in, 
            std::weak_ptr<Object> part_in,
            std::vector<ContactPoint>& contact_points);
    void GetContactPoints(
            RobotBaseWPtr robot_in, 
            std::weak_ptr<Object> part_in,
            std::vector<ContactPoint>& contact_points,
            const int link = -2);

    void AddObjectWithLabel(const std::string& label, const int id);
    void RemoveObjectWithLabel(const int id);

    void QueryObjectByLabel(const std::string& label,
            std::vector<ObjectAttributes>& result);
    
public:
    size_t size() const override { return id_to_robot_.size(); }

    void robot_iteration_begin() override;

    RenderBody* next_robot() override;
    
    bool has_next_robot() const override;

private:
    decltype(id_to_robot_)::iterator robot_it_;
};

}

#endif // WORLD_H_
