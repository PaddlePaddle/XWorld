#include "world.h"
#include "utils.h"

namespace xrobot {

using namespace render_engine;
using namespace bullet_engine;

World::World() : BulletWorld(),
                 id_to_robot_(),
                 recycle_robot_map_(),
                 model_cache_(),
                 object_locations_(),
                 reset_count_(0),
                 pickable_list_(),
                 tag_list_() {}

World::~World() {
    CleanEverything();
    ClearCache();
}

void World::UpdatePickableList(const std::string& tag, const bool pick) {
    pickable_list_[tag] = pick;
}

void World::AssignTag(const std::string& path, const std::string& tag) {
    tag_list_[path] = tag;
}

void World::LoadMetadata(const char* filename) {
    errno = 0;
    FILE *fp = fopen(filename, "r");
    if (!fp) {
        fprintf(stderr, "[%d] Unable to open metadata file %s\n", errno, filename);
        return;
    }

    int line_number = 1;
    char key_buffer[4096];
    std::vector<char *> keys;
    if (fgets(key_buffer, 4096, fp)) {
        char *token = strtok(key_buffer, ",\n");
        while (token) {
            keys.push_back(token);
            token = strtok(NULL, ",\n");
        }
    }

    // Check index
    int model_id_k = -1;
    for (int i = 0; i < keys.size(); i++) {
        if (!strcmp(keys[i], "index")) {
            model_id_k = i;
            break;
        }
    }

    if (model_id_k < 0) {
        fprintf(stderr, "Did not find \"index\" in header on line %d of %s\n",
                line_number, filename);
        return;
    }

    // Check path
    int path_k = -1;
    for (int i = 0; i < keys.size(); i++) {
        if (!strcmp(keys[i], "path")) {
            path_k = i;
            break;
        }
    }

    if (path_k < 0) {
        fprintf(stderr, "Did not find \"path\" in header on line %d of %s\n",
                line_number, filename);
        return;
    }

    // Check tag
    int tag_k = -1;
    for (int i = 0; i < keys.size(); i++) {
        if (!strcmp(keys[i], "tag")) {
            tag_k = i;
            break;
        }
    }

    if (tag_k < 0) {
        fprintf(stderr, "Did not find \"tag\" in header on line %d of %s\n",
                line_number, filename);
        return;
    }

    // Check pickable
    int pickable_k = -1;
    for (int i = 0; i < keys.size(); i++) {
        if (!strcmp(keys[i], "pickable")) {
            pickable_k = i;
            break;
        }
    }

    if (pickable_k < 0) {
        fprintf(stderr, "Did not find \"pickable\" in header on line %d of %s\n",
                line_number, filename);
        return;
    }

    char value_buffer[4096];
        while (fgets(value_buffer, 4096, fp)) {
        line_number++;

        std::vector<char *> values;
        char *token = strtok(value_buffer, ",\n");
        while (token) {
            values.push_back(token);
            token = strtok(NULL, ",\n");
        }

        if (values.size() == 0) continue;
        if (values.size() != keys.size()) {
            fprintf(stderr, "Invalid number of entries at line %d in %s\n",
                    line_number, filename);
            return;
        }

        const char *model_id = values[model_id_k];
        if (!model_id) continue;
        int model_id_length = strlen(model_id);
        if (model_id_length == 0) continue;

        const char *path = values[path_k];
        if (!path) continue;
        int path_length = strlen(path);
        if (path_length == 0) continue;

        const char *tag = values[tag_k];
        if (!tag) continue;
        int tag_length = strlen(tag);
        if (tag_length == 0) continue;

        const char *pickable = values[pickable_k];
        if (!pickable) continue;
        int pickable_length = strlen(pickable);
        if (pickable_length == 0) continue;

        std::string rel_path(path);
        // std::string dir_str(filename);
        // size_t p = dir_str.find_last_of("/");
        // std::string abs_path(dir_str.substr(0, p) + std::string(path));

        bool pickable_bool = !strcmp(pickable, "0");

        pickable_list_[std::string(tag)] = pickable_bool;
        tag_list_[rel_path] = std::string(tag);        
    }

    fclose(fp);
}
std::weak_ptr<RobotBase> World::LoadRobot(
        const std::string& filename,
        const glm::vec3& pos,
        const glm::vec3& rot_axis,
        const xScalar rot_angle,
        const glm::vec3& scale,
        const std::string& label,
        const bool fixed_base,
        const xScalar mass,
        const bool flip,
        const bool concave) {

    double d = glm::length(rot_axis);
    double s = sin(rot_angle*0.5) / d;
    double c = cos(rot_angle*0.5);
    glm::vec4 quat(rot_axis[0]*s, rot_axis[1]*s, rot_axis[2]*s, c);

    return LoadRobot(filename, pos, quat, scale,
            label, fixed_base, mass, flip, concave);
}


std::weak_ptr<RobotBase> World::LoadRobot(
        const std::string& filename,
        const glm::vec3& pos,
        const glm::vec4& quat, // quat
        const glm::vec3& scale,
        const std::string& label,
        const bool fixed_base,
        const xScalar mass,
        const bool flip,
        const bool concave) {
    assert(filename.size());

    // If label is valid overide
    // Otherwise, load meta
    std::string tag(label);
    bool pickable = false;

    if(!label.length() || !label.compare("unlabeled")) {
        if(tag_list_.find(filename) != tag_list_.end()) {
            tag = tag_list_[filename];
        }
    }

    if(pickable_list_.find(tag) != pickable_list_.end()) {
        pickable = pickable_list_[tag];
    }

    int find = (int) filename.find(".obj");
    if(find > -1) {
        // Load OBJ
        std::string filename_with_scale =
                filename + ":" + 
                std::to_string(scale[0]) + ":" + 
                std::to_string(scale[1]) + ":" + 
                std::to_string(scale[2]);
        auto robot = LoadModelFromCache(filename_with_scale, pos, quat);
        if (!robot) {
            robot = std::make_shared<Robot>(shared_from_this());
            robot->LoadOBJFile(filename, pos, quat, scale,
                    tag, mass, flip, concave);
        }

        if (mass > 0) {
            robot->root_part_->SetMass(mass);
            robot->Wake();
        }

        load_icon(filename);

        robot->UpdatePickable(pickable);
        robot->reuse();
        AddObjectWithLabel(tag, robot->body_data_.body_uid);

        return robot;
    
    } 

    find = (int) filename.find(".urdf");
    if(find > -1) {
        // Load URDF
        auto robot = LoadModelFromCache(filename, pos, quat);
        if (robot) {
            robot->root_part_->RecoverFromStatic();
            robot->root_part_->Wake();
            for (auto joint : robot->joints_) {
                if(joint) {
                    joint->ResetJointState(0.0f, 0.005f);
                }
            }
            for (auto part : robot->parts_) {
                part->RecoverFromStatic();
            }
        } else {
            robot = std::make_shared<Robot>(shared_from_this());
            robot->LoadURDFFile(
                    filename, pos, quat, scale[0], tag, fixed_base, concave);
            robot->reuse();
            robot->Wake();
        }

        load_icon(filename);

        robot->UpdatePickable(pickable);
        robot->reuse();
        AddObjectWithLabel(tag, robot->body_data_.body_uid);

        return robot;
    }

    find = (int) filename.find(".json");
    if (find > -1) {
        auto robot = LoadModelFromCache(filename, pos, quat);

        if (!robot) {
            Json::Value json_root;
            if (!json_parse_text(filename, json_root)) {
                fprintf(stderr, "Unable to parse %s\n", filename.c_str());
                return std::weak_ptr<RobotBase>();
            }
            // Check Type
            std::string type = json_get_string(&json_root, "type", "NoType"); 
            if (type == "convert") {
                robot = std::make_shared<RobotWithConvertion>(
                        shared_from_this());
                auto robot_conv =
                        std::dynamic_pointer_cast<RobotWithConvertion>(robot);
                robot_conv->LoadConvertedObject(
                        filename, pos, quat, scale[0], tag, concave, fixed_base);

                load_icon(robot_conv->path());

                robot_conv->reuse();
                robot_conv->Wake();
                robot_conv->UpdatePickable(pickable); // update pickable
                AddObjectWithLabel(tag, robot_conv->body_data_.body_uid);
                return robot_conv;
            } else if (type == "animate") {
                robot = std::make_shared<RobotWithAnimation>(shared_from_this());
                auto robot_anim =
                        std::dynamic_pointer_cast<RobotWithAnimation>(robot);
                robot_anim->LoadAnimatedObject(
                        filename, pos, quat, scale[0], tag, concave, fixed_base);

                load_icon(robot_anim->path());

                robot_anim->reuse();
                robot_anim->Wake();
                robot_anim->UpdatePickable(pickable); // update pickable
                AddObjectWithLabel(tag, robot_anim->body_data_.body_uid);
                return robot_anim;
            } else {
                fprintf(stderr, "Incorrect Action File [%s]\n", type.c_str());
                return std::weak_ptr<RobotBase>();
            }
        } else {
            robot->root_part_->RecoverFromStatic();
            robot->root_part_->Wake();

            for (auto joint : robot->joints_) {
                if(joint) {
                    joint->ResetJointState(0.0f, 0.005f);
                }
            }
            for (auto part : robot->parts_) {
                part->RecoverFromStatic();
                part->Wake();
            }

            auto robot_anim =
                    std::dynamic_pointer_cast<RobotWithAnimation>(robot);
            auto robot_conv = 
                    std::dynamic_pointer_cast<RobotWithConvertion>(robot);

            if (robot_anim) {
                if(robot_anim->unlock_tag_.size()) {
                    robot_anim->SetLock(false);
                    robot_anim->TakeAction(1);
                    robot_anim->SetLock(true);
                } else {
                    robot_anim->TakeAction(1);
                }

                load_icon(robot_anim->path());

                BulletStep();
            } else if (robot_conv) {
                bool cycle = robot_conv->GetCycle();
                bool lock  = robot_conv->GetUnlockedTage().size();

                if(!cycle)
                    robot_conv->SetCycle(true);

                if(lock)
                    robot_conv->SetLock(false);

                robot_conv->TakeAction(0);

                if(!cycle)
                    robot_conv->SetCycle(false);

                if(lock)
                    robot_conv->SetLock(true);

                load_icon(robot_conv->path());

                BulletStep();
            }

            robot->reuse();
            robot->UpdatePickable(pickable);
            AddObjectWithLabel(tag, robot->body_data_.body_uid);

            return robot;
        }
    }

    // printf("file: %s [%d]\n", filename.c_str(), find);
    // printf("Format Not Support!\n");
    return std::weak_ptr<RobotBase>();
}


void World::ResetSimulation() {
    reset();
}

void World::RemoveRobot(const RobotBaseWPtr& rm_robot) {
    
    auto robot = wptr_to_sptr(rm_robot);
    int uid = robot->body_data_.body_uid;
    if (id_to_robot_.find(uid) != id_to_robot_.end()) {
        robot->root_part_->Sleep();
        for (auto& part : robot->parts_) {
            if (part) {
                part->Sleep();
            }
        }
    }
    robot->body_data_.attach_to_id = -2;
    RemoveObjectWithLabel(robot->body_data_.body_uid);
    id_to_robot_.erase(robot->body_data_.body_uid);
}

void World::CleanEverything2() {

    for (auto& kv : id_to_robot_) {
        kv.second->reset_move();
        if(!kv.second->is_recycled())
            kv.second->recycle();
    }

    for(auto &recycle_robot : recycle_robot_map_) {
        if(recycle_robot.second.size() >= kCacheSize) {
            while(recycle_robot.second.size() > kCacheSize - 4) {
                auto robot = recycle_robot.second.back();
                robot->RemoveRobotFromBullet();
                RemoveRobot(robot);
                recycle_robot.second.pop_back();
            }
        }
    }

    reset_count_++;

    remove_all_cameras();

    tag_list_.clear();
    pickable_list_.clear();
    icon_inventory_.clear();
}

void World::PrintCacheInfo() {
    for(auto& recycle_robot : recycle_robot_map_) {
        printf("urdf: %s   ", recycle_robot.first.c_str());
        printf("size: %d\n", (int) recycle_robot.second.size());
    }
}

void World::CleanEverything() {
    reset_count_++;

    remove_all_cameras();

    tag_list_.clear();
    pickable_list_.clear();
    icon_inventory_.clear();

    recycle_robot_map_.clear();
    id_to_robot_.clear();
    object_locations_.clear();
    ResetSimulation();
}

void World::UpdateAttachObjects(const RobotBaseSPtr& robot) {
    if(robot->body_data_.attach_to_id < -1) 
        return;

    if(cameras_size() < 1 || !camera(0)) 
        return;

    if (robot->body_data_.attach_to_id == -1) {
        if(auto attach_object_sptr = robot->root_part_->attach_object_.lock()) {

            bool contact = false;
            std::vector<ContactPoint> contact_points;
            GetContactPoints(
                    attach_object_sptr, 
                    attach_object_sptr->root_part_,
                    contact_points);

            for (int i = 0; i < contact_points.size(); ++i) {
                ContactPoint cp = contact_points[i];
                if(cp.contact_distance < -0.008f) {
                    robot->Detach();
                    contact = true;
                    break;
                }
            }
            // TODO
            // camera 
            if(!contact) {
                float pitch = glm::radians(camera(0)->pre_pitch_);
                glm::vec3 offset = camera(0)->offset_;

                btTransform mat_rc_r;
                btQuaternion cam_q(pitch, 0, 0);
                mat_rc_r.setIdentity();
                mat_rc_r.setRotation(cam_q);

                btTransform mat_rc_t;
                mat_rc_t.setIdentity();
                mat_rc_t.setOrigin(btVector3(0,offset.y,0));

                btTransform new_transform = 
                        robot->root_part_->object_position_;
                btTransform attach_transform = 
                        robot->body_data_.attach_transform;
                btTransform object_orn = 
                        robot->body_data_.attach_orientation;
                btTransform obj_tr = 
                        mat_rc_t * new_transform * mat_rc_r * attach_transform;

                obj_tr.setRotation(object_orn.getRotation());
                SetTransformation(attach_object_sptr, obj_tr);
            }
        } 
    }
}

void World::FixLockedObjects(const RobotBaseSPtr& robot) {
    if(auto anim_robot = std::dynamic_pointer_cast<RobotWithAnimation>(robot)) {
        if(anim_robot->GetLock()) {
            int joint = anim_robot->GetJoint();
            float position = anim_robot->GetPosition(1);
            anim_robot->SetJointPosition(joint, position, 1.0f, 0.1f, 1000000.0f);
        }
    }
}

void World::QueryPose(RobotBaseSPtr& robot) {
    const xScalar* root_inertial_frame;
    const xScalar* q;
    const xScalar* q_dot;

    robot->query_pose(client_, &root_inertial_frame, &q, &q_dot);

    robot->root_part_->object_position_ = TransformFromReals(q, q+3);
    robot->root_part_->object_speed_[0] = q_dot[0];
    robot->root_part_->object_speed_[1] = q_dot[1];
    robot->root_part_->object_speed_[2] = q_dot[2];
    robot->root_part_->object_angular_speed_[0] = q_dot[3];
    robot->root_part_->object_angular_speed_[1] = q_dot[4];
    robot->root_part_->object_angular_speed_[2] = q_dot[5];
    robot->root_part_->object_local_inertial_frame_ = 
        TransformFromReals(root_inertial_frame, root_inertial_frame+3);
    robot->root_part_->object_link_position_ = TransformFromReals(q, q+3);

    for (auto& part : robot->parts_)
    {
        struct b3LinkState link_state;
        if(!part) continue;
        if(part->bullet_link_id_ == -1) continue;

        robot->query_link(client_, part->bullet_link_id_, link_state);

        part->object_position_ = TransformFromReals(link_state.m_worldPosition,
                                                    link_state.m_worldOrientation);
        part->object_local_inertial_frame_ =
                TransformFromReals(link_state.m_localInertialPosition,
                                     link_state.m_localInertialOrientation);
        part->object_link_position_ =
                TransformFromReals(link_state.m_worldLinkFramePosition,
                                     link_state.m_worldLinkFrameOrientation);
        part->object_speed_[0] = link_state.m_worldLinearVelocity[0];
        part->object_speed_[1] = link_state.m_worldLinearVelocity[1];
        part->object_speed_[2] = link_state.m_worldLinearVelocity[2];
        part->object_angular_speed_[0] = link_state.m_worldAngularVelocity[0];
        part->object_angular_speed_[1] = link_state.m_worldAngularVelocity[1];
        part->object_angular_speed_[2] = link_state.m_worldAngularVelocity[2];
    }

    for (auto& joint : robot->joints_)
    {
        if(!joint) continue;
        joint->joint_current_position_ = q[joint->bullet_q_index_];
        joint->joint_current_speed_ = q_dot[joint->bullet_u_index_];
    }
}

void World::BulletStep(const int skip_frames)
{
    if(cameras_size() < 1 || !camera(0)) 
        return;

    // Check Object at Camera Center is Movable
    if(get_highlight_center() > -1) 
        QueryMovable();

    for (auto& kv : id_to_robot_) {
        auto robot = kv.second;
        if (!robot || robot->is_recycled() || robot->is_hiding()) 
            continue;

        // Check Object at Camera Center is Interactable
        if(get_highlight_center() == 0)
            QueryInteractable(robot);

        UpdateAttachObjects(robot);
        FixLockedObjects(robot);

        for (auto& joint : robot->joints_) {
            if(joint) {
                robot->update_joints(client_);
                break;
            }
        }
    }

    step();

    for (auto& kv : id_to_robot_) {
        if (!kv.second) continue;
        QueryPose(kv.second);
    }

    render_step();
}

void World::QueryMovable() {
    // TODO
    // Camera
    glm::vec3 from_3d = camera(0)->position_;
    glm::vec3 frnt_3d = camera(0)->front_;

    std::vector<RayTestInfo> raytest_result;
    std::vector<Ray> ray;
    ray.push_back({from_3d, from_3d + kDistanceThreshold * frnt_3d});
    BatchRayTest(ray, raytest_result);

    if(raytest_result[0].bullet_id >= 0) {
        auto object = id_to_robot_[raytest_result[0].bullet_id];

        if(object->body_data_.pickable) {
            set_highlight_center(2);
            return;
        }
    }

    set_highlight_center(0);
}

void World::QueryInteractable(const std::shared_ptr<RobotBase>& robot) {
    // TODO
    // Camera
    glm::vec3 from_3d = camera(0)->position_;
    glm::vec3 frnt_3d = camera(0)->front_;

    if (!robot->is_recycled() && !robot->is_hiding()) {

        if(std::dynamic_pointer_cast<RobotWithAnimation>(robot) || 
           std::dynamic_pointer_cast<RobotWithConvertion>(robot)) {

            glm::vec3 aabb_min, aabb_max;
            robot->root_part_->GetAABB(aabb_min, aabb_max);

            bool intersect = RayAABBIntersect({from_3d, from_3d + frnt_3d},
                                              aabb_min,
                                              aabb_max);

            glm::vec3 object_pos = (aabb_max - aabb_min) * 0.5f + aabb_min;
            float dist = glm::distance(object_pos, from_3d);

            if(intersect && dist < kDistanceThreshold) {
                set_highlight_center(1);
                return;
            }
        }
    }

    set_highlight_center(0);
}

void World::BulletInit(const float gravity, const float timestep) {
    init(gravity, timestep);    
}

void World::SetTransformation(const RobotBaseWPtr& robot, const btTransform& tr) {

    if (auto robot_sptr = robot.lock()) {
        xScalar p[3];
        p[0] = tr.getOrigin()[0];
        p[1] = tr.getOrigin()[1];
        p[2] = tr.getOrigin()[2];

        xScalar q[4];
        q[0] = tr.getRotation()[0];
        q[1] = tr.getRotation()[1];
        q[2] = tr.getRotation()[2];
        q[3] = tr.getRotation()[3];

        set_pose(client_, robot_sptr->id(), p, q);
    }
}

void World::GetRootClosestPoints(
        const RobotBaseWPtr& robot_in,
        const std::weak_ptr<Object>& part_in,
        std::vector<ContactPoint>& contact_points) {
    if (auto robot = robot_in.lock()) {
        robot->get_closest_points(client_, contact_points);
    }        
}

void World::GetContactPoints(
        const std::weak_ptr<RobotBase>& robot_in,
        const std::weak_ptr<Object>& part_in,
        std::vector<ContactPoint>& contact_points,
        const int link) {
    if(auto robot = robot_in.lock()) {
        robot->get_contact_points(client_, contact_points, link);
    } 
}

void World::BatchRayTest(
        const std::vector<Ray>& rays,
        std::vector<RayTestInfo>& result,
        const int num_threads) {
    cast_rays(rays, result, num_threads);
}

void World::RayTest(
        const glm::vec3& from, const glm::vec3& to, RayTestInfo& result) {
    cast_ray({from, to}, result);
}

std::shared_ptr<RobotBase> World::LoadModelFromCache(
        const std::string& filename,
        const glm::vec3& position,
        const glm::vec4& rotation) {

    std::shared_ptr<RobotBase> robot;

    if (recycle_robot_map_.find(filename) != recycle_robot_map_.end() && 
            !recycle_robot_map_[filename].empty()) {

        robot = recycle_robot_map_[filename].back();
        recycle_robot_map_[filename].pop_back();
        robot->reuse();

        set_pose(client_, robot->id(), position, rotation);
        set_velocity(client_, robot->id(), kFloat3Zero);
    }
    return robot;
}

void World::QueryObjectByLabel(
        const std::string& label, std::vector<ObjectAttributes>& result) {
    assert(!label.empty());

    if(object_locations_.find(label) != object_locations_.end()) {

        auto find = object_locations_[label];

        result.clear();
        for (int i = 0; i < find.size(); ++i) {
            int bullet_id = find[i];
            auto object = id_to_robot_[bullet_id];

            glm::vec3 root_aabb_min, root_aabb_max;
            object->root_part_->GetAABB(root_aabb_min, root_aabb_max);

            ObjectAttributes attribs;
            attribs.aabb_min = root_aabb_min;
            attribs.aabb_max = root_aabb_max;
            attribs.bullet_id = bullet_id;

            result.push_back(attribs);
        }
    }
}

void World::AddObjectWithLabel(const std::string& label, const int id) {
    assert(!label.empty());

    if(object_locations_.find(label) == object_locations_.end()) {
        std::vector<int> bullet_id_list;
        bullet_id_list.push_back(id);
        object_locations_[label] = bullet_id_list;
    } else {
        object_locations_[label].push_back(id);
    }
}

void World::RemoveObjectWithLabel(const int id) {
    for (auto it = object_locations_.begin(); it != object_locations_.end(); ++it) {
        if (it->second.size() > 0) {
            auto find = std::find(it->second.begin(), it->second.end(), id);
            if (find != it->second.end()) {
                it->second.erase(find);
            }
        }
    }
}

render_engine::ModelDataSPtr World::FindInCache(
        const std::string& key, 
        std::vector<render_engine::ModelDataSPtr>& model_list,
        bool& reset) {
    auto it = model_cache_.find(key);
    if (it != model_cache_.end()) {
        auto model_data = it->second;
        model_list.push_back(model_data);
        reset = false;
        if (model_data) {
            return model_data;
        }
    } else {
        auto model_data  = std::make_shared<ModelData>();
        model_list.push_back(model_data);
        model_cache_[key] = model_data;
        return model_data;
    }
    return nullptr;
}

void World::ClearCache() {
    model_cache_.clear();
}

void World::robot_iteration_begin() {
    robot_it_ = id_to_robot_.begin();
}

RenderBody* World::next_robot() {
    if (robot_it_ == id_to_robot_.end()) {
        return NULL;
    } else {
        RenderBody* ret = static_cast<RenderBody*>(robot_it_->second.get());
        robot_it_++;
        return ret;
    }
}

bool World::has_next_robot() const {
    return robot_it_ != id_to_robot_.end();
}

}
