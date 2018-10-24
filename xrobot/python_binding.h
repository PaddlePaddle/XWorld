#ifndef PLAYGROUND_PY_H_
#define PLAYGROUND_PY_H_

#include <memory>
#include <vector>
#include <functional>
#include <unordered_map>

#include <boost/python.hpp>

#include "render_engine/render.h"
#include "map_grid.h"
#include "map_suncg.h"
#include "lidar.h"
#include "task.h"
#include "navigation.h"

using namespace xrobot;

typedef render_engine::GLContext CTX;

void list2vec(const boost::python::list& ns, std::vector<float>& v)
{
	int L = len(ns);
	v.resize(L);
	for (int i=0; i<L; ++i) {
		v[i] = boost::python::extract<float>(ns[i]);
	}
}

inline boost::python::tuple vec2tuple(const glm::vec3 v)
{
	return boost::python::make_tuple(v.x, v.y, v.z);
}

inline boost::python::tuple vec2tuple(const glm::vec4 v)
{
	return boost::python::make_tuple(v.x, v.y, v.z, v.w);
}

inline glm::vec3 list2vec3(const boost::python::list& ns)
{
	std::vector<float> ns_v;
	list2vec(ns, ns_v);

	assert(ns_v.size() == 3);
	return glm::vec3(ns_v[0], ns_v[1], ns_v[2]);
}

inline glm::vec4 list2vec4(const boost::python::list& ns)
{
	std::vector<float> ns_v;
	list2vec(ns, ns_v);

	assert(ns_v.size() == 4);
	return glm::vec4(ns_v[0], ns_v[1], ns_v[2], ns_v[3]);
}

inline glm::quat list2quat(const boost::python::list& ns)
{
	std::vector<float> ns_v;
	list2vec(ns, ns_v);

	assert(ns_v.size() == 4);
	return glm::angleAxis(ns_v[3], glm::vec3(ns_v[0], ns_v[1], ns_v[2]));
}

// Range hold AABB struct
struct Range {
	glm::vec3 min, max;

	Range() {
		min = glm::vec3(0);
		max = glm::vec3(0);
	}

	Range(boost::python::list min_py, boost::python::list max_py) {
		min = list2vec3(min_py);
		max = list2vec3(max_py);
	}

	// Two AABBs are equal
	bool __eq__(const Range& other) { 
		return min == other.min && max == other.max;
	}

	// Two AABBs are intersect
	bool __ne__(const Range& other) { 
		return glm::all(glm::lessThanEqual(min, other.max)) &&
			   glm::all(glm::greaterThanEqual(max, other.min));
	}

	// Contains
	bool __gt__(const Range& other) {
		return glm::all(glm::lessThan(min, other.min)) &&
			   glm::all(glm::greaterThan(max, other.max));
	}

	// Contains
	bool __lt__(const Range& other) {
		return glm::all(glm::lessThan(other.min, min)) &&
			   glm::all(glm::greaterThan(other.max, max));
	}

	// To String
	static std::string __str__(const Range& self) {
		return std::string( 
			"(" +
				std::to_string(self.min.x) + ", " + 
				std::to_string(self.min.y) + ", " +
				std::to_string(self.min.z) + ", " +
				std::to_string(self.max.x) + ", " +
				std::to_string(self.max.y) + ", " +
				std::to_string(self.max.z) +
			")"
		);
	}

	boost::python::tuple GetMin() const { return vec2tuple(min); }
	boost::python::tuple GetMax() const { return vec2tuple(max); }
};

// Thing hold object or robot in playground
class Thing {
public:
	Thing();

	// Returns current position in (x y z)
	boost::python::tuple GetPosition();

	// Return current orientation in (x y z w). It uses 
	// normalized quaternion for orientation representation!
	boost::python::tuple GetOrientation();

	// Return the label
	std::string GetLabel();
	
	long __hash__() { return (long) (uintptr_t) robot_.lock().get(); }
	bool __eq__(const Thing& other) { 
		return robot_.lock().get() == other.robot_.lock().get();
	}
	static std::string __str__(const Thing& self) { return self.label_; }

	std::weak_ptr<RobotBase> GetPtr() const { return robot_; }
	void SetPtr(std::weak_ptr<RobotBase> robot) { robot_ = robot; Sync(); }

private:
	//  Update all the status
	void Sync();
	std::string label_;
	boost::python::tuple position_;
	boost::python::tuple orientation_;
	std::weak_ptr<RobotBase> robot_;
};

// NavAgent hold a object with navigation functionality
class NavAgent {
public:
	explicit NavAgent(const int uid, const std::string& label);

	std::string GetLabel() const { return label_; }
	int GetUid() const { return uid_; }
	bool __eq__(const NavAgent& other) { return uid_ == other.uid_; }
	static std::string __str__(const NavAgent& self) { return self.label_; }

private:
	// Used to validate and access the agent in crowd. -1 means 
	// this agent is invalid
	int uid_;
	std::string label_;
};

// Playground defines the scene and renderer. Used to control the robot,
// generate a scene, enable certain feature and query the object.
//
// If you are going to seek some special functionalities, 
// such as IK, constraints, carving while path-finding, multi-rays lidar 
// and high quality rendering this wrapper is not supported, use C++ instead.
class Playground {
public:

	// Create a empty playground with basic rendering parameters.
	// 
	// If you are going to flyover visualization, use DEBUG_VISUALIZATION
	// 
	// Quality also can be adjust by switch RENDER_QUALITY_LOW and 
	// RENDER_QUALITY_NORMAL. However, low quality rendering 
	// does not have anti-aliasing which means images could have jagged edges.
	Playground(const int w, const int h,
			   const int headless = 0, 
			   const int quality = 0,
			   const int device = 0); 

	~Playground();

	// Adjust directional light setting. Use python dict to update
	// the configurations
	//
	// Example in Python:
	// 	light_conf = dict()
	//	light_conf["ambient"] = 0.1
	// 	lifht_conf["exposure"] = 1.0
	//  env.SetLighting(light_conf)
	//
	// Use key "direction_x", "direction_y", "direction_z" to update direction
	// Use key "ambient" to update ambient factor
	// Use key "exposure" to update exposure factor
	void SetLighting(const boost::python::dict lighting);

	// Create a camera and attach it to a certain object in the scene
	void AttachCameraTo(Thing object, const boost::python::list offset_py);

	// Create a free camera
	void FreeCamera(const boost::python::list position, const float yaw, const float pitch);
	void UpdateFreeCamera(const boost::python::list position, const float yaw, const float pitch);

	// Enable single ray lidar in the scene
	// 
	// A desired number of rays is less than 720
	void EnableLidar(const int num_rays, const float max_distance);

	// Update lidar and return the results. -1 means no-hit
	//
	// Front, up vectors and attached position are necessary
	boost::python::list UpdateLidar(const boost::python::list front_py, 
								    const boost::python::list up_py,
								    const boost::python::list position_py);

	// Enable inventory for robot temporary storage. 
	//
	// This member function is nessecary for use "Pickup" and "Putdown" actions
	void EnableInventory(const int max_capacity = 1);

	// Enable navigations to use path-finding for a object or robot
	//
	// The range of the baking area is defined by two 'vectors'. 
	// The minimum y must smaller than the ground, and the maximum y cannot
	// greater than the ceiling.
	//
	// Path-finding relies on grid map which is generated by a special
	// depth camera on top of the scene. The range of the baking area
	// needs to be passed into this member function.
	void EnableNavigation(const boost::python::list min_corner, 
						  const boost::python::list max_corner,
						  const bool kill_after_arrived);

	// Bake the grid map for path-finding.
	void BakeNavigationMesh();

	// Assign a threshould for determine ground when baking the grid map
	// 
	// The threshould is a log-based value between 0 to 1
	//
	// The surface threshould will be automatically calculated base on
	// the major depth samples in the grid map. However, you may need to
	// assign it by yourself in some scenario
	void AssignSurfaceLevel(const float level);

	// Assign a scale up value to dilate the grid map
	//
	// In some cases, a large size agent cannot pass a small gap. You need
	// dilate the grid map to fill the gaps
	//
	// A negative value for erode the grid map
	void AssignAgentRadius(const float radius);


	// Assign a target to an agent
	//
	// If an agent is no longer available, it will be neglected automatically
	void AssignNavigationAgentTarget(const NavAgent& agent,
		                             const boost::python::list position);

	// Spawn an agent with path-finding feature
	NavAgent SpawnNavigationAgent(const std::string& path,
							      const std::string& label,
							      const boost::python::list position,
							      const boost::python::list orientation);

	// Clear everything in the scene, including the camera
	void Clear();

	// Generate a empty scene with checkerboard style floors and walls
	//
	// Default size is 5x5 (10m x 10m)
	void CreateAnTestScene();

	// Generate a empty SUNCG scene
	void CreateSceneFromSUNCG();

	// Generate a empty scene
	void CreateEmptyScene(const float min_x = -5, const float max_x = 5,
                          const float min_z = -5, const float max_z = 5);

	// Generate a random size room
	void CreateRandomGenerateScene();

	// Load a profile to generate random size room
	//
	// Example in Python:
	//	conf = dict()
	//	conf["room"] = [floor_path, wall_apth, door_path,...]
	//	conf["on_floor"] = [crate_path, "crate_label",...]
	//  conf["on_object"] = [crate_path, "crate_label",...]
	//  conf["on_either"] = [crate_path, "crate_label",...]
	//  env.LoadRandomSceneConfigure(conf)
	//
	void LoadRandomSceneConfigure(const boost::python::dict conf);

	// Generate radom scene
	//
	// "on_floor" means how many random object will generate on floor
	// "on_object" means how many random object will generate on object
	// "on_either" means how many random object will generate either on 
	// floor or object
	boost::python::list LoadRandomScene(const int size, 
										const int on_floor,
										const int on_object,
										const int on_either);

	// Load SUNCG
	//
	// "filter" means remove a certain type or some types object while 
	// loading the scene
	void LoadSUNCG(const std::string& house,
				   const std::string& metadata,
				   const std::string& suncg_data_dir,
				   const int filter = -1);

	// Generate object at position with orientation. It uses 
	// normalized quaternion for orientation representation!
	// 
	// Only uniform scale is supported in Python
	Thing SpawnAnObject(const std::string& file, 
					    const boost::python::list position_py,
					    const boost::python::list orentation_py,
					    const float scale,
					    const std::string& label,
					    const bool fixed = true);

	// Initialize camera
	//
	// Make sure call this member function before rendering loop
	void Initialize();

	// Update simulation and renderer
	void Update();

	// Update renderer 
	//
	// This also swap buffer in back
	void UpdateRenderer();

	// Update simulation Only
	//
	// This member function will not return any of available actions for 
	// robot. To "UpdateSimulationWithAction" for actions return
	boost::python::dict UpdateSimulation();

	// Update simulation with action applied
	//
	// This member function will return the actions which available to robot
	// in Python dict
	//
	// The number of actions are limited to 10! It cannot execute any action
	// id are larger than 10
	boost::python::dict UpdateSimulationWithAction(const int action);

	// Get observation
	//
	// Velocities and Accelerations are not supported
	boost::python::dict GetObservationSpace();

	// Get actions
	//
	// Action ids are the continuous order numbers in the list
	//
	// Attach/Detach and Pickup/Putdown cannot use simultaneously.
	boost::python::dict GetActionSpace();

	// Get camera position and orientation
	boost::python::tuple GetCameraPosition() const;
	boost::python::tuple GetCameraFront() const;
	boost::python::tuple GetCameraRight() const;
	boost::python::tuple GetCameraUp() const;

	// Get camera near clipping plane's distance
	//
	// Could be useful for calculating real depth
	float GetNearClippingDistance();

	// Get camera far clipping plane's distance
	//
	// Could be useful for calculating real depth
	float GetFarClippingDistance();

	// Get raw renderered images
	//
	// 8-bit unsigned char RGBA raw 
	boost::python::object GetCameraRGBDRaw();

	// Move the object or robot 
	//
	// The actual distance offset per step is 0.005 * speed
	void MoveForward(const float speed = 1);
	void MoveBackward(const float speed = 1);
	
	// Turn the object or robot
	//
	// The actual radius offset per step is 0.005 * speed
	void TurnLeft(const float speed = 1);
	void TurnRight(const float speed = 1);
	
	// Pitch up or down in 0.5 deg angle
	//
	// The angle will be clamped down in -45 to 45 deg 
	void LookUp();
	void LookDown();

	// Pick up the object in the center of camera (also with in 3 unit length)
	void Grasp();

	// Put down the object in the center of camera (also with in 3 unit length)
	void PutDown();

	// Attach or stick the object root base at the center of camera (also with in 3 unit length)
	void Attach();

	// Detach the object which already been attached
	void Detach();

	// Rotate an object a certain degree angles
	void Rotate(const boost::python::list angle_py);

	// Take an action to a object with action id
	//
	// action id are limited from 0 to 10
	void TakeAction(const int action_id);

	// Teleport an object or robot to a certain position in scene
	void Teleport(Thing object, const boost::python::list position_py);

	// Move or rotate the joint in certain position with maximum forces	
	//
	// "joint_position" should follow { joint_id : joint_position }
	void ControlJoints(const Thing& object, 
					   const boost::python::dict joint_positions,
					   const float max_force);

	// Enable or disable the interaction for further actions
	//
	// Actions only can be executed after enable interaction
	boost::python::list EnableInteraction();
	void DisableInteraction();

	// Query two objects' (Bouding Box) are intersect
	bool QueryObjectAABBIntersect(Thing& object_a, Thing& object_b);

	// Query a certain object is at camera center
	//
	// This will use ray-cast to test ray intersection with object 
	// concave or convex hull
	bool QueryObjectWithLabelAtCameraCenter(const std::string& label);

	// Query a certain object is at forward
	//
	// Only use distance and angle
	bool QueryObjectWithLabelAtForward(const std::string& label);

	// Query a certain object is near the robot
	bool QueryObjectWithLabelNearMe(const std::string& label);

	// Return object at camera center
	Thing QueryObjectAtCameraCenter();

	// Return a list contains all object with certain label
	boost::python::list QueryObjectByLabel(const std::string& label);

	// Get the framerate, rendered frames count and cache information
	boost::python::dict GetStatus() const;


	// Debug and Unfinished Member Functions
	//
	// You should not use any of this member functions
	bool GetKeyPressUp() { return ctx_->GetKeyPressUp(); }
	bool GetKeyPressDown() { return ctx_->GetKeyPressDown(); }
	bool GetKeyPressRight() { return ctx_->GetKeyPressRight(); }
	bool GetKeyPressLeft() { return ctx_->GetKeyPressLeft(); }
	bool GetKeyPress1() { return ctx_->GetKeyPress1(); }
	bool GetKeyPress2() { return ctx_->GetKeyPress2(); }
	bool GetKeyPress3() { return ctx_->GetKeyPress3(); }
	bool GetKeyPress4() { return ctx_->GetKeyPress4(); }
	bool GetKeyPressKP9() { return ctx_->GetKeyPressKP9(); }
	bool GetKeyPressKP6() { return ctx_->GetKeyPressKP6(); }
	bool GameOver() const { return gameover_; }

private:
	int w_, h_;
	int iterations_;
	float camera_aspect_;
	float camera_pitch_;
	
	bool kill_after_arrived_;
	bool gameover_;
	bool interact_;
	Thing agent_;

	boost::python::list current_actions_;

	std::shared_ptr<Map> scene_;
	std::shared_ptr<Inventory> inventory_;
	std::shared_ptr<Navigation> crowd_;
	std::shared_ptr<Lidar> lidar_;
	std::shared_ptr<render_engine::Render> renderer_;
	render_engine::GLContext * ctx_;
	render_engine::Camera * main_camera_;
};

// Python Binding

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(EnableInventory_member_overloads, 
									   Playground::EnableInventory, 0, 1)

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(CreateEmptyScene_member_overloads, 
									   Playground::CreateEmptyScene, 0, 4)

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(LoadSUNCG_member_overloads, 
									   Playground::LoadSUNCG, 3, 4)

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(SpawnAnObject_member_overloads, 
									   Playground::SpawnAnObject, 5, 6)

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(MoveForward_member_overloads, 
									   Playground::MoveForward, 0, 1)

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(MoveBackward_member_overloads, 
									   Playground::MoveBackward, 0, 1)

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(TurnLeft_member_overloads, 
									   Playground::TurnLeft, 0, 1)

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(TurnRight_member_overloads, 
									   Playground::TurnRight, 0, 1)

BOOST_PYTHON_MODULE(libxrobot)
{
	using namespace boost::python;

	class_<Range>("Range", init<boost::python::list, boost::python::list>())
	.add_property("GetMin", &Range::GetMin)
	.add_property("GetMax", &Range::GetMax)
	.def("__eq__", &Range::__eq__)
	.def("__ne__", &Range::__ne__)
	.def("__lt__", &Range::__lt__)
	.def("__gt__", &Range::__gt__)
	.def("__str__", &Range::__str__)
	;

	class_<Thing>("Thing", no_init)
	.def("GetPosition", &Thing::GetPosition)
	.def("GetOrientation", &Thing::GetOrientation)
	.def("GetLabel", &Thing::GetLabel)
	.def("__hash__", &Thing::__hash__)
	.def("__eq__", &Thing::__eq__)
	.def("__str__", &Thing::__str__)
	;

	class_<NavAgent>("NavAgent", no_init)
	.def("GetLabel", &NavAgent::GetLabel)
	.def("__eq__", &NavAgent::__eq__)
	.def("__str__", &NavAgent::__str__)
	;

	class_<Playground>("Playground", init<int,int,optional<int,int,int>>())

	.def("EnableInventory", &Playground::EnableInventory,
		EnableInventory_member_overloads(
			args("max_capacity"), "capacity"
		)
	)
	.def("CreateEmptyScene", &Playground::CreateEmptyScene, 
		CreateEmptyScene_member_overloads(
			args("min_x", "max_x", "min_z", "max_z"), "range"
		)
	)
	.def("LoadSUNCG", &Playground::LoadSUNCG,
		LoadSUNCG_member_overloads(
			args("house", "metadata", "suncg_data_dir", "filter"), "suncg"
		)
	)
	.def("SpawnAnObject", &Playground::SpawnAnObject,
		SpawnAnObject_member_overloads(
			args("file", "position_py", "orentation_py", "scale", "label", "fixed"),
			"spawn"
		)
	)
	.def("MoveForward", &Playground::MoveForward,
		MoveForward_member_overloads(
			args("speed"), "speed"
		)
	)
	.def("MoveBackward", &Playground::MoveBackward,
		MoveBackward_member_overloads(
			args("speed"), "speed"
		)
	)
	.def("TurnLeft", &Playground::TurnLeft,
		TurnLeft_member_overloads(
			args("speed"), "speed"
		)
	)
	.def("TurnRight", &Playground::TurnRight,
		TurnRight_member_overloads(
			args("speed"), "speed"
		)
	)

	.def("SetLighting", &Playground::SetLighting)
	.def("EnableLidar", &Playground::EnableLidar)
	.def("UpdateLidar", &Playground::UpdateLidar)
	.def("EnableNavigation", &Playground::EnableNavigation)
	.def("AssignSurfaceLevel", &Playground::AssignSurfaceLevel)
	.def("AssignAgentRadius", &Playground::AssignAgentRadius)
	.def("BakeNavigationMesh", &Playground::BakeNavigationMesh)
	.def("AssignNavigationAgentTarget", &Playground::AssignNavigationAgentTarget)
	.def("SpawnNavigationAgent", &Playground::SpawnNavigationAgent)
	.def("Clear", &Playground::Clear)
	.def("CreateAnTestScene", &Playground::CreateAnTestScene)
	.def("CreateSceneFromSUNCG", &Playground::CreateSceneFromSUNCG)
	.def("CreateRandomGenerateScene", &Playground::CreateRandomGenerateScene)
	.def("LoadRandomSceneConfigure", &Playground::LoadRandomSceneConfigure)
	.def("LoadRandomScene", &Playground::LoadRandomScene)
	.def("AttachCameraTo", &Playground::AttachCameraTo)
	.def("FreeCamera", &Playground::FreeCamera)
	.def("UpdateFreeCamera", &Playground::UpdateFreeCamera)
	.def("Initialize", &Playground::Initialize)
	.def("UpdateSimulation", &Playground::UpdateSimulation)
	.def("UpdateRenderer", &Playground::UpdateRenderer)
	.def("Update", &Playground::Update)
	.def("LookUp", &Playground::LookUp)
	.def("LookDown", &Playground::LookDown)
	.def("Grasp", &Playground::Grasp)
	.def("PutDown", &Playground::PutDown)
	.def("Attach", &Playground::Attach)
	.def("Detach", &Playground::Detach)
	.def("Rotate", &Playground::Rotate)
	.def("TakeAction", &Playground::TakeAction)
	.def("ControlJoints", &Playground::ControlJoints)
	.def("Teleport", &Playground::Teleport)
	.def("GetCameraRGBDRaw", &Playground::GetCameraRGBDRaw)
	.def("QueryObjectAABBIntersect", &Playground::QueryObjectAABBIntersect)
	.def("QueryObjectWithLabelAtCameraCenter", &Playground::QueryObjectWithLabelAtCameraCenter)
	.def("QueryObjectWithLabelAtForward", &Playground::QueryObjectWithLabelAtForward)
	.def("QueryObjectWithLabelNearMe", &Playground::QueryObjectWithLabelNearMe)
	.def("QueryObjectAtCameraCenter", &Playground::QueryObjectAtCameraCenter)
	.def("QueryObjectByLabel", &Playground::QueryObjectByLabel)
	.def("UpdateSimulationWithAction", &Playground::UpdateSimulationWithAction)
	.def("GetObservationSpace", &Playground::GetObservationSpace)
	.def("GetActionSpace", &Playground::GetActionSpace)
	.def("GetCameraPosition", &Playground::GetCameraPosition)
	.def("GetCameraRight", &Playground::GetCameraRight)
	.def("GetCameraFront", &Playground::GetCameraFront)
	.def("GetCameraUp", &Playground::GetCameraUp)
	.def("GetStatus", &Playground::GetStatus)
	// .def("GetKeyPressUp", &Playground::GetKeyPressUp)
	// .def("GetKeyPressDown", &Playground::GetKeyPressDown)
	// .def("GetKeyPressRight", &Playground::GetKeyPressRight)
	// .def("GetKeyPressLeft", &Playground::GetKeyPressLeft)
	// .def("GetKeyPress1", &Playground::GetKeyPress1)
	// .def("GetKeyPress2", &Playground::GetKeyPress2)
	// .def("GetKeyPress3", &Playground::GetKeyPress3)
	// .def("GetKeyPress4", &Playground::GetKeyPress4)
	// .def("GetKeyPressKP9", &Playground::GetKeyPressKP9)
	// .def("GetKeyPressKP6", &Playground::GetKeyPressKP6)
	;

	scope().attr("ENABLE_INTERACTION")  = 11;
	scope().attr("DISABLE_INTERACTION") = 12;
	scope().attr("NO_ACTION")           = 13;
	scope().attr("HEADLESS")            = 1;
	scope().attr("DEBUG_VISUALIZATION") = 0;
	scope().attr("GRID")                = 0;
	scope().attr("SUNCG")               = 1;
	scope().attr("REMOVE_NONE")         = -1;
	scope().attr("REMOVE_STAIR")        = 1;
	scope().attr("REMOVE_DOOR")         = 2;
	scope().attr("WORLD_UP")            = boost::python::make_tuple(0, 1, 0);
	scope().attr("METACLASS_WALL")      = std::string("Wall");
	scope().attr("METACLASS_FLOOR")     = std::string("Floor");
	scope().attr("METACLASS_CEILING")   = std::string("Ceiling");
	scope().attr("RENDER_QUALITY_LOW")     = render_engine::kLowQuality;
	scope().attr("RENDER_QUALITY_NORMAL")  = render_engine::kNormalQuality;
	scope().attr("RENDER_QUALITY_HIGH")    = render_engine::kHighQuality;
}

#endif // PLAYGROUND_PY_H_