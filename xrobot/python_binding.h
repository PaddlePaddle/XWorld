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

// Range hold AABB struct
struct Range {
	glm::vec3 min, max;

	boost::python::tuple GetMin() const { return vec2tuple(min); }
	boost::python::tuple GetMax() const { return vec2tuple(max); }
	bool __eq__(const Range& other) { min == max; }
};

// Thing hold object or robot in playground
class Thing {
public:
	Thing();

	// Returns current position in (x y z)
	boost::python::tuple GetPosition();

	// Return current orientation in (x y z w). It uses 
	// normalized quaternion for orientation representation
	boost::python::tuple GetOrientation();

	// Return the label
	std::string GetLabel();
	
	long __hash__() { return (long) (uintptr_t) robot_.lock().get(); }
	bool __eq__(const Thing& other) { 
		return robot_.lock().get() == other.robot_.lock().get();
	}
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
			   const int quality = 0); 
	~Playground();

	void SetLighting(const boost::python::dict lighting);
	void AttachCameraTo(Thing object, const boost::python::list offset_py);

	void EnableLidar(const int num_rays, const float max_distance);
	boost::python::list UpdateLidar(const boost::python::list front_py, 
								    const boost::python::list up_py,
								    const boost::python::list position_py);

	void EnableInventory(const int max_capacity);
	void EnableNavigation(const boost::python::list min_corner, 
						  const boost::python::list max_corner,
						  const bool kill_after_arrived);

	// Navigation
	void BakeNavigationMesh();
	void AssignAgentRadius(const float radius);
	void AssignNavigationAgentTarget(const NavAgent& agent,
		                             const boost::python::list position);
	NavAgent SpawnNavigationAgent(const std::string& path,
							      const std::string& label,
							      const boost::python::list position);

	// Scene Generation
	void Clear();
	void CreateAnTestScene();
	void CreateSceneFromSUNCG();
	void CreateRandomGenerateScene();
	void CreateEmptySceneWithBuildingBlock();

	// Random Scene
	void LoadRandomSceneConfigure(const boost::python::dict conf);
	boost::python::list LoadRandomScene(const int size, 
										const int on_floor,
										const int on_object,
										const int on_either);

	// Load SUNCG
	void LoadSUNCG(const std::string& house,
				   const std::string& metadata,
				   const std::string& suncg_data_dir,
				   const int filter = -1);

	// Object Generation
	Thing SpawnAnObject(const std::string& file, 
					    const boost::python::list position_py,
					    const boost::python::list orentation_py,
					    const float scale,
					    const std::string& label,
					    const bool fixed = true);

	// Update
	void Initialize();
	void Update();
	void UpdateRenderer();
	boost::python::dict UpdateSimulation();
	boost::python::dict UpdateSimulationWithAction(const int action);

	// Observation and Action Space
	boost::python::dict GetObservationSpace();
	boost::python::dict GetActionSpace();

	// Sensors
	float GetNearClippingDistance();
	float GetFarClippingDistance();
	boost::python::object GetCameraRGBDRaw();

	// Object (Robot) Movement
	void MoveForward(const float speed = 1);
	void MoveBackward(const float speed = 1);
	void TurnLeft( const float speed = 1);
	void TurnRight(const float speed = 1);
	void LookUp();
	void LookDown();

	// Object (Robot) Actions
	void Grasp();
	void PutDown();
	void Attach();
	void Detach();
	void Rotate(const boost::python::list angle_py);
	void TakeAction(const int action_id);
	void Teleport(Thing object, const boost::python::list position_py);	
	void ControlJoints(const Thing& object, 
					   const boost::python::dict joint_positions,
					   const float max_force);
	boost::python::list EnableInteraction();
	void DisableInteraction();

	// Query
	bool QueryObjectAABBIntersect(Thing& object_a, Thing& object_b);
	bool QueryObjectWithLabelAtCameraCenter(const std::string& label);
	bool QueryObjectWithLabelAtForward(const std::string& label);
	bool QueryObjectWithLabelNearMe(const std::string& label);
	Thing QueryObjectAtCameraCenter();
	boost::python::list QueryObjectByLabel(const std::string& label);

	// Utils
	boost::python::dict GetStatus() const;
	bool GameOver() const { return gameover_; }

	// Debug
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

private:
	int w_, h_;
	int iterations_;
	float camera_aspect_;
	float camera_pitch_;
	
	bool kill_after_arrived_;
	bool gameover_;
	bool interact_;
	Thing agent_;

	std::shared_ptr<Map> scene_;
	std::shared_ptr<Inventory> inventory_;
	std::shared_ptr<Navigation> crowd_;
	std::shared_ptr<Lidar> lidar_;
	std::shared_ptr<render_engine::Render> renderer_;
	render_engine::GLContext * ctx_;
	render_engine::Camera * main_camera_;
};

void def_py_init()
{
	using namespace boost::python;

	class_<Range>("Range", no_init)
	.add_property("GetMin", &Range::GetMin)
	.add_property("GetMax", &Range::GetMax)
	.def("__eq__", &Thing::__eq__)
	;

	class_<Thing>("Thing", no_init)
	.def("GetPosition", &Thing::GetPosition)
	.def("GetOrientation", &Thing::GetOrientation)
	.def("GetLabel", &Thing::GetLabel)
	.def("__hash__", &Thing::__hash__)
	.def("__eq__", &Thing::__eq__)
	;

	class_<NavAgent>("NavAgent", no_init)
	.def("GetLabel", &NavAgent::GetLabel)
	.def("__eq__", &NavAgent::__eq__)
	;

	class_<Playground>("Playground", init<int,int,int,int>())
	.def("SetLighting", &Playground::SetLighting)
	.def("EnableLidar", &Playground::EnableLidar)
	.def("UpdateLidar", &Playground::UpdateLidar)
	.def("EnableInventory", &Playground::EnableInventory)
	.def("EnableNavigation", &Playground::EnableNavigation)
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
	.def("LoadSUNCG", &Playground::LoadSUNCG)
	.def("SpawnAnObject", &Playground::SpawnAnObject)
	.def("AttachCameraTo", &Playground::AttachCameraTo)
	.def("Initialize", &Playground::Initialize)
	.def("UpdateSimulation", &Playground::UpdateSimulation)
	.def("UpdateRenderer", &Playground::UpdateRenderer)
	.def("Update", &Playground::Update)
	.def("MoveForward", &Playground::MoveForward)
	.def("MoveBackward", &Playground::MoveBackward)
	.def("TurnLeft", &Playground::TurnLeft)
	.def("TurnRight", &Playground::TurnRight)
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

	.def("GetStatus", &Playground::GetStatus)

	.def("GetKeyPressUp", &Playground::GetKeyPressUp)
	.def("GetKeyPressDown", &Playground::GetKeyPressDown)
	.def("GetKeyPressRight", &Playground::GetKeyPressRight)
	.def("GetKeyPressLeft", &Playground::GetKeyPressLeft)
	.def("GetKeyPress1", &Playground::GetKeyPress1)
	.def("GetKeyPress2", &Playground::GetKeyPress2)
	.def("GetKeyPress3", &Playground::GetKeyPress3)
	.def("GetKeyPress4", &Playground::GetKeyPress4)
	.def("GetKeyPressKP9", &Playground::GetKeyPressKP9)
	.def("GetKeyPressKP6", &Playground::GetKeyPressKP6)
	;

	// scope().attr("NO_AA")               = 0;
	// scope().attr("AA")                  = 1;
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

BOOST_PYTHON_MODULE(libxrobot)
{
	def_py_init();
}

#endif // PLAYGROUND_PY_H_