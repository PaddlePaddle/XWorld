#ifndef PLAYGROUND_PY_H_
#define PLAYGROUND_PY_H_

#include <memory>
#include <vector>
#include <functional>
#include <unordered_map>

#include <boost/python.hpp>

#include "render_engine/render.h"
#include "map.h"
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

struct Range {
	glm::vec3 min, max;

	boost::python::tuple GetMin() const { return vec2tuple(min); }
	boost::python::tuple GetMax() const { return vec2tuple(max); }
	bool __eq__(const Range& other) { min == max; }
};

class Thing {
public:
	Thing();

	boost::python::tuple GetPosition();
	boost::python::tuple GetOrientation();
	std::string GetLabel();
	
	long __hash__() { return (long) (uintptr_t) robot_.lock().get(); }
	bool __eq__(const Thing& other) { 
		return robot_.lock().get() == other.robot_.lock().get();
	}

	std::weak_ptr<RobotBase> GetPtr() const { return robot_; }
	void SetPtr(std::weak_ptr<RobotBase> robot) { robot_ = robot; Sync(); }

private:
	void Sync();
	std::string label_;
	boost::python::tuple position_;
	boost::python::tuple orientation_;
	std::weak_ptr<RobotBase> robot_;
};

class Playground {
public:
	Playground(const int w, const int h, const int headless = 0, const int quality = 0); 
	~Playground();

	// Rendering
	void SetLighting();
	void AttachCameraTo(Thing object, const boost::python::list offset_py);

	// Functionality
	void EnableLidar(const int num_rays, const float max_distance);
	boost::python::list UpdateLidar(const boost::python::list front_py, 
								    const boost::python::list up_py,
								    const boost::python::list position_py);

	void EnableInventory(const int max_capacity);
	void EnableCrowds();

	// Scene Generation
	void Clear();
	void CreateAnTestScene();
	void CreateSceneFromSUNCG();
	void CreateRandomGenerateScene();
	void CreateEmptySceneWithBuildingBlock();

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

	// Utils
	bool GameOver() const { return gameover_; }

	// Sensors
	float GetNearClippingDistance();
	float GetFarClippingDistance();
	boost::python::object GetCameraRGBDRaw();
	boost::python::object GetCameraRGBRaw();
	boost::python::object GetCameraDepthRaw();

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
	void ControlJoints(const Thing& object, const boost::python::list joint_positions);
	boost::python::list EnableInteraction();
	void DisableInteraction();

	// Query
	bool QueryObjectAABBIntersect(Thing& object_a, Thing& object_b);
	bool QueryObjectWithLabelAtCameraCenter(const std::string& label);
	bool QueryObjectWithLabelAtForward(const std::string& label);
	bool QueryObjectWithLabelNearMe(const std::string& label);
	Thing QueryObjectAtCameraCenter();
	boost::python::list QueryObjectByLabel(const std::string& label);

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

	class_<Playground>("Playground", init<int,int,int,int>())
	.def("SetLighting", &Playground::SetLighting)
	.def("EnableLidar", &Playground::EnableLidar)
	.def("UpdateLidar", &Playground::UpdateLidar)
	.def("EnableInventory", &Playground::EnableInventory)
	.def("Clear", &Playground::Clear)
	.def("CreateAnTestScene", &Playground::CreateAnTestScene)
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

	scope().attr("HEADLESS")            = 1;
	scope().attr("VISUALIZATION")       = 0;
	scope().attr("WORLD_UP")            = boost::python::make_tuple(0, 1, 0);
	scope().attr("METACLASS_WALL")      = std::string("Wall");
	scope().attr("METACLASS_FLOOR")     = std::string("Floor");
	scope().attr("METACLASS_CEILING")   = std::string("Ceiling");
	scope().attr("RENDER_QUALITY_LOW")  = render_engine::kLowQuality;
	scope().attr("RENDER_QUALITY_MED")  = render_engine::kNormalQuality;
	scope().attr("RENDER_QUALITY_HIGH") = render_engine::kHighQuality;
}

BOOST_PYTHON_MODULE(libxrobot)
{
	def_py_init();
}

#endif // PLAYGROUND_PY_H_