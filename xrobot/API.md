XRobot Python API
=================

# Rendering

## Playground

Playground is essential to the simulation environment and rendering engine. Here we define the width and height of the virtual camera, and also select the rendering quality and assign it to a dedicated gpu (if you have a multi-gpu system)

Rendering Quality can be chosen between low and normal. Low quality does not have any of post-processing but is quite fast. However, normal quality use SSAO to increase some small occlusion-like details and use FXAA to reduce some of jagged edges. Exposure control is also available in normal rendering quality. To switch to low-quality mode, change the "HEADLESS" to "RENDER_QUALITY_LOW"

Headless context is used for rendering a scene without associating a window. The default rendering backend is EGL, and GLX Headless is also supported. To disable headless context, change the "HEADLESS" to "Debug_Visualization" for visualizing the scene with a flyover camera. (This only available when rendering is debugged mode)

To create a playground, use `Playground(width, height, mode, quality, gpu)`

* Example:
```python
	your_playground = Playground(640, 480, HEADLESS, RENDER_QUALITY_NORMAL, 0)
```

# Scene

## Empty Scene

To create a completely empty scene without any building block, use `CreateEmptyScene()`

* Example:
```python
	your_playground.CreateEmptyScene()
```

* Example for assigning a boundary which could be useful in high quality rendering:
```python
	your_playground.CreateEmptyScene(min_x, max_x, min_z, max_z)
```

This is quite helpful when you want to load a fully constructed scene with OBJ files. However, you cannot load any SUNCG house into this type of scene. Jump to "Load Object into Scene" for loading a scene

## SUNCG Scene

To create a scene can load SUNCG JSON file, use `CreateSceneFromSUNCG()` and `LoadSUNCG(suncg_house_path, model_category_path, suncg_dir, filter)`

You also can use "filter" to remove some speical types of objects while loading the scene. Use `REMOVE_DOOR` and `REMOVE STAIR` to remove all the door and stairs respectively. `REMOVE DOOR | REMOVE_STAIR` is also acceptable. 

* Example:
```python
	your_playground.CreateSceneFromSUNCG()
	your_playground.LoadSUNCG(suncg_house, suncg_metadata, suncg_dir, REMOVE_DOOR)
```

## Arena Scene

To create a square arena (10 x 10 unit) with checkerboard-pattern for testing, use `CreateAnTestScene()`.

Currently the arena size and pattern cannot been changed.

* Example:
```python
	your_playground.CreateAnTestScene()
```

## Build-in Rooms with Randomly Generate Tiles

To create a room with randomly generate tiles, use `CreateRandomGenerateScene()`. And a basic room configuration need to be assigned before generating the room.

a) First, we need create a profile using python dictionary. The format must follow the example below:

* Example of creating a profile:
```python
	room_configuration = {
		"room"      : [floor1_path, wall1_apth, door1_path, floor2_path, wall2_apth, door2_path,...]
		"on_floor"  : [object1_path, "object1_label", object2_path, "object2_label",...]
		"on_object" : [object1_path, "object1_label",...]
		"on_either" : []
	}
```

The `room` defines the number of the rooms and the models. The real defination is a list constitudes with serveral rooms, and each room is a tuple of three models (`floor_model_path`, `wall_model_path` and `door_model_path`) in the list.

The `on_floor` defines which object will randomly spawn on floor. A pair (`object_model_path` followed by `object_label`) for each object.

Similarly, `on_object` defines which object will randomly spawn on another object and `on_either` defines which object will spawn on either object or floor. And mostly, a large size object will not spawn on a small size object.

b) Generate the rooms

* Example of generating the rooms using a profile:

```python
	your_playground.LoadRandomSceneConfigure(room_configuration)
	your_playground.LoadRandomScene(max_length_or_width, num_on_floor, num_on_object, num_on_either)
```

`max_length_or_width` defines the maximum length and width of the scene

`num_on_floor`, `num_on_object`, `num_on_either` defines number of object will spawn on floor, object and either cases. The actual number of object spawn could smaller than your defined number.

For the tile arrangement, check out `xrobot/room_generator.cpp` and in details.

## Load Object into Scene

* Example to load static scene:

```python
	position = [0, 0, 0]
	orientation = [1, 0, 0, 0]
	scale = 1
	your_playground.SpawnAnObject("room.obj", position, orientation, scale, "room", fixed)
```

* Example to load active rigidbody object:

```python
	your_playground.SpawnAnObject("crate.obj", [0,0,0], [1,0,0,0], 1.0, "crate", False)
```

`fixed` defines the root node in the model whether or not treated as static object and rest node will still be actived

**Be aware of orientation representation!** It uses valid quaternion for orientation! [0, 0, 0, 0] is invalid in quaternion!

# Reset and Initialize

## Reset Environment

This will remove any object in the environment including the camera.

* Example to reset the environment:

```python
	your_playground.Clear()
```

**Reset the scene before create a new one!**

## Initialization

Initialize the scene. It tells the playground ready for rendering

* Example to reset the environment:

```python
	
	## Generate Scene

	your_playground.Initialize()

	## Simulation Loop

```

**Initialization before simulation start**

# Update

## Only Simulate One Step

* Example:

```python
	your_playground.UpdateSimulation()
```

* Example of simulating one step with actions applied :

```python
	actions = your_playground.UpdateSimulationWithAction(applied_action)
```

`applied_action` is a action id which uses to apply action before simulation step forward. You can get a list of action id use `GetActionSpace()`

The return value `actions` is a python dictionary with which special actions can be applied int next step.

The number of actions are limited to 10! It cannot execute any action id are larger than 10!

## Only Render One Frame

* Example:

```python
	your_playground.UpdateRenderer()
```

## Do Both

* Example:

```python
	your_playground.Update()
```

# Control

## Movement

Move the object or robot

* Example of Moving:

```python
	your_playground.MoveForward()
	your_playground.MoveBackward(scale)
```

The actual linear offset per step is 0.005 * `scale` and the linear offset per step is 0.005 unit.

* Example of Turning:

```python
	your_playground.TurnLeft()
	your_playground.TurnRight(scale)
```

The actual angular offset per step is 0.005 * `scale` and the angular offset per step is 0.005 rad.

* Example use `UpdateSimulationWithAction` to control the movement:

```python
	your_playground.UpdateSimulationWithAction(0) # Move Forward
	your_playground.UpdateSimulationWithAction(1) # Move Backward
	your_playground.UpdateSimulationWithAction(2) # Turn Left
	your_playground.UpdateSimulationWithAction(3) # Turn Right
```

## Camera Control

For attached camera, user only can control the pitch angle of the camera.

* Example:

```python
	your_playground.LookUp()
	your_playground.LookDown()
```

The actual pitch angle offset per step is 0.5 deg. And the angle will be clamped down in -45 to 45 deg.

* Example use `UpdateSimulationWithAction` to control the camera:

```python
	your_playground.UpdateSimulationWithAction(4) # Look Up
	your_playground.UpdateSimulationWithAction(5) # Look Down
```

## Joint Control

* Example:

```python
	joint_position = {
		0 : 0.5,
		2 : 0.7,
		3 : 0
	}
	your_playground.ControlJoints(your_robot, joint_position, max_force)
```

`joint_position` is a python dictionary, and each entry is a pair of "joint id" and "joint position" 

## Grasp / Drop

Grasp or drop the object in the center of camera (also with in 3 unit length). Enable inventory before using "Grasp / Drop", check out "Inventory" for more details. 

* Example:

```python
	your_playground.Grasp()
	your_playground.PutDown()
```

* Example use `UpdateSimulationWithAction` to Grasp/Drop:

```python
	your_playground.UpdateSimulationWithAction(8) # Grasp
	your_playground.UpdateSimulationWithAction(9) # Drop
```

**Mix using Grasp / Drop and Attach / Detach is not recommended!**

## Attach / Detach

 Attach or detach the object root base at the center of camera (also with in 3 unit length).

* Example:

```python
	your_playground.Attach()
	your_playground.Detach()
```

* Example use `UpdateSimulationWithAction` to Grasp/Drop:

```python
	your_playground.UpdateSimulationWithAction(6) # Grasp
	your_playground.UpdateSimulationWithAction(7) # Drop
```

**Mix using Grasp / Drop and Attach / Detach is not recommended!**

## Teleport

Teleport an object or robot to a certain position in scene

* Example:

```python
	teleport_to_position = [4, 0, 4]
	your_playground.Teleport(object, teleport_to_position)
```

## Rotate

Rotate an object a certain radius angles

* Example:

```python
	rotate_to = [0, 1.57, 0]
	your_playground.Rotate(rotate_to)
```

* Example use `UpdateSimulationWithAction` to rotate 90 degree along world up vector:

```python
	your_playground.UpdateSimulationWithAction(10) # Rotate 90 deg along Y-Axis
```

## Actions

You can create two special types of object capable of changing its pose. Both of two types need a json file to pre-define their behaviors.

1) The first type of object can transfer to another object instantly. The json file can be defined as:

* Example:

```json
	{
		"label" : "object label",
		"type" : "convert",
		"cycle" : false,
		"actions" : [
			{ 
			  "name" : "convert_to_crate",
			  "object" : "./crate_0.3/crate.urdf"
			},
			{ 
			  "name" : "convert_to_apple",
			  "object" : "./apple/apple.urdf"
			}
		]
	}
```

You also can have more than two stages and convert it from one to the another.

2) The second type of object can change its joint pose smoothly. The json file can be defined as:

* Example:

```json
	{
		"label" : "object label",
		"type" : "animate",
		"object" : "/home/ziyuli/model/door.urdf",
		"joint_id" : 1,
		"actions" : [
			{ 
			  "name" : "open",
			  "position" : 1.0
			},
			{ 
			  "name" : "close",
			  "position" : 0.0
			}
		]
	}

```

Currently, multi joints position assignment is not supported.

After you load those types of objects, you can use `TakeAction(id)` to them. However, `TakeAction(id)` is not working for a normal object.

* Example of Taking Action:

```python
	your_playground.TakeAction(my_crate, 1)
```

* Example use `UpdateSimulationWithAction` to take action:

```python
	available_action_to_target = your_playground.UpdateSimulationWithAction(ENABLE_INTERACTION) # Enable Interaction
	## select an action from "available_action_to_target"
	## we select action '0' for example
	your_playground.UpdateSimulationWithAction(0)  # Action 0
	your_playground.UpdateSimulationWithAction(DISABLE_INTERACTION) # Disable Interaction
```

You have to select a target first use `ENABLE_INTERACTION` before take any action. Actions only can be executed after enable interaction. `ENABLE_INTERACTION` will choose the target at the center of virtual camera (also within 3 unit length). And after taking the action, use `DISABLE_INTERACTION` to deselect the target

# Query

## Intersection Between Two Object

Query two objects (Bouding Box) are intersect

* Example:

```python
	result = your_playground.QueryObjectAABBIntersect(my_crate, my_agent)
```

## Object At Forward Direction

Query a certain object is at virtual camera forward direction

* Example:

```python
	result = your_playground.QueryObjectWithLabelAtForward(my_object_label)
```

## Object At Virtual Camera Center

Query a certain object is at virtual camera center. This will use ray-cast to test ray intersection with object concave or convex hull.

* Example:

```python
	result = your_playground.QueryObjectWithLabelAtCameraCenter(my_object_label)
```

* Example of Quering Object at Center Without Object Label:

```python
	my_object = your_playground.QueryObjectAtCameraCenter()
```

## Object Near Virtual Camera

Query a certain object is near to the virtual camera

* Example:

```python
	result = your_playground.QueryObjectWithLabelNearMe(my_object_label)
```

## Object Query

Return a list with all the object with certain label

* Example:

```python
	results = your_playground.QueryObjectByLabel(my_object_label)
```

# Sensor

## Attached Camera

## Free Camera

## Single-Ray Lidar

## Position and Orientation

# Other Features

## Inventory

## Navigation Agent

# Miscellaneous

## Lighting

Adjust basic light setting.

* Example:

```python

my_playground.SetLighting({
	"ssr"      : False,
	"ambient"  : 0.1,
	"exposure" : 1.0
})

```

Use key `direction_x`, `direction_y`, `direction_z` to update direction
Use key `ambient` to update ambient factor
Use key `exposure` to update exposure factor
Use key `ssr` to update screen space reflections

**"ssr" and "exposure" are normal rendering mode only!**

## Thing

Object or robot in playground

* Example:

```python

agent = my_playground.SpawnAnObject("husky.urdf", [0,0,0], [-1,0,0,1.57], 0.6, "Agent", True)

## Print the Label of the Thing
## console output: Agent
print str(agent)

print hash(agent)

current_position = agent.GetPosition()

current_orientation = agent.GetOrientation()

my_label = agent.GetLabel()

if agent == another_agent:
	print "They are the same!"

```

## Range

AABB struct

* Example:

```python
	
object_range   = Range([0,0,0], [1,1,1])
object_range_a = Range([0,0,0], [5,5,5])
object_range_b = Range([1,1,1], [6,6,6])

## Print AABB
## console output: (0, 0, 0, 1, 1, 1)
print str(object_range)

aabb_min = object_range.GetMin()
aabb_max = object_range.GetMax()

if object_range_a == object_range_b:
	print "The AABBs between Object A and Object B are equal!"

if object_range_a != object_range_b:
	print "The AABBs between Object A and Object B are intersect!"

if object_range_a > object_range_b:
	print "The AABB of Object A contains the AABB of Object B!"

if object_range_a < object_range_b:
	print "The AABB of Object B contains the AABB of Object A!"

```

**Be aware of "!=" between two range! The not equal operator is actually intersection test!**

## NavAgent

Special type of object or robot with navigation ability

* Example:

```python

nav_agent = my_playground.SpawnNavigationAgent(model_path, "model_label", position, orientation)

## Print the Label of the Thing
## console output: model_label
print str(nav_agent)

if nav_agent == another_nav_agent:
	print "They are the same!"

```

## Get Status and Information

You can get some performance information use `GetStatus()`.

* Example:

```python

rendered_frames = my_playground.GetStatus()["frames"]
current_framerate = my_playground.GetStatus()["framerate"]

```

# Other Features are NOT in python interface

* Multi-Ray Lidar
* More Sophicated Query
* Inverse Kinematics
* Changing Object Physics Properties
* High Quality Rendering