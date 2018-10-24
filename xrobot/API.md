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

This is quite helpful when you want to load an object or a fully constructed scene with OBJ file. However, you cannot load any SUNCG house into this type of scene.

## SUNCG Scene



## Build-in Randomly Generate Grid

# Update



# Control

## Movement

## Camera Control

## Joint Control

## Character Movement

## Grasp / Drop

## Attach / Detach

## Teleport

## Rotate

## Actions

# Query

## Forward

# Sensor

## Attached Camera

## Free Camera

## Single-Ray Lidar

## Position and Orientation

# Miscellaneous

## Navigation Agent

## Lighting

## Reset and Initialize

## Range

## Get Status and Information

# Other Features are NOT in python interface

## Multi-Ray Lidar

## More Sophicated Query

## Inverse Kinematics

## Physics Properties

## High Quality Rendering