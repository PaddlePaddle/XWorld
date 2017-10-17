# <img src="doc/xworld_logo.png" alt="XWorld">
This repository currently contains a collection of simulators for Reinforcement Learning research.

|**Name**|**Description**|**Thread-compatible?***|**Optional?**|
|:--------------|---------------|:---------------:|:---------------:|
|**SimpleGame**|A simple 1D array-walking game.|Yes|No|
|**SimpleRace**|A simple synthetic car racing game.|Yes|No|
|**XWorld2D**|A 2D environment for an agent to learn vision and language abilities.|No|No|
|**Atari**|Wrappers for the Arcade Learning Environment ([ALE](http://www.arcadelearningenvironment.org/)) environment. For stability, we use a fork version.|Yes|Yes|

*If yes, then multithreading can be used; otherwise multiprocessing is needed.

# Requirements
* Compiler: GCC 4.8 or above
* CMake: CMake 3.0 or above
* Python: Python 2.7

# Dependencies
The following softwares must be installed before building XWorld.
* [Boost](http://www.boost.org/)
* [Glog](https://github.com/google/glog)
* [GFlags](https://github.com/gflags/gflags)
* [GTest](https://github.com/google/googletest)
* [Python](https://www.python.org/)

In Ubuntu 14.04 and 16.04, you can do
```
sudo apt-get install libboost-all-dev libgflags-dev libgoogle-glog-dev libgtest-dev python-dev
```

# Build
First get this git repository
```
git clone https://github.com/PaddlePaddle/XWorld
```

Suppose the directory is `xworld_path`, then do
```
cd <xworld_path>
mkdir -p build
cd build
cmake [<optional parameters>] ..
```
For example,
```
cd ~/XWorld; mkdir build; cd build
cmake ..
```

Finally, in the build directory do
```
make
make test
```

By default, XWorld only builds the first three games: SimpleGame, SimpleRace, and XWorld2D. Optionally, you can install [Atari](http://www.arcadelearningenvironment.org/) by:

```
cmake -DWITH_ATARI=ON ..
```
which will automatically download and build Atari.

# Use
You can compile a C++ project with XWorld. The C++ simulator APIs are located in simulator.h ```GameSimulator```.
Alternatively, we provide a set of simple Python APIs for interacting with the simulators. After building XWorld, you need to export the path of the python module:
```
export PYTHONPATH=<xworld_path>/python:$PYTHONPATH
```
You can add the above line to ~/.bashrc to avoid doing the export in the future.

To get started, several examples of the simulator Python APIs can be found in
```
<xworld_path>/python/examples
```
And several C++ examples (run the .sh scripts inside) can be found in
```
<xworld_path>/examples
```
Generally, C++ APIs are more flexible but expose more details than Python APIs.

# Instructions for creating games
Below we explain the flags that the user can set for each game. If you use Python APIs, these flags are passed in as a dictionary when creating the game (see <xworld_path>/python/examples for details). If you use C++ APIs, these flags are either set globally from command line with GFlags, or are passed as arguments of the class constructor (see <xworld_path>/examples for details).

|**ID**|**Python Keyword**|**C++ Class Name**|**Flags**|
|---|---|-----|-----------|
|**a**|```simple_game```|```SimpleGame```|```pause_screen```,```array_size```|
|**b**|```simple_race```|```SimpleRaceGame```|```pause_screen```,```window_width```,```window_height```,<br>```track_type```,```track_width```,```track_length```,<br>```track_radius```,```race_full_manouver```,```random```,<br>```difficulty```|
|**c**|```xworld```|```XWorldSimulator```|```pause_screen```,```conf_path```,```curriculum```,<br>```task_mode```,```task_groups_exclusive```,```context```|
|**d**|```atari```|```ArcadeGame```|```pause_screen```,```ale_rom```,```context```|

The meanings of the above flags are listed below. Each flag applies to certain games indicated by the IDs.
* ```pause_screen``` (**a-d**)

    Pause the shown screen when show_screen() is called, until any key is pressed. (Default: false)

* ```array_size``` (**a**)

    The array size of simple game. (Default: 0)

* ```window_width``` (**b**)

    Width of display window for Simple Race. (Default: 0)

* ```window_height``` (**b**)

    Height of display window for Simple Race. (Default: 0)

* ```track_type``` (**b**)

    Track type used in Simple Race: circular track ("circle") or straight track ("straight"). (Default: "straight")

* ```track_width``` (**b**)

    The width of the track in Simple Race. (Default: 0)

* ```track_length``` (**b**)

    The length of the track in Simple Race, if ```track_type``` is "straight". (Default: 0)

* ```track_radius``` (**b**)

    The radius of the circluar track in Simple Race, if ```track_type``` is "circle". (Default: 0)

* ```race_full_manouver``` (**b**)

    In Simple Race, whether to allow turning and moving forward/backward at the same time (true) or not (false). (Default: false)

* ```random``` (**b**)

    In Simple Race, whether to enable random start positiona and facing direction (true) or not (false). (Default: false)

* ```difficulty``` (**b**)

    Difficulty level: "easy" level provides negative rewards when moving away from the center line of the track, and "hard" level only provides rewards when out-of-boundary or reaching finish line. (Default: "easy")

* ```conf_path``` (**c**)

    The conf JSON file for XWorld2D, or the conf XML file for MALMO. (Default: "")

* ```curriculum``` (**c**), can be extended to any of (**a-f**) that employs curriculum learning

    This number indicates the maximum number of games that has curriculum learning. Given any difficulty range that can be quantized by two integers [*low*, *high*], if ```curriculum``` is 0, then the difficulty number is randomly selected from this range; otherwise, the difficulty is linearly increased from *low* to *high* until ```curriculum``` games have been played. (Default: 0)

* ```task_mode``` (**c**)

    This flag has three possible values.

    "arxiv_lang_acquisition": replicate the environment used in arXiv:1703.09831 (used with conf file ```<xworld_path>/games/xworld/confs/navigation.json``` and dictionary ```<xworld_path>/games/xworld/dicts/nav_dict.txt```);

    "arxiv_interactive": replicate the environment used in arXiv:1705.09906 (used with conf file ```<xworld_path>/games/xworld/confs/lang.json``` and dictionary ```<xworld_path>/games/xworld/dicts/lang_dict.txt```);

    "one_channel": integrate the above two environments into a single one.

    (Default: "one_channel")

* ```task_groups_exclusive``` (**c**), can be extended to any of (**a-f**) that incorporates a teacher

    In XWorld2D, whether the agent handles multiple tasks simultaneously (false) or not (true). (Default: true)

* ```context``` (**c-d**)

    How many consecutive frames are used to represent the current state. (Default: 1)

* ```ale_rom``` (**d**)

    The Atari ROM file path. You need to download the ROMs (.bin files) yourself. (Default: "")

# Code for training XWorld2D
Currently there is a PyTorch [reimplementation](https://github.com/zihangdai/pytorch_xworld) (by [@zihangdai](https://github.com/zihangdai)) of the framework used in arXiv:1703.09831. The code runs on the whole vocabulary and does not include the zero-shot experiments, but you can choose to do so according to the zero-shot setups discussed in the paper.

# Write your own XWorld2D tasks
You can customize XWorld2D tasks in a flexible way. To define a new task, you need three things:
1. A Python class that defines the map
  * This class must be defined in a file with the same name and put in
  ```
  <xworld_path>/games/xworld/maps/
  ```
  The class has to inherit from the base class ```XWorldEnv``` (defined in ```xworld_env.py```) and overwrite the member function ```_configure``` to specify how the map is configured. For an example, please take a look at ```XWorldNav.py```.

2. A Python class that defines the task
  * This class must be defined in a file with the same name and put in
  ```
  <xworld_path>/games/xworld/tasks/
  ```
  The class has to inherit from the base class ```XWorldTask``` (defined in ```xworld_task.py```). For an example, please take a look at ```XWorldNavTarget.py```.

3. A JSON conf file. This file specifies three aspects of the world:
  * ```item_path```: where the icon images are stored. Change this variable if you have new icons.
  * ```map``` : the name of the Python class that defines the map. It should be one of the Python defined maps.
  * ```task_groups``` : how the teacher assigns multiple tasks to the agent. Each task should be one of the Python defined tasks.
  For an example, please take a look at ```<xworld_path>/confs/walls.json```.

Generally, the teacher can dynamically change the environment at every time step, potentially according to the agent's performance and/or behaviors, which is important if you want to implement curriculum learning. The teacher's sentences can be generated by a context-free grammar (```<xworld_path>/python/context_free_grammar.py```) at each time step of each task. You have to define the grammar and decide when to generate what sentence when writing a task.

# Citations
If you use the XWorld2D environment for your research, please consider citing

* Haonan Yu, Haichao Zhang, Wei Xu, [*A Deep Compositional Framework for Human-like Language Acquisition in Virtual Environment*](https://arxiv.org/abs/1703.09831), arXiv:1703.09831, 2017.
* Haichao Zhang, Haonan Yu, Wei Xu, [*Listen, Interact and Talk: Learning to Speak via Interaction*](https://arxiv.org/abs/1705.09906), arXiv:1705.09906, 2017.

If you use our wrappers for the third-party simulators, please follow their original guide for citation.

# License
This repository has the Apache2.0 license, except that the third-party simulator ALE has its own license.
