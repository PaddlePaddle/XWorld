# <img src="doc/xworld_logo.png" alt="XWorld">
This repository currently contains a collection of simulators for Reinforcement Learning research.

|**Name**|**Description**|**Thread-compatible?***|**Optional?**|**Build-on-the-fly?**|
|:--------------|---------------|:---------------:|:---------------:|:---------------:|
|**SimpleGame**|A simple 1D array-walking game.|Yes|No|Yes|
|**SimpleRace**|A simple synthetic car racing game.|Yes|No|Yes|
|**XWorld2D**|A 2D environment for an agent to learn vision and language abilities.|Yes|No|Yes|
|**Atari**|Wrappers for the Arcade Learning Environment ([ALE](http://www.arcadelearningenvironment.org/)) environment. For stability, we use a fork version.|Yes|No|Yes|
|**Malmo**|Wrappers for Microsoft [Malmo](https://github.com/Microsoft/malmo) Project.|*TBD*|Yes|No, Malmo has to be installed manually first.|
|**DeepMind Lab**|Wrappers for DeepMind [Lab](https://github.com/deepmind/lab). We use a fork version of the original repository to address a [compiling issue](https://github.com/deepmind/lab/pull/71).|Yes|Yes|No, external dependencies of DeepMind Lab have to be manually installed first|

*If each thread has a different environment instance, then they don't interfere with each other. But if all threads share the same instance, then there is interference.

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
sudo apt-get install libboost-all-dev libgflags-dev libgoogle-glog-dev libgtest-dev
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

By default, XWorld only builds the first four games: SimpleGame, SimpleRace, Atari, and XWorld2D. Optionally, you can install [DeepMind Lab](https://deepmind.com/blog/open-sourcing-deepmind-lab) and [Project Malmo](https://www.microsoft.com/en-us/research/project/project-malmo).

To use DeepMind Lab, please install the external dependencies by following the instructions included in this [DeepMind Lab build documentation](https://github.com/deepmind/lab/blob/master/docs/build.md#how-to-build-deepmind-lab) and run the cmake command
```
cmake -DWITH_DEEPMIND_LAB=ON ..
```
which will automatically download and build DeepMind Lab.

To use Malmo, please first follow the instructions on [https://github.com/Microsoft/malmo](https://github.com/Microsoft/malmo) to install Malmo, and then run the cmake command
```
cmake -DMALMO_ROOT=/path/to/MALMO ..
```

# Use
You can compile a C++ project with XWorld. The C++ simulator APIs are located in simulator.h.
Alternatively, we provide a set of simple Python APIs for interacting with the simulators. After building XWorld, you need to export the path of the python module:
```
export PYTHONPATH=<xworld_path>/python:$PYTHONPATH
```
You can add the above line to ~/.bashrc to avoid doing the export in the future.

To get started, several examples of the simulator Python APIs can be found in
```
<xworld_path>/python/examples
```

# Instructions for creating games
Below we explain the flags that the user can set for each game. If you use Python APIs, these flags are passed in as a dictionary when creating the game (see <xworld_path>/python/examples for details). If you use C++ APIs, these flags are either set globally from command line with GFlags, or are passed as arguments of the class constructor (see <xworld_path>/examples for details).

|**ID**|**Python Keyword**|**C++ Class Name**|**Flags**|
|---|---|-----|-----------|
|**a**|```simple_game```|```SimpleGame```|```pause_screen```,```array_size```|
|**b**|```simple_race```|```SimpleRace```|```pause_screen```,```window_width```,```window_height```,```track_type```,```track_width```,```track_length```,```track_radius```,```race_full_manouver```,```random```,```difficulty```|
|**c**|```xworld```|```XWorldSimulator```|```pause_screen```,```conf_path```,```curriculum```,```task_mode```,```task_groups_exclusive```,```context```|
|**d**|```atari```|```ArcadeGame```|```pause_screen```,```ale_rom```,```context```|
|**e**|```minecraft```|```MinecraftSimulator```|```pause_screen```,```context```,```mission```,```conf_path```,```minecraft_client_ip```,```minecraft_client_port```|
|**f**|```deepmind_lab```|```DeepmindLabSimulatorBase```|```pause_screen```,```context```,```runfiles_path```,```level_script```|

The meanings of the above flags are listed below. Each flag applies to certain games indicated by the IDs.
* ```pause_screen``` (**a-f**)
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
    Difficulty level for Simple Race: "easy" level provides negative rewards when moving away from the center line of the track, "hard" level only provides rewards when out-of-boundary or reaching finish line. (Default: "easy")
* ```conf_path``` (**c**,**e**)
    The conf JSON file for XWorld2D, or the conf XML file for MALMO. (Default: "")
* ```curriculum``` (**c**), can be extended to any of (**a-f**) that employs curriculum learning
    This number indicates the maximum number of games that has curriculum learning. Given any difficulty range that can be quantized by two integers [*low*, *high*], if ```curriculum``` is 0, then the difficulty number is randomly selected from this range; otherwise, the difficulty is linearly increased from *low* to *high* until ```curriculum``` games have been played. (Default: 0)
* ```task_mode``` (**c**)
    This flag has three possible values.
    "arxiv_lang_acquisition": replicate the environment used in arXiv:1703.09831;
    "arxiv_interactive": replicate the environment used in arXiv:1705.09906;
    "one_channel": incoporate the above two environments into a single one.
    (Default: "one_channel")
* ```task_groups_exclusive``` (**c**), can be extended to any of (**a-f**) that incorporates a teacher
    In XWorld2D, whether the agent handles multiple tasks simultaneously (false) or not (true). (Default: true)
* ```context``` (**c-f**)
    How many consecutive frames are used to represent the current state. (Default: 1)
* ```ale_rom``` (**d**)
    The Atari ROM file path. (Default: "")
* ```mission``` (**e**)
    MALMO mission name, currently only support "demo". (Default: "demo")
* ```minecraft_client_ip``` (**e**)
    IP address of Minecraft mod. (Default: "127.0.0.1")
* ```minecraft_client_port``` (**e**)
    pport number of Minecraft mod. (Default: 10000)
* ```runfiles_path``` (**f**)
    Path of DeepMind Lab run files. (Default: "")
* ```level_script``` (**f**)
    The Lua level script for DeepMind Lab. (Default: "")

# Citations
If you use the XWorld2D environment for your research, please consider citing

* Haonan Yu, Haichao Zhang, Wei Xu, [*A Deep Compositional Framework for Human-like Language Acquisition in Virtual Environment*](https://arxiv.org/abs/1703.09831), arXiv 1703.09831, 2017.
* Haichao Zhang, Haonan Yu, Wei Xu, [*Listen, Interact and Talk: Learning to Speak via Interaction*](https://arxiv.org/abs/1705.09906), arXiv 1705.09906, 2017.

If you use our wrappers for the third-party simulators, please follow their original guide for citation.

# License
This repository has the Apache2.0 license, except that all the third-party simulators (i.e., ALE, Malmo, DeepMind Lab) have their own licenses.
