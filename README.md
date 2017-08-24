# XWorld: a simulator package for Reinforcement Learning
This repository currently contains a collection of simulators for Reinforcement Learning research.

|**Name**|**Description**|**Thread-compatible?***|**Optional?**|**Build-on-the-fly?**|
|:--------------|---------------|:---------------:|:---------------:|:---------------:|
|**SimpleGame**|A simple 1D array-walking game.|Yes|No|Yes|
|**SimpleRace**|A simple synthetic car racing game.|Yes|No|Yes|
|**XWorld2D**|A 2D environment for an agent to learn vision and language abilities.|Yes|No|Yes|
|**Atari**|Wrappers for the Arcade Learning Environment ([ALE](http://www.arcadelearningenvironment.org/)) environment. For stability, we use a fork version.|Yes|No|Yes|
|**Malmo**|Wrappers for Microsoft [Malmo](https://github.com/Microsoft/malmo) Project.|*TBD*|Yes|No, has to be installed manually first.|
|**DeepMind Lab**|Wrappers for DeepMind [Lab](https://github.com/deepmind/lab). We use a fork version of the original repository to address a [compiling issue](https://github.com/deepmind/lab/pull/71).|Yes|Yes|Yes|

*If each thread has a different environment instance, then they don't interfere with each other. But if all threads share the same instance, then there is interference.

# Dependencies
The following softwares must be installed before building XWorld.
* [Boost](http://www.boost.org/)
* [Glog](https://github.com/google/glog)
* [GFlags](https://github.com/gflags/gflags)
* [GTest](https://github.com/google/googletest)

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
cmake -DWITH_DEEPMIND_LAB=ON ..
```

Finally, in the build directory do
```
make
make test
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

# Citations
If you use the XWorld2D environment for your research, please consider citing

* Haonan Yu, Haichao Zhang, Wei Xu, [*A Deep Compositional Framework for Human-like Language Acquisition in Virtual Environment*](https://arxiv.org/abs/1703.09831), arXiv 1703.09831, 2017.
* Haichao Zhang, Haonan Yu, Wei Xu, [*Listen, Interact and Talk: Learning to Speak via Interaction*](https://arxiv.org/abs/1705.09906), arXiv 1705.09906, 2017.

If you use our wrappers for the third-party simulators, please follow their original guide for citation.

# License
This repository has the Apache2.0 license, except that all the third-party simulators (i.e., ALE, Malmo, DeepMind Lab) have their own licenses.
