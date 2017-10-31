Roboschool
==========

First, define a `ROBOSCHOOL_PATH` variable in the current shell. It will be used in this README but not anywhere in the Roboschool code.

```bash
ROBOSCHOOL_PATH=/path/to/roboschool
```

```bash
apt install cmake ffmpeg pkg-config libpython3.5-dev libboost-python-dev libtinyxml-dev
```
Install Qt 5.5 from here:
https://download.qt.io/archive/qt/5.5/5.5.1/qt-opensource-linux-x64-5.5.1.run

Download and build Assimp from here:
https://github.com/assimp/assimp

Compile and install bullet as follows. Note that `make install` will merely copy files into the roboschool directory.

```bash
git clone https://github.com/olegklimov/bullet3 -b roboschool_self_collision
mkdir bullet3/build
cd    bullet3/build
cmake -DBUILD_SHARED_LIBS=ON -DUSE_DOUBLE_PRECISION=1 -DCMAKE_INSTALL_PREFIX:PATH=$ROBOSCHOOL_PATH/roboschool/cpp-household/bullet_local_install -DBUILD_CPU_DEMOS=OFF -DBUILD_BULLET2_DEMOS=OFF -DBUILD_EXTRAS=OFF  -DBUILD_UNIT_TESTS=OFF -DBUILD_CLSOCKET=OFF -DBUILD_ENET=OFF -DBUILD_OPENGL3_DEMOS=OFF ..
make -j4
make install
cd ../..
```

Finally, install project itself: __NOTE__ for Linux user, please set the compiler to gcc-5 before using pip to install.
```bash
pip3 install --user -e $ROBOSCHOOL_PATH
cd $ROBOSCHOOL_PATH/roboschool/cpp-househould
sh make.sh
```

XWorld3D
========
```bash
mkdir build
cd build
cmake -DROBOSCHOOL_ROOT=$ROBOSCHOOL_PATH ..
make -j4
```

Download 3d models and glsl codes from the following links and put the folders under games/xworld3d/
- 3d models: https://www.dropbox.com/sh/4tulfzgybejinnf/AACGQ0bhFfHwR1JxfntX9ynpa?dl=0
- glsl codes: https://www.dropbox.com/sh/pc7yhg8jod0ganx/AABXkGiIYrgTaBJpPgmp9OFaa?dl=0


