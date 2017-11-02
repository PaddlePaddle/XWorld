# XWorld3D

We use our fork of [Roboschool](https://blog.openai.com/roboschool/) to implement the XWorld3D environment.
```bash
git clone https://github.com/skylian/roboschool -b cpp_api
```

First, export `ROBOSCHOOL_PATH` for later use in the install process. In your `~/.bashrc` file, append the line

```bash
export ROBOSCHOOL_PATH=/path/to/roboschool
```

## Basic dependencies

Install [FFmpeg](https://www.ffmpeg.org/) and [TinyXML](http://www.grinninglizard.com/tinyxml/),

```bash
sudo apt install ffmpeg libtinyxml-dev
```

Roboschool requires gcc 5.x. On Ubuntu 16.04, this requirement is already satisfied. On Ubuntu 14.04, you need do
```bash
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo apt-get update
sudo apt-get install gcc-5 g++-5
```
The command ```update-alternatives``` can be used to switch between gcc 4.x and 5.x if necessary. (Note: you don't have to set gcc 5.x as the default compiler.)

## Qt
Install Qt 5.5:
```bash
cd /tmp/
wget https://download.qt.io/archive/qt/5.5/5.5.1/qt-opensource-linux-x64-5.5.1.run
chmod 755 qt-opensource-linux-x64-5.5.1.run
sudo ./qt-opensource-linux-x64-5.5.1.run
rm qt-opensource-linux-x64-5.5.1.run
```
(In the GUI installer, select "skip" for the account, and use the default installation settings.)


## Assimp

Install [Assimp](https://github.com/assimp/assimp) (suppose you have selected a location `[assimp_root]`):
```html
cd [assimp_root]
git clone https://github.com/assimp/assimp
cd assimp
```
Inside `assimp`, we need to modify `assimp.pc.in`. Change line 4 to
```
includedir=@CMAKE_INSTALL_PREFIX@/@ASSIMP_INCLUDE_INSTALL_DIR@
```
Then
```
cmake CMakeLists.txt -CMAKE_INSTALL_PREFIX=.
make
```

Suppose the locations that you install Qt and Assimp are ```[qt_path]``` and ```[assimp_root]/assimp```, respectively. You need now append their pkg-config directories to your ```.bashrc``` file:
```bash
(in ~/.bashrc)
export PKG_CONFIG_PATH=[qt_path]/5.5/gcc_64/lib/pkgconfig:$PKG_CONFIG_PATH
export PKG_CONFIG_PATH=[assimp_root]/assimp:$PKG_CONFIG_PATH
```

Then:
```bash
source ~/.bashrc
```

## Bullet Physics

Compile and install Bullet Physics. Note that `make install` will copy files into the Roboschool directory.

```bash
cd /tmp/
git clone https://github.com/olegklimov/bullet3 -b roboschool_self_collision
mkdir bullet3/build
cd    bullet3/build
cmake -DBUILD_SHARED_LIBS=ON -DUSE_DOUBLE_PRECISION=1 -DCMAKE_INSTALL_PREFIX:PATH=$ROBOSCHOOL_PATH/roboschool/cpp-household/bullet_local_install -DBUILD_CPU_DEMOS=OFF -DBUILD_BULLET2_DEMOS=OFF -DBUILD_EXTRAS=OFF  -DBUILD_UNIT_TESTS=OFF -DBUILD_CLSOCKET=OFF -DBUILD_ENET=OFF -DBUILD_OPENGL3_DEMOS=OFF ..
make -j4
make install
cd ../..; rm -rf bullet3
```

## Roboschool

Finally, install the Roboschool itself.
```bash
pip install --user -e $ROBOSCHOOL_PATH
cd $ROBOSCHOOL_PATH/roboschool/cpp-househould
sh make.sh
```

## Build

To enable the build of XWorld3D, you need to add an option when doing cmake in the `XWorld/build` directory:
```bash
cmake -DROBOSCHOOL_ROOT=$ROBOSCHOOL_PATH ..
```

## Download 3D models
```bash
cd <xworld_path>/games/xworld3d
bash ./download_3d_models.sh
```

## Test
```bash
cd <xworld_path>/examples; ./test_xworld3d
```
