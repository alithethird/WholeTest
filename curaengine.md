# Build CuraEngine on Raspbian or Ubuntu

[CuraEngine](https://github.com/Ultimaker/CuraEngine) requires
[libArcus](https://github.com/Ultimaker/libArcus) which requires
[protobuf](https://github.com/google/protobuf). Let's build them in turn.

Make sure your `python3` is Python 3.8.10
[install python 3.8.10](https://computingforgeeks.com/how-to-install-python-on-ubuntu-linux-system/)
Make sure `cmake` needs to be at least `3.20.0`
I used `3.20.2`
[install cmake 3.20.2](https://graspingtech.com/upgrade-cmake/)

Install utilities used by the build process:
```
sudo apt-get install dh-autoreconf cmake git
```

Additionally, protobuf requires `python3-setuptools` and libArcus requires
`python3-sip-dev`. Install them:
```
sudo apt-get install python3-setuptools python3-sip-dev
```

I used `~/Ambots/slicerTools` as my main directory

## protobuf

Download protobuf's [release 3.17.1](https://github.com/google/protobuf/releases).
Choose the one with Python. The filename should be like
`protobuf-python-N.N.N.tar.gz`, where `N.N.N` is the version number.

I use `3.17.1` as an example. You should use this version.
```
cd ~/Ambots/slicerTools
wget https://github.com/google/protobuf/releases/download/v3.17.1/protobuf-python-3.17.1.tar.gz
tar zxf protobuf-python-3.17.1.tar.gz
```

Build and install for C++:
```
cd protobuf-3.17.1
./autogen.sh
./configure
make
sudo make install
```

Build and install for Python:
```
cd python
python3 setup.py build
sudo python3 setup.py install
```

Make sure shared libraries can be found:
```
sudo ldconfig
```

## libArcus

Clone repository:
```
cd ~/Ambots/slicerTools
git clone https://github.com/Ultimaker/libArcus.git
```

Build and install:
```
cd libArcus
mkdir build
cd build
cmake ..
make
sudo make install
```

Make sure shared libraries can be found:
```
sudo ldconfig
```

## CuraEngine, finally

Clone repository:
```
cd ~/Ambots/slicerTools
git clone https://github.com/Ultimaker/CuraEngine.git
```

Build:
```
cd CuraEngine
mkdir build
cd build
cmake ..
make
```

There should be an executable `CuraEngine` in the directory. After [downloading 
a definition file](https://github.com/Ultimaker/Cura/issues/798), try it:
```
./CuraEngine slice -v -p -j fdmprinter.def.json -o example.gcode -l example.stl --next
```
