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

if you have error:
```
ubuntu@ubuntu:~/Ambots/slicerTools/libArcus/build$ cmake ..
-- The C compiler identification is GNU 9.4.0
-- The CXX compiler identification is GNU 9.4.0
-- Detecting C compiler ABI info
-- Detecting C compiler ABI info - done
-- Check for working C compiler: /usr/bin/cc - skipped
-- Detecting C compile features
-- Detecting C compile features - done
-- Detecting CXX compiler ABI info
-- Detecting CXX compiler ABI info - done
-- Check for working CXX compiler: /usr/bin/c++ - skipped
-- Detecting CXX compile features
-- Detecting CXX compile features - done
-- Setting BUILD_SHARED_LIBS to ON
-- Setting build type to 'Release' as none was specified.
-- Generating compile commands to /home/ubuntu/Ambots/slicerTools/libArcus/build/compile_commands.json
-- Setting POSITION_INDEPENDENT_CODE: ON
-- Found Protobuf: /usr/local/lib/libprotobuf.so;-lpthread (found suitable version "3.17.1", minimum required is "3.17.1") 
-- Enabling threading support for Arcus
-- Check if compiler accepts -pthread
-- Check if compiler accepts -pthread - yes
-- Found Threads: TRUE  
-- Setting Python version to 3.10. Set Python_VERSION if you want to compile against an other version.
CMake Error at /usr/local/share/cmake-3.20/Modules/FindPackageHandleStandardArgs.cmake:230 (message):
  Could NOT find Python: Found unsuitable version "3.8.10", but required is
  exact version "3.10" (found /usr/bin/python3, found components: Interpreter
  Development Development.Module Development.Embed)
Call Stack (most recent call first):
  /usr/local/share/cmake-3.20/Modules/FindPackageHandleStandardArgs.cmake:592 (_FPHSA_FAILURE_MESSAGE)
  /usr/local/share/cmake-3.20/Modules/FindPython/Support.cmake:3165 (find_package_handle_standard_args)
  /usr/local/share/cmake-3.20/Modules/FindPython.cmake:514 (include)
  CMakeLists.txt:77 (find_package)


-- Configuring incomplete, errors occurred!
See also "/home/ubuntu/Ambots/slicerTools/libArcus/build/CMakeFiles/CMakeOutput.log".
See also "/home/ubuntu/Ambots/slicerTools/libArcus/build/CMakeFiles/CMakeError.log".
```
askes for python 3.10
how to install 3.10
[python 3.10](https://computingforgeeks.com/how-to-install-python-on-ubuntu-linux-system/)
After that you also need to instal distutils for python 3.10
```
sudo apt install python3.10-distutils
sudo apt install python3.10-dev 
```
Right now you should have 2 different python3 directories
and then you will use this command to call to the cmake

```
cmake .. -DPYTHON_INCLUDE_DIR= /usr/include/python3.10 -DPYTHON_LIBRARY= /usr/lib/aarch64-linux-gnu

```
after doing that the error should change to 
```
ubuntu@ubuntu:~/Ambots/slicerTools/libArcus/build$ cmake .. -DPYTHON_INCLUDE_DIR= /usr/include/python3.10 -DPYTHON_LIBRARY= /usr/lib/aarch64-linux-gnu
-- Setting BUILD_SHARED_LIBS to ON
-- Generating compile commands to /home/ubuntu/Ambots/slicerTools/libArcus/build/compile_commands.json
-- Setting POSITION_INDEPENDENT_CODE: ON
-- Enabling threading support for Arcus
-- Found Python: /usr/bin/python3.10 (found suitable exact version "3.10.5") found components: Interpreter Development Development.Module Development.Embed 
-- Linking and building  against Python 3.10.5
Traceback (most recent call last):
  File "/home/ubuntu/Ambots/slicerTools/libArcus/cmake/FindSIP.py", line 34, in <module>
    import sipbuild
ModuleNotFoundError: No module named 'sipbuild'

During handling of the above exception, another exception occurred:

Traceback (most recent call last):
  File "/home/ubuntu/Ambots/slicerTools/libArcus/cmake/FindSIP.py", line 44, in <module>
    import sipconfig
ModuleNotFoundError: No module named 'sipconfig'
CMake Error at cmake/FindSIP.cmake:58 (MESSAGE):
  Could not find SIP
Call Stack (most recent call first):
  CMakeLists.txt:80 (find_package)


-- Configuring incomplete, errors occurred!
See also "/home/ubuntu/Ambots/slicerTools/libArcus/build/CMakeFiles/CMakeOutput.log".
See also "/home/ubuntu/Ambots/slicerTools/libArcus/build/CMakeFiles/CMakeError.log".
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
