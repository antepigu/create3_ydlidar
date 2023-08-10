# create3_ydlidar
Integration of YDLidar Tmini for iRobot Create3 in ROS2 Humble

## Build YDLidar-SDK
### Ubuntu 18.04/16.04/14.04 LTS
In the YDLidar SDK directory, run the following commands to compile the project:
```
git clone https://github.com/YDLIDAR/YDLidar-SDK.git
cd YDLidar-SDK/build
cmake ..
make
sudo make install
```
Note:
  If already installed python and swig, `sudo make install` command will also install python API without the following operations.

### python API install separtately:
The Next operation only installs the python API, if the above command has been executed, there is no need to perform the next operation.
```
cd YDLidar-SDK
pip install .

# Another method
python setup.py build
python setup.py install
```## Build YDLidar-SDK
### Ubuntu 18.04/16.04/14.04 LTS
In the YDLidar SDK directory, run the following commands to compile the project:
```
git clone https://github.com/YDLIDAR/YDLidar-SDK.git
cd YDLidar-SDK/build
cmake ..
make
sudo make install
```
Note:
  If already installed python and swig, `sudo make install` command will also install python API without the following operations.

### python API install separtately:
The Next operation only installs the python API, if the above command has been executed, there is no need to perform the next operation.
```
cd YDLidar-SDK
pip install .

# Another method
python setup.py build
python setup.py install
```