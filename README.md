## Description

Yada Yada this is a semester project we'll make this more official later i promise.


Idea: having the arm hold a plate, on which a ball resides, and to try to move it around a pre determined obstacle, to a predetermined goal.

- The first challenge we would face with this project is planning a path where we have a constraint that is not just robot-robot and robot-obstacle collisions, but also an allowed angle of the plate, that may have some leeway. (we are not entirely sure if this is non trivial however).
- The second challenge we want to try to explore, is using the real-sense camera to check how far the ball is from the center of the plate, and adjusting the angle to prevent it from falling out. this closed loop is akin to a local avoidance among a global planner.

### TODO List:
1. Conifgure the masks in plate_detect() to be able to detect the plate
2. Check the correctness of kinematics.py functions
3. Try executing a path with the help of the kinematics.py functionality while keeping the ball balanced

## Requirements

pip install -r requirements.txt


## Dependencies

### Linux Installation
sudo apt-get install libblas-dev liblapack-dev

pip install --user numpy Cython

### Windows Installation

#### * this could be a little buggy as we only tested the linux installation. But go to your C:\ and clone the following


git clone https://github.com/Microsoft/vcpkg.git

cd vcpkg

.\bootstrap-vcpkg.bat

#### * after this add your vcpkg path to PATH Environment Variable and restart

vcpkg install lapack:x64-windows

vcpkg install blas:x64-windows

vcpkg integrate install

pip install --user numpy Cython


### Kinematics Library Download

git clone https://github.com/cambel/ur_ikfast.git

cd ur_ikfast

pip install -e .
