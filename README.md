# TrajOpt
Integration of TrajOpt into ROS

## Solvers support
`trajopt_ros` implements sequential convex optimization to solve the motion planning problem.
It implements a penalty method to optimize for joint velocities while satisfying a set of constraints.
Internally, it makes use of convex solvers that are able to solve linearly constrained quadratic problems.
At the moment, the following solvers are supported:
- `BPMPD` (interior point method, free for non-commercial use only)
- `Gurobi` (simplex and interior point/parallel barrier, license required)
- `OSQP` (ADMM, BSD2 license)
- `qpOASES` (active set, LGPL 2.1 license)

While the `BPMPD` library is bundled in the distribution, `Gurobi`, `OSQP` and `qpOASES` need to be installed in the system.
To compile with `Gurobi` support, a `GUROBI_HOME` variable needs to be defined.
Once `trajopt_ros` is compiled with support for a specific solver, you can select it by properly setting the `TRAJOPT_CONVEX_SOLVER` environment variable. Possible values are `GUROBI`, `BPMPD`, `OSQP`, `QPOASES`, `AUTO_SOLVER`.
The selection to `AUTO_SOLVER` is the default and automatically picks the best between the available solvers.

## Build Branch Sphinx Documentation

```
cd gh_pages
sphinx-build . output
```
Now open gh_pages/output/index.rst and remove *output* directory before commiting changes.

## Installation
- Install [tesseract](https://github.com/janneyow/tesseract)
  - In the same workspace:
    ```bash
    git clone https://github.com/ros-industrial/ros_industrial_cmake_boilerplate # this is required for tesseract
    git clone -b noetic https://github.com/janneyow/tesseract
    git submodule update --init --recursive
    ```
  - If pyconfig.h cannot be found, locate your python include path
    ```bash
    find /usr/include -name pyconfig.h 
      # returns /usr/include/python3.8/pyconfig.h
    export CPLUS_INCLUDE_PATH="$CPLUS_INCLUDE_PATH:/usr/include/python3.8/" 
      # you can add this line to your bashrc
    ```
    <!-- only for PyObject -->
    <!-- - trajopt_ros depends on torch in python, set up catkin config to import torch
    - no need this, just need to set pythonhome 
    ```
    catkin config -DPYTHON_EXECUTABLE=~/anaconda3/envs/mujoco/bin/python3.10 -DPYTHON_INCLUDE_DIR=~/anaconda3/envs/mujoco/include/python3.10 -DPYTHON_LIBRARY=~/anaconda3/envs/mujoco/lib/libpython3.10.so -DCMAKE_BUILD_TYPE=Release
    ``` -->

- Clone this package:
  - In the same workspace:
  ```bash
      git clone -b noetic https://github.com/janneyow/trajopt
  ```

- To build:
```bash
# to speed up trajopt
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
```

When running in Ubuntu 20.04 (*Note: these have been updated in this repo*)
1. Need to update tesseract/tesseract_rviz/src/render_tools/env/robot_link.cpp
- Change "rviz" to Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME
1. Update compile options to c++14


## Running an example
```bash
roslaunch trajopt_examples test_trajopt.launch 
```

## JSON Config files
- [Industrial training/Tesseract](https://industrial-training-master.readthedocs.io/en/melodic/_source/demo3/Introduction-to-trajopt.html)
- [Original trajopt](https://rll.berkeley.edu/trajopt/doc/sphinx_build/html/tutorial.html#move-arm-to-pose-target)
    - *Note some original functionalities were not transferred to the tesseract implementation*

Basic info:
- **n_steps** (int): number of waypoints generated in the trajectory
- **manip** (str): name of the robot manipulator to plan for
- **start_fixed** (bool): whether to force the first trajectory state to be the first state given


Init info:
*Note start state is given by ferl_mj's config file*
- Increase reliability with multiple initializations to decrease probability of converging to a local minimum that is not collision-free.
- **type**(str): type of initialization. valid values are
    - *"stationary"*: initializes entire trajectory to current joint states of the robot. No data is needed
    - *"given_traj"*: the entire initial trajectory must be provided in the **data** member
    - *"joint_interpolated"*: the **endpoint** member is required. the trajectory is the joint interpolated between the current state and the endpoint
- **data** (trajArray, optional): Array containing the initialization information
- **endpoint** (array, optional): joint states for the end point

Costs and Constraints: *Refer to problem_description.hpp for their term_type*
- **type**
    - **joint_pos**
    - **joint_vel**
    - **joint_acc**
    - **joint_jerk**
- *Only added as costs in Tesseract's implementation, original author's implementation only includes the following as constraints* 
    - **cart_pose** 
    - **dynamic_cart_pose** - for when the goal frame is not fixed in space
    - **cart_vel**
    - **collision**
    - **total_time**


### Examples
1. Moving arm to a joint-position target, set constraints and init_info
```
...
"constraints" :
  [
    {
      "type": "joint_pos",
      "params":{
        "targets": [0.5, -0.6, 0, 0.5, -1.3988, -0.2]
      }
    }
  ],
  "init_info" :
  {
    "type" : "joint_interpolated",
    "endpoint": [0.5, -0.6, 0, 0.5, -1.3988, -0.2]
  }
```
2. Moving arm to one or many pose targets, set constraints. Init info can be left as **"stationary"**
- Alternatively, provide a joint space trajectory as a seed in init_info, then include the cartesian pose targets as costs. 

```
...
 "constraints" :
  [
    {
      "name" : "waypoint_cart_1",
      "type" : "cart_pose",
      "params" :
      {
        "timestep" : 0,
        "xyz" : [0.2, 0.0, -0.112],
        "wxyz" : [0.0, 1.0, 0.0, 0.0],
        "link" : "link_eef",
        "pos_coeffs" : [10, 10, 10],
        "rot_coeffs" : [10, 10, 10]
      }
    },
    {
      "name" : "waypoint_cart_2",
      "type" : "cart_pose",
      "params" :
      {
        "timestep" : 1,
        "xyz" : [0.2, 0.2, -0.112],
        "wxyz" : [0.0, 1.0, 0.0, 0.0],
        "link" : "link_eef",
        "pos_coeffs" : [10, 10, 10],
        "rot_coeffs" : [10, 10, 10]
      }
    },
    {
      "name" : "waypoint_cart_3",
      "type" : "cart_pose",
      "params" :
      {
        "timestep" : 2,
        "xyz" : [0.2, 0.2, 0.112],
        "wxyz" : [0.0, 1.0, 0.0, 0.0],
        "link" : "link_eef",
        "pos_coeffs" : [10, 10, 10],
        "rot_coeffs" : [10, 10, 10]
      }
    }
  ],
  "init_info" :
  {
    "type" : "stationary"
  }
```

## Trajoptpy
Uses the Boost Python library to generate python bindings.
- [Reference](http://wiki.ros.org/ROS/Tutorials/Using%20a%20C%2B%2B%20class%20in%20Python) for the CMakeLists.txt 
- trajoptpy.cpp contains the wrapper

