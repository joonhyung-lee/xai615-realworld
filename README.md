# xai615-realworld
This repo provides minimal hands-on code for the class XAI615 in RealWorld Robot for Pick-n-Place Demo.

MuJoCo related code is employed from following DeepMind's repos: 

* [MuJoCo] https://github.com/deepmind/mujoco
* [Perception] https://github.com/NVlabs/UnseenObjectClustering
* [UniversalRobot] https://github.com/ros-industrial/ur_modern_driver

## Prerequisites

This repo is tested on following environment:

* Ubuntu: 20.04
* Python: 3.8.10
* mujoco: 2.3.2

### Install dependencies

Mujoco Engine
```bash
pip install mujoco

pip install mujoco-python-viewer
```

RG2 - Gripper
```bash
pip install pymodbus==2.4.0
```

### Descriptions
Below is a list of files and their descriptions:

* Realworld
    1. [Gripper](https://github.com/joonhyung-lee/xai615-realworld/blob/main/code/realworld/demo_realworld_gripper.ipynb): OnRobot RG2 Gripper usage
    2. [Get Topic](https://github.com/joonhyung-lee/xai615-realworld/blob/main/code/realworld/demo_realworld_get_topic.ipynb): Get Topic `/joint_states`(=current joint value)
    3. [Solve IK](https://github.com/joonhyung-lee/xai615-realworld/blob/main/code/realworld/demo_realworld_solve_ik.ipynb): Solve Inverse Kinematics
    4. [Perception](https://github.com/joonhyung-lee/xai615-realworld/blob/main/code/realworld/demo_realworld_perception.ipynb): Segmentation using UnseenObjectClustering method
    5. [Pick-n-Place](https://github.com/joonhyung-lee/xai615-realworld/blob/main/code/realworld/demo_realworld_pick_n_place.ipynb): Pick-n-Place Demo

* Simulation
    1. Solve inverse kinematics in various method with [[General](https://github.com/joonhyung-lee/xai615-realworld/blob/main/code/simulation/demo_sim_solve_ik.ipynb), [Augmented](https://github.com/joonhyung-lee/xai615-realworld/blob/main/code/simulation/demo_sim_solve_ik_augmented.ipynb), [Nullspace projection](https://github.com/joonhyung-lee/xai615-realworld/blob/main/code/simulation/demo_sim_solve_ik_nullspace_projection.ipynb), [Repulse](https://github.com/joonhyung-lee/xai615-realworld/blob/main/code/simulation/demo_sim_solve_ik_repelling.ipynb), [RRT-Star](https://github.com/joonhyung-lee/xai615-realworld/blob/main/code/simulation/demo_sim_solve_ik_rrt.ipynb)]
    2. [Trajectory planning method](https://github.com/joonhyung-lee/xai615-realworld/blob/main/code/simulation/demo_sim_trajectory_planning.ipynb): [Quintic, Minimum Jerk, Linear movement]
    3. [Velocity profile method](https://github.com/joonhyung-lee/xai615-realworld/blob/main/code/simulation/demo_sim_trapezoidal_spline.ipynb): [Trapezoidal, s-Spline method]

* Final Demo
    1. [Stack object](https://github.com/joonhyung-lee/xai615-realworld/blob/main/code/final_demo.py): Stack one object over another.
    2. [Pick-n-Place only one object](https://github.com/joonhyung-lee/xai615-realworld/blob/main/code/final_demo_pick-n-place.ipynb): Pick-n-Place only one object via Finite-State-Machine.
    3. [Pick-n-Place multiple objects](https://github.com/joonhyung-lee/xai615-realworld/blob/main/code/final_demo_pick-n-place_mutiple.ipynb): Pick-n-Place multiple objects.
    4. [Pick-n-Place multiple objects only using mujoco ](https://github.com/joonhyung-lee/xai615-realworld/blob/main/code/final_demo_sim_pick-n-place_mutiple.ipynb): Pick-n-Place multiple objects using only mujoco simulator