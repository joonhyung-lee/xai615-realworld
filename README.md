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