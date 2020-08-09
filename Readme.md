# MADER: Trajectory Planner in Multi-Agent and Dynamic Environments #

Single-Agent               |  Multi-Agent           | 
:-------------------------:|:-------------------------:|
[![MADER: Trajectory Planner in Multi-Agent and Dynamic Environments](./mader/imgs/single_agent1.gif)](https://www.youtube.com/user/AerospaceControlsLab "MADER: Trajectory Planner in Multi-Agent and Dynamic Environments")      |  [![MADER: Trajectory Planner in Multi-Agent and Dynamic Environments](./mader/imgs/circle.gif)](https://www.youtube.com/user/AerospaceControlsLab "MADER: Trajectory Planner in Multi-Agent and Dynamic Environments") |  
[![MADER: Trajectory Planner in Multi-Agent and Dynamic Environments](./mader/imgs/single_agent2.gif)](https://www.youtube.com/user/AerospaceControlsLab "MADER: Trajectory Planner in Multi-Agent and Dynamic Environments")       |  [![MADER: Trajectory Planner in Multi-Agent and Dynamic Environments](./mader/imgs/sphere.gif)](https://www.youtube.com/user/AerospaceControlsLab "MADER: Trajectory Planner in Multi-Agent and Dynamic Environments")    |  

## Citation

When using MADER, please cite [this paper](https://www.google.com/):

```bibtex
@inproceedings{tordesillas2019faster,
  title={{MADER}: Trajectory Planner in Multi-Agent and Dynamic Environments},
  author={Tordesillas, Jesus and How, Jonathan P},
  journal={arXiv preprint},
  year={2020}
}
```

## General Setup
MADER has been tested with 
* Ubuntu 18.04/ROS Melodic 

### Dependencies

Install [NLopt](https://nlopt.readthedocs.io/en/latest/) following [these instructions](https://nlopt.readthedocs.io/en/latest/#download-and-installation) (MADER has been tested with NLopt v2.6.2).

Install v4.12.4 of [CGAL](https://www.cgal.org/): 

```
sudo apt-get install libgmp3-dev libmpfr-dev
wget https://github.com/CGAL/cgal/releases/download/releases%2FCGAL-4.14.2/CGAL-4.14.2.tar.xz
tar -xf CGAL-4.14.2.tar.xz
cd CGAL-4.14.2/
cmake . -DCMAKE_BUILD_TYPE=Release
sudo make install
```

### Compilation

Create a workspace, and clone this repo and its submodules:
```
mkdir ws && cd ws && mkdir src && cd src
git clone https://github.com/mit-acl/mader.git
cd mader && git submodule init && git submodule update
cd ..
catkin config -DCMAKE_BUILD_TYPE=Release
catkin build
```

Now add this to your `~/.bashrc`: 
```
source PATH_TO_YOUR_WS/devel/setup.bash
```

### Running Simulations

#### Single-agent
```
roslaunch mader single_agent_simulation.launch
```

#### Multi-agent
```
roslaunch mader single_agent_simulation.launch
```

And finally open 5 terminals and execute these commands:
```
roslaunch acl_sim start_world.launch
roslaunch acl_sim perfect_tracker_and_sim.launch
roslaunch global_mapper_ros global_mapper_node.launch
roslaunch faster faster_interface.launch
roslaunch faster faster.launch
```


## Credits:
This package uses some C++ classes from the [JPS3D](https://github.com/KumarRobotics/jps3d) and [DecompROS](https://github.com/sikang/DecompROS) repos (included in the `thirdparty` folder), so credit to them as well. 

