# MADER: Fast and Safe Trajectory Planner for Flights in Unknown Environments #

For one drone:
```
roslaunch mader all.launch
roslaunch mader mader.launch
```

For many drones:
```
roslaunch mader mader_general.launch
roslaunch mader many_drones.launch action:=start
roslaunch mader many_drones.launch action:=mader
(Take off)
roslaunch mader many_drones.launch action:=send_goal
```

Note that 'CGal' takes a lot to compile if you use RelWithDebugInfo. Use Release instead (see [this](https://www.cgal.org/FAQ.html#compilation_speed)  )

Code used for the paper **MADER: Fast and Safe Trajectory Planner for Flights in Unknown Environments** (IROS 2019) ([pdf](https://arxiv.org/abs/1903.03558), [video](https://www.youtube.com/watch?v=gwV0YRs5IWs))

```
@inproceedings{tordesillas2019mader,
  title={{MADER}: },
  author={Tordesillas, Jesus and How, Jonathan P},
  booktitle={2019 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  year={2019},
  organization={IEEE}
}

```

## Instructions
Install [Gurobi](https://www.gurobi.com/) (you can test your installation typing `gurobi.sh` in the terminal).

Create a workspace, clone this repo and its dependencies, and compile the workspace:
```
mkdir ws && cd ws && mkdir src && cd src
git clone https://github.com/mit-acl/mader.git
wstool init
wstool merge ./mader/mader/install/mader.rosinstall
wstool update -j8
cd ..
catkin config -DCMAKE_BUILD_TYPE=Release
catkin build
```

And finally open 5 terminals and execute these commands:
```
roslaunch acl_sim start_world.launch
roslaunch acl_sim perfect_tracker_and_sim.launch
roslaunch global_mapper_ros global_mapper_node.launch
roslaunch mader mader_interface.launch
roslaunch mader mader.launch
```
Now you can click `Takeoff` in the GUI, and then, in RVIZ, press `G` (or click the option `2D Nav Goal` on the top bar of RVIZ) and click any goal for the drone. 

## Architecture:


![](./mader/imgs/diagram.png) 


## Credits:
This package uses code from the [JPS3D](https://github.com/KumarRobotics/jps3d) and [DecompROS](https://github.com/sikang/DecompROS) repos (included in the `thirdparty` folder), so credit to them as well. 





