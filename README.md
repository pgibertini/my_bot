## Robot Package

This is a simulation a package for running simulation on a differential robot.
It uses the template provided my Articulated Robotics - a huge thanks to him and his awesome tutorials!

![Robot display in rviz2](https://i.imgur.com/0iKLL6t.png)

*The robot has a lidar on the top and a camera at the front.*

## Install

1. Clone the repo in the `src` directory of your ROS workspace.
2. Build the project: `colcon build`
> You can use the `--symlink-install` argument to not have to re-build after every change (if you add a new file, you will have to re-build anyway).
4. Source the directory: `source install/setup.bash` (from the root of your ROS workspace)

## Usage

### Lauch the simulation

```bash
ros2 launch my_bot launch_sim.launch.py
```

> You can load a provided world using the argument `world:=src/my_bot/worlds/obstacles.world`

```bash
ros2 launch my_bot launch_sim.launch.py world:=src/my_bot/worlds/obstacles.world
```

Run `rviz2` to visualize the different topics of the simulation (`RobotModel`, `LaserScan`, `Image`).

> You can also load the `sensors.rviz` config saved in `config/rviz` directory.

### Slam the world

With the simulation launched, you can slam the world using slam toolbox with the following command:
```bash
ros2 launch slam_toolbox online_async_launch.py params_file:=./src/my_bot/config/mapper_params_online_async.yaml use_sim_time:=true
```
This will allow you to create a map of your world using the lidar of the robot. Add a `Map` topic in `rviz2` to see the map beeing created.

> Alternatively, you can use the `slam.rviz` config saved in `config/rviz` directory.

To slam the map, you will have to control the bot manually using the teleop twist keyboard.

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard 
```

### Save a map

Then, you can save the map using 
```bash
ros2 run nav2_map_server map_saver_cli -f my_map
```

This will save you map as `my_map` in the current directory.

### Load a map

The launch script `load_map.launch.py` allows you to load a saved map.
```bash
ros2 launch my_bot load_map.launch.py 
```

By default, it will load the map called `labyrinth` in the config directory.

To see the map in `rviz2`, select `map` as Fixed Frame and `Transient Local` as the durability policy in the Update Topic section of the Map topic. 

> Alternatively, you can use the `map.rviz` config saved in `config/rviz` directory.

### Navigate using `nav2`

After closing all you terminals, you can use the `launch_nav` launch script to navigate throw the labyrinth using `nav2` plugin. 

```bash
ros2 launch my_bot launch_nav.launch.py 
```

> By default, it will load the `labyrinth` world and load the associated map on the map topic. You can edit the `launch_nav.launch.py` file to change that.

Launch `rviz2` and use the `nav.rviz` config saved in `config/rviz` directory.

You can select a `2D Goal Pose` on the map to have the robot navigate to it.
