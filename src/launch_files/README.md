# ROS 2 Launch files
### Sintax:
```sh
ros2 launch $PACKAGE_NAME $LAUNCH_FILE ($PARAMETER:=$VALUE)*
```

#### quick_simulation_launch
```sh
ros2 launch launch_files quick_simulation_launch.py
ros2 launch launch_files quick_simulation_launch.py interface:=False side:=left color:=yellow n_robots:=3
```
#### sim_main_launch
```sh
ros2 launch launch_files sim_main_launch.py
ros2 launch launch_files sim_main_launch.py interface:=True side:=left color:=yellow n_robots:=3
```