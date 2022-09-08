# Ararabots Verysmall - LIA
This is a Ros2 Workspace ported from the previous [VSS Repository](https://github.com/Ararabots-UFMS/vsss). 
The main package was splitted in to 12 packages:
- **interface:** Simple MVC Interface built upon the FLTK framework.
- **launch_files:** Contains only the launch files for quickly starting the system.
- **message_server(WIP):** Responsible for sending the messages from *message_server* topic to the **Bluetooth** Hardware.
- **parameters:** Not exacly a package since it only stores common json files.
- **referee(WIP):** Reads commands from the [Referee](https://github.com/VSSSLeague/VSSReferee) for publishing in the *game_topic*. 
- **robot:** The main robot node that reads from the *game_topic* and *things_position* topics, execute
- **sim_cam:**
- **sim_message_server:**
- **strategy:**
- **sys_interfaces:**
- **utils:**
- **vision:**

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

# Strategy
The strategy module aims to implement all the robots behaviours (goalkeeper, defender and attacker). Currently (2019) we use behaviour trees to model all the robots actions. For a getting started with behavior trees see: https://www.pirobot.org/blog/0030/.

- **Current implemented trees:** attack with univector

- ## **Submodules**
    - **Behaviour:** In this file there is the building blocks of our behaviour tree architecture. It has the implementation of the Sequence class wich represents a sequence node, the Selector class represents the Selector node and the Blackboard class contains all the variables that compose de game state.
    - **Base Trees:** The base trees file contains some base actions like the Penalty and the Freeball, that is, actions that every robot in the team must know to perform.
    - **Strategy utils:** This file contains lots of macros and helper functions that could me used to speed up the process of creating new actions.
    - **Actions:**
        - **Decorators:** The decorators file contains classes that "overrides" the default output of the run method of its inheriting childs. These are used when the Sequence and Selector nodes aren't sufficient t implement the desired action.
        - **State Behaviours:** The state behaviours file relates to logic state related things, like if the the current game state is equal to a target state. The change state class modifies the game state to a target state and returns TaskStatus.FAILURE so the tree can move on to the next node.
        - **Game behaviours:** The game behaviours has classes related to the phisical game state, like if a robot is or not behind the ball.
        - **Movement bahaviours:** This file contains the true leaf ands the most low level nodes of the behaviour trees and all of them outputs a task status and an ACTION TUPLE, the ACTION TUPLE is the common interface between the behaviour and the Control inside the Robot Module, it consists of (OpCodes, angle, speed, distance). These four parameters are used to model every robot movment.