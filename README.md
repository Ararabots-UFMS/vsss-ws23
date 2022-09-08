# Ararabots Verysmall - LIA

## Project Repository:
* Robot
* Vision
* Logical Board
* Interfaces

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
        - **Movment bahaviours:** This file contains the true leaf ands the most low level nodes of the behaviour trees and all of them outputs a task status and an ACTION TUPLE, the ACTION TUPLE is the common interface between the behaviour and the Control inside the Robot Module, it consists of (OpCodes, angle, speed, distance). These four parameters are used to model every robot movment.
