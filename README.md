# Ararabots Verysmall - LIA
This is a Ros2 Workspace ported from the previous [VSS Repository](https://github.com/Ararabots-UFMS/vsss). 
The main package was splitted in to 14 packages:
- **interface:** Simple MVC Interface built upon the FLTK framework.
- **launch_files:** Contains only the launch files for quickly starting the system.
- **parameters:** Not exacly a package since it only stores common json files.
- **referee(WIP):** Reads commands from the [Referee](https://github.com/VSSSLeague/VSSReferee) for publishing in the *game_topic*. 
- **robot:** The main robot node that reads from the *game_topic* and *things_position* topics, execute

- **strategy:**
- **sys_interfaces:**
- **utils:**
- **Computer Vision Systems**:
    - **vision:**
    - **sim_cam:**
- **Comunication Systems**:
    - **sim_message_server:** 
    - **ble_message_server(WIP):** Responsible for sending the messages from *message_server* topic to the **Bluetooth** Hardware.
    - **now_message_server:** Responsible for sending the messages from *message_server* topic to the **Bluetooth** Hardware.
    - **message_server(DEPRECATED):** Responsible for sending the messages from *message_server* topic to the **Bluetooth** Hardware.