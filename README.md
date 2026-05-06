# Ararabots Verysmall - LIA
## VSSS — Ambiente Dockerizado

## Pré-requisitos

- Docker instalado
- Câmera USB conectada (`/dev/video*`)
- ESP32 conectado via USB (`/dev/ttyUSB0`)
- Arquivo `pyFltk-1.3.4.1_py3.tar.gz` no mesmo diretório do `Dockerfile`

## Build da imagem

```bash
docker build -t vsss-ros .
```

> O build pode demorar bastante, pois instala pacotes ROS, compila o workspace com colcon e o FLTK.

## Executando o container

```bash

xhost +local:docker && docker run -it \
  --env DISPLAY=$DISPLAY \
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  --device=/dev/video* \
  --device=/dev/ttyUSB* \
  --network host \
  vsss-ros bash

```

- `xhost +local:docker` — libera acesso ao display para o container abrir janelas gráficas
- `--network host` — necessário para comunicação entre nós ROS2 via DDS
- `--device` — passa câmeras e ESP32 para dentro do container

## Variáveis de ambiente

execute manualmente:

```bash
source /opt/ros/humble/setup.bash
source /vsss/vsss_ws/install/local_setup.bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
```

## Rodando o sistema

### Launch completo (visão + interface + servidor de mensagens)

```bash
ros2 launch launch_files main_launch.py
```

### Apenas o publisher de visão

```bash
ros2 run vision publisher
```

## Informações

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
