# This is an auto generated Dockerfile for ros:ros-base
# generated from docker_images_ros2/create_ros_image.Dockerfile.em
FROM ros:humble-ros-core-jammy

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    git \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# install pip
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
RUN rosdep init && \
  rosdep update --rosdistro $ROS_DISTRO

# setup colcon mixin and metadata
RUN colcon mixin add default \
      https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update && \
    colcon metadata add default \
      https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
    colcon metadata update

# colcon override check
RUN pip install colcon-override-check

# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-base=0.10.0-1* \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /vsss
RUN git clone https://github.com/Ararabots-UFMS/vsss_ws.git
WORKDIR /vsss/vsss_ws

# requirements
# TODO: fix this file on main branch
RUN git checkout now_msg_server
RUN pip install -r requirements.txt
RUN pip install opencv-python
RUN pip install Pillow

# environment variables
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
RUN echo "export ROS_ARARA_ROOT=$PWD/" >> ~/.bashrc
RUN echo "source $PWD/install/local_setup.bash" >> ~/.bashrc

# build the workspace
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build --packages-select sys_interfaces --allow-overriding sys_interfaces && \
    colcon build --symlink-install --packages-skip sys_interfaces

WORKDIR /vsss/vsss_ws/scripts
# TODO: add this fix to fltk install script
RUN apt-get update && apt-get install wget -y
RUN sh install-fltk.sh

WORKDIR /vsss/vsss_ws

RUN echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib" >> ~/.bashrc






