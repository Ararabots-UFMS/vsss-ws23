FROM ros:humble-ros-core-jammy

# ------------------------
# Dependências básicas
# ------------------------
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    git \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    python3-pip \
    wget \
    && rm -rf /var/lib/apt/lists/*

# ------------------------
# rosdep
# ------------------------
RUN rosdep init && \
    rosdep update --rosdistro $ROS_DISTRO

# ------------------------
# colcon config
# ------------------------
RUN colcon mixin add default \
    https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update && \
    colcon metadata add default \
    https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
    colcon metadata update

RUN pip install colcon-override-check

# ------------------------
# Pacotes ROS
# ------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-base=0.10.0-1* \
    && rm -rf /var/lib/apt/lists/*

# ------------------------
# Workspace
# ------------------------
WORKDIR /vsss
RUN git clone -b feat/new_docker_image https://github.com/Ararabots-UFMS/vsss-ws23.git vsss_ws

WORKDIR /vsss/vsss_ws

# ------------------------
# Python deps
# ------------------------
RUN pip install -r requirements.txt
RUN pip install opencv-python Pillow

# ------------------------
# Build do workspace
# ------------------------
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build --packages-select sys_interfaces --allow-overriding sys_interfaces && \
    colcon build --symlink-install --packages-skip sys_interfaces"

# ------------------------
# FLTK
# ------------------------
WORKDIR /vsss/vsss_ws/scripts
RUN sh install-fltk.sh

# ------------------------
# pyFltk
# ------------------------
COPY pyFltk-1.3.4.1_py3.tar.gz /opt/
RUN cd /opt && \
    tar -xzf pyFltk-1.3.4.1_py3.tar.gz && \
    cd pyFltk-1.3.4.1 && \
    python3 setup.py build install

# ------------------------
# Variáveis de ambiente
# ------------------------
ENV ROS_ARARA_ROOT=/vsss/vsss_ws
ENV LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

# ------------------------
# Entrypoint (ESSENCIAL)
# ------------------------
RUN echo '#!/bin/bash\n\
source /opt/ros/$ROS_DISTRO/setup.bash\n\
if [ -f /vsss/vsss_ws/install/local_setup.bash ]; then\n\
  source /vsss/vsss_ws/install/local_setup.bash\n\
fi\n\
exec "$@"' > /ros_entrypoint.sh

RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]