# setup sources.list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# setup keys
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# install
sudo apt-get update
sudo apt-get install ros-melodic-ros-base
sudo rosdep init
rosdep update

# environment setup
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# dependencies for building packages
sudo apt install python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

# python
sudo apt-get install python3-pip python3-yaml
sudo pip3 install rospkg catkin_pkg
