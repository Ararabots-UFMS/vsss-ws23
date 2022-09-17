apt-get -y update
apt-get install -y build-essential cmake

# GUI (if you want to use GTK instead of Qt, replace 'qt5-default' with 'libgtkglext1-dev' and remove '-DWITH_QT=ON' option in CMake):
apt-get install -y qt5-default libvtk6-dev

# Media I/O:
apt-get install -y zlib1g-dev libjpeg-dev libwebp-dev libpng-dev libtiff5-dev libjasper-dev libopenexr-dev libgdal-dev

# Video I/O:
apt-get install -y libdc1394-22-dev libavcodec-dev libavformat-dev libswscale-dev libtheora-dev libvorbis-dev libxvidcore-dev libx264-dev yasm libopencore-amrnb-dev libopencore-amrwb-dev libv4l-dev libxine2-dev

# Parallelism and linear algebra libraries:
apt-get install -y libtbb-dev libeigen3-dev

# Python:
apt-get install -y python-dev python-tk python-numpy python3-dev python3-tk python3-numpy python3-pip python3-testresources

# Java:
apt-get install -y ant default-jdk

# Documentation:
apt-get install -y doxygen

apt-get install -y unzip wget

# 3. INSTALL THE LIBRARY
# Download Opencv
wget https://codeload.github.com/opencv/opencv/zip/master -O /opt/opencv.zip
unzip /opt/opencv.zip -d /opt
mv /opt/opencv-master /opt/opencv
rm /opt/opencv.zip

# Download contrib module
wget https://codeload.github.com/opencv/opencv_contrib/zip/master -O /opt/opencv-contrib.zip
unzip /opt/opencv-contrib.zip -d /opt
mv /opt/opencv_contrib-master /opt/opencv-contrib
rm /opt/opencv-contrib.zip

cd /opt/opencv
mkdir build
cd build
cmake -DWITH_QT=ON -DWITH_OPENGL=ON -DFORCE_VTK=ON -DWITH_TBB=ON -DWITH_GDAL=ON -DWITH_XINE=ON -DBUILD_EXAMPLES=ON \
	-DENABLE_PRECOMPILED_HEADERS=OFF -DWITH_V4L=ON -DWITH_LIBV4L=ON \
	-DOPENCV_EXTRA_MODULES_PATH=/opt/opencv-contrib/modules ..
make -j4
make install
ldconfig
