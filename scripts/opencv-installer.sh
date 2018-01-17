#!/bin/bash

#if [$# = 0]
#	echo "Please specify install dir"
#	exit 0
#fi

install_prefix=$1
opencv_folder="opencv-3.3"

# update system
sudo apt-get -y update
sudo apt-get -y upgrade

# Install dependencies
echo "Install cmake, git, gcc etc..."
sudo apt-get install -y build-essential && \
sudo apt-get install -y cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev && \
sudo apt-get install -y python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev && \
echo "Finished cmake, git, gcc etc... installation!" && \

# Download OpenCV sources
echo "Downloading OpenCV..." && \
mkdir $opencv_folder && \
cd $opencv_folder && \
git clone https://github.com/opencv/opencv.git && \
echo "OpenCV download completed!" && \

echo "Downloading OpenCV extra-modules..." && \
git clone https://github.com/opencv/opencv_contrib.git && \
echo "OpenCV extra-modules download completed!!" && \

# Build OpenCV release
echo "Building OpenCV (release)..." && \
mkdir Release && \
cd Release && \
# TODO: enable options to speedup opencv
cmake -DCMAKE_BUILD_TYPE=RELEASE -DCMAKE_INSTALL_PREFIX=$install_prefix -D OPENCV_EXTRA_MODULES_PATH=../opencv_contrib/modules -DWITH_V4L=ON -DWITH_LIBV4L=ON -DBUILD_PERF_TESTS=OFF -DBUILD_TESTS=OFF ../opencv && \
make -j3 && \
echo "Finished OpenCV (release) build!" && \

# install OpenCV release
echo "Installing OpenCV" && \
sudo make install && \
echo "Done!" && \

# remove temporary files
echo "Removing temporary files..." && \
cd ../.. && \
sudo rm -r $opencv_folder && \
echo "Done!" && \
echo "End." && \

# add OpenCV_DIR environment variable
sudo echo "export OpenCV_DIR=/usr/local/$opencv_folder" > /etc/profile.d/opencv_envvars.sh
