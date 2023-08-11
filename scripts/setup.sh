#!/bin/bash

echo "
Installing gazebo
"
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
    http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    | sudo tee /etc/apt/sources.list.d/gazebo-stable.list \
    > /dev/null
sudo apt update && sudo apt upgrade -y
sudo apt-get install gz-garden -y

echo "
Installing Tesseract and its source dependencies
"
vcs import < src/tesseract_raster_demo/dependencies.repos src/ --debug

echo "
Installing rosdep dependencies
"
rosdep update
rosdep install --from-paths src --ignore-src -y -r \
  --skip-keys="taskflow gz-math7 gz-common5 gz-rendering7 gz-common5"
