#!/bin/bash

echo "╔══╣ Setup: SOBITS VLA ROSBAG CONVERSION (STARTING) ╠══╗"

sudo apt-get update
sudo apt install ffmpeg

pip3 install rosbags rosbags-image opencv-python pandas pyarrow

echo "╚══╣ Setup: SOBITS VLA ROSBAG CONVERSION (FINISHED) ╠══╝"