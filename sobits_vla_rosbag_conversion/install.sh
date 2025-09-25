#!/bin/bash

echo "╔══╣ Setup: SOBITS VLA ROSBAG CONVERSION (STARTING) ╠══╗"

sudo apt-get update
sudo apt install ffmpeg

python3 -m pip install rosbags rosbags-image pandas pyarrow

echo "╚══╣ Setup: SOBITS VLA ROSBAG CONVERSION (FINISHED) ╠══╝"
