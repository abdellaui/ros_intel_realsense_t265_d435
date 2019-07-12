#!/usr/bin/env bash

rosbag record $(< topics_recording.txt) -o $(pwd)/bags/
# -o /media/abdullah/128GB/
