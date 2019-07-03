# dependencies: 
- realsense-ros https://github.com/IntelRealSense/realsense-ros


# usage 

firstly run `.\connect.sh`, if you want to see the current dataflow run `.\vizualize.sh`.
r
for recording a rosbag:
- run `.\record_async.sh` or `.\record_sync.sh` while you are connected to you device.
- you can specify topics on `topics_recording.txt` or `../src/sync_record.py`

for replaying a rosbag:
- disconnect from your device
- run `rosbag play <filename>`
- sometimes it is usefull to restart the vizualizer
