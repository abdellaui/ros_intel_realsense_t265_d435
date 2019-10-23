connecting and reading data stream from intel realsense t265 and d435 synchronously.


# some nice images:

t265 mounted over the d435:

![real](images/t265_d435.png)

real image (d435):

![real](images/dc_frame000005.png)

depth image (d435):

![depth](images/dt_frame000005.png)

fish eye left (t265):

![depth](images/f1_frame000005.png)

fish eye right (t265):

![depth](images/f1_frame000005.png)

odometry (t265) + pointcloud (d435):

![depth](images/pointcloud1.png)

odometry (t265) + pointcloud (d435):

![depth](images/pointcloud2.png)

# some nice informations:

this is a catkin package.

firstly run ```cd <catkin_workspace>```

in your catkin worksapce run ```catkin_create_pkg t265_d435```

copy all files to `<catkin_workspace>/src/t265_d435`

usage is under `scripts/README.md` explained

# known issues:

sometimes there exists frame drops which causes an interrupt and exiting the recording process, while recording and vizualizing via rviz in the same time. 
