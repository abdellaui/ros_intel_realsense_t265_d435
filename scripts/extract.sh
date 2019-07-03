#!/usr/bin/env bash

rm -rf result
echo "creating folders"
mkdir result
mkdir result/static
mkdir result/images
mkdir result/pointclouds

echo "creating csv: /d435/color/camera_info"
rostopic echo -b $1 -p /d435/color/camera_info > result/static/d435_color_camera_info.csv

echo "creating csv: /d435/depth/camera_info"
rostopic echo -b $1 -p /d435/depth/camera_info > result/static/d435_depth_camera_info.csv

#echo "creating csv: /d435/infra1/camera_info"
#rostopic echo -b $1 -p /d435/infra1/camera_info > result/static/d435_infra1_camera_info.csv

#echo "creating csv: /d435/infra2/camera_info"
#rostopic echo -b $1 -p /d435/infra2/camera_info > result/static/d435_infra2_camera_info.csv

echo "creating csv: /t265/fisheye1/camera_info"
rostopic echo -b $1 -p /t265/fisheye1/camera_info > result/static/t265_fisheye1_camera_info.csv

echo "creating csv: /t265/fisheye2/camera_info"
rostopic echo -b $1 -p /t265/fisheye2/camera_info > result/static/t265_fisheye2_camera_info.csv

echo "creating csv: /t265/accel/sample"
rostopic echo -b $1 -p /t265/accel/sample > result/t265_accel_sample.csv

echo "creating csv: /t265/gyro/sample"
rostopic echo -b $1 -p /t265/gyro/sample > result/t265_gyro_sample.csv

echo "creating csv: /t265/odom/sample"
rostopic echo -b $1 -p /t265/odom/sample > result/t265_odom_sample.csv

echo "creating csv: /tf"
rostopic echo -b $1 -p /tf > result/tf.csv

echo "creating csv: /tf_static"
rostopic echo -b $1 -p /tf_static > result/static/tf_static.csv

echo "extracting pointclouds: /d435/depth/color/points"
rosrun pcl_ros bag_to_pcd $1 /d435/depth/color/points result/pointclouds

echo "extracting images"
source ~/anaconda3/etc/profile.d/conda.sh
conda deactivate
rosrun t265_d435 image_extraction.py -p $(pwd) -b $1
