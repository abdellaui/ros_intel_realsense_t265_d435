#!/usr/bin/env python

import rosbag
import argparse
import cv2
import os
import pandas
import glob
import shutil
import sys

parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
parser.add_argument("-p","--path", default=os.getcwd(), help="path to bag")
opt = parser.parse_args()

csv_dict = {
    'd435_depth_color_points': []
    }



path_root = os.path.join(opt.path, 'result/pointclouds')

print("creating csv-files and renaming points")

for topic in csv_dict:
    current_path = os.path.join(path_root, topic)
    print(current_path)
    frame_count = 0
    for f in glob.glob(current_path+"/*.pcd"):
        frame_count += 1
        t=1
        frame_name = "frame%06i.pcd" % frame_count
        path = os.path.join(topic, frame_name)
        t = os.path.basename(f)[0:-4].replace('.', '')
        if(t[0:5]=="frame"):
            print("rename process was applied!")
            sys.exit()    
        shutil.move(f, os.path.join(path_root, path))
        csv_dict[topic].append([t, path])
        print(t,path)
    path_abs = os.path.join(path_root, topic+'.csv')
    pandas.DataFrame(csv_dict[topic]).to_csv(path_abs, header=['time', 'dest'], index=False)



