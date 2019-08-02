#!/usr/bin/env python

import rosbag
import argparse
import cv2
import os
import pandas
import shutil
from cv_bridge import CvBridge

parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
parser.add_argument("-p","--path", default=os.getcwd(), help="path to bag")
parser.add_argument("-b","--bag", required=True, help="name of bag")
opt = parser.parse_args()

topics = [
    "/d435/color/image_raw",
    #"/d435/depth/image_rect_raw",
    #"/d435/infra1/image_rect_raw",
    #"/d435/infra2/image_rect_raw",
    #"/t265/fisheye1/image_raw",
    #"/t265/fisheye2/image_raw",
]

csv_dict = {
    'd435_color_image_raw': [],
    #'d435_depth_image_rect_raw': [],
    #'d435_infra1_image_rect_raw': [],
    #'d435_infra2_image_rect_raw': [],
    #'t265_fisheye1_image_raw': [],
    #'t265_fisheye2_image_raw': []
    }


bridge = CvBridge()
bag = rosbag.Bag(opt.bag)

print("creating: subfolders")

path_root = os.path.join(opt.path, 'result/images')
if os.path.exists(path_root):
    shutil.rmtree(path_root)
os.mkdir(path_root)

for t in csv_dict:
    path_abs = os.path.join(path_root, t)
    if os.path.exists(path_abs):
        shutil.rmtree(path_abs)
    os.mkdir(path_abs)

print("extracting: images")

len_topics = len(topics)
frame_count = 0
for topic, msg, t in bag.read_messages(topics=topics):

    new_topic_name = topic[1:].replace('/', '_')
    frame_name = "frame%06i.png" % ( (frame_count // len_topics) + 1)

    frame_count += 1
    path = os.path.join(new_topic_name, frame_name)
    path_abs = os.path.join(path_root, path)

    csv_dict[new_topic_name].append([t.__str__(), path])

    cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    if new_topic_name == 'd435_color_image_raw':
        cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)

    cv2.imwrite(path_abs, cv_img)
    print(t,new_topic_name)





print("creating: csv-files")
for t in csv_dict:
    path_abs = os.path.join(path_root, t+'.csv')
    pandas.DataFrame(csv_dict[t]).to_csv(path_abs, header=['time', 'dest'], index=False)

bag.close()
