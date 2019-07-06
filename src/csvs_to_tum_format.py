#!/usr/bin/env python

import argparse
import os
import numpy as np
import datetime

timestamp = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
parser.add_argument("-i","--imu", required=True, help="path to imu.csv")
parser.add_argument("-p","--path", default=os.getcwd(), help="path to bag")
parser.add_argument("-d","--dataset", required=True, help="path to image dataset.csv")
parser.add_argument("-o","--output", default=timestamp+"_tum.csv", help="path to output.csv")
parser.add_argument("-of","--offset", nargs='+', required=True, help="xyz offset")
opt = parser.parse_args()


path = os.path.join(opt.path,"tum_files")
if not os.path.exists(path):
    os.mkdir(path)
storepath = os.path.join(path, opt.output)
offset = opt.offset
offset[0] = float(offset[0])
offset[1] = float(offset[1])
offset[2] = float(offset[2])

dataImu= np.loadtxt(opt.imu, dtype=np.float64, delimiter=",", skiprows=1, usecols = (5,6,7,8,9,10,11))
dataImage = np.loadtxt(opt.dataset, dtype=str, delimiter=",", skiprows=1, usecols = (1))

dataset = [ [x, *dataImu[i]] for i,x in enumerate(dataImage)]

with open(storepath, "w+") as file:
    newString = "dest x y z w p q r\n"+timestamp+"\noffset: "+ str(offset) +"\n"
    file.write(newString)
    for i, data in enumerate(dataset):
        newString = "{} {} {} {} {} {} {} {}\n".format(data[0], data[1]+offset[0], data[2]+offset[1], data[3]+offset[2], data[4], data[5], data[6], data[7])
        file.write(newString)
        
    file.close()
print("saved to: "+storepath)