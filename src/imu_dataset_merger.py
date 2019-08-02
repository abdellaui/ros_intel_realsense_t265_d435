#!/usr/bin/env python

import argparse
import os
import numpy as np
import datetime
import math 
def rotateZ(_x, _y, _z, angle):
    """ Rotates the point around the Z axis by the given angle in degrees. """
    rad = angle * math.pi / 180
    cosa = math.cos(rad)
    sina = math.sin(rad)
    x = _x * cosa - _y * sina
    y = _x * sina + _y * cosa
    z = _z
    return x, y, z

def quaternion_multiply(quaternion1, quaternion0):
    w0, x0, y0, z0 = quaternion0
    w1, x1, y1, z1 = quaternion1
    return np.array([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                     x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                     -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                     x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0], dtype=np.float64)

timestamp = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
parser.add_argument("-i","--imu", required=True, help="path to imu.csv")
parser.add_argument("-p","--path", default=os.getcwd(), help="path to bag")
parser.add_argument("-d","--dataset", required=True, help="path to image dataset.csv")
parser.add_argument("-o","--output", default=timestamp+"_tum.csv", help="path to output.csv")
parser.add_argument("-s","--scale", nargs='+', default=[1.0, 1.0, 1.0], help="xyz scale")
parser.add_argument("-of","--offset", nargs='+', required=True, help="xyz offset")
parser.add_argument("-q","--quaternion", nargs='+', default=[0.5, 0.5, -0.5, -0.5], help="wpqr (wxyz) offset")

opt = parser.parse_args()



path = os.path.join(opt.path,"tum_files")
if not os.path.exists(path):
    os.mkdir(path)
storepath = os.path.join(path, opt.output)


scale = opt.scale
scale[0] = float(scale[0])
scale[1] = float(scale[1])
scale[2] = float(scale[2])

offset = opt.offset
offset[0] = float(offset[0])
offset[1] = float(offset[1])
offset[2] = float(offset[2])

offset_qt = opt.quaternion
offset_qt[0] = float(offset_qt[0])
offset_qt[1] = float(offset_qt[1])
offset_qt[2] = float(offset_qt[2])
offset_qt[3] = float(offset_qt[3])

dataImu= np.loadtxt(opt.imu, dtype=np.float64, delimiter=",", skiprows=1, usecols = (5,6,7,8,9,10,11))
dataImage = np.loadtxt(opt.dataset, dtype=str, delimiter=",", skiprows=1, usecols = (1))

dataset = [ [x, *dataImu[i]] for i,x in enumerate(dataImage)]

with open(storepath, "w+") as file:
    newString = "dest x y z w p q r\n"+timestamp+"\noffset: "+ str(offset) +"; quaternion: "+ str(offset_qt) +"\n"
    file.write(newString)
    for i, data in enumerate(dataset):
        q1 = np.array([data[7], -1*data[5], data[4], data[6]], dtype=np.float64)
        #q2 = np.array(offset_qt, dtype=np.float64)
        #q = quaternion_multiply(q1, q2)
        q = quaternion_multiply(q1, np.array([0.5, 0.5, 0, 0], dtype=np.float64)) # z-rotation
        x = data[1]*scale[0]
        y = data[2]*scale[0]
        z = data[3]*scale[0]
        x,y,z = rotateZ(x,y,z, 90) # z-rotation
        x += offset[0]
        y += offset[1]
        z += offset[2]
        newString = "{} {} {} {} {} {} {} {}\n".format(data[0], x, y, z, q[0], q[1], q[2], q[3])
        
        file.write(newString)
        
    file.close()
print("saved to: "+storepath)
