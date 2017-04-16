#!/usr/bin/env python

import time
import serial
from math import sin, cos, asin, acos, atan, sqrt, pi
import logging
logging.basicConfig(level=logging.INFO)

import time
import numpy as np
import cv2
import pyrealsense as pyrs
from scipy.stats import threshold
from scipy.ndimage.filters import gaussian_filter
from scipy.ndimage.morphology import grey_dilation, grey_erosion

#angles are expressed as follows:
#a0: rotation angle from straight towards controller
#a1: shoulder angle where -pi/2 is flat back and 0 is straight up
#a2: elbow angle where 0 folds into shoulder, pi is straight continue of shoulder
#a3: wrist up/down where pi/2 is right angle "out", pi continues elbow and 3pi/2 is straight angle "in"
#a4: wrist rotate where -pi/2 is right angle counter clockwise, 0 is straight, pi/2 is clockwise
#a5: just 0-pi where 0 is open and pi is closed

servo_angles = [[0, pi, -1800/pi, 2200], 
                [-pi/2, pi/2, 1650/pi, 1370], 
                [0, pi, 1560/pi, 740], 
                [pi/2, 3*pi/2, -1850/pi, 3275], 
                [-pi/2,pi/2, 1800/pi, 1500],
                [0,pi, 1300/pi, 700]]

l1 = 15   #cm, == 5"7/8
l2 = 18.5 #cm == 7"3/8
l3 = 11.5 #cm == 4"1/2

def xform_to_arm(coor):
    return([-coor[0]/10, -coor[1]/10, -coor[2]/10])
    
def servo_angle(i, a):
    min = servo_angles[i][0]
    max = servo_angles[i][1]
    if a < min:
        a = a + 2*pi
    if a > max:
        a = a - 2*pi
#    print 'i, a: ', i, a
    if (a < min or a > max):
        return -1
    sa = (a * servo_angles[i][2] + servo_angles[i][3])
#    print 'sa: ', sa
    return int(sa)

def move_to_angle(ser, angles, time=None, spd=None):
# angles is a 5 tuple of angles in radians or None for no instruction to that servo
    print angles
    out = ''
    if (len(angles) != 6):
        print('angles should be a 6-tuple')
        return -1
    if spd == None:
        s = ''
    else:
        s = 'S{}'.format(spd)
    for (i,a) in enumerate(angles):
        if a!=None:
            sa = servo_angle(i,a)
            if (sa == -1):
                print('bad servo angle for servo', i)
                return -1
            out = out + '#{}P{}{}'.format(i, sa, s)
    if time != None:
        out = out + 'T{}'.format(time)
    out = out+'\r'
    print out
    ser.write(out)

def wait_stop(ser):
    while True:
        ser.write('Q\r')
        if (ser.read(1) == '.'):
            break
        
def zero_position(ser):
    ser.write('#0P1300S500#1P1000S500#2P1200S500#3P1450S500#4P1500S500#5P2000S500T1000\r')
    wait_stop(ser)
def rest(ser):
    zero_position(ser)
    ser.write('#0P0#1P0#2P0#3P0\r')
    wait_stop(ser)

def stop_now(ser):
    ser.write('#0P0#1P0#2P0#3P0#4P0#5P0\r')
    
def find_rotation(T3):
    if T3[0] == 0:
        b0 = pi/2
        T2 = (T3[2], T3[1])
    elif T3[0] > 0:
        b0 = atan(T3[2] / T3[0])
        T2 = (T3[0]*cos(b0) + T3[2] * sin(b0), T3[1])
    else:
        b0 = atan(T3[0] / -T3[2])
        T2 = (-T3[0]*sin(b0) + T3[2] * cos(b0), T3[1])
        b0 = pi/2 + b0
    return b0, T2
    
#T3 is the (x,y,z) coordinates in cm where (0,0,0) is the base of the arm
#x is parallel to the base board towards the controller
#y is up
#z is out
def find_angles(T3, a):
    print 'find_angles( ', T3, a, ')'
    T3 = (float(T3[0]), float(T3[1]), float(T3[2]))
    b0, T2 = find_rotation(T3)
    try:
        P2 = (T2[0] - l3*cos(a), T2[1] -l3*sin(a))
        d_sq = P2[0]*P2[0] + P2[1]*P2[1]
        d = sqrt(d_sq)
        b2 = acos( (P2[0]*P2[0]+P2[1]*P2[1]-l1*l1-l2*l2) / (-2*l1*l2) )
        c1 = asin(P2[1]/d)
        c0 = acos( (l2*l2 - l1*l1 - d_sq) / (-2*l1*d) )
        b1 = pi/2 - c1 - c0
        b3 = 3*pi/2 + a + b1 - b2
        print b0/pi*180, b1/pi*180, b2/pi*180, b3/pi*180
    except ValueError:
        b0,b1,b2,b3 = (None, None, None, None)
        print 'Angles Error'        
    return b0, b1, b2, b3

if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyUSB0', 9600)
    zero_position(ser)
    T_init = np.array([0,25,10])
    b0,b1,b2,b3 = find_angles(T_init,pi/2)
    #b0,b1,b2,b3 = find_angles(T_init,0)
    move_to_angle(ser, (b0,b1,b2,b3,0,pi), 1000)
    wait_stop(ser)
    T_1 = T_init+np.array([-10,0,-5])
    b0,b1,b2,b3 = find_angles(T_1,pi/2)
    move_to_angle(ser, (b0,b1,b2,b3,pi/2,pi), 5000)
    #wait_stop(ser)
    time.sleep(2)
    T_2 = T_init+np.array([10,0,-5])
    b0,b1,b2,b3 = find_angles(T_2,pi/2)
    move_to_angle(ser, (b0,b1,b2,b3,-pi/2,pi), 5000)
    #wait_stop(ser)
    time.sleep(2)
    stop_now(ser)
    time.sleep(2)
    rest(ser)