#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Fri Apr  7 18:26:05 2017

@author: rafi
"""

import numpy as np
import cv2
import time
import serial
from math import sin, cos, asin, acos, atan, sqrt, pi
import logging
logging.basicConfig(level=logging.INFO)

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
                [pi/2, 3*pi/2, -1850/pi, 3400], 
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

def angles_from_pos(pos):
    ret = [0,0,0,0,0,0]
    for i in range(6):
        ret[i] = (pos[i] - servo_angles[i][3]) / servo_angles[i][2]
    #print 'pos: ', pos, ' ret: ', ret
    return ret
    
def move_to_angle(ser, angles, mov_time=None, spd=None):
# angles is a 5 tuple of angles in radians or None for no instruction to that servo
    #print angles
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
                print'bad servo angle for servo', i, ' requested ', a, ' (min,max): ', (servo_angles[i][0], servo_angles[i][1])
                return -1
            out = out + '#{}P{}{}'.format(i, sa, s)
    if time != None:
        out = out + 'T{}'.format(mov_time)
    out = out+'\r'
    #print out
    ser.write(out)

def still_moving(ser):
    ser.write('Q\r')
    return (ser.read(1) != '.')
    
def wait_stop(ser):
    while still_moving(ser):
        continue

def get_pos(ser):
    res = [0]*6
    ser.write('QP0QP1QP2QP3QP4QP5\r')
    for i in range(6):
        res[i] = ord(ser.read(1))*10
    return res
    
def zero_position(ser):
    ser.write('#0P1300S500#1P1000S500#2P1200S500#3P600S500#4P1500S500#5P2000S800T1000\r')
    wait_stop(ser)
def rest(ser):
    zero_position(ser)
    ser.write('#0P0#1P0#2P0#3P0\r')
    wait_stop(ser)

def stop_now(ser):
    ser.write('STOP0\r')
    ser.write('STOP1\r')
    ser.write('STOP2\r')
    ser.write('STOP3\r')
    ser.write('STOP4\r')
    ser.write('STOP5\r')
    
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
    #print 'find_angles( ', T3, a, ')'
    T3 = (float(T3[0]), float(T3[1]), float(T3[2]))
    b0, T2 = find_rotation(T3)
    try:
        P2 = (T2[0] - l3*cos(a), T2[1] -l3*sin(a))
        #print P2
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

def xyz_from_angles(angles):
    #find (y,z) in the arm plane
    elbow = np.array([l1*sin(pi/2+angles[1]),-l1*cos(pi/2+angles[1])])
    #print 'elbow ', elbow
    wrist = elbow+np.array([l2*sin(angles[2]-pi/2-angles[1]), l2*cos(angles[2]-pi/2-angles[1])])
    #print 'wrist ', wrist
    tip = wrist+np.array([-l3*sin(-angles[3]-angles[2]+angles[1]+3*pi/2), l3*cos(-angles[3]-angles[2]+angles[1]+3*pi/2)])
    #print 'tip ', tip
    x = tip[1]*cos(angles[0])
    z = tip[1]*sin(angles[0])
    y = tip[0]
    return(x,y,z)
    

green_lower = np.array([40, 20, 10],np.uint8)
green_upper = np.array([110, 255, 255],np.uint8)
red_lower = np.array([150, 80, 30],np.uint8)
red_upper = np.array([250, 255, 255],np.uint8)

def detect_color(color,  lower_thresh, upper_thresh):
    #look for saturated green and return its contour and center
    hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
    greenMask = cv2.inRange(hsv, lower_thresh, upper_thresh)
    # apply a series of erosions and dilations to the mask
    # using an elliptical kernel
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    greenMask = cv2.erode(greenMask, kernel, iterations = 2)
    greenMask = cv2.dilate(greenMask, kernel, iterations = 2)
     
    # blur the mask to help remove noise, then apply the
    # mask to the frame
    greenMask = cv2.GaussianBlur(greenMask, (3, 3), 0)
    
    #now find the contours of the mask
    _, contours, _ = cv2.findContours(greenMask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    #print 'found ', len(contours), ' green contours'
    #and keep the biggest one:
    c = None
    c_area = 0
    c_center = None
    for contour in contours:
        M = cv2.moments(contour)
        if M['m00'] > c_area:
            c = contour
            c_area = M['m00']
            c_center = (int(M['m10']/M['m00']), int(M['m01']/M['m00']))

    return c, c_center



#testing xyz_from_angles
#angles = find_angles([5,20,15],0)
#print angles
#pos = xyz_from_angles(angles)
#print pos
#exit()

cap = cv2.VideoCapture(3)

ser = serial.Serial('/dev/ttyUSB0', 9600)
zero_position(ser)
T_init = np.array([0,30,5])
T = T_init
angles = list(find_angles(T,pi/2)) + [0,pi]
#b0,b1,b2,b3 = find_angles(T_init,0)
move_to_angle(ser, angles, 1000)
wait_stop(ser)

state = 'wait_target'
cnt = 0
move_spd = 100
next_move = [0,0,0]
cur_move = [0,0,0]
plug_d_bh = 280 #plug height that I want to see, which is then plug_d_cm away
plug_d_cm = 19 #when I see the plug height at plug_bh it is plug_cm centimeters away

while(True):
    ret, frame = cap.read()

    if (ret):
        blob, blob_cent = detect_color(frame, green_lower, green_upper)
        if blob != None:
            cv2.drawContours(frame, [blob], 0, (255, 0, 0), 2)
            x,y,bw,bh = cv2.boundingRect(blob)
            cv2.rectangle(frame,(x,y),(x+bw,y+bh),(0,255,0),2)
            cv2.line(frame, (frame.shape[1]/2,frame.shape[0]/2-plug_d_bh/2),(frame.shape[1]/2,frame.shape[0]/2+plug_d_bh/2), (0,0,255))
            cv2.putText(frame, 'State: {}, bh: {}, bw: {}'.format(state, bh, bw), (0,100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0))
            if state == 'wait_target':
                cv2.putText(frame, 'press <space> to start', (0,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0))
            if state == 'find_center':
                cv2.putText(frame, 'moving to center', (0,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0))
                next_move[0] = np.sign( int(((x+bw/2) - (frame.shape[1]/2))/20) ) #shape center minus frame center, accuracy of 20 pixels
                next_move[1] = -np.sign( int(((y+bh/2) - (frame.shape[0]/2))/20) )
                next_move[2] = -np.sign( int ((bh-plug_d_bh)/5)) #determine Z by the shape height, assuming no roll, accuracy of 5 pixels
                if next_move == [0,0,0]: #we found center!
                    state = 'find_yaw_init'
                    stop_now(ser)
                elif cur_move != next_move or not still_moving(ser):
                    print 'change from ', cur_move, ' to ', next_move
                    if still_moving(ser):
                        print 'stop first'
                        stop_now(ser)
                    cur_pos = xyz_from_angles(angles_from_pos(get_pos(ser)))
                    next_pos = np.array(cur_pos)+(np.array(next_move)*2)
                    print 'next_pos: ', next_pos
                    move_to_angle(ser, list(find_angles(next_pos,pi/2)) + [0,pi], spd = move_spd)
                    cur_move = list(next_move)
            elif state == 'find_yaw_init':
                prev_bw = bw
                cur_pos = xyz_from_angles(angles_from_pos(get_pos(ser)))
                cur_zx = np.array([cur_pos[2], cur_pos[0]])
                cur_wrist_angle = 0 #that's how we got here
                cur_shoulder_angle = atan(cur_zx[1]/cur_zx[0])
                cur_angle = cur_shoulder_angle #current angle is shoulder_angle since wrist_angle=0
                plug_center_zx = cur_zx + np.array([cos(cur_wrist_angle-cur_shoulder_angle),sin(cur_wrist_angle-cur_shoulder_angle)])*plug_d_cm
                angle_step = 10.0/180*pi #start arbitrarily with 10 degrees left
                next_angle = cur_angle + angle_step 
                new_zx = plug_center_zx - np.array([cos(next_angle),-sin(next_angle)])*plug_d_cm
                new_pos = [new_zx[1],cur_pos[1],new_zx[0]]
                new_shoulder_angle = atan(new_zx[0]/new_zx[1])
                if new_shoulder_angle < 0:
                    new_shoulder_angle = pi + new_shoulder_angle
                new_wrist_angle = (pi/2 - new_shoulder_angle)+next_angle
                cur_angle = next_angle
                print 'I am in ', cur_pos, ' wrist_angle: ', cur_wrist_angle, ' cur_shoulder_angle: ', cur_shoulder_angle
                print 'estimate plug_center_zx ', plug_center_zx
                print 'Now want to see from angle ', next_angle
                print 'So moving to new_zx ', new_zx, ' with new_wrist_angle ', new_wrist_angle
                state = "find_yaw"
                move_to_angle(ser, list(find_angles(new_pos,pi/2)) + [-new_wrist_angle,pi], spd = move_spd)
            elif state == 'find_yaw':
                if not still_moving(ser):
                    if bw > prev_bw: #that's good, continue
                        angle_step = angle_step #no-op
                        print 'good direction, trying another step of ', angle_step
                    else:
                        angle_step = -(angle_step*0.8) #go the other way at lower speed
                        print 'wrong direction, trying instead a step of ', angle_step
                    if abs(angle_step) < 1.0/180*pi:
                        #we're down to tiny adjustments, say we found it
                        state = 'found_yaw'
                    else:
                        prev_bw = bw
                        cur_pos = xyz_from_angles(angles_from_pos(get_pos(ser)))
                        cur_zx = np.array([cur_pos[2], cur_pos[0]])
                        cur_wrist_angle = new_wrist_angle #that's how we got here
                        cur_shoulder_angle = atan(cur_zx[1]/cur_zx[0])
                        next_angle = cur_angle + angle_step 
                        new_zx = plug_center_zx - np.array([cos(next_angle),-sin(next_angle)])*plug_d_cm
                        new_pos = [new_zx[1],cur_pos[1],new_zx[0]]
                        new_shoulder_angle = atan(new_zx[0]/new_zx[1])
                        if new_shoulder_angle < 0:
                            new_shoulder_angle = pi + new_shoulder_angle
                        new_wrist_angle = (pi/2 - new_shoulder_angle)+next_angle
                        cur_angle= next_angle
                        print 'I am in ', cur_pos, ' wrist_angle: ', cur_wrist_angle, ' cur_shoulder_angle: ', cur_shoulder_angle
                        print 'estimate plug_center_zx ', plug_center_zx
                        print 'Now want to see from angle ', next_angle
                        print 'So moving to new_zx ', new_zx, ' with new_wrist_angle ', new_wrist_angle
                        #cv2.waitKey(0)
                        move_to_angle(ser, list(find_angles(new_pos,pi/2)) + [-new_wrist_angle,pi], spd = move_spd)
                                    
        cv2.imshow('frame',frame)
    key = cv2.waitKey(1) & 0xFF
    if key == ord(' '):
        if state == 'wait_target':
            state = 'find_center'
            cur_move = [0,0,0]
    if key == ord('q'):
        break
    elif key == ord('f'):
        cv2.imwrite('frame-{}.jpg'.format(cnt), frame)
        cnt+=1
        cv2.waitKey(0)
# When everything done, release the capture
rest(ser)
cap.release()
cv2.destroyAllWindows()