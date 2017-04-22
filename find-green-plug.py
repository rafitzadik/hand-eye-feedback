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
                [pi/2, 3*pi/2, -1870/pi, 3400], #should be -1850, but not working as well
                [-pi/2,pi/2, 1800/pi, 1500],
                [0,pi, 1300/pi, 700]] 

l1 = 14.5   #cm, == 5"7/8
l2 = 18.0 #cm == 7"3/8
l3 = 11.0 #cm == 4"1/2

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
        print "P2: ", P2
        d_sq = P2[0]*P2[0] + P2[1]*P2[1]
        d = sqrt(d_sq)
        b2 = acos( (d_sq-l1*l1-l2*l2) / (-2*l1*l2) )
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
green_upper = np.array([110, 255, 120],np.uint8)
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

def find_rect_contour_orientation(contour):
    #Assuming the contour is a rectangle looked at from an angle left or right (so yaw, no pitch or roll)
    #find whether the yaw is left or right
    #1 means left, -1 means right, 0 means it seems straight on
    bx,by,bw,bh = cv2.boundingRect(blob)
    leftx = bx+5
    rightx = bx+bw-5
    first_left = 0
    first_right = 0
    for dy in range(1,bh/2):
        if first_left == 0:
            if cv2.pointPolygonTest(contour, (leftx, by+bh-dy), False) >= 0:
                first_left = by+bh-dy
        if first_right == 0:
            if cv2.pointPolygonTest(contour, (rightx, by+bh-dy), False) >= 0:
                first_right = by+bh-dy
        if first_left != 0 and first_right != 0:
            break
    print 'find_rect_contour_orientation: ({},{})'.format(first_left, first_right)
    if first_left - first_right > 5:
        return -1
    if first_right - first_left > 5:
        return 1
    return 0

def find_rect_contour_orientation_alt(contour):
    #Assuming the contour is a rectangle looked at from an angle left or right (so yaw, no pitch or roll)
    #find whether the yaw is left or right
    #1 means left, -1 means right, 0 means it seems straight on
    #alternative implementation, using approxPolyDP
    approx = cv2.approxPolyDP(contour, 30, True)
    pts_from_top = sorted(approx, key = lambda pt: pt[0][1])
    print pts_from_top[0][0][1], pts_from_top[1][0][1]
    if pts_from_top[1][0][1] - pts_from_top[0][0][1] > 5: #is there more than a 5 pixel difference?
        if pts_from_top[1][0][0] > pts_from_top[0][0][0]: #is the higher on the left or right?
            return -1
        else:
            return 1
    else:
        return 0

#testing xyz_from_angles
#angles = find_angles([5,20,15],0)
#print angles
#pos = xyz_from_angles(angles)
#print pos
#exit()

cap = cv2.VideoCapture(3)
#cap2 = cv2.VideoCapture(5)

#outFile = cv2.VideoWriter('/home/rafi/Videos/find-plug.avi', cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 20, (1280, 480), True)

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
next_move = [0,0,0,0]
cur_move = [0,0,0,0]
plug_d_bh = 280 #plug height that I want to see
pixel_per_cm = 20 #pixels per cm close to the center
wait_1_time = 5
wait_2_time = 2
get_close_l = 5
num_wrong = 0

while(True):
    ret, frame = cap.read()

    if (ret):
        blob, blob_cent = detect_color(frame, green_lower, green_upper)
        if blob != None:
            cv2.drawContours(frame, [blob], 0, (255, 0, 0), 2)
            x,y,bw,bh = cv2.boundingRect(blob)
            cv2.rectangle(frame,(x,y),(x+bw,y+bh),(0,255,0),2)
            approx = cv2.approxPolyDP(blob, 30, True)
            cv2.drawContours(frame, [approx], 0, (0, 0, 255), 2)
            cv2.line(frame, (frame.shape[1]/2,frame.shape[0]/2-plug_d_bh/2),(frame.shape[1]/2,frame.shape[0]/2+plug_d_bh/2), (0,0,255))
            cv2.circle(frame, (frame.shape[1]/2, frame.shape[0]/2), 3, (0,0,255), -1)
            cv2.putText(frame, 'State: {}, num_wrong: {}, bh: {}, bw: {}'.format(state, num_wrong, bh, bw), (0,100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0))
            if state == 'wait_target':
                cv2.putText(frame, 'press <space> to start', (0,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0))
            if state == 'find_center':
                next_move[0] = np.sign( int(((x+bw/2) - (frame.shape[1]/2))/20) ) #shape center minus frame center, accuracy of 20 pixels
                next_move[1] = -np.sign( int(((y+bh/2) - (frame.shape[0]/2))/20) )
                next_move[2] = -np.sign( int ((bh-plug_d_bh)/5)) #determine Z by the shape height, assuming no roll, accuracy of 5 pixels
                next_move[3] = find_rect_contour_orientation_alt(blob)
                if cur_move != next_move:
                    num_wrong += 1
                else:
                    num_wrong = 0
                cv2.putText(frame, 'moving to center, {}'.format(next_move), (0,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0))
                if next_move == [0,0,0,0]: #we found center!
                    stop_now(ser)
                    state = 'wait_1'
                    wait_start_time = time.time()
                elif num_wrong >= 5 or not still_moving(ser):
                    print 'change from ', cur_move, ' to ', next_move
                    if still_moving(ser):
                        print 'stop first'
                        stop_now(ser)
                    cur_angles = angles_from_pos(get_pos(ser))
                    cur_pos = xyz_from_angles(cur_angles)
                    next_pos = [0,0,0,0]
                    next_pos[0] = cur_pos[0] - next_move[3] * 1 #if I see left yaw, move right
                    next_pos[1] = cur_pos[1] + next_move[1] * 2 #if I see the center low, move low
                    next_pos[2] = cur_pos[2] + next_move[2] * 2 #if I see the blob far, move out
                    next_pos[3] = cur_angles[4] + next_move[0] * (20.0/180*pi) #if I see the blob left, rotate angle to left
                    print 'next_pos: ', next_pos
                    move_to_angle(ser, list(find_angles(next_pos[0:3],pi/2)) + [next_pos[3],pi], 1000)
                    cur_move = list(next_move)                                    
                    num_wrong = 0
            if state == 'wait_1':
                if time.time() - wait_start_time >= wait_1_time:
                    state = 'get_close'
                    cur_angles = angles_from_pos(get_pos(ser))
                    cur_pos = xyz_from_angles(cur_angles)
                    #move_to_angle(ser,list(find_angles(cur_pos,pi/2)) + [cur_angles[4],pi], 500)
                    angle = atan(cur_pos[0]/cur_pos[2])+cur_angles[4]
                    #calculate delta_x, delta_z assuming we're looking to the correct center
                    delta_z = get_close_l*cos(angle)
                    delta_x = get_close_l*sin(angle)
                    delta_y = 0
                    #but we're likely a bit off, so correct for that:
                    fix_x = float(((x+bw/2) -frame.shape[1]/2))/pixel_per_cm
                    fix_y = float(((y+bh/2) -frame.shape[0]/2))/pixel_per_cm
                    print 'x+bw/2 ', x+bw/2, ' x_center: ', frame.shape[1]/2
                    print '(fix_x, fix_y): ', (fix_x, fix_y)
                    next_pos = [0,0,0]
                    next_pos[0] = cur_pos[0]+delta_x+fix_x
                    next_pos[1] = cur_pos[1]+delta_y+fix_y
                    next_pos[2] = cur_pos[2]+delta_z
                    new_angle = angle - atan(next_pos[0]/next_pos[2])
                    print 'cur_pos: ', cur_pos
                    print 'cur_angles: ', cur_angles
                    print 'angle: ', angle
                    print 'next_pos: ', next_pos
                    print 'new_angle: ', new_angle
                    move_to_angle(ser,list(find_angles(next_pos,pi/2)) + [new_angle,pi], 2000)
            if state == 'get_close':
                if not still_moving(ser):
                    state = 'DONE'
        #ret2, frame2 = cap2.read()
        #img2 = cv2.resize(frame2, (frame.shape[1], frame.shape[0]))
        #both = np.concatenate((frame,img2), axis=1)
        #cv2.imshow('frame',both)
        #outFile.write(both)
        cv2.imshow('frame',frame)
    key = cv2.waitKey(1) & 0xFF
    if key == ord(' '):
        if state == 'wait_target':
            state = 'find_center'
            cur_move = [0,0,0,0]
            num_wrong = 0
        if state =='DONE':
            angles = list(find_angles(T_init,pi/2)) + [0,pi]
            #b0,b1,b2,b3 = find_angles(T_init,0)
            move_to_angle(ser, angles, 1000)
            state = 'wait_target'
            cur_move = [0,0,0,0]
    if key == ord('q'):
        break
    elif key == ord('f'):
        cv2.imwrite('frame-{}.jpg'.format(cnt), frame)
        cnt+=1
        cv2.waitKey(0)
# When everything done, release the capture
rest(ser)
cap.release()
#cap2.release()
#outFile.release()
cv2.destroyAllWindows()