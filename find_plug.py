#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Fri Apr  7 18:26:05 2017

@author: rafi
"""

import numpy as np
import cv2

process_params = [[200,240,3,5,10]]

#process_params = [[150,230,3,5,10],
#                  [50,100,3,5,10],
#                  [200,240,3,5,10],
#                  [150,230,5,10,10],
#                  [50,100,5,10,10]]

cap = cv2.VideoCapture(3)
cnt = 0
while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    if (ret):
        blur = cv2.blur(frame, (3,3))
        #now remove everything that isn't black:
        hsv = cv2.cvtColor(blur,cv2.COLOR_BGR2HSV)
        [H, S, V] = cv2.split(hsv)
        ret,threshV = cv2.threshold(V,120,255,cv2.THRESH_BINARY_INV) #keep only dark regions, and invert
        ret,threshS = cv2.threshold(V,100,255,cv2.THRESH_BINARY_INV) #keep only low saturation values, and invert
        thresh = cv2.bitwise_and(threshV, threshS) #keep only places that are both dark and low saturation
        kernel = np.ones((3,3),np.uint8)
        closing = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel) #remove tiny noise
        #closing = cv2.bitwise_and(thresh, thresh)
        _, contours, _ = cv2.findContours(closing,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        
        #print len(contours)
        possible_contours = []
        big_circle = None
        
        print 'found ', len(contours), ' contours'
        for contour in contours: #of the dark patches, find one that has a circle of the right size
            mask = np.zeros(frame.shape[:-1], np.uint8)
            cv2.drawContours(mask, [contour], 0, 255, -1)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            pic = cv2.bitwise_and(gray, gray, mask=mask)
            equalized = cv2.convertScaleAbs(gray, None, 4, 0)
            blurred = cv2.medianBlur(equalized,5)
#            edged = cv2.Canny(blurred, 200, 230)
#            kernel = np.ones((3,3),np.uint8)
#            dilated = cv2.dilate(edged, kernel, iterations = 1)
            dilated = blurred
            
            rect = cv2.minAreaRect(contour) #returns (center, size, angle in degrees, 0 - (-90))
            max_w = max(rect[1])
            circles = cv2.HoughCircles(dilated,cv2.HOUGH_GRADIENT,1,50,
                                    param1=200,param2=20,minRadius=int(max_w*0.3),maxRadius=int(max_w*0.5))
            if circles != None and len(circles[0]) == 1:        
                possible_contours.append((contour,circles[0][0]))
                #break
        print 'of these ', len(possible_contours), ' are possible'
        if len(possible_contours) > 0:
            areas = [cv2.contourArea(contour) for (contour,c) in possible_contours]
            largest_possible_idx = np.argmax(areas)
            print 'best possible contour is index ', largest_possible_idx
            best_cnt = possible_contours[largest_possible_idx]
            big_circle = np.uint16(np.around(best_cnt[1]))
            if big_circle != None:
                mask = np.zeros(frame.shape[:-1], np.uint8)
                cv2.circle(mask,(big_circle[0],big_circle[1]),big_circle[2]+10,255,-1)
                masked = cv2.bitwise_and(blurred,blurred,mask=mask)
                #plug = masked[big_circle[1]-big_circle[2]-1:big_circle[1]+big_circle[2]+5,big_circle[0]-big_circle[2]-1:big_circle[0]+big_circle[2]+5]
                plug = masked
                for i, params in enumerate(process_params): #try several processing options
                    edged = cv2.Canny(plug, params[0], params[1])
                    kernel = np.ones((params[2],params[2]),np.uint8)
                    #dilated = cv2.dilate(edged, kernel, iterations = 1)
                    dilated = edged
        #            ret,thresh_plug = cv2.threshold(plug, 150, 255, cv2.THRESH_BINARY)
        #            edged = cv2.Canny(thresh_plug, 200, 230)
        #            dilated = thresh_plug
                    
                    small_circles = cv2.HoughCircles(dilated,cv2.HOUGH_GRADIENT,1,params[3],
                                                     param1=100,param2=params[4],minRadius=int(big_circle[2]/5),maxRadius=int(big_circle[2]/4))
                    
                    filtered = cv2.cvtColor(plug,cv2.COLOR_GRAY2BGR)
                    # draw the big circle
                    cv2.circle(filtered,(big_circle[0],big_circle[1]),big_circle[2],(0,255,0),2)
                    
                    if small_circles != None:
                        #print len(small_circles)
                        for small in small_circles[0]:
                            cv2.circle(filtered,(small[0],small[1]),small[2],(255,0,0),2)
            
                    cv2.imshow('processed-{}'.format(i),filtered)
                    cv2.imshow('dilated-{}'.format(i),dilated)
    cv2.imshow('frame',thresh)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('f'):
        cv2.imwrite('frame-{}.jpg'.format(cnt), frame)
        cnt+=1
        cv2.waitKey(0)
# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()