import cv2
import numpy as np
import matplotlib.pyplot as plt
import random
import lane_detection


def pick_lane(lanes):
    '''
    helper function for get_lane_center
    finds the closet lane from a list of lanes
    
    '''

    maxDiff = 0
    for addedLanes in lanes:
        diff = abs(addedLanes[0][0]  - addedLanes[1][0])
        if (maxDiff < diff):
            maxDiff = diff
            pickedLane = addedLanes
    
    return pickedLane


def get_lane_center(lanes):
    '''
    takes a list of lanes as an input and returns the center of the closest lane and its slope

    parameters:
        lanes: the list of lanes to process

    
    '''

    clane = pick_lane(lanes)


    center = (clane[0][0]+clane[1][0])/2

    x1, y1, x2, y2 = clane[0]

    slope1 = (y1-y2)/(x1-x2)

    x1, y1, x2, y2 = clane[1]

    slope2 = (y1-y2)/(x1-x2)

    slope = (slope1+slope2)/2

    

    return center, slope


def recommend_direction(center, slope):
    '''
    takes the center of the closest lane and its slope as inputs and returns a direction


    parameters:
        center: the center of the closest lane
        slope: the slope of the closest lane
    
    
    '''

    halfOfRes = 4096/2
    if center == halfOfRes:
        direction = "forward"
    elif center > halfOfRes: # more than halfway
        direction = "right"
    else:
        direction = "left"

    if 1/slope > 0:
        print("turn right")
    if 1/slope < 0:
        print("turn Left")
   
    return direction