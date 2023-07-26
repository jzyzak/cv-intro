import cv2
import numpy as np
import matplotlib.pyplot as plt
from random import randrange
import lane_detection

def get_lane_center(lanes):
    center_intercept = (lanes[0][0]+lanes[1][0])/2
    x1, y1, x2, y2 = lanes[0]
    slope1 = (y1-y2)/(x1-x2)
    x1, y1, x2, y2 = lanes[1]
    slope2 = (y1-y2)/(x1-x2)
    center_slope = (slope1+slope2)/2
    return center_intercept, center_slope

def recommend_direction(center, slope):
    
    halfOfRes = 1920/2
    if center == halfOfRes:
        direction = "forward"
    elif center > halfOfRes:# more than halfway
        print("strafe right")
        direction = "right"
    else:
        print("strafe left")
        direction = "left"
    if 1/slope > 0:
        print("turn right")
    if 1/slope < 0:
        print("turn Left")
    return direction
    