import cv2
from random import randrange
import numpy as np
import matplotlib.pyplot as plt
#from dt_apriltags import Detector
from lane_detection import *
import matplotlib.cm as cm
from PIDcontrols import *
from statistics import median


def get_lane_center(lanes):
    center_intercept = (lanes[0][0]+lanes[1][0])/2
    x1, y1, x2, y2 = lanes[0]
    slope1 = (y1-y2)/(x1-x2)
    x1, y1, x2, y2 = lanes[1]
    slope2 = (y1-y2)/(x1-x2)
    center_slope = 1/(((1/slope1)+(1/slope2))/2)
    center_intercept = ((((1080 - y2)/center_slope)  )+ x2)
    return center_intercept, center_slope

def draw_center_lane(img, center_intercept, center_slope, xPoint = 0, yPoint = 0):
    global imgPixelHeight 
    imgPixelHeight  = img.shape[0]
    cv2.line(img, (int(center_intercept), imgPixelHeight), (int(xPoint), int(yPoint)), (0,0,255), 6)
    return img

def recommend_direction(center, slope):
    
    halfOfRes = 1920/2
    HorizontalDiff = halfOfRes-center
    centerTolerance = 2.5
    if abs(HorizontalDiff) < centerTolerance:
        direction = "Go Forward!"
    elif HorizontalDiff > 0:# more than halfway
        #print("strafe right")
        direction = f"Strafe Left by {HorizontalDiff}"
    else:
        #print("strafe left")
        direction = f"Strafe Right by {HorizontalDiff}"

    AproxAUVAngle = 90 - angle_between_lines(slope, 0)  
    # get the approx angle of the auv with the line by calculating the slope of the 
    #center line with a horizontal line relative to the rov
    if slope > 0:
        AproxAUVAngle = -AproxAUVAngle
        direction += f" turn Left by: {AproxAUVAngle} degrees"
        pass#print("turn right")
    if slope < 0:
        direction += f" turn Right by: {AproxAUVAngle} degrees"
        pass#print("turn Left")
    return direction

def low_pass_filter_moving_median(input_data, window_size):
    """
    Apply a first-order low-pass filter using the Moving Median method to the input data.

    Parameters:
        input_data (list or numpy array): Input data to be filtered.
        window_size (int): The size of the moving window used for computing the median.

    Returns:
        list: Filtered output data.
    """
    half_window = window_size // 2
    filtered_data = []

    for i in range(len(input_data)):
        start_idx = max(0, i - half_window)
        end_idx = min(len(input_data), i + half_window + 1)
        window = input_data[start_idx:end_idx]
        filtered_value = median(window)
        filtered_data.append(filtered_value)

    return filtered_data

def PIDoutputPosition(horizontal_diff, pid):
    pid_input = low_pass_filter_moving_median(horizontal_diff, 100)
    #pid = PID(50, 1, -27.5, 100)

    pid_output = pid.update(pid_input)
    return pid_output

#def thrusterDirections(center, slope):

def set_control(horizontal_diff, pid_output):
    # Calculate the power contribution for a thruster based on its angle and PID output
    thruster_power = pid_output * np.cos(np.pi/4)

    # Scale the power to the range of -100 to 100
    scaled_power = thruster_power * 100.0

    # Limit the scaled power to the range of -100 to 100
    #power_percentage = max(min(scaled_power, 100.0), -100.0)
    power_percentage = np.clip(scaled_power, -100, 100)

    thruster_magnitudes = []
    if(horizontal_diff < 0):
        thruster_magnitudes = [-power_percentage, power_percentage, -power_percentage, power_percentage, 0, 0]
    elif(horizontal_diff > 0):
        thruster_magnitudes = [power_percentage, -power_percentage, power_percentage, -power_percentage, 0, 0]
    else:
        thruster_magnitudes = [0, 0, 0, 0, 0, 0]
    return thruster_magnitudes
