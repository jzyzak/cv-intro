import cv2
import numpy as np
import matplotlib.pyplot as plt
from random import randrange

# Problem 1

def literallyEverything(img, threshold1=90, threshold2=100, apertSize=3, minLength=10, maxGap=60, color=(0,255,0)):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, threshold1, threshold2, apertureSize=apertSize) 
    lines = cv2.HoughLinesP(
                    edges, #described above
                    1, #1 pixel resolution parameter
                    np.pi/180, # 1 degree resolution parameter
                    60, #min number of intersections/votes
                    minLineLength=minLength,
                    maxLineGap=maxGap,
            ) # detect lines
    try:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(img, (x1, y1), (x2, y2), color, 2)
            slope = (y2-y1)/(x2-x1)
            print(str(slope))
    except TypeError:
        pass
    
    return img

# Part 1
def detect_lines(img, threshold1=50, threshold2=150, apertureSize=3, minLineLength=100, maxLineGap=10):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, threshold1, threshold2, apertureSize)
    plt.imshow(edges)
    lines = cv2.HoughLinesP(
                    edges, #described above
                    rho = 1, #1 pixel resolution parameter
                    theta = np.pi/180, # 1 degree resolution parameter
                    threshold = 125, #min number of intersections/votes
                    minLineLength=minLineLength,
                    maxLineGap=maxLineGap,
            ) # detect lines
    return lines

# Part 2
def draw_lines(img, lines, color=(0,255,0)):
    try:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(img, (x1, y1), (x2, y2), color, 2)
    except TypeError:
        pass
    plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    plt.show()

# Part 3
def get_slopes_intercepts(lines):
    '''
    takes a list of lines as an input and returns a list of slopes and a list of intercepts

    parameters:
        lines: the list of lines to process
    
    '''

    slopes = []
    intercepts = []

    for line in lines:
        x1, y1, x2, y2 = line[0]
        slope = (y2-y1)/(x2-x1)
        slopes.append(slope)
        intercept = ((((2160 - y1)/slope)  )+ x1)
        intercepts.append(intercept)

    return slopes, intercepts

# Part 4
def detect_lanes(lines):
    slopeList, xInterceptList = get_slopes_intercepts(lines)
    print (f"slopeList:{slopeList}")
    print (f"xInterceptList:{xInterceptList}")
    lanes = []
    #check of the lines intersect on the screen
    if len(slopeList)> 1:
        for i in range(0,len(slopeList)):
            # if (len(slopeList) > 1):
            #     i += 1
            #     print("added i")
            for j in range (i+1,len(slopeList)):
                print(f"DistREQ:{abs(xInterceptList[i]-xInterceptList[j])}")
                print(f"slopeREQ:{abs(1/ slopeList[i]-1 /slopeList[j])}")
                if(abs(xInterceptList[i]-xInterceptList[j])< 10000 and abs(1/ slopeList[i]-1 /slopeList[j]) < 1):
                    
                    xPoint = ((slopeList[i] * xInterceptList[i]) - (slopeList[j] * xInterceptList[j]))/(slopeList[i]-slopeList[j])
                    yPoint = slopeList[i]*(xPoint - xInterceptList[i]) + 2160
                    
                    # avgSlope = (slopeList[i]+ slopeList[j])/2
                    # avgInterecept = (xInterceptList[i]+xInterceptList[j])/2
                    lane1 = [xInterceptList[i], 2160, xPoint,yPoint]
                    lane2 = [xInterceptList[j], 2160, xPoint,yPoint]
                    addedlanes = [lane1,lane2]
                    #print (f"thiasdfee:{(slopeList[i] * xInterceptList[i]) - slopeList[j] * xInterceptList[j]}")
                    lanes.append(addedlanes)


            #lanes.append(lane)

            #

            # if (yPoint> -500 and yPoint< 1080):
            #     avgInterceptX = (xInterceptList[i] + xInterceptList[j])/2
            #     lane = [xPoint.item(), avgInterceptX.item(), yPoint.item(), 1080.00]
            #     lanes.append(lane)

    return lanes

def pick_lane(lanes):
    maxDiff = 0
    for addedLanes in lanes:
        diff = abs(addedLanes[0][0]  - addedLanes[1][0])
        if (maxDiff < diff):
            maxDiff = diff
            pickedLane = addedLanes
    print(f"picked: {pickedLane}")
    return pickedLane

def draw_lanes(img,lanes,color = (255, 0, 0)):
    for addedLanes in lanes:
        color = (randrange(255),randrange(255),randrange(255))
        for lane in addedLanes:
            
            x1, y1, x2, y2 = lane
            print ("type(x1)")
            print (lane)
            cv2.line(img, (int(x1), int(y1)), (int(x2), int(y2)), color, 6)
    return img

def draw_Single_lane(img,lanes,color = (255, 0, 0)):
    color = (randrange(255),randrange(255),randrange(255))
    for lane in lanes:
        
        x1, y1, x2, y2 = lane
        print ("type(x1)")
        print (lane)
        cv2.line(img, (int(x1), int(y1)), (int(x2), int(y2)), color, 6)
    return img