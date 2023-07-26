import cv2
import numpy as np
import matplotlib.pyplot as plt

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
def get_slopes_intercept(lines):
    slopes = []
    intercepts = []

    try:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            #cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            slope = (y2-y1)/(x2-x1)
            slopes.append(slope)
            x_intercept = y1-slope*x1
            intercepts.append(x_intercept)
    except TypeError:
        pass

    return slopes, intercepts

# Part 4
def detect_lanes(lines):
    lanes = []

    slopes, intercepts = get_slopes_intercept(lines)

    #finished = False
    
    while len(slopes) > 4:
        for i in range(0, len(slopes)-1):
            if(abs(slopes[i]-slopes[i+1]) < 0.1):
                slopes.remove(slopes[i])
                i-=1
        break
    return slopes

# Part 5
def draw_lanes(img, lanes):
    try:
        for lane in lanes:
            x1, y1, x2, y2 = lane[0]
            cv2.line(img, (x1, y1), (x2, y2), (0,255,0), 2)
    except TypeError:
        pass
    plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))