import cv2
import numpy as np

###### CONSTANT VALUE CONFIGURATION ######
#COLOR MASK
lower = np.array([150, 128, 30])
upper = np.array([180, 255, 255])
#GET ANGLE
ANGLE_THRESHOLD = 1.05
##########################################

cap = cv2.VideoCapture(0)

while cap.isOpened():
    # get a frame
    _, frame = cap.read()
    height, width, channels = frame.shape

    # BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)

    # calc mask
    mu = cv2.moments(mask, False)

    if mu["m00"] != 0:
        x, y = int(mu["m10"]/mu["m00"]) , int(mu["m01"]/mu["m00"])
    else :
        x, y = -1, -1

    cv2.circle(frame, (x,y), 4, 255, 2, 4)

    cv2.imshow('frame' , frame)
    cv2.imshow('mask', mask)

    #calc areas and angle
    area = cv2.countNonZero(mask)

    leftArea = -1
    rightArea = -1
    if x != -1 and y != -1:
        leftMask = mask[:, 0:x]
        
        leftArea = cv2.countNonZero(leftMask)
        rightArea = area - leftArea

    print(str(leftArea) + ":" + str(rightArea))
    if leftArea <= 0 or rightArea <= 0:
        print("None")
    elif 1/ANGLE_THRESHOLD < leftArea/rightArea and leftArea/rightArea < ANGLE_THRESHOLD:
        print("Front")
    else:
        if leftArea > rightArea:
            print("Left")
        else:
            print("Right")

    #Esc
    if cv2.waitKey(5) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
