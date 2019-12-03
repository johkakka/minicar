import cv2
import numpy as np

###### CONSTANT VALUE CONFIGURATION ######
#COLOR MASK
lower = np.array([150, 128, 30])
upper = np.array([180, 255, 255])

#AREA and DISTANCE
AREA_REFERENCE = 20000  # [px]
DIST_REFERENCE = 1000   # [mm]
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

    # calc areas and dist
    area = cv2.countNonZero(mask)
    if area != 0:
        rate = AREA_REFERENCE/area
    else:
        rate = np.Infinity

    dist = DIST_REFERENCE * np.sqrt(rate)
    print(dist)

    # Esc
    if cv2.waitKey(5) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
