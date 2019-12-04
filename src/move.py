import cv2
import numpy as np
import time
import RPi.GPIO as GPIO

###### CONSTANT VALUE CONFIGURATION ######
COLOR = "yellow"

#COLOR MASK
R_lower = np.array([150, 64, 30])
R_upper = np.array([180, 255, 255])
Y_lower = np.array([15, 80, 120])
Y_upper = np.array([40, 200, 255])
G_lower = np.array([60, 0, 30])
G_upper = np.array([90, 80, 255])

#AREA and DISTANCE
R_AREA_REFERENCE = 183  # [px]
R_DIST_REFERENCE = 4870   # [mm]
Y_AREA_REFERENCE = 183  # [px]
Y_DIST_REFERENCE = 4870   # [mm]
G_AREA_REFERENCE = 183  # [px]
G_DIST_REFERENCE = 4870   # [mm]

#target
DIST_TARGET = -2000 # [mm]

#GAIN
DIST_P_GAIN = 0.01
ANGLE_P_GAIN = 0.04

##########################################

# conf
if COLOR == "red":
    lower = R_lower
    upper = R_upper
    AREA_REFERENCE = R_AREA_REFERENCE
    DIST_REFERENCE = R_DIST_REFERENCE
elif COLOR == "green":
    lower = G_lower
    upper = G_upper
    AREA_REFERENCE = G_AREA_REFERENCE
    DIST_REFERENCE = G_DIST_REFERENCE
else:
    lower = Y_lower
    upper = Y_upper
    AREA_REFERENCE = Y_AREA_REFERENCE
    DIST_REFERENCE = Y_DIST_REFERENCE


def main():
    cap = cv2.VideoCapture(0)

    while cap.isOpened():
        # get a frame
        _, frame = cap.read()
        height, width, channels = frame.shape

        # BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower, upper)

        # calc mask
        target = analysis_blob(mask)

        # get areas and calc dist
        area = target["area"]
        if area != 0:
            rate = AREA_REFERENCE/area
        else:
            rate = 0

        dist = DIST_REFERENCE * np.sqrt(rate)
        # print(dist)

        if dist < 400:
            break

        # get center
        if area != 0:
            x = int(target["center"][0])
            y = int(target["center"][1])
        else:
            x = int(width/2)
            y = 0
        cv2.circle(frame, (x,y), 4, 255, 2, 4)

        cv2.imshow('frame' , frame)
        cv2.imshow('mask', mask)

        # calc error
        e_dist = dist - DIST_TARGET
        e_angle = x - width/2

        # calc input [P]
        l_input = e_dist * DIST_P_GAIN + e_angle * ANGLE_P_GAIN
        r_input = e_dist * DIST_P_GAIN - e_angle * ANGLE_P_GAIN

        print("e_dist = " + str(dist) + " ,e_x = " + str(e_angle) + " -> " + str(l_input) + ", " + str(r_input))

        run(l_input, r_input)

        # Esc
        if cv2.waitKey(5) & 0xFF == 27:
            break

    cap.release()
    cv2.destroyAllWindows()


def analysis_blob(binary_img):
    mu = cv2.moments(binary_img, False)
    if mu["m00"] != 0:
        x, y = int(mu["m10"]/mu["m00"]) , int(mu["m01"]/mu["m00"])
    else:
        x, y = -1, -1

    maxblob = {}

    maxblob["area"] = cv2.countNonZero(binary_img)
    maxblob["center"] = (x, y)

    return maxblob


def run(l, r):
    if l > 0:
        if l > 100:
            l = 100
        pwmL.ChangeDutyCycle(0)
        GPIO.output(17, 1)
        GPIO.output(27, 0)
        pwmL.ChangeDutyCycle(l)
    else:
        if l < -100:
            l = -100
        pwmL.ChangeDutyCycle(0)
        GPIO.output(17, 0)
        GPIO.output(27, 1)
        pwmL.ChangeDutyCycle(-l)
    if r > 0:
        if r > 100:
            r = 100
        pwmR.ChangeDutyCycle(0)
        GPIO.output(9, 0)
        GPIO.output(10, 1)
        pwmR.ChangeDutyCycle(r)
    else:
        if r < -100:
            r = -100
        pwmR.ChangeDutyCycle(0)
        GPIO.output(9, 1)
        GPIO.output(10, 0)
        pwmR.ChangeDutyCycle(-r)


if __name__ == '__main__':
    time.sleep(3)

    GPIO.setmode(GPIO.BCM)

    GPIO.setup(22, GPIO.OUT)
    GPIO.setup(17, GPIO.OUT)
    GPIO.setup(27, GPIO.OUT)
    GPIO.setup(9, GPIO.OUT)
    GPIO.setup(10, GPIO.OUT)
    GPIO.setup(0, GPIO.OUT)
    GPIO.setup(24, GPIO.OUT)

    pwmL = GPIO.PWM(0, 1000)
    pwmR = GPIO.PWM(22, 1000)
    pwmL.start(0)
    pwmR.start(0)
    GPIO.output(24, 1)

    main()

    pwmL.stop()
    pwmR.stop()

    GPIO.cleanup()
    time.sleep(1)


