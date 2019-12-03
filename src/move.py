import cv2
import numpy as np
import time
import RPi.GPIO as GPIO

###### CONSTANT VALUE CONFIGURATION ######
#COLOR MASK
lower = np.array([150, 128, 30])
upper = np.array([180, 255, 255])

#AREA and DISTANCE
AREA_REFERENCE = 8500  # [px]
DIST_REFERENCE = 1000   # [mm]

#target
DIST_TARGET = -1200 # [mm]

#GAIN
DIST_P_GAIN = 0.003
ANGLE_P_GAIN = 0.001

#othor
d = 70
##########################################

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
        l_input = e_dist * DIST_P_GAIN + d * e_angle * ANGLE_P_GAIN
        r_input = e_dist * DIST_P_GAIN - d * e_angle * ANGLE_P_GAIN

        print("e_dist = " + str(e_dist) + " ,e_x = " + str(e_angle) + " -> " + str(l_input) + ", " + str(r_input))

        run(l_input, r_input)

        # Esc
        if cv2.waitKey(5) & 0xFF == 27:
            break

    cap.release()
    cv2.destroyAllWindows()


def analysis_blob(binary_img):
    if cv2.countNonZero(binary_img) <= 0:
        maxblob = {}

        maxblob["upper_left"] = {0, 0}
        maxblob["width"] = 0
        maxblob["height"] = 0
        maxblob["area"] = 0
        maxblob["center"] = {0, 0}

        return maxblob

    label = cv2.connectedComponentsWithStats(binary_img)

    n = label[0] - 1
    data = np.delete(label[2], 0, 0)
    center = np.delete(label[3], 0, 0)

    max_index = np.argmax(data[:, 4])

    maxblob = {}

    maxblob["upper_left"] = (data[:, 0][max_index], data[:, 1][max_index])
    maxblob["width"] = data[:, 2][max_index]
    maxblob["height"] = data[:, 3][max_index]
    maxblob["area"] = data[:, 4][max_index]
    maxblob["center"] = center[max_index]

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
        if r < 100:
            r = -100
        pwmR.ChangeDutyCycle(0)
        GPIO.output(9, 1)
        GPIO.output(10, 0)
        pwmR.ChangeDutyCycle(-r)

    time.sleep(0.1)


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


