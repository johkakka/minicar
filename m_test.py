import time
import RPi.GPIO as GPIO

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


pwmL.ChangeDutyCycle(0)
GPIO.output(17, 0)
GPIO.output(27, 1)
pwmR.ChangeDutyCycle(0)
GPIO.output(9, 1)
GPIO.output(10, 0)


pwmL.ChangeDutyCycle(100)
pwmR.ChangeDutyCycle(100)
time.sleep(3)

GPIO.output(17, 0)
GPIO.output(27, 0)
GPIO.output(9, 0)
GPIO.output(10, 0)
time.sleep(3)

pwmL.ChangeDutyCycle(0)
GPIO.output(17, 1)
GPIO.output(27, 0)
pwmR.ChangeDutyCycle(0)
GPIO.output(9, 0)
GPIO.output(10, 1)

pwmL.ChangeDutyCycle(100)
pwmR.ChangeDutyCycle(100)
time.sleep(3)

GPIO.output(17, 1)
GPIO.output(27, 1)
GPIO.output(9, 1)
GPIO.output(10, 1)
time.sleep(3)

GPIO.output(17, 0)
GPIO.output(27, 0)
GPIO.output(9, 0)
GPIO.output(10, 0)
pwmL.stop()
pwmR.stop()

GPIO.cleanup()
time.sleep(1)