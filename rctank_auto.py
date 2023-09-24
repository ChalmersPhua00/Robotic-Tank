import RPi.GPIO as GPIO
from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory
import cv2
from picamera import PiCamera
from picamera.array import PiRGBArray
import time

factory = PiGPIOFactory()
servo1 = Servo(13, pin_factory=factory)
servo2 = Servo(12, pin_factory=factory)

camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 30
raw_capture = PiRGBArray(camera, size=(640, 480))
time.sleep(2)

GPIO.setmode(GPIO.BOARD)

motor1a = 7
motor1b = 11
motor1e = 22
motor2a = 13
motor2b = 16
motor2e = 15

GPIO.setup(motor1a, GPIO.OUT)
GPIO.setup(motor1b, GPIO.OUT)
GPIO.setup(motor1e, GPIO.OUT)
GPIO.setup(motor2a, GPIO.OUT)
GPIO.setup(motor2b, GPIO.OUT)
GPIO.setup(motor2e, GPIO.OUT)

cap = cv2.VideoCapture(0)
face_classifier = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')

try:
    for frame in camera.capture_continuous(raw_capture, format="bgr", use_video_port=True):
        img = frame.array
        gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        faces = face_classifier.detectMultiScale(gray_img, scaleFactor=1.1, minNeighbors=1)
        if len(faces) > 0:
            x, y, w, h = faces[0]
            face_center_x = x + w // 2
            face_center_y = y + h // 2
            region_width = 640 // 3
            region_height = 480 // 3
            region_x = face_center_x // region_width
            region_y = face_center_y // region_height
            cv2.rectangle(img, (x, y), (x + w, y + h), (15, 150, 50), 2)
            cv2.circle(img, (face_center_x, face_center_y), 5, (15, 150, 50), -1)
            if region_x == 0 and region_y == 0: # Top Left
                print("Top Left")
                if servo1.value > -0.9:
                    servo1.value -= 0.1
                if servo2.value < 0.9:
                    servo2.value += 0.1
            elif region_x == 1 and region_y == 0: # Top Center
                print("Top Center")
                if servo2.value < 0.9:
                    servo2.value += 0.1
            elif region_x == 2 and region_y == 0: # Top Right
                print("Top Right")
                if servo1.value < 0.9:
                    servo1.value += 0.1
                if servo2.value < 0.9:
                    servo2.value += 0.1
            elif region_x == 0 and region_y == 1: # Center Left
                print("Center Left")
                if servo1.value > -0.9:
                    servo1.value -= 0.1
            elif region_x == 1 and region_y == 1: # Center Center
                print("Center Center")
            elif region_x == 2 and region_y == 1: # Center Right
                print("Center Right")
                if servo1.value < 0.9:
                    servo1.value += 0.1
            elif region_x == 0 and region_y == 2: # Bottom Left
                print("Bottom Left")
                if servo1.value > -0.9:
                    servo1.value -= 0.1
                if servo2.value > -0.9:
                    servo2.value -= 0.1
            elif region_x == 1 and region_y == 2: # Bottom Center
                print("Bottom Center")
                if servo2.value > -0.9:
                    servo2.value -= 0.1
            elif region_x == 2 and region_y == 2: # Bottom Right
                print("Bottom Right")
                if servo1.value < 0.9:
                    servo1.value += 0.1
                if servo2.value > -0.9:
                    servo2.value -= 0.1
            if w < 100 and h < 100:
                GPIO.output(motor1a, GPIO.HIGH)
                GPIO.output(motor1b, GPIO.LOW)
                GPIO.output(motor1e, GPIO.HIGH)
                GPIO.output(motor2a, GPIO.HIGH)
                GPIO.output(motor2b, GPIO.LOW)
                GPIO.output(motor2e, GPIO.HIGH)
            else:
                GPIO.output(motor1e, GPIO.LOW)
                GPIO.output(motor2e, GPIO.LOW)
        else:
            GPIO.output(motor1e, GPIO.LOW)
            GPIO.output(motor2e, GPIO.LOW)
        cv2.imshow('Webcam', img)
        if cv2.waitKey(1) == ord('q'):
            break
        raw_capture.truncate(0)

finally:
    camera.close()
    cap.release()
    cv2.destroyAllWindows()
    GPIO.cleanup()
