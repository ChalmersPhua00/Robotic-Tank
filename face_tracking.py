from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory
import cv2
from picamera2 import Picamera2
import time, libcamera

factory = PiGPIOFactory()
servo1 = Servo(13, pin_factory=factory)
servo2 = Servo(12, pin_factory=factory)

camera = Picamera2()
config = camera.create_preview_configuration(main={"size": (640, 480)})
config["transform"] = libcamera.Transform(vflip=1)
camera.configure(config)
camera.start()
camera.resolution = (640, 480)
camera.framerate = 30
time.sleep(2)

cap = cv2.VideoCapture(0)
face_classifier = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')

try:
    while True:
        img = camera.capture_array()
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        ret, buffer = cv2.imencode('.jpg', img)
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
                if servo1.value < 0.9:
                    servo1.value += 0.1
                if servo2.value < 0.9:
                    servo2.value += 0.1
            elif region_x == 1 and region_y == 0: # Top Center
                print("Top Center")
                if servo2.value < 0.9:
                    servo2.value += 0.1
            elif region_x == 2 and region_y == 0: # Top Right
                print("Top Right")
                if servo1.value > -0.9:
                    servo1.value -= 0.1
                if servo2.value < 0.9:
                    servo2.value += 0.1
            elif region_x == 0 and region_y == 1: # Center Left
                print("Center Left")
                if servo1.value < 0.9:
                    servo1.value += 0.1
            elif region_x == 1 and region_y == 1: # Center Center
                print("Center Center")
            elif region_x == 2 and region_y == 1: # Center Right
                print("Center Right")
                if servo1.value > -0.9:
                    servo1.value -= 0.1
            elif region_x == 0 and region_y == 2: # Bottom Left
                print("Bottom Left")
                if servo1.value < 0.9:
                    servo1.value += 0.1
                if servo2.value > -0.9:
                    servo2.value -= 0.1
            elif region_x == 1 and region_y == 2: # Bottom Center
                print("Bottom Center")
                if servo2.value > -0.9:
                    servo2.value -= 0.1
            elif region_x == 2 and region_y == 2: # Bottom Right
                print("Bottom Right")
                if servo1.value > -0.9:
                    servo1.value -= 0.1
                if servo2.value > -0.9:
                    servo2.value -= 0.1
        cv2.imshow('Camera', img)
        if cv2.waitKey(1) == ord('q'):
            break

finally:
    camera.close()
    cv2.destroyAllWindows()
