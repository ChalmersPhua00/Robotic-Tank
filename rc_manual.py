import curses
import RPi.GPIO as GPIO
from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory
import cv2
from picamera2 import Picamera2
import time, libcamera

camera = Picamera2()
config = camera.create_preview_configuration(main={"size": (640, 480)})
config["transform"] = libcamera.Transform(vflip=1)
camera.configure(config)
camera.start()
camera.resolution = (640, 480)
camera.framerate = 30
time.sleep(2)

GPIO.setmode(GPIO.BOARD)

factory = PiGPIOFactory()
servo1 = Servo(13, pin_factory=factory)
servo2 = Servo(12, pin_factory=factory)

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

screen = curses.initscr()
curses.noecho()
curses.cbreak()
curses.halfdelay(1)
screen.keypad(True)

try:
    while True:
        image = camera.capture_array()
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        cv2.imshow('Camera', image)
        ret, buffer = cv2.imencode('.jpg', image)
        char = screen.getch()
        if char == ord('q') or cv2.waitKey(1) & 0xFF == ord('q'):
            break
        elif char == 119:
            GPIO.output(motor1a, GPIO.HIGH)
            GPIO.output(motor1b, GPIO.LOW)
            GPIO.output(motor1e, GPIO.HIGH)
            GPIO.output(motor2a, GPIO.HIGH)
            GPIO.output(motor2b, GPIO.LOW)
            GPIO.output(motor2e, GPIO.HIGH)
            while screen.getch() == 119:
                pass
            GPIO.output(motor1e, GPIO.LOW)
            GPIO.output(motor2e, GPIO.LOW)
        elif char == 115:
            GPIO.output(motor1a, GPIO.LOW)
            GPIO.output(motor1b, GPIO.HIGH)
            GPIO.output(motor1e, GPIO.HIGH)
            GPIO.output(motor2a, GPIO.LOW)
            GPIO.output(motor2b, GPIO.HIGH)
            GPIO.output(motor2e, GPIO.HIGH)
            while screen.getch() == 115:
                pass
            GPIO.output(motor1e, GPIO.LOW)
            GPIO.output(motor2e, GPIO.LOW)
        elif char == 97:
            GPIO.output(motor1a, GPIO.LOW)
            GPIO.output(motor1b, GPIO.HIGH)
            GPIO.output(motor1e, GPIO.HIGH)
            GPIO.output(motor2a, GPIO.HIGH)
            GPIO.output(motor2b, GPIO.LOW)
            GPIO.output(motor2e, GPIO.HIGH)
            while screen.getch() == 97:
                pass
            GPIO.output(motor1e, GPIO.LOW)
            GPIO.output(motor2e, GPIO.LOW)
        elif char == 100:
            GPIO.output(motor1a, GPIO.HIGH)
            GPIO.output(motor1b, GPIO.LOW)
            GPIO.output(motor1e, GPIO.HIGH)
            GPIO.output(motor2a, GPIO.LOW)
            GPIO.output(motor2b, GPIO.HIGH)
            GPIO.output(motor2e, GPIO.HIGH)
            while screen.getch() == 100:
                pass
            GPIO.output(motor1e, GPIO.LOW)
            GPIO.output(motor2e, GPIO.LOW)
        elif char == curses.KEY_UP:
            servo2.max()
        elif char == curses.KEY_DOWN:
            servo1.mid()
            servo2.mid()
        elif char == curses.KEY_LEFT:
            servo1.min()
        elif char == curses.KEY_RIGHT:
            servo1.max()

finally:
    curses.nocbreak()
    screen.keypad(False)
    curses.echo()
    curses.endwin()
    camera.close()
    cv2.destroyAllWindows()
    GPIO.cleanup()
