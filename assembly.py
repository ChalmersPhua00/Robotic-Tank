import curses
import RPi.GPIO as GPIO
from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory
import cv2
from picamera import PiCamera
from picamera.array import PiRGBArray
import time

camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 30
raw_capture = PiRGBArray(camera, size=(640, 480))
time.sleep(2)

GPIO.setmode(GPIO.BOARD)

factory = PiGPIOFactory()
servo = Servo(12, pin_factory=factory)

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
    for frame in camera.capture_continuous(raw_capture, format="bgr", use_video_port=True):
        image = frame.array
        cv2.imshow("Camera", image)
        raw_capture.truncate(0)
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
            servo.mid()
        elif char == curses.KEY_LEFT:
            servo.min()
        elif char == curses.KEY_RIGHT:
            servo.max()

finally:
    curses.nocbreak()
    screen.keypad(False)
    curses.echo()
    curses.endwin()
    camera.close()
    cv2.destroyAllWindows()
    GPIO.cleanup()