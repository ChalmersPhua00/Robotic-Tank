import curses
import RPi.GPIO as GPIO
from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory
import cv2
from picamera2 import Picamera2
import time
import libcamera
import pygame
import numpy as np

# Initialize the camera
camera = Picamera2()
camera.start()
camera.resolution = (640, 480)
camera.framerate = 30
time.sleep(2)

# Set up GPIO
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

# Set up PWM for motor enable pins
motor1_pwm = GPIO.PWM(motor1e, 1000)
motor2_pwm = GPIO.PWM(motor2e, 1000)
motor1_pwm.start(0)
motor2_pwm.start(0)

pygame.init()
screen = pygame.display.set_mode((640, 480))
pygame.display.set_caption("Robotic Tank")

font = pygame.font.SysFont("Arial", 24)

speed = 50
stationary = True

def set_motor_speed(pwm, s):
    pwm.ChangeDutyCycle(s)

try:
    while True:
        # Capture image from the camera
        image = camera.capture_array()
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        # Convert the image for Pygame
        image = np.flip(image, axis=2) # Convert from BGR to RGB
        image = pygame.surfarray.make_surface(image)
        image = pygame.transform.rotate(image, 90) # The camera feed initially appears to be rotated 90 degrees clockwise
        # Display the image
        screen.blit(image, (0, 0))
        # Display the current speed
        speed_text = font.render(f"Speed: {speed}", True, (255, 255, 255)) # True enables anti-aliasing which makes the text smoother
        screen.blit(speed_text, (10, 10)) # Draws speed_text onto the Pygame screen at (10, 10) the top-left corner
        pygame.display.update()
        # Handle events (key presses)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                raise KeyboardInterrupt
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_q:
                    raise KeyboardInterrupt
                elif event.key == pygame.K_w:
                    GPIO.output(motor1a, GPIO.HIGH)
                    GPIO.output(motor1b, GPIO.LOW)
                    GPIO.output(motor1e, GPIO.HIGH)
                    GPIO.output(motor2a, GPIO.HIGH)
                    GPIO.output(motor2b, GPIO.LOW)
                    GPIO.output(motor2e, GPIO.HIGH)
                    set_motor_speed(motor1_pwm, speed)
                    set_motor_speed(motor2_pwm, speed)
                    stationary = False
                elif event.key == pygame.K_s:
                    GPIO.output(motor1a, GPIO.LOW)
                    GPIO.output(motor1b, GPIO.HIGH)
                    GPIO.output(motor1e, GPIO.HIGH)
                    GPIO.output(motor2a, GPIO.LOW)
                    GPIO.output(motor2b, GPIO.HIGH)
                    GPIO.output(motor2e, GPIO.HIGH)
                    set_motor_speed(motor1_pwm, speed)
                    set_motor_speed(motor2_pwm, speed)
                    stationary = False
                elif event.key == pygame.K_a:
                    GPIO.output(motor1a, GPIO.LOW)
                    GPIO.output(motor1b, GPIO.HIGH)
                    GPIO.output(motor1e, GPIO.HIGH)
                    GPIO.output(motor2a, GPIO.HIGH)
                    GPIO.output(motor2b, GPIO.LOW)
                    GPIO.output(motor2e, GPIO.HIGH)
                    set_motor_speed(motor1_pwm, speed)
                    set_motor_speed(motor2_pwm, speed)
                    stationary = False
                elif event.key == pygame.K_d:
                    GPIO.output(motor1a, GPIO.HIGH)
                    GPIO.output(motor1b, GPIO.LOW)
                    GPIO.output(motor1e, GPIO.HIGH)
                    GPIO.output(motor2a, GPIO.LOW)
                    GPIO.output(motor2b, GPIO.HIGH)
                    GPIO.output(motor2e, GPIO.HIGH)
                    set_motor_speed(motor1_pwm, speed)
                    set_motor_speed(motor2_pwm, speed)
                    stationary = False
                elif event.key == pygame.K_SPACE:
                    set_motor_speed(motor1_pwm, 0)
                    set_motor_speed(motor2_pwm, 0)
                    stationary = True
                elif event.key == pygame.K_UP:
                    servo2.max()
                elif event.key == pygame.K_DOWN:
                    servo1.mid()
                    servo2.mid()
                elif event.key == pygame.K_LEFT:
                    servo1.min()
                elif event.key == pygame.K_RIGHT:
                    servo1.max()
                elif event.key == pygame.K_z:
                    if speed > 0:
                        speed -= 1
                        if not stationary:
                            set_motor_speed(motor1_pwm, speed)
                            set_motor_speed(motor2_pwm, speed)
                elif event.key == pygame.K_x:
                    if speed < 100:
                        speed += 1
                        if not stationary:
                            set_motor_speed(motor1_pwm, speed)
                            set_motor_speed(motor2_pwm, speed)
        # Wait a small time to avoid high CPU usage
        pygame.time.wait(10)

finally:
    pygame.quit()
    camera.close()
    GPIO.cleanup()
