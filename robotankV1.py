import curses
import RPi.GPIO as GPIO
from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory
import cv2
from picamera2 import Picamera2
import time
import pygame
import numpy as np

camera = Picamera2()
camera.start()
camera.resolution = (640, 480)
camera.framerate = 30
time.sleep(2)

GPIO.setmode(GPIO.BOARD)

factory = PiGPIOFactory()
servo1 = Servo(13, pin_factory=factory)
servo2 = Servo(12, pin_factory=factory)

motor_pins = {
    "motor1": {"a": 7, "b": 11, "e": 22},
    "motor2": {"a": 13, "b": 16, "e": 15}
}

for motor in motor_pins.values():
    GPIO.setup(motor["a"], GPIO.OUT)
    GPIO.setup(motor["b"], GPIO.OUT)
    GPIO.setup(motor["e"], GPIO.OUT)

motor1_pwm = GPIO.PWM(motor_pins["motor1"]["e"], 1000)
motor2_pwm = GPIO.PWM(motor_pins["motor2"]["e"], 1000)
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

def handle_motor(motor, direction):
    GPIO.output(motor_pins[motor]["a"], GPIO.HIGH if direction == "forward" else GPIO.LOW)
    GPIO.output(motor_pins[motor]["b"], GPIO.LOW if direction == "forward" else GPIO.HIGH)
    GPIO.output(motor_pins[motor]["e"], GPIO.HIGH)
    set_motor_speed(motor1_pwm if motor == "motor1" else motor2_pwm, speed)

try:
    while True:
        image = camera.capture_array()
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        image = np.flip(image, axis=2)
        image = pygame.surfarray.make_surface(image)
        image = pygame.transform.rotate(image, 90)
        screen.blit(image, (0, 0))
        speed_text = font.render(f"Speed: {speed}", True, (255, 255, 255))
        screen.blit(speed_text, (10, 10))
        pygame.display.update()
        for event in pygame.event.get():
            if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_q):
                raise KeyboardInterrupt
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_w:
                    handle_motor("motor1", "forward")
                    handle_motor("motor2", "forward")
                    stationary = False
                elif event.key == pygame.K_s:
                    handle_motor("motor1", "backward")
                    handle_motor("motor2", "backward")
                    stationary = False
                elif event.key == pygame.K_a:
                    handle_motor("motor1", "backward")
                    handle_motor("motor2", "forward")
                    stationary = False
                elif event.key == pygame.K_d:
                    handle_motor("motor1", "forward")
                    handle_motor("motor2", "backward")
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
                elif event.key == pygame.K_z and speed > 0:
                    speed -= 1
                    if not stationary:
                        set_motor_speed(motor1_pwm, speed)
                        set_motor_speed(motor2_pwm, speed)
                elif event.key == pygame.K_x and speed < 100:
                    speed += 1
                    if not stationary:
                        set_motor_speed(motor1_pwm, speed)
                        set_motor_speed(motor2_pwm, speed)
        pygame.time.wait(10)

finally:
    pygame.quit()
    camera.close()
    GPIO.cleanup()
