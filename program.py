#!/usr/bin/env pybricks-micropython
from time import sleep
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, GyroSensor, UltrasonicSensor, ColorSensor
from pybricks.parameters import Port, Stop, Color, ImageFile, SoundFile, Button
from pybricks.robotics import DriveBase
from pybricks.tools import wait
from pybricks.media.ev3dev import Image, ImageFile
from functions import drift, gyro_straight, gyro_turn
# Initialize the EV3 Brick.
ev3 = EV3Brick()
gyro_sensor = GyroSensor(Port.S4)
gyro_sensor.reset_angle(0)
#Timp
#Motor
motorA = Motor(Port.A)
motorC = Motor(Port.C)
right_motor = Motor(Port.B)
left_motor = Motor(Port.D)
robot = DriveBase(right_motor, left_motor, wheel_diameter = 62.4, axle_track = 100) #105

counter = 0
main_1_state = False
while counter <= 1 and main_1_state == False:
    if Button.CENTER in ev3.buttons.pressed():
        sleep(1)
        main_1_state = False
        counter += 1
        while counter == 1 and main_1_state == False:
            print("1")
            main_1_state = True
print("salut")
