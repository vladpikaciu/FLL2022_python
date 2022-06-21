#!/usr/bin/env pybricks-micropython
from functions import drift, gyro_straight, gyro_turn
from pybricks.hubs import EV3Brick
from pybricks.parameters import Button
from main_IV import main_IV
from main_III import main_III
from main_II import main_II
from main_I import main_I
from time import sleep
from time import sleep
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, GyroSensor, UltrasonicSensor, ColorSensor
from pybricks.parameters import Port, Stop, Color, ImageFile, SoundFile, Button
from pybricks.robotics import DriveBase
from pybricks.tools import wait
from pybricks.media.ev3dev import Image, ImageFile
ev3 = EV3Brick()
gyro_sensor = GyroSensor(Port.S4)
#color_sensor1 = ColorSensor(Port.S1)##Frontal
color_sensor_dreapta = ColorSensor(Port.S2)##Dreapta
color_sensor_stanga = ColorSensor(Port.S3)##Stanga
gyro_sensor.reset_angle(0)

#Motor
motorA = Motor(Port.A)
motorC = Motor(Port.C)
right_motor = Motor(Port.B)
left_motor = Motor(Port.D)
robot = DriveBase(right_motor, left_motor, wheel_diameter = 62.4, axle_track = 100) #105

counter = 0
# Reset all robot sensors 
def reset():
    robot.stop()
    robot.reset()
    gyro_sensor.reset_angle(0)
# Make missions start when center button is pressed
while counter < 5:
    if Button.CENTER in ev3.buttons.pressed():
        sleep(2)
        main_1_state = False
        main_2_state = False
        main_3_state = False
        main_4_state = False
        counter += 1
        print(counter)
        # Run first mission
        while counter == 1 and main_1_state == False:
            reset()
            print("1")
            main_I()
            main_1_state = True
        # Run second mission
        while counter == 2 and main_2_state == False:
            print("2")
            reset()
            main_II()
            main_2_state = True
        # Run third mission
        while counter == 3 and main_3_state == False:
            reset()
            main_III()
            print("3")
            main_3_state = True
        # Run fourth mission
        while counter == 4 and main_4_state == False:
            reset()
            main_IV()
            print("4")
            main_4_state = True
