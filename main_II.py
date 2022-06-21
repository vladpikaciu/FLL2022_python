#!/usr/bin/env pybricks-micropython
from time import sleep
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, GyroSensor, UltrasonicSensor, ColorSensor
from pybricks.parameters import Port, Stop, Color, ImageFile, SoundFile
from pybricks.robotics import DriveBase
from pybricks.tools import wait
from pybricks.media.ev3dev import Image, ImageFile
from functions import drift, gyro_straight, gyro_turn
# Initialize the EV3 Brick.
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


#Start

def straight_nogyro(distance=-100, dreapta=-100, stanga=-100):
    while robot.distance() < distance:
        right_motor.run(dreapta)
        left_motor.run(stanga)
    right_motor.stop()
    left_motor.stop()

def straight_motorC(distance, speed):
    while robot.distance() < distance:
            right_motor.run(speed)
            left_motor.run(speed)
            if robot.distance() >= 250:
                motorA.run_time(350, 3000, then=Stop.HOLD)
                motorA.stop()
    right_motor.stop()
    left_motor.stop()

    robot.stop()
    robot.reset()

def straight_motorA(distance, speed):
    while robot.distance() < distance:
            right_motor.run(speed)
            left_motor.run(speed)
            motorA.run(9000)
    ##Trebuie de implementat miscare in spate
    right_motor.stop()
    left_motor.stop()
    motorA.stop()
def ridica_motor(distance, speed):
    robot.stop()
    robot.reset()
    gyro_sensor.reset_angle(0)
    sfarsit = False
    jos_motor = False
    stop = False
    stop2 = True
    while robot.distance() < distance:
        print(robot.distance())
        right_motor.run(speed)
        left_motor.run(speed)
        while jos_motor==False:
            sleep(2)
            motorC.run_until_stalled(400, then=Stop.COAST)
            motorC.stop()
            jos_motor = True
            print("actiune 1")
            stop2 = False
        while robot.distance() > 20 and stop2 == False:
            motorC.run_time(-350, 1000)
            print(robot.distance(), "actiune 2")
            jos_motor = True
            stop = True
            stop2 = True
        while robot.distance() < distance and stop == True:
            print(robot.distance(), "motorA")
            motorA.run_time(speed=2000, time=2500, then=Stop.HOLD)
            motorA.stop()
            stop = False
            sfarsit = True
            break
        print(robot.distance())
    robot.stop()
    robot.reset()
    gyro_sensor.reset_angle(0)


# Misiune motor Ferrari
def main_II():
    gyro_sensor.reset_angle(0)
    gyro_straight(distance=-200, speed=-200, GSKP=9)
    gyro_turn(13)
    gyro_straight(distance=-140, speed=-200, GSKP=9)
    gyro_turn(-14) ## uneori 13, 17 sau 26
    gyro_sensor.reset_angle(0)
    gyro_straight(distance=-580, speed=-400, GSKP=9)

    ridica_motor(distance=150, speed=50)
    motorA.run_time(speed=-300, time=1000)
    motorA.stop()
    gyro_straight(distance=-10, speed=-200)
    drift(40, -500, 0) # 45

    straight_motorC(distance=400, speed=3000)
    robot.stop()
    robot.reset()


# main_II()
