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
gyro_sensor.reset_angle(0)
#Timp
#Motor
motorA = Motor(Port.A)
motorC = Motor(Port.C)
right_motor = Motor(Port.B)
left_motor = Motor(Port.D)
robot = DriveBase(right_motor, left_motor, wheel_diameter = 62.4, axle_track = 100) #105



#Start

def straight_nogyro(distance=-100):
    motorA.run_angle(1800, 3000)#4600
    while robot.distance() < distance:
        corection = (0 - gyro_sensor.angle())*4 #Viteza de corectare 1-7
        robot.drive(200, corection)
        print(robot.distance(), "gyro", gyro_sensor.angle())
    robot.stop()
    right_motor.stop()
    left_motor.stop()
    robot.reset()

def straight_motorC(distance, speed):
    while robot.distance() > distance:
            right_motor.run(speed)
            left_motor.run(speed)
            motorC.run(900)
    right_motor.stop()
    left_motor.stop()
    motorC.stop()
def straight_motorA(distance, speed):
    while robot.distance() > distance:
            right_motor.run(speed)
            left_motor.run(speed)
            motorA.run(900)
    right_motor.stop()
    left_motor.stop()
    motorA.stop()
    motorC.stop()

def viteza_schimbatoare(distance):
    actiune1 = False
    robot.stop()
    robot.reset()
    while robot.distance() < distance:
        while robot.distance() < 900:
            right_motor.run(300)
            left_motor.run(300)
            action1 = True
        while robot.distance() > 700 and robot.distance() < distance:
            # motorA.run_angle(1800, 4600)
            right_motor.run(300)
            left_motor.run(310)
    print(robot.distance())
    right_motor.stop()
    left_motor.stop()
    robot.stop()
    robot.reset()

# Se apropie de destinatie
def main_I():
    gyro_sensor.reset_angle(0)
    ## Ajungi la cucos
    gyro_straight(distance=-1150, speed=-500)
    ##prinde cucosul
    straight_motorC(distance=-50, speed=-100)
    ## Ajungi la centru de sortare
    gyro_straight(distance=-150, speed=-100)
    ## Sorteaza blocurile
    motorA.run_target(speed=400, target_angle=-4300, wait=True)
    motorA.stop()
    ## Mergi inainte
    gyro_straight(distance=-20, speed=-300)
    ## Mergi la baza
    gyro_sensor.reset_angle(0)
    ## Mergi cu gyro inapoi
    straight_nogyro(distance=1750)
    robot.stop()
    robot.reset()

# main_I()
