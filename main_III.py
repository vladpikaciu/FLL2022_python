#!/usr/bin/env pybricks-micropython
from time import sleep
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, GyroSensor, UltrasonicSensor, ColorSensor
from pybricks.parameters import Port, Stop, Color, ImageFile, SoundFile
from pybricks.robotics import DriveBase
from pybricks.tools import wait
from pybricks.media.ev3dev import Image, ImageFile
from functions import drift, gyro_straight, gyro_turn, indreptare_culoare_continua
# Initialize the EV3 Brick.
ev3 = EV3Brick()
negru = [10,11,12]
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

def straight_motorC(distance, speed, viteza=300, misiune=1):
    motorA_state = True
    while robot.distance() > distance and misiune == 1:
        right_motor.run(speed)
        left_motor.run(speed)

    ##La distanta de 205 lasa proiect inovativ in cerc
    while robot.distance() < distance and misiune == 2:
        right_motor.run(speed)
        left_motor.run(speed)
        if robot.distance() > 190:
            motorC.run_until_stalled(300)
            motorC.stop()
    print("DIstanta", robot.distance())
    right_motor.stop()
    left_motor.stop()
    motorC.stop()
    robot.reset()

def motor_A_straight(distance, GSKP, deaccelerare, speed):
    robot.reset()
    actiune1 = False
    sfarsit = False
    actiune1_state = False
    while robot.distance() < distance:
        corection = (0 - gyro_sensor.angle())*GSKP #Viteza de corectare 1-3
        robot.drive(speed, corection)
        print(robot.distance())
        ## Reitereaza primul while, dar nu mai executa actiune 1
        if actiune1_state == True:
            actiune1 = True
        while robot.distance() > deaccelerare and actiune1 == False:
            ## Ne asiguram ca blocul sur din masina este in cerc facand virare
            robot.turn(-30)
            sleep(1)
            ## Da drumu la constructie cu masini folosind Motorul A in sus
            motorA.run_target(speed=200, target_angle=-100, then=Stop.HOLD)
            motorA.stop()
            ## Nu executa prima actiune
            actiune1_state = True
            print("Actiune 1")
            ## Iesim din acest while si reiteram while principal
            break
        ## Executam actiune 2 cand actiune 1 a fost finisata
        while actiune1 == True and sfarsit == False:
            print("Actiune 2")
            ## Dam motorul A inapoi in jos
            motorA.run_until_stalled(speed=400, then=Stop.HOLD)
            motorA.stop()
            ## Iesim din actiune 2
            sfarsit = True
    robot.stop()
    robot.reset()

def viteza_schimbatoare(distance):
    actiune1 = False
    robot.stop()
    robot.reset()
    while robot.distance() < distance:
        while robot.distance() < 150:
            right_motor.run(300)
            left_motor.run(380)
            action1 = True
        while robot.distance() > 300 and robot.distance() < distance:
            right_motor.run(380)
            left_motor.run(300)
    right_motor.stop()
    left_motor.stop()
    robot.stop()
    robot.reset()
def miscare_motor_A(distance, speed):
    robot.stop()
    robot.reset()
    actiune1 = False
    while robot.distance() > distance:
        robot.drive(-100, 0)
        print(robot.distance())
        while robot.distance() < -100 and actiune1==False:
            motorA.run_time(speed=-500, time=1500)
            actiune1 = True
    robot.stop()
    print(robot.distance())





def main_III():
    gyro_sensor.reset_angle(0)
    gyro_straight(distance=12, speed=100)
    drift(grade=93, stanga=0, dreapta=200)
    gyro_sensor.reset_angle(0)

    motor_A_straight(distance=1210, GSKP=3, deaccelerare=780, speed=200)
    gyro_straight(distance=-110, speed=-150)
    drift(grade=-56, stanga=100, dreapta=0)
    straight_motorC(distance=792, speed=400, misiune=2)
    gyro_straight(distance=-15, speed=-100)
    drift(grade=-39, stanga=150, dreapta=-150)
    gyro_straight(distance=20, speed=100)
    robot.settings(straight_speed=-400, straight_acceleration=-100)
    robot.straight(-510)
    robot.stop()
    ## la inceput mergi cu virare intro parte iar apoi in alta parte
    viteza_schimbatoare(650)
    miscare_motor_A(distance=-255, speed=-100)
    robot.straight(20)
    robot.stop()
# main_III()

