#!/usr/bin/env pybricks-micropython
from time import sleep
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, GyroSensor, UltrasonicSensor, ColorSensor
from pybricks.parameters import Port, Stop, Color, ImageFile, SoundFile, Button
from pybricks.robotics import DriveBase
from pybricks.tools import wait
from pybricks.media.ev3dev import Image, ImageFile
from functions import drift, gyro_straight, gyro_turn, indreptare_culoare_continua

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


def miscare_gyro_motor(distance=-100, speed=-100, GSKP=4, deaccelerare=0, motor_state = 1, motor_speed=3000):
    #In spate cu plus
    if distance > 0:
        while robot.distance() <=  distance:
            corection = (0 - gyro_sensor.angle())*GSKP #Viteza de corectare 1-3
            robot.drive(speed, corection)
            # while distance - robot.distance() <= deaccelerare:
            #     corection = (0 - gyro_sensor.angle())*GSKP #Viteza de corectare 1-3
            #     robot.drive(viteza_II, corection)
        robot.stop()
    ##In fata cu minus
    elif distance < 0:
        while robot.distance() >=  distance:
            corection = (0 - gyro_sensor.angle())*GSKP #Viteza de corectare 1-3
            robot.drive(speed, 0)
            print(robot.distance())
            ###Aici scrii ce face
            if robot.distance() < deaccelerare and  robot.distance() > deaccelerare-20 and motor_state == 1:
                motorC.run_until_stalled(-500, then=Stop.HOLD)
                print(robot.distance())
                motorC.stop()
            ### AIci scrii ce face
            elif robot.distance() < deaccelerare and  robot.distance() > deaccelerare-20 and motor_state == 2:
                motorA.run_time(motor_speed, 400, then=Stop.HOLD)
                print("motorA")
                motorA.stop()
        robot.stop()
    print("Inainte gyro:", gyro_sensor.angle())
    print("Inainte distanta:", robot.distance())
    right_motor.stop()
    left_motor.stop()
    robot.reset()
def straight_motorC(distance, speed, deaccelerare, misiune):
    while robot.distance() > distance and misiune == 1:
        right_motor.run(speed)
        left_motor.run(speed)
        print(robot.distance())
        if robot.distance() < deaccelerare and  robot.distance() > deaccelerare-20:
            motorC.run_target(5000, 150)
            motorC.stop()
    while robot.distance() < distance and misiune == 2:
        right_motor.run(speed)
        left_motor.run(speed)
        while robot.distance() > deaccelerare and robot.distance() < deaccelerare+20:
            motorC.run_target(5000, 180, then=Stop.HOLD)
            motorC.stop()
    print("DIstanta", robot.distance())
    robot.stop()
    right_motor.stop()
    left_motor.stop()
    robot.reset()
def drift_fara_gyro(grade, stanga, dreapta):
    if grade > 0:
        while gyro_sensor.angle() < grade:
            right_motor.run(dreapta)
            left_motor.run(stanga)
    elif grade < 0:
        while gyro_sensor.angle() > grade:
            right_motor.run(dreapta)
            left_motor.run(stanga)
    right_motor.stop()
    left_motor.stop()
    sleep(0.5)
    print("Intoarcere gyro drift:", gyro_sensor.angle())
    robot.reset()
    robot.stop()
    gyro_sensor.reset_angle(0)

def distanta_culoare():
    robot.reset()
    gyro_sensor.reset_angle(0)
    while color_sensor_stanga.reflection() not in negru:
        robot.drive(-100, 0)
    robot.stop()
    robot.reset()
    straight_motorC(distance=-140, speed=-200, viteza=-300)



def main_IV():
    gyro_straight(distance=-900, speed=-500, GSKP=7)
    gyro_turn(-45, speed= 200)
    #drift_fara_gyro(-45, stanga=250, dreapta=-250)

    gyro_straight(distance=-240, speed=-300)
    gyro_sensor.reset_angle(0)
    drift_fara_gyro(60, stanga=0, dreapta=220)
    drift_fara_gyro(60, stanga=-200, dreapta=50)
    gyro_straight(distance=100, speed=200)
    drift_fara_gyro(-34, stanga=300, dreapta=0)

    motorC.run_until_stalled(-500)
    gyro_straight(distance=-250, speed=-200)
    ## DROP la cutii sure
    motorA.run_time(-100, 2500)
    gyro_straight(distance=15, speed = 200)
    motorC.run_target(5000, 180)
    motorC.stop()
    robot.reset()
    robot.settings(straight_speed=400, straight_acceleration=100)
    robot.straight(210)
    print(robot.distance())
    # robot.reset()
    # robot.stop()


##END

# main_IV()
