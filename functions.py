#!/usr/bin/env pybricks-micropython
from time import sleep
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, GyroSensor, UltrasonicSensor, ColorSensor
from pybricks.parameters import Port, Stop, Color, ImageFile, SoundFile
from pybricks.robotics import DriveBase
from pybricks.tools import wait
from pybricks.media.ev3dev import Image, ImageFile

# Initialize the EV3 Brick.
ev3 = EV3Brick()
gyro_sensor = GyroSensor(Port.S4)
#color_sensor1 = ColorSensor(Port.S1)##Frontal
color_sensor_dreapta = ColorSensor(Port.S2)##Dreapta
color_sensor_stanga = ColorSensor(Port.S3)##Stanga
gyro_sensor.reset_angle(0)
## Lista cu valori care reprezinta culoarea neagra
negru = [10,11,12]
#Motor
motorA = Motor(Port.A)
motorC = Motor(Port.C)
right_motor = Motor(Port.B)
left_motor = Motor(Port.D)
robot = DriveBase(right_motor, left_motor, wheel_diameter = 62.4, axle_track = 100) #105


def drift(grade, stanga, dreapta):
    gyro_sensor.reset_angle(0)
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
    while gyro_sensor.angle() < grade:
        robot.turn(2)
    while gyro_sensor.angle() > grade:
        robot.turn(-2)
    print("Inoarcere gyro drift:", gyro_sensor.angle())
    robot.reset()
    robot.stop()
    gyro_sensor.reset_angle(0)

def gyro_straight(distance=-100, speed=-100, GSKP=4, deaccelerare=0, viteza_II=-50):
    gyro_sensor.reset_angle(0)
    robot.reset()
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
            robot.drive(speed, corection)
            while distance - robot.distance() >= deaccelerare and distance - robot.distance() < 0:
                corection = (0 - gyro_sensor.angle())*GSKP #Viteza de corectare 1-3
                robot.drive(viteza_II, corection)
        robot.stop()
    right_motor.stop()
    left_motor.stop()
    robot.stop()
    print("Inainte gyro:", gyro_sensor.angle())
    print("Inainte distanta:", robot.distance())
    robot.reset()

def gyro_turn(grade, speed=100):
    gyro_sensor.reset_angle(0)
    robot.settings(turn_rate =speed)
    robot.turn(grade)
    while gyro_sensor.angle() < grade:
        robot.turn(2)
    while gyro_sensor.angle() > grade:
        robot.turn(-2)
    print("Inoarcere gyro:", gyro_sensor.angle())
    gyro_sensor.reset_angle(0)
    robot.reset()
    robot.stop()

def indreptare_culoare(virare):
    ##Merge inainte sau inapoi in functie de valoare predefinita de parametrul virare cu plus sau cu minus
    while color_sensor_dreapta.reflection() and color_sensor_stanga.reflection() not in negru:
        right_motor.run(virare)
        left_motor.run(virare)
        ##Daca color.dreapta este negru si color.stanga nu este negru
        ## atunci motor dreapta pastreaza pozitia si stanga merge inainte, cand stanga este negru atunci iese din while si intra in alt while
        while color_sensor_dreapta.reflection() in negru and color_sensor_stanga.reflection() not in negru:
            right_motor.hold()
            left_motor.run(virare)
        ## Tot acelasi lucru doar cu stanga
        while color_sensor_stanga.reflection() in negru and color_sensor_dreapta.reflection() not in negru:
            left_motor.hold()
            right_motor.run(virare)
        ## Daca ambele senzore sunt negru atunci opreste robotul si iesi din while
        while color_sensor_dreapta.reflection() and color_sensor_stanga.reflection() in negru:
            robot.stop()
            break
    robot.stop()
    right_motor.stop()
    left_motor.stop()

def indreptare_culoare_continua(numar=4, speed=50, virare=-50, distance=10):
    robot.settings(straight_speed=speed)
    print('Begin Color indreptare', gyro_sensor.angle())
    ## Functie pentru ca sa se indrepte de mai multe ori pentru mai multa precizie
    for x in range(0,numar):
        print("Color gyro" + str(x) +" " , gyro_sensor.angle())
        ## Virarea care este necesara pentru indreptare culoare
        indreptare_culoare(virare=virare)
        ## Mergi in spate sau in fata
        robot.straight(distance)
        robot.stop()

## Exemplu in spate: indreptare_culoare_continua(speed=-50, distance=-10, virare=50, numar=4)
## Exemplu in fata: indreptare_culoare_continua(speed=50, distance=10, virare=-50, numar=4)
