#!/usr/bin/env python3
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_D,OUTPUT_B, SpeedPercent, MoveTank
from ev3dev2.sensor import INPUT_1
from ev3dev2.sensor.lego import TouchSensor
from ev3dev2.sensor.lego import UltrasonicSensor
from ev3dev2.sensor.lego import GyroSensor
from ev3dev2.led import Leds

class RobotGripper:
    def __init__(self):
        self.angleToRunTo = 0
        self.motorB = MediumMotor(OUTPUT_B)
        self.CurrentAngle = 0
        self.Hold = False
    
    def setHold(status):
        self.




tankDrive = MoveTank(OUTPUT_A,OUTPUT_D)
motorA = LargeMotor(OUTPUT_A)
motorD = LargeMotor(OUTPUT_D)
gyro = GyroSensor()

def getAverageDistance():
    average = (motorA.position + motorD.position)/2
    return average

def moveStraight(distance):
    motorA.reset()
    motorD.reset()
    tankDrive.reset()
    GyroSensor.reset()
    currentDistance = 0
    while currentDistance < distance:
        currentDistance = getAverageDistance()
        tankDrive.on_for_degrees(SpeedPercent(),SpeedPercent(), 10)
        #movesteer('rotations',steer = , pwr=50, rots=.1)
    #Set to brake

def collisonDistance():
    distance = UltrasonicSensor.distance_inches_ping
    return distance

def turnToAngle(angle):
    if angle > 0:
        #Do something
    else:
        #do another thing
