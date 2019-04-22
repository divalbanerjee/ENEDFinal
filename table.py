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
        self.motorAngle = 0
        self.motorDestinationAngle = 0
        self.Hold = False
           
    def findMotorAngle(self):
        motorArm.on_for_degrees(-50,angle)

    def extend(self):
        self.motorDestinationAngle = (angleToRunTo-CurrentAngle)/0.142857
        self.motorArm.on_for_degrees(-50,angle)

    def setHold(status):
        self.motorB.on_for_degrees(100,0,brake = status, block = True)
    
        

class IGPS:
    def __init__(self):
        self.radiusA = 0
        self.radiusB = 0
        self.radiusC = 0

class myRobot:
    def __init__(self):
        self.tankDrive = MoveTank(OUTPUT_A,OUTPUT_D)
        self.motorA = LargeMotor(OUTPUT_A)
        self.motorD = LargeMotor(OUTPUT_D)
        self.gyro = GyroSensor()
        self.myGripper = RobotGripper()
        self.Turning = False
        self.Moving = False
        self.Angle = False
        self.positionX = 0
        self.positionY = 0
        self.myDefaultSpeed = 50;
    def getAverageDistance(self):
        average = (motorA.position + motorD.position)/2
        return average

    def moveStraight(self,distance):
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

    def collisonDistance(self):
        distance = UltrasonicSensor.distance_inches_ping
        return distance

    def collisionAvoidance(self):
        if self.Turning == False :
            #Normal collision avoidance
        else:
            #Collision avoidance disabled
            print('Not checking for collisions')

    def turnToAngle(self,angle):
        if angle > 0:
            print('Positive angle')
            #Do something
            
        else:
            #do another thing
            print ('Negative Angle')

    def readBarCode(self):
        #String kmessage is 3 letters, 'W' denotates white, "B" denotates black
        output = ""
        
        return output

class RobotRunner
    def __init__(self):
        self.myBot = myRobot
    def task1(self):
        #Do task 1
    def task2(self):
    
    def mainDemo(self):

 #main 

