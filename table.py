#!/usr/bin/env python3
from ev3dev2.motor import LargeMotor, MediumMotor, OUTPUT_A, OUTPUT_D,OUTPUT_B, SpeedPercent, MoveTank
from ev3dev2.sensor import INPUT_1
from ev3dev2.sensor.lego import TouchSensor
from ev3dev2.sensor.lego import UltrasonicSensor
from ev3dev2.sensor.lego import GyroSensor
from ev3dev2.led import Leds
from time import sleep, time

class RobotGripper:
    def __init__(self):
        self.angleToRunTo = 0
        self.motorArm = MediumMotor(OUTPUT_B)
        self.CurrentAngle = 0
        self.motorAngle = 0
        self.motorDestinationAngle = 0
        self.Hold = False
        self.gainAngle = 0
           
    def findMotorAngle(self,angle):
        self.angleToRunTo = angle
        self.motorDestinationAngle = angle/0.142857
        if angle > 0:
            self.gainAngle = (self.angleToRunTo - self.CurrentAngle)/0.142857
            sleep(.1)
        elif angle < 0:
            self.gainAngle = (self.angleToRunTo + self.CurrentAngle)/0.142857
            sleep(.1)
    def extend(self):
        self.motorArm.on_for_degrees(-50,self.gainAngle)
        sleep(3)

    def setHold(self,status):
        self.motorArm.on_for_degrees(100,0,brake = status, block = True)

    def retract(self):
        self.setHold(True)
        self.motorArm.on_for_degrees(50,self.gainAngle)
        sleep(3)
         

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
        self.myGyroSensor = GyroSensor()
        self.myGripper = RobotGripper()
        self.myUltrasonicSensor = UltrasonicSensor()
        self.Turning = False
        self.Moving = False
        self.myAngle = 0
        self.positionX = 0
        self.positionY = 0
        self.myDefaultSpeed = 50;

    def getAverageDistance(self):
        average = (self.motorA.position + self.motorD.position)/2
        return average

    def moveStraight(self,distance):
        self.motorA.reset()
        self.motorD.reset()
        self.tankDrive.reset()
        GyroSensor.reset()
        GyroSensor.mode='GYRO-ANG'
        currentDistance = 0
        SpeedPercentLeft = .5
        SpeedPercentRight = .5
        while currentDistance < distance:
            currentDistance = self.getAverageDistance()
            self.tankDrive.on_for_degrees(SpeedPercentLeft * self.myDefaultSpeed, SpeedPercentRight * self.myDefaultSpeed, 10)
            #movesteer('rotations',steer = , pwr=50, rots=.1)
        #Set to brake

    def collisonDistance(self):
        distance = self.myUltrasonicSensor.distance_inches_ping
        return distance

    def collisionAvoidance(self):
        if self.Turning == False:
            #Normal collision avoidance
            print('Checking for collisions')
            distance = self.collisonDistance()
            if distance < 5:
                self.motorA.stop()

        else:
            #Collision avoidance disabled
            print('Not checking for collisions')

    def turnToAngle(self,angle):
        self.myGyroSensor.reset()
        if angle > 0:
            print('Positive angle')
            #Turns right
            self.Turning = True
            while self.myGyroSensor.value() < angle:
                self.tankDrive.on_for_degrees((-.2*self.myDefaultSpeed), (.2*self.myDefaultSpeed),2,brake = False, block = True)
                sleep(.1)
            self.tankDrive.STOP_ACTION_BRAKE()
            self.Turning = False
            self.myAngle += self.myGyroSensor.value
            self.myGyroSensor.reset
        else:
            #do another thing
            print ('Negative Angle')
            self.Turning = True
            while self.myGyroSensor.value() < angle:
                self.tankDrive.on_for_degrees((.2*self.myDefaultSpeed), (-.2*self.myDefaultSpeed),2,brake = False, block = True)
                sleep(.1)   
            self.tankDrive.STOP_ACTION_BRAKE()
            self.Turning = False
            self.myAngle -= self.myGyroSensor.value
            self.myGyroSensor.reset

    def readBarCode(self):
        #String kmessage is 3 letters, 'W' denotates white, "B" denotates black
        output = ""
        
        return output

class RobotRunner:
    def __init__(self):
        self.myBot = myRobot()
    def task1(self):
        #Do task 1

        self.myBot.myGripper.findMotorAngle(220)
        self.myBot.myGripper.extend()
        self.myBot.myGripper.retract()


        #Startpoint can = 1 or 2
mySession = RobotRunner()
mySession.task1()

 #main 

