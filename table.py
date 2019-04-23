#!/usr/bin/env python3
from ev3dev2.motor import LargeMotor, MediumMotor, OUTPUT_A, OUTPUT_D,OUTPUT_B, SpeedPercent, MoveTank
from ev3dev2.sensor import INPUT_1
from ev3dev2.sensor.lego import TouchSensor, ColorSensor
from ev3dev2.sensor.lego import UltrasonicSensor
from ev3dev2.sensor.lego import GyroSensor
from ev3dev2.led import Leds
import ev3dev.ev3 as ev3
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
    def calculate(self):
        print ('Yeet')

class myRobot:
    def __init__(self):
        self.tankDrive = MoveTank(OUTPUT_A,OUTPUT_D)
        self.motorA = LargeMotor(OUTPUT_A)
        self.motorD = LargeMotor(OUTPUT_D)
        self.myGyroSensor = GyroSensor()
        self.myGripper = RobotGripper()
        self.myUltrasonicSensor = UltrasonicSensor()
        self.myColorSensor = ColorSensor()
        self.Turning = False
        self.Moving = False
        self.myAngle = 0
        self.positionX = 0
        self.positionY = 0
        self.myDefaultSpeed = 50;
        self.desiredBarCode = "BWWB"
        self.myGPS = IGPS()

    def getAverageDistance(self):
        #Returns average distance in inches
        average = (self.motorA.position + self.motorD.position)/2
        average = (average * 3.14 * (2.71*2))
        return average

    def moveStraight(self,distance):
        self.motorA.reset()
        self.motorD.reset()
        self.tankDrive.reset()
        #self.myGyroSensor.angle = 0
        self.myGyroSensor.mode='GYRO-ANG'
        currentDistance = 0
        speedPercentLeft = .5
        speedPercentRight = .5
        while currentDistance < distance:
            currentDistance = self.getAverageDistance()

            if self.myGyroSensor.angle < 0:
                #Turn left, left gain
                gain = (-.00005 * (self.myGyroSensor.angle))
                speedPercentLeft = speedPercentLeft - gain
                speedPercentRight = speedPercentRight + gain
            else:
                #Turn moe right, right gain
                gain = (.00005 * (self.myGyroSensor.angle))
                speedPercentLeft = speedPercentLeft + gain
                speedPercentRight = speedPercentRight - gain

            self.tankDrive.on_for_degrees(self.myDefaultSpeed, self.myDefaultSpeed, 10,brake = False, block = True)
            sleep(.2)
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
                self.motorD.stop()
                print('Collision avoided')

        else:
            #Collision avoidance disabled
            print('Not checking for collisions')

    def turnToAngle(self,angle):
        #self.myGyroSensor.reset()
        if angle > 0:
            print('Positive angle')
            #Turns right
            self.Turning = True
            while self.myGyroSensor.value() < angle:
                self.tankDrive.on_for_degrees((-.2*self.myDefaultSpeed), (.2*self.myDefaultSpeed),3,brake = False, block = True)
                sleep(.2)
            self.tankDrive.STOP_ACTION_BRAKE()
            self.Turning = False
            self.myAngle += self.myGyroSensor.value
            self.myGyroSensor.reset
        else:
            #do another thing
            print ('Negative Angle')
            self.Turning = True
            while self.myGyroSensor.value() > angle:
                self.tankDrive.on_for_degrees((.2*self.myDefaultSpeed), (-.2*self.myDefaultSpeed),2,brake = False, block = True)
                sleep(.1)   
            self.tankDrive.STOP_ACTION_BRAKE()
            self.Turning = False
            self.myAngle -= self.myGyroSensor.value
            self.myGyroSensor.reset

    def readBarCode(self):
        #String kmessage is 3 letters, 'W' denotates white, "B" denotates black
        output = ""
        ev3.Sound.speak('starting').wait()
        for i in range(4):
            self.moveStraight(.25)
            self.myColorSensor.mode = 'COL-COLOR'

            if self.myColorSensor.value == 1:
                output = output + 'B'
                ev3.Sound.speak('black').wait()

            elif self.myColorSensor.value == 6:
                output = output + 'W'
                ev3.Sound.speak('white').wait()
            sleep(1)
        ev3.Sound.speak('Here is the barcode' + output).wait()

        print(output)
        sleep(5)

        return output

class RobotRunner:
    def __init__(self):
        self.myBot = myRobot()

    def testTask(self):
        self.myBot.readBarCode()
        #self.myBot.moveStraight(100)
        #self.myBot.turnToAngle(90)
        self.myBot.myGripper.findMotorAngle(220)
        self.myBot.myGripper.extend()
        self.myBot.myGripper.retract()

    def task1(self):
        #Do task 1
        print('Task 1')

    def task12(self):
        #Do task 1
        print('Task 2')
        
        #Startpoint can = 1 or 2

mySession = RobotRunner()
mySession.testTask()


 #main 

