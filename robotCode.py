#!/usr/bin/env python3
from ev3dev2.motor import LargeMotor, MediumMotor,OUTPUT_A, OUTPUT_D, OUTPUT_B,SpeedPercent, MoveTank
from ev3dev2.sensor import INPUT_1
from ev3dev2.sensor.lego import TouchSensor
from ev3dev2.led import Leds
from time import sleep

from ev3dev2.sound import Sound

sound = Sound()
#ound.speak('ENED Bad')

motorArm = MediumMotor(OUTPUT_B)


angle = 220/0.142857
motorArm.on_for_degrees(-50,angle)
sleep(5)
motorArm.on_for_degrees(50,angle)

#tank_drive = MoveTank(OUTPUT_A, OUTPUT_D)

# drive in a turn for 5 rotations of the outer motor
# the first two parameters can be unit classes or percentages.
#tank_drive.on_for_rotations(SpeedPercent(50), SpeedPercent(50), 3)

# drive in a different turn for 3 seconds

#tank_drive.on_for_degrees(SpeedPercent(-30), SpeedPercent(30), 90)