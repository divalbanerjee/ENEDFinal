#!/usr/bin/env python3
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_D, OUTPUT_B,SpeedPercent, MoveTank
from ev3dev2.sensor import INPUT_1
from ev3dev2.sensor.lego import TouchSensor
from ev3dev2.led import Leds

from ev3dev2.sound import Sound

sound = Sound()
sound.speak('ENED Bad')

motorArm = MediumMotor(OUTPUT_B)

motorArm.

#tank_drive = MoveTank(OUTPUT_A, OUTPUT_D)

# drive in a turn for 5 rotations of the outer motor
# the first two parameters can be unit classes or percentages.
#tank_drive.on_for_rotations(SpeedPercent(50), SpeedPercent(50), 3)

# drive in a different turn for 3 seconds

tank_drive.on_for_degrees(SpeedPercent(-30), SpeedPercent(30), 90)