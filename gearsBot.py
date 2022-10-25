#!/usr/bin/env python3

# Import the necessary libraries
import math
import time
from pybricks.ev3devices import *
from pybricks.parameters import *
from pybricks.robotics import *
from pybricks.tools import wait
from pybricks.hubs import EV3Brick

ev3 = EV3Brick()
motorA = Motor(Port.A)
motorB = Motor(Port.B)
left_motor = motorA
right_motor = motorB
robot = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=152)
robot.settings(straight_speed=200, straight_acceleration=100, turn_rate=100)

left_light = ColorSensor(Port.S1)
obstacle_sensor = UltrasonicSensor(Port.S2)
right_light = ColorSensor(Port.S3)


# Here is where your code starts

def drive_robot(velocity):
    """Drives robot forward"""
    left_motor.run(speed=velocity[0])
    right_motor.run(speed=velocity[1])

def calc_velocity(color_line, color_base, color_current, steering_offset, drive_side, base_velocity):
    """Returns a tuple with right and left velocity"""
    f = (color_current-min(color_line,color_base)-
        .5*abs(color_line-color_base))/(.5*(color_line-color_base))
    f *= drive_side
    f = min(f,1)
    f = max(f,-1)
    
    #print(f)
    if f > 0:
        return (base_velocity, base_velocity-2*f*base_velocity)
    return (base_velocity+2*f*base_velocity, base_velocity)

def drive_forward(color_line, color_base, velocity):
    """Drives robot forward until on the line"""
    drive_robot(velocity)
    difference = abs(color_base-color_line)
    while True:
        color_current = left_light.reflection()
        if color_line - difference/4 < color_current < color_line + difference/4:
            break

def main():
    """Main Function"""
    color_line, color_base = 0, 100
    base_velocity = 200
    steering_offset = 1
    drive_side = 1 # -1 = left side, 1 = right side

    drive_forward(color_line, color_base, (base_velocity,base_velocity))

    #print(calc_velocity(color_line, color_base, 80, steering_offset, drive_side, base_velocity))

    while True:
        drive_robot(calc_velocity(color_line, color_base, 
            left_light.reflection(), steering_offset, drive_side, base_velocity))

if __name__ == '__main__':
    main()


