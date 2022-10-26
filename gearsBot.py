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

def standardize(color_left, color_right, color_current):
    """Returns a number between -1 and 1"""
    t = (color_current-min(color_left, color_right) -
         .5*abs(color_left-color_right))
    return (2*t)/(color_left-color_right)


def velocity_fn(x, base_velocity, steering_offset):
    """Calculates left and right velocity"""
    velocity_left = (min(base_velocity, base_velocity+2*base_velocity*x) +
                     x*base_velocity*min(steering_offset, 0) +
                        0.4*base_velocity*min(0, x))
    velocity_right = (min(base_velocity, base_velocity-2*base_velocity*x) +
                      x*base_velocity*max(steering_offset, 0) -
                        0.4*base_velocity*max(0, x))
    return (velocity_left, velocity_right)


def drive_robot(velocity):
    """Drives robot forward"""
    left_motor.run(speed=velocity[0])
    right_motor.run(speed=velocity[1])


def get_color_furthest(color_left, color_right, color_current):
    """Returns the color furthest away from color_current"""
    if abs(color_current-color_left) > abs(color_current-color_right):
        return color_right
    return color_left


def drive_forward(color_left, color_right, velocity):
    """Drives robot forward until it has passed the line"""
    drive_robot(velocity)
    color_current = left_light.reflection()
    color_target = get_color_furthest(color_left, color_right, color_current)
    distance = abs(color_current-color_target)
    while True:
        color_current = left_light.reflection()
        new_distance = abs(color_current-color_target)
        if new_distance > distance:
            break
        wait(200)


def main():
    """Main Function"""
    color_left, color_right = 100, 0
    base_velocity = 300
    steering_offset = -1
    drive_forward(color_left, color_right, (base_velocity, base_velocity))

    while True:
        s = standardize(color_left, color_right, left_light.reflection())
        velocity = velocity_fn(s, base_velocity, steering_offset)
        drive_robot(velocity)


if __name__ == '__main__':
    main()
