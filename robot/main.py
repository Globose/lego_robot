#!/usr/bin/env pybricks-micropython
import time
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port 
from pybricks.tools import wait

# Robot definition
ev3 = EV3Brick()

# Motor definitions
left_motor = Motor(Port.A)
right_motor = Motor(Port.B)

# Sensor definitions
left_light = ColorSensor(Port.S1)  # S3
obstacle_sensor = UltrasonicSensor(Port.S2)  # S4
right_light = ColorSensor(Port.S3)  # S2

# Here is where your code starts


def norm(color_left, color_right, color_current):
    """Returns a number between -1 and 1
        -1 => Turn left, 1 => turn right, 0 => drive straight
    """
    t = (color_current-min(color_left, color_right) -
         .5*abs(color_left-color_right))
    t = (2*t)/(color_left-color_right)
    t = (t**3+t)/2
    return t


def velocity_fn(x, base_velocity, steering_offset):
    """Calculates left and right velocity"""
    velocity_left = (min(base_velocity, base_velocity+2*base_velocity*x) +
                     x*base_velocity*min(steering_offset, 0))
    velocity_right = (min(base_velocity, base_velocity-2*base_velocity*x) +
                      x*base_velocity*max(steering_offset, 0))
    return (velocity_left, velocity_right)


def drive_robot(velocity):
    """Drives robot forward"""
    left_motor.run(speed=velocity[0])
    right_motor.run(speed=velocity[1])


def drive_to_line(color_line, color_base, light, velocity):
    """Drives robot and stops before the line"""
    drive_robot(velocity)
    while True:
        light_refl = light.reflection()
        if abs(color_line-light_refl) < abs(color_base-light_refl):
            drive_robot((0, 0))
            return


def drive_over_line(color_line, color_base, light, velocity):
    """Drives robot and stops after the line"""
    drive_to_line(color_line, color_base, light, velocity)
    drive_robot(velocity)
    while True:
        light_refl = light.reflection()
        if abs(color_line-light_refl) > abs(color_base-light_refl):
            drive_robot((0, 0))
            return


def park(color_line, color_base):
    """Parks the robot"""
    drive_over_line(color_line, color_base, left_light, (150, 150))
    start_time = time.time()
    while True:
        drive_robot(velocity_fn(norm(color_line, color_base, left_light.reflection()), 200, -1))
        if time.time() - start_time > 6.6:
            break
    drive_robot((0, 0))


def back(color_line, color_base):
    """Backs the robot straight"""
    drive_over_line(color_line, color_base, right_light, (-150, -150))
    drive_robot((150, 0))
    wait(1700)


def parking_mode(steering_offset_inv, color_line, color_base):
    """Attempts to park the robot"""
    drive_robot(velocity_fn(-1, 150, -1))
    wait(1700)
    drive_robot(velocity_fn(1, 150, -1))
    wait(1700)

    # Check radar
    parking_empty = False

    if parking_empty:
        park(color_line, color_base)
        wait(3000)
        back(color_line, color_base)
    else:
        drive_robot((0, 150))
        wait(150)


def calibrate():
    """Calibrates the color sensor"""
    ev3.speaker.beep()
    color_left = left_light.reflection()
    color_right = right_light.reflection()
    ev3.speaker.beep()
    wait(2000)
    return color_left, color_right


def light_on_line(color_line, color_base, light):
    return abs(light.reflection()-color_line) < abs(light.reflection()-color_base)


def main():
    """Main Function"""
    color_left, color_right = 0, 100  # calibrate()

    base_velocity = 200
    steering_offset = 0

    drive_to_line(color_left, color_right, right_light, (180, 180))
    timer = time.time()

    while True:
        distance = obstacle_sensor.distance()*10
        velocity = base_velocity*min(1, ((distance-100)/200))

        vel = velocity_fn(norm(color_left, color_right, right_light.reflection()), velocity, steering_offset)
        drive_robot(vel)

        if light_on_line(color_left, color_right, left_light) and time.time()-timer > 3:
            parking_mode(-steering_offset, color_left, color_right)
            drive_to_line(color_left, color_right, right_light, (200, 200))
            timer = time.time()


if __name__ == '__main__':
    main()
