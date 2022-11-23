#!/usr/bin/env pybricks-micropython
import time
import random
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

# Driving definitions
DRIVING_MODE = -1
BASE_VELOCITY = 180


# Driving
def driving_mode(color_line, color_base, mode):
    """Returns constants for choosen driving mode"""
    if mode == 1:  # Left sensor on left side of the line
        return left_light, right_light, color_base, color_line, -1
    if mode == -1:  # Right sensor on right side of the line
        return right_light, left_light, color_line, color_base, 1


def norm(color_left, color_right, color_current):
    """Returns a number between -1 and 1
        -1 => Turn left, 1 => turn right, 0 => drive straight
    """
    t = (color_current-min(color_left, color_right) -
         .5*abs(color_left-color_right))
    t = (2*t)/(color_left-color_right)
    # t = 0.88*(t**3+t)/2
    return t


def velocity_fn(x, base_velocity, steering_offset):
    """Calculates left and right velocity"""
    velocity_left = (min(base_velocity, base_velocity+2*base_velocity*x) +
                     x*base_velocity*min(steering_offset, 0))
    velocity_right = (min(base_velocity, base_velocity-2*base_velocity*x) +
                      x*base_velocity*max(steering_offset, 0))
    return (velocity_left, velocity_right)


def drive_robot(velocity):
    """Drives robot with left and right velocity"""
    left_motor.run(speed=velocity[0])
    right_motor.run(speed=velocity[1])


# Line Driving
def follow_line(base_velocity, color_left, color_right, driving_sensor, steering_offset):
    """Robot follows the line"""
    distance = obstacle_sensor.distance()
    velocity = base_velocity*min(1, max(0, ((distance-10)/20)))
    drive_robot(velocity_fn(norm(color_left, color_right, driving_sensor.reflection()), velocity, steering_offset))


def drive_to_line(color_line, color_base, sensor, velocity):
    """Drives robot and stops before the line"""
    drive_robot(velocity)
    while True:
        light_refl = sensor.reflection()
        if abs(color_line-light_refl) < abs(color_base-light_refl):
            drive_robot((0, 0))
            return


def drive_over_line(color_line, color_base, sensor, velocity):
    """Drives robot and stops after the line"""
    drive_to_line(color_line, color_base, sensor, velocity)
    drive_robot(velocity)
    while True:
        light_refl = sensor.reflection()
        if abs(color_line-light_refl) > abs(color_base-light_refl):
            drive_robot((0, 0))
            return


def rotate_on_line(color_line, sensor, velocity=(190, -190)):
    """Rotates robot until a sensor is on the line"""
    drive_robot(velocity)
    while True:
        if sensor_on_line(color_line, sensor):
            break
    drive_robot((0, 0))


def follow_line_straight(color_left, color_right, color_base, sensor, steering_offset):
    """Follows a line straight to the end of it"""
    while abs(norm(color_left, color_right, sensor.reflection())) > 0.01:
        follow_line(180, color_left, color_right, sensor, steering_offset)

    drive_robot((180, 180))
    while True:
        if sensor_on_line(color_base, sensor):
            break
    drive_robot((0, 0))


# Sensors
def calibrate():
    """Calibrates the color sensor"""
    color_left = left_light.reflection()
    color_right = right_light.reflection()
    ev3.speaker.beep()
    wait(2000)
    return color_left, color_right


def sensor_on_line(color_line, sensor, limit=19):
    """Returns true if sensor is on the line"""
    return abs(sensor.reflection()-color_line) < limit


# Parking
def unpark(color_line, color_base, driving_sensor):
    """Unparks the robots"""
    color_left, color_right, steering_offset = color_line, color_base, -1
    if driving_sensor == right_light:
        color_left, color_right, steering_offset = color_base, color_line, 1

    rotate_on_line(color_line, driving_sensor, (180*steering_offset, -180*steering_offset))
    follow_line_straight(color_left, color_right, color_base, driving_sensor, steering_offset)
    drive_over_line(color_line, color_base, driving_sensor, (180, 180))


def park_line(color_line, color_base, parking_sensor):
    """Parks robot"""
    color_left, color_right, steering_offset = color_line, color_base, -1
    if parking_sensor == right_light:
        color_left, color_right, steering_offset = color_base, color_line, 1

    drive_over_line(color_line, color_base, parking_sensor, (180, 180))
    follow_line_straight(color_left, color_right, color_base, parking_sensor, steering_offset)


def empty_parking_spot(color_line, parking_sensor):
    """Returns true if the parking spot is empty"""
    velocity = (150, -150)
    if parking_sensor == left_light:
        velocity = (-150, 150)

    drive_robot(velocity)
    distances = []
    for i in range(16):
        wait(50)
        distances.append(obstacle_sensor.distance())

    rotate_on_line(color_line, parking_sensor, (velocity[1], velocity[0]))
    return min(distances) > 21


def parking_mode(color_line, color_base, driving_sensor, parking_sensor):
    """Attempts to park the robot. Returns False if parking spot was occupied"""
    parking_empty = empty_parking_spot(color_line, parking_sensor)

    if random.randint(1,2) == 2:
        parking_empty = False
    
    print("PE", parking_empty)

    if parking_empty:
        park_line(color_line, color_base, parking_sensor)
        wait(random.randint(1, 7)*1000)
        unpark(color_line, color_base, driving_sensor)
        return True
    return False


def main():
    """Main Function"""
    base_velocity = BASE_VELOCITY
    color_line, color_base = 0,100 #calibrate()  # Left on line, right on base
    mode = DRIVING_MODE

    driving_sensor, parking_sensor, color_left, color_right, steering_offset = driving_mode(color_line, color_base, mode)

    drive_to_line(color_line, color_base, driving_sensor, (180, 180))
    # drive_over_line(color_line, color_base, driving_sensor, (180, 180))
    timer = time.time()

    while True:
        follow_line(base_velocity, color_left, color_right, driving_sensor, steering_offset)

        if sensor_on_line(color_line, parking_sensor) and time.time()-timer > 1.6:
            parking_enabled = not parking_mode(color_line, color_base, driving_sensor, parking_sensor)
            timer = time.time()


if __name__ == '__main__':
    main()
