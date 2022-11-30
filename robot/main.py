#!/usr/bin/env pybricks-micropython
import time
import random
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Color
from pybricks.tools import wait
from pybricks.messaging import BluetoothMailboxServer, TextMailbox

# Robot definition
ev3 = EV3Brick()

# Motor definitions
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)

# Sensor definitions
left_light = ColorSensor(Port.S3)  # S3
obstacle_sensor = UltrasonicSensor(Port.S4)  # S4
right_light = ColorSensor(Port.S2)  # S2

# Messages
MSG_PARK = 'park'
MSG_BOTH_PARKED = 'both_parked'
MSG_UNPARK = 'unpark'
MSG_ROTATE = 'rotate'
MSG_UNROTATE = 'unrotate'
REC_PARKED = 'parked'
REC_UNPARKED = 'unparked'

# Color definitions
COLOR_DRIVING = Color.GREEN
COLOR_PARKING_ENABLED = Color.YELLOW
COLOR_PARKING = Color.YELLOW
COLOR_UNPARKING = Color.GREEN
COLOR_PARKED = Color.YELLOW
COLOR_WAITING = Color.RED
COLOR_BOTH_PARKED = Color.RED
COLOR_REVERSED = Color.GREEN

# Driving definitions
DRIVING_MODE = -1
BASE_VELOCITY = 200


# Driving
def driving_mode(color_line, color_base, mode):
    """Returns constants for choosen driving mode"""
    if mode == 1:  # Left sensor on left side of the line
        return left_light, right_light, color_base, color_line, -1
    if mode == -1:  # Right sensor on right side of the line
        return right_light, left_light, color_line, color_base, 1


def norm(color_left, color_right, color_current):
    """Returns a number between -1 and 1 indicating turn rate"""
    t = (color_current-min(color_left, color_right)-.5*abs(color_left-color_right))
    t = (2*t)/(color_left-color_right)
    t = 0.88*(t**3+t)/2
    return t


def velocity_fn(x, velocity, steering_offset):
    """Calculates left and right velocity"""
    velocity_left = (min(velocity, velocity+2*velocity*x)+x*velocity*min(steering_offset, 0))
    velocity_right = (min(velocity, velocity-2*velocity*x)+x*velocity*max(steering_offset, 0))
    return (velocity_left, velocity_right)


def drive_robot(velocity):
    """Drives robot with left and right velocity"""
    left_motor.run(speed=velocity[0])
    right_motor.run(speed=velocity[1])


def rotate180():
    """Rotates robot 180 deg"""
    drive_robot((200,200))
    wait(1000)
    drive_robot((-200,200))
    wait(2520)
    drive_robot((0,0))


# Line Following
def follow_line(color_left, color_right, driving_sensor, steering_offset, cc = True):
    """Robot follows the line with cc"""
    velocity = BASE_VELOCITY
    if cc:
        distance = obstacle_sensor.distance()
        velocity = BASE_VELOCITY*min(1, max(0,((distance-10)/20)))
    drive_robot(velocity_fn(norm(color_left, color_right, driving_sensor.reflection()), velocity, steering_offset))


def follow_line_straight(color_left, color_right, color_base, sensor, steering_offset, limit=2):
    """Follows a line straight to the end of it"""
    parking_timer = time.time()
    
    while time.time() - parking_timer < limit:
        follow_line(color_left, color_right, sensor, steering_offset, False)

    drive_robot((BASE_VELOCITY, BASE_VELOCITY))
    while True:
        if sensor_on_line(color_base, sensor):
            break
    drive_robot((0, 0))


def stop_before_line(color_line, color_base, sensor, velocity):
    """Drives robot and stops before the line"""
    drive_robot(velocity)
    while True:
        light_refl = sensor.reflection()
        if abs(color_line-light_refl) < abs(color_base-light_refl):
            drive_robot((0, 0))
            return


def stop_past_line(color_line, color_base, sensor, velocity):
    """Drives robot and stops after the line"""
    stop_on_line(color_line, sensor, velocity)
    drive_robot(velocity)
    while True:
        light_refl = sensor.reflection()
        if abs(color_line-light_refl) > abs(color_base-light_refl):
            drive_robot((0, 0))
            return


def stop_on_line(color_line, sensor, velocity):
    """Drives robot and stops on the line"""
    drive_robot(velocity)
    while True:
        if sensor_on_line(color_line, sensor):
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


def sensor_on_line(color_line, sensor, limit=2):
    """Returns true if sensor is on the line"""
    return abs(sensor.reflection()-color_line) < limit


# Parking
def wait_for_client(mbox, msg):
    """Waits for a matching client message"""
    while not mbox.read() == msg:
        wait(1000)


def unpark(color_line, color_base, driving_sensor, mbox):
    """Unparks the robots"""
    mbox.send(MSG_UNPARK)
    wait_for_client(mbox, REC_UNPARKED)
    ev3.light.on(COLOR_UNPARKING)

    color_left, color_right, steering_offset = color_line, color_base, -1
    if driving_sensor == right_light:
        color_left, color_right, steering_offset = color_base, color_line, 1

    stop_before_line(color_line, color_base, driving_sensor, (BASE_VELOCITY*steering_offset, -BASE_VELOCITY*steering_offset))
    follow_line_straight(color_left, color_right, color_base, driving_sensor, steering_offset, 1)
    stop_past_line(color_line, color_base, driving_sensor, (BASE_VELOCITY, BASE_VELOCITY))


def park(color_line, color_base, parking_sensor):
    """Parks robot"""
    ev3.light.on(COLOR_PARKING)
    color_left, color_right, steering_offset = color_line, color_base, -1
    if parking_sensor == right_light:
        color_left, color_right, steering_offset = color_base, color_line, 1

    stop_on_line(color_base, parking_sensor, (BASE_VELOCITY,BASE_VELOCITY))
    follow_line_straight(color_left, color_right, color_base, parking_sensor, steering_offset, 2.5)


def empty_parking_spot_old():
    drive_robot(velocity_fn(-1, 150, -1))
    distances = []
    for i in range(13):
        wait(100)
        distances.append(obstacle_sensor.distance())

    drive_robot(velocity_fn(1, 150, -1))
    wait(1300)
    distance = min(distances)
    return distance > 210


def empty_parking_spot(color_line, parking_sensor):
    """Returns true if the parking spot is empty"""
    velocity = (180, -180)
    if parking_sensor == left_light:
        velocity = (-180, 180)

    drive_robot(velocity)
    distances = []
    for i in range(12):
        wait(30)
        distances.append(obstacle_sensor.distance())

    stop_on_line(color_line, parking_sensor, (velocity[1], velocity[0]))
    return min(distances) > 210


def parking_mode(color_line, color_base, driving_sensor, parking_sensor, mbox):
    """Attempts to park and unpark the robot. Returns False if parking spot is occupied"""
    parking_empty = empty_parking_spot(color_line, parking_sensor)
    #parking_empty = empty_parking_spot_old()

    if parking_empty:
        park(color_line, color_base, parking_sensor)
        ev3.light.on(COLOR_PARKED)

        wait_for_client(mbox, REC_PARKED)
        mbox.send(MSG_BOTH_PARKED)
        ev3.light.on(COLOR_BOTH_PARKED)

        wait(random.randint(1, 7)*1000)
        unpark(color_line, color_base, driving_sensor, mbox)
        return True
    return False


# Bluetooth
def connect():
    """Connects to another robot via Bluetooth"""
    ev3.light.on(COLOR_WAITING)
    server = BluetoothMailboxServer()
    mbox = TextMailbox('greeting', server)
    print("Waiting for connection..")
    server.wait_for_connection()
    print("Connected..")
    return mbox


# Reverse
def reverse(mode, mbox):
    """Sets new values when reversing"""
    reversed_limit = random.randint(30, 60)
    reverse_mode = False 
    if mode == DRIVING_MODE:
        ev3.light.on(COLOR_DRIVING)
        mbox.send(MSG_UNROTATE)
    else:
        mbox.send(MSG_ROTATE)
        ev3.light.on(COLOR_REVERSED)
        reversed_limit = random.randint(10, 20)
        reverse_mode = True
    return reversed_limit, reverse_mode


def main():
    """Main Function"""
    parking_enabled = False
    reverse_mode = False
    color_line, color_base = calibrate()  # Left on line, right on base
    mode = DRIVING_MODE

    driving_sensor, parking_sensor, color_left, color_right, steering_offset = driving_mode(color_line, color_base, mode)
    mbox = connect()
    ev3.light.on(COLOR_DRIVING)

    stop_on_line(color_line, driving_sensor, (BASE_VELOCITY, BASE_VELOCITY))

    timer = time.time()
    reversed_timer = time.time()
    reversed_limit = random.randint(20, 60)

    while True:
        follow_line(color_left, color_right, driving_sensor, steering_offset, True)

        # Parking
        if sensor_on_line(color_line, parking_sensor) and parking_enabled and time.time()-timer > 1.6:
            parking_enabled = not parking_mode(color_line, color_base, driving_sensor, parking_sensor, mbox)
            timer = time.time()
            if reverse_mode and not parking_enabled:
                ev3.light.on(COLOR_REVERSED)
            elif not parking_enabled:
                ev3.light.on(COLOR_DRIVING)

        # Reverse
        if time.time()-reversed_timer > reversed_limit and time.time()-timer > 4:
            mode *= -1
            driving_sensor, parking_sensor, color_left, color_right, steering_offset = driving_mode(color_line, color_base, mode)
            reversed_limit, reverse_mode = reverse(mode, mbox)
            rotate180()
            reversed_timer = time.time()
            timer = time.time()
            parking_enabled = False

        # Enable parking
        if time.time() - timer > 15 and not parking_enabled and not reverse_mode:
            mbox.send(MSG_PARK)
            parking_enabled = True
            ev3.light.on(COLOR_PARKING_ENABLED)


if __name__ == '__main__':
    main()
