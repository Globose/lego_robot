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

# Here is where your code starts


def norm(color_left, color_right, color_current):
    """Returns a number between -1 and 1
        -1 => Turn left, 1 => turn right, 0 => drive straight
    """
    t = (color_current-min(color_left, color_right) -
         .5*abs(color_left-color_right))
    t = (2*t)/(color_left-color_right)
    t = 0.88*(t**3+t)/2
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


def park():
    """Parks the robot"""
    drive_robot((70, 150))
    wait(5000)
    drive_robot((150, 150))
    wait(100)
    drive_robot((0, 0))


def parking_mode(color_line, color_base, mbox):
    """Attempts to park the robot"""
    drive_robot(velocity_fn(-1, 150, -1))
    distances = []
    for i in range(13):
        wait(100)
        distances.append(obstacle_sensor.distance())

    drive_robot(velocity_fn(1, 150, -1))
    wait(1300)
    distance = min(distances)
    parking_empty = distance > 210

    if parking_empty:
        park()
        ev3.light.on(Color.RED)
        #wait(5000)
        client_parked = False
        while not client_parked:
            msg_from_client = mbox.read()
            last_msg = msg_from_client
            if last_msg == 'parked':
                client_parked = True
            wait(1000)
        
        mbox.send('both_parked')
        ev3.light.on(Color.YELLOW)
        wait(random.randint(1, 7)*1000)
        mbox.send('unpark')
        wait(7000)
        drive_over_line(color_line, color_base, right_light, (-120, -170))
        ev3.light.on(Color.GREEN)
        return True
    else:
        drive_robot((0, 150))
        wait(200)
        return False


def calibrate():
    """Calibrates the color sensor"""
    ev3.speaker.beep()
    color_left = left_light.reflection()
    color_right = right_light.reflection()
    ev3.speaker.beep()
    wait(2000)
    return color_left, color_right


def light_on_line(color_line, light):
    """Returns true if sensor is on the line"""
    ref = light.reflection()
    return abs(ref-color_line) < 2


def connect():
    server = BluetoothMailboxServer()
    mbox = TextMailbox('greeting', server)
    print("Waiting for connection..")
    server.wait_for_connection()
    print("Connected..")
    return mbox


def main():
    """Main Function"""
    ev3.light.on(Color.GREEN)
    color_left, color_right = calibrate()
    mbox = connect()
    
    parking_enabled = False
    
    last_msg = mbox.read()
    
    base_velocity = 170
    steering_offset = 1

    drive_to_line(color_left, color_right, right_light, (180,180))
    #drive_over_line(color_left, color_right, right_light, (180, 180))
    timer = time.time()
    
    while True:
        distance = obstacle_sensor.distance()
        velocity = base_velocity*min(1, ((distance-100)/200))

        vel = velocity_fn(norm(color_left, color_right, right_light.reflection()), velocity, steering_offset)
        drive_robot(vel)
        
        if parking_enabled and light_on_line(color_left, left_light) and time.time()-timer > 1.6:
            parked = parking_mode(color_left, color_right, mbox)
            timer = time.time()
            parking_enabled = not parked
        
        if time.time() - timer > 15 and not parking_enabled:
            print("sending parking")
            mbox.send('park')
            mbox.send('park')
            mbox.send('park')
            mbox.send('park')
            mbox.send('0')
            parking_enabled = True
        
        msg_from_client = mbox.read()
        if msg_from_client != last_msg:
            print(msg_from_client)
            last_msg = msg_from_client
            if last_msg == 'parked':
                client_parked = True
                

if __name__ == '__main__':
    main()
