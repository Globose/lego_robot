#!/usr/bin/env pybricks-micropython
from pybricks import robotics
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Stop, Direction, Color
from pybricks.tools import wait, StopWatch
from pybricks.robotics import DriveBase

# Robot definitions
ev3 = EV3Brick()

# Motor definitions
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)

# Sensor definitions
left_light = ColorSensor(Port.S3)

# Write your program here.

velocity = 100

def drive_robot(vel_left, vel_right):
    left_motor.run(speed=vel_left)
    right_motor.run(speed=vel_right)
   
def calibrate():
    """Calibrates the color sensor"""
    #print("Ställ sensor på banan")
    ev3.speaker.beep()
    #robot.turn(44)
    wait(5000)
    ref_on = left_light.reflection()
    ev3.speaker.beep()
    #print("Ställ sensor utanför banan")
    wait(5000)
    ref_off = left_light.reflection()
    ev3.speaker.beep()
    wait(5000)
    return ref_on, ref_off

def turn(ref_now, ref_mean, ref_diff):
    """Returns the turn velocity based on color reflection"""
    x = abs(ref_now-ref_mean)/(ref_diff/2)
    x1 = 5-(100/19)*x
    x2 = 1-x
    x = min(x1,x2)
    return x*velocity

def drive_forward(ref_on, ref_diff):
    """Drives robot forward until on the line"""
    drive_robot(velocity,velocity)
    while True:
        ref_now = left_light.reflection()
        if ref_now - ref_diff/2 < ref_on < ref_now + ref_diff/2:
            break
        
def main():
    """Main Function"""
    ref_on, ref_off = calibrate()
    ref_mean = (ref_on+ref_off)/2
    ref_diff = abs(ref_on-ref_off)
    
    drive_forward(ref_on, ref_diff)
    
    while True:
        ref_now = left_light.reflection()
        if ref_now < ref_mean:
            drive_robot(turn(ref_now,ref_mean, ref_diff),velocity)
        elif ref_now > ref_mean:
            drive_robot(velocity,turn(ref_now,ref_mean, ref_diff))
        else:
            drive_robot(velocity,velocity)
        wait(20)
     
if __name__ == '__main__':
    main()
