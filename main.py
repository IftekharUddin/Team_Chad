#!/usr/bin/env pybricks-micropython
from pybricks import ev3brick as brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import (Port, Stop, Direction, Button, Color,
                                 SoundFile, ImageFile, Align)
from pybricks.tools import print, wait, StopWatch

import math 

# set sensors
bump_sensor = TouchSensor(Port.S1)
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
distanceSensor = UltrasonicSensor(Port.S4)

def main():
    # The wheel diameter of the robot EDucator is 56 milimeters
    wheel_diameter = 56

    # The axle track is the distance between the centers of each of th ewheels.
    # For the Robot Educator this is 114 milimeters
    axle_track = 114

    #const for number of rotations to travel 1.2 m
    NUM_ROTATIONS = (1.2*1000)/(math.pi * wheel_diameter)
    rotationString = str(NUM_ROTATIONS)
    brick.display.clear()
    brick.display.text(rotationString, (60, 50))

    # Objective 1
    wait_for_buttonpress()
    drive_robot_for_distance(NUM_ROTATIONS, NUM_ROTATIONS) 

    # Objective 2
    wait_for_buttonpress()
    ultrasonic_test()

    # Objective 3
    wait_for_buttonpress()
    bump_wall()
    
# beep and then wait until button is pressed
def wait_for_buttonpress():
    brick.sound.beep()
    while not any(brick.buttons()):
        wait(10)

def drive_robot_forward():
    right_motor.run(150)
    left_motor.run(150)

def drive_robot_backward():
    right_motor.run(-150)
    left_motor.run(-150)

def stop_robot():
    right_motor.stop(Stop.HOLD)
    left_motor.stop(Stop.HOLD)

# go forward 1.2m
# then stop
def drive_robot_for_distance(rightRotations, leftRotations):
    right_motor.run_target(150, 360*leftRotations, Stop.HOLD, False)
    left_motor.run_target(150, 360*rightRotations, Stop.HOLD, True)

# go forward until 50cm away from the wall infront of you
# then stop
def ultrasonic_test():
    drive_robot_forward()
    while(distanceSensor.distance() > 500):
        wait(10)
    
    stop_robot()

# go forward until bump sensor is hit
# then back up until 50cm away from the wall
def bump_wall():
    drive_robot_forward()
    while not bump_sensor.pressed():
        pass
    stop_robot()

    drive_robot_backward()
    while(distanceSensor.distance() < 500):
        drive_robot_forward()
    stop_robot()
    
main()