#!/usr/bin/env pybricks-micropython
# Mohammad: 7302-31240
# Iftekhar: 7204-73469
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

# The wheel diameter of the robot EDucator is 56 milimeters
wheel_diameter = 56


def main():
    batteryCheck()


    # const for number of rotations to travel 1.2 m
    NUM_ROTATIONS = (1.2*1000)/(math.pi * wheel_diameter)
    rotationString = str(NUM_ROTATIONS)
    brick.display.clear()
    brick.display.text("ready", (60, 50))

    # Objective 1: Drive forward until hit wall. Then drive a little back
    drive_until_bumped_then_retreat(200)
    # Objective 2: Turn to the Right 90 degrees
    # Objective 3: Go forward until the wall to the left is gone.
    # Objective 4: Go forward for x amount of distance to clear the wall
    # Objective 5: Turn to the left 90 degrees
    # Objective 6: Go forward 1.2m



# set right motor to given power level
def left_motor_power_set_at(power):

# set left motor to given power level
def right_motor_power_set_at(power):

# turn for given degrees
def turn_for(degrees):

# follow a wall until a break is detected
def follow_wall():

    
# beep and then wait until button is pressed
def wait_for_buttonpress():
    # waiting for next command
    brick.sound.file('OOT_PauseMenu_Select.wav')
    brick.light(Color.RED)
    while not any(brick.buttons()):
        wait(10)

    # running next command acknowledgement
    brick.light(Color.GREEN)


# All functions for general robot use
def batteryCheck():
    if brick.battery.voltage() < 7000:
        brick.sound.beep(3)

# drives forward at a constant given rotational speed
def drive_robot_at_angular_speed(rotational_speed):
    right_motor.run(rotational_speed)
    left_motor.run(rotational_speed)

# stops robot by actively braking
def stop_robot():
    right_motor.stop(Stop.HOLD)
    left_motor.stop(Stop.HOLD)

# go forward given distance
# for a number of given speed ( 150 = baseline) 
# then stop
def drive_robot_for_distance(rightRotations, leftRotations, speed):
    right_motor.run_target(speed, 360*leftRotations, Stop.HOLD, False)
    left_motor.run_target(speed, 360*rightRotations, Stop.HOLD, True)

#drive straight with proportion from PID
#distance in meters
def drive_straight_with_pid(distance):
    NUM_ROTATIONS = (distance*1000)/(math.pi * wheel_diameter)
    kp = 1





    

# sense for distance in cm 
# then stop
def ultrasonic_sense_for_distance(distance):
    drive_robot_forward()
    while(distanceSensor.distance() > distance):
        wait(10)
    stop_robot()

# go forward until bump sensor is hit
# then back up until 50cm away from the wall
def drive_until_bumped():
    while not bump_sensor.pressed():
        drive_robot_at_angular_speed(125)
    stop_robot()


# go forward until bump sensor is hit
# then back up until distance (mm) away from the wall
def drive_until_bumped_then_retreat(distance):
    while not bump_sensor.pressed():
        drive_robot_at_angular_speed(125)
    stop_robot()

    while(distanceSensor.distance() < distance):
        drive_robot_backward()
    stop_robot()
    brick.sound.file('OOT_Song_Correct_Mono.wav')

main()
    