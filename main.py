#!/usr/bin/env pybricks-micropython
# Mohammad: 7302-31240
# Iftekhar: 7204-73469
from pybricks import ev3brick as brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor, InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import (Port, Stop, Direction, Button, Color,
                                 SoundFile, ImageFile, Align)
from pybricks.tools import print, wait, StopWatch

import math 

MOVE_TO_WALL=1
TURN_RIGHT=2
TRACK_WALL=3
TURN_LEFT=4
MOVE_TO_GOAL=5
BACK_UP=7
END=6


# set sensors
bump_sensor = TouchSensor(Port.S1)
left_motor = Motor(Port.A)
right_motor = Motor(Port.C)
distanceSensor = UltrasonicSensor(Port.S4)

#we will measure time in SECONDS to match angular velocity, which is degress/sec
watch = StopWatch()

# The wheel diameter of the robot EDucator is 56 milimeters
wheel_diameter = 56/1000
wheel_radius = wheel_diameter/2

# The axle track is the distance between the centers of each of th ewheels.
# For the Robot Educator this is 114 milimeters
axle_track = 114/1000

currState = MOVE_TO_WALL

wall_counter = 0

init = False
back_init = False
final_init = False

last_distance = 0

def loop():
    global wall_counter
    global init 
    global back_init
    global final_init
    global currState
    while currState != END:
        if not init: #initialize program by creating start time and setting angular heading to 0 degrees
            currTime = watch.time()/1000
            heading = 0
            init = True
        else:
            # Objective 1: Drive forward until hit wall. Then drive a little back
            if currState == MOVE_TO_WALL:
                if drive_until_bumped():
                    currState = BACK_UP

            elif currState == BACK_UP:
                if not back_init:
                    startTime = watch.time()
                    drive_robot_at_angular_speed(-100)
                    back_init = True
                elif (watch.time() - startTime) > 1000:
                    stop_robot()
                    currState = TURN_RIGHT

            # Objective 2: Turn to the Right 90 degrees
            elif currState == TURN_RIGHT:
                if turned_right(heading):
                    currState = TRACK_WALL

            # Objective 3: Go forward until the wall to the left is gone.
            # Objective 4: Go forward for x amount of distance to clear the wall
            elif currState == TRACK_WALL:
                if tracked_wall():
                    currState = TURN_LEFT

            # Objective 5: Turn to the left 90 degrees
            elif currState == TURN_LEFT:
                if turned_left(heading):
                    currState = MOVE_TO_GOAL

            # Objective 6: Go forward .7m
            elif currState == MOVE_TO_GOAL:
                if not final_init:
                    right_motor.reset_angle(0)
                    left_motor.reset_angle(0)
                    final_init = True
                if drive_robot_for_distance(.85, 150):
                    currState = END
            elif currState == END:
                stop_robot()

            #DEAD RECKONING CALCULATIONS
            #integrate (vr-vl)/L by summation and multiplying by change in time
            vr = right_motor.speed() * wheel_radius
            vl = left_motor.speed() * wheel_radius
            # print(str(vr) + " " + str(vl))

            newTime = watch.time()/1000
            dt = newTime - currTime

            # print(str(heading))
            heading = heading + (vr - vl)*.9*(dt/axle_track)

            currTime = newTime
        

    
def turned_right(heading):
    error = abs(-90 - heading)
    kp = 5

    right_motor.run(-kp*error)
    left_motor.run(kp*error)

    if abs(heading+90) < 8:
        return True
    else:
        return False
    

def turned_left(heading):
    error = abs(heading)
    kp = 4

    right_motor.run(15+kp*error)
    left_motor.run(-(15+kp*error))

    if abs(heading) < 2:
        return True
    else:
        return False

def tracked_wall():
    kp = 130 
    global wall_counter
    global last_distance 

    distance = distanceSensor.distance()/1000

    dampener = 1
    if wall_counter >= 125:
        return True

    if not last_distance == 0 and abs(distance - last_distance) > .05:
        print("CHANGED")
        stop_robot()
        wait(1000)
        last_distance = distance
        return False

    if distance >= .5:
        dampener = .7
        wall_counter += 1
        left_motor.run(110)
        right_motor.run(90)
        print("ERROR" + str(distance))
        last_distance = distance
        return False
    else: 
        wall_counter = 0


    speed = 150 * dampener
    error = .20 - distance

    if error > .05:
        left_motor.run(speed)
        right_motor.run(speed/2)
    elif error < -.05:
        left_motor.run(speed/2)
        right_motor.run(speed)
    else:
        right_motor.run(speed - kp*error)
        left_motor.run(speed + kp*error)

    # print(str(distance))
    print(str(last_distance))

    last_distance = distance
    # turn = 25
    # speed = 200
    # if distance > .2:
    #     left_motor.run(speed - turn)
    #     right_motor.run(speed + turn)
    # elif distance < .15:
    #     left_motor.run(speed + turn)
    #     right_motor.run(speed - turn)
    

    # print(str(speed-kp*error) + " " + str(speed+kp*error))
    # print(str(distance))

    return False





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
def drive_robot_for_distance(distance, speed):
    NUM_ROTATIONS = (distance)/(math.pi * wheel_diameter)
    right_motor.run_target(speed, 360*NUM_ROTATIONS, Stop.HOLD, False)
    left_motor.run_target(speed, 360*NUM_ROTATIONS, Stop.HOLD, True)
    return True

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
    if not bump_sensor.pressed():
        drive_robot_at_angular_speed(200)
        return False
    stop_robot()
    print(str("bumped"))
    return True



# go forward until bump sensor is hit
# then back up until distance (mm) away from the wall
def drive_until_bumped_then_retreat(distance):
    while not bump_sensor.pressed():
        drive_robot_at_angular_speed(125)
    stop_robot()

    while(distanceSensor.distance() < distance):
        drive_robot_backward()
    stop_robot()
    return True
    
loop()
