#!/usr/bin/env pybricks-micropython
# Mohammad: 7302-31240
# Iftekhar: 7204-73469
from pybricks import ev3brick as brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor, InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import (Port, Stop, Direction, Button, Color,
                                 SoundFile, ImageFile, Align)
from pybricks.tools import print, wait, StopWatch

import math 

# set ENUMS
MOVE_TO_WALL=1
TURN_RIGHT=2
TRACK_WALL=3
TURN_AWAY_FROM_WALL = 8
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

# set initial state
currState = MOVE_TO_WALL

# counter to indicate when robot has passed the wall
wall_counter = 0
stall_counter = 0

init = False
back_init = False
final_init = False
turn_away_init = False

# temp variable to check delta distance
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
                # start going backward
                if not back_init:
                    startTime = watch.time()
                    drive_robot_at_angular_speed(-100)
                    back_init = True

                # go backward for one second
                elif (watch.time() - startTime) > 1500:
                    stop_robot()
                    currState = TURN_RIGHT

            # Objective 2: Turn to the Right 90 degrees
            elif currState == TURN_RIGHT:
                if turned_right(heading):
                    currState = TRACK_WALL

            # Objective 3: Go forward until the wall to the left is gone.
            # Objective 4: Go forward for x amount of distance to clear the wall
            elif currState == TRACK_WALL:
                if bump_sensor.pressed():
                    turn_away_init = False
                    currState = TURN_AWAY_FROM_WALL
                if tracked_wall():
                    currState = TURN_LEFT

            elif currState == TURN_AWAY_FROM_WALL:
                if turn_from_wall(5000):
                    currState = TRACK_WALL

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
                if drive_robot_for_distance(.90, 150):
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

            print(str(heading))
            heading = heading + (vr - vl)*.9*(dt/axle_track)

            currTime = newTime
        

# robot pivots on left wheel so the button isn't pressed anymore
def turn_from_wall(time):
    global turn_away_init
    global turn_away_start_time

    # start stopwatch of state
    if not turn_away_init:
        turn_away_start_time = watch.time()
        turn_away_init = True

    # stop after "time" miliseconds
    if watch.time() - turn_away_start_time > time:
        print("DONE")
        stop_robot()
        return True
    
    # set left motor to 0 and right motor to x
    left_motor.run(0)
    right_motor.run(-70)
    print("TURNING AWAY")
    return False
    


    
# turn right by adding 90 to the heading (turning right is negative) and multiplying that error by the constant kp
def turned_right(heading):
    error = abs(-90 - heading)
    kp = 5

    right_motor.run(-kp*error)
    left_motor.run(kp*error)

    # stop turning when the heading is within 8mm of our target heading
    if abs(heading+90) < 8:
        return True
    else:
        return False
    

# turn left using PID for the total angle we've accumulated (heading)
def turned_left(heading):
    error = abs(heading)
    kp = 4

    right_motor.run(15+kp*error)
    left_motor.run(-(15+kp*error))

    # stop if within 3.5 of our heading
    if abs(heading) < 3.5:
        return True
    else:
        return False

def tracked_wall():
    brick.light(Color.GREEN)
    global wall_counter
    global last_distance 
    kp = 1000 

    # set distance to seconds
    distance = distanceSensor.distance()/1000

    # print(str(distance))
    # set stall counter
    global stall_counter

    # print(stall_counter)
    # registered that the robot has passed the wall.
    # used because the bad sensor sometimes registered incorrect high values
    if wall_counter >= 85:
        return True

    # if stalled, move back slightly 
    if stall_counter > 0:
        brick.light(Color.RED)
        right_motor.run(45)
        left_motor.run(45)
        stall_counter -= 1
        return False

    # if too far from wall, turn left SHARP
    if distance >= .7:
        wall_counter += 1
        left_motor.run(110)
        right_motor.run(70)
        # print("ERROR" + str(distance))
        last_distance = distance
        return False
    else: 
        wall_counter = 0

    # stall and double check if the sudden change in distance is reading correctly
    if not last_distance == 0 and abs(distance - last_distance) > .05:
        # print("CHANGED")
        stop_robot()
        wait(100)
        last_distance = distance
        stall_counter = 15
        return False


    # set normal speed and error distance
    speed = 150 
    error = .17 - distance

    # too far, turn left normal
    if error > .05:
        left_motor.run(speed)
        right_motor.run(speed/2)
    # too close, turn right normal
    elif error < -.05:
        left_motor.run(speed/2)
        right_motor.run(speed)
    # just right, turn right/left proportional to the error. (PID)
    else:
        right_motor.run(speed - kp*error)
        left_motor.run(speed + kp*error)

    # print(str(distance))
    # print(str(last_distance))

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
    
wait_for_buttonpress()
loop()
