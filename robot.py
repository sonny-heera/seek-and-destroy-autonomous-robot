# Sandeep Heera
# robot.py
# This program drives a robot via the Brick Pi shield
# interface. The robot performs a variety of tasks including
# avoiding obstacles, searching for magnets, searching for 
# a white line, and scanning for specific objects 
# (spheres and cones) at particular intervals using the Pi
# camera in conjunction with OpenCV. All of these tasks are 
# accomplished via multiprocessing.
import RPi.GPIO as GPIO
from BrickPi import *
import time, math, sys, os, FaBo9Axis_MPU9250, multiprocessing, cv2, imutils
from multiprocessing import *

mpu9250 = FaBo9Axis_MPU9250.MPU9250()	# MPU 9250 object

# Ports for the Lego motors and sensors
MOTOR1 = PORT_B
MOTOR2 = PORT_C
LIGHT_SENSOR_PORT = PORT_4
PORTS_TO_ROTATE = [PORT_B, PORT_C]
TURN_SPEED_ENCODER = [100, 100]

# GPIO pins for the additional sensors and peripheral devices
TRIG_L = 19
ECHO_L = 23
TRIG_R = 24
ECHO_R = 25
TRIG_F = 21
ECHO_F = 20
MOTOR_3_A = 4
MOTOR_4_A = 17
MOTOR_1_A = 27
MOTOR_2_A = 22
FIRE_TURRET_MISSILE_PIN = 26
MISSILE_FIRED_TURRET_PIN = 6
RIGHT_LIMIT_TURRET_PIN = 13
LEFT_LIMIT_TURRET_PIN = 5
UPPER_LIMIT_TURRET_PIN = 16
LOWER_LIMIT_TURRET_PIN = 12
CANNON_FIRE_PIN = 18
LIGHT_SENSOR_PIN = 8

# General constants
TURN_90_ENCODER = [150, -150]
TURN_45_ENCODER = [75, -75]
TURN_30_ENCODER = [50, -50]
TURN_90_ENCODER_R = [-150, 150]
TURN_45_ENCODER_R = [-75, 75]
TURN_30_ENCODER_R = [-50, 50]
ROTATE_DEG_TIMEOUT = 2
REVERSE = [-80, -80]
SPEED_DELAY = 0.01
MAX_FRONT_DISTANCE = 20
MIN_SIDE_DISTANCE = 40
MAX_COLOR = 500
INFRARED_THRESHOLD = 0.00035
INFRARED_TIMEOUT = 0.003
PROCESS_DELAY = 0.10
POSITIVE_MAGNET_VALUE = 400
NEGATIVE_MAGNET_VALUE = -400
MAGNET_FOUND_VALUE = POSITIVE_MAGNET_VALUE
MAGNET_CLEAR_DELAY = 2
MAGNET_TIMEOUT = 25
LEFT_SWEEP_LIMIT = 12
RIGHT_SWEEP_LIMIT = -12
Z_OFFSET = 70
SENSOR_DELAY = 0.02
FIRE_MISSILE_DELAY = 3.1
FIRE_CANNON_MISSILE_DELAY = 0.200
MAX_TURRET_MISSILES = 4
MAX_CANNON_MISSILES = 6
MISSILE_TIMEOUT = 3
TURRET_DELAY = 0.2
FRONT = 0
LEFT = 1
RIGHT = 2
FAST = 125
SLOW = 40
MAX_US_DISTANCE = 255
MAX_PULSE_TIME = 1
MAX_RELOAD_OBJECT_DISTANCE = 5
MAX_RELOAD_OBJECT_TIMEOUT = 5
RELOAD_TIME = 30
MAX_TURRET_POSITION_L = 17
MAX_TURRET_POSITION_R = -17
TURRET_LEFT_90_THRESHOLD = 14
TURRET_LEFT_45_THRESHOLD = 9
TURRET_LEFT_30_THRESHOLD = 4
TURRET_RIGHT_90_THRESHOLD = -14
TURRET_RIGHT_45_THRESHOLD = -9
TURRET_RIGHT_30_THRESHOLD = -4
BRICK_PI_TIMEOUT = 3000

# OpenCV constants
CAMERA_WIDTH = 200
FRAME_RATE = 20
CV2_FRAME_RATE = 5
MIN_WIDTH = 5
MAX_WIDTH = 150
MIN_HEIGHT = 5
MAX_HEIGHT = 150
X_CENTER = CAMERA_WIDTH / 2
Y_CENTER = CAMERA_WIDTH * 3 / 4 / 2
TOLERANCE = 30
SPHERE = 0
UP_CONE = 1
DOWN_CONE = 2
SCALE_FACTOR_SPHERE_UP_CONE = 1.3
SCALE_FACTOR_DOWN_CONE = 1.3
MIN_NEIGHBORS_SPHERE_UP_CONE = 4
MIN_NEIGHBORS_DOWN_CONE = 2

# Initialize the camera
camera = cv2.VideoCapture(0)
camera.set(CV2_FRAME_RATE, FRAME_RATE)

# Rotates the ports given by the desire degrees. This function was taken 
# from: http://forum.dexterindustries.com/t/would-like-to-improve-motor-encoder-readings-from-python/1914/5
# A timeout of 2 seconds is enforced. This is due to the PID algorithm which in certain situations
# can generate a pwm output to the motors which is insufficient to turn 
# the robot.
def motor_rotate_deg(power,deg,port,sampling_time=.01,delay_when_stopping=.05):
    """Rotate the selected motors by specified degre
    Args:
      power    : an array of the power values at which to rotate the motors (0-255)
      deg    : an array of the angle's (in degrees) by which to rotate each of the motor
      port    : an array of the port's on which the motor is connected
      sampling_time  : (optional) the rate(in seconds) at which to read the data in the encoders
      delay_when_stopping:  (optional) the delay (in seconds) for which the motors are run in the opposite direction before stopping
        Returns:
        0 on success
        Usage:
        Pass the arguments in a list. if a single motor has to be controlled then the arguments             should be
      passed like elements of an array,e.g, motorRotateDegree([255],[360],[PORT_A]) or
      motorRotateDegree([255,255],[360,360],[PORT_A,PORT_B])
    """  
    debug = False
    num_motor=len(power)    #Number of motors being used
    #print num_motor
    init_val=[0]*num_motor
    curr_val=[0]*num_motor
    final_val=[0]*num_motor
    last_encod=[0]*num_motor
    
    delta=0
    gain=0.005
    idelta=0.0
    alpha=10
    smulti=0
    BrickPiUpdateValues()
    for i in range(num_motor):
        BrickPi.MotorEnable[port[i]] = 1        #Enable the Motors
        power[i]=abs(power[i])
        
        init_val[i]=BrickPi.Encoder[port[i]]        #Initial reading of the encoder  
        
        final_val[i]=init_val[i]+(deg[i]*2)        #Final value when the motor has to be stopped;One encoder value counts for 0.5 degrees
     
        #For running clockwise and anticlockwise
        if deg[i]>0:
           BrickPi.MotorSpeed[port[i]] = power[i]
        elif deg[i]<0:
           BrickPi.MotorSpeed[port[i]] = -power[i]
        else:
           BrickPi.MotorSpeed[port[i]] = 0
        
        
    run_stat=[0]*num_motor

    time_start = time.time()
    time_end = time.time()
    time_total = time_end - time_start
    
    while True:
        time_end = time.time()
        time_total = time_end - time_start
        if time_total >= ROTATE_DEG_TIMEOUT:
            break
        
        result = BrickPiUpdateValues()          #Ask BrickPi to update values for sensors/motors
        time.sleep(sampling_time)          #sleep for the sampling time given (default:10 ms)
        i = 0
        #if debug:
            #print "Result of Update Values: " + `result`
        if not result :
            for i in range(num_motor):        #Do for each of the motors
                #The FIRST thing we should do is check our encoders!
                curr_val[i]=BrickPi.Encoder[port[i]]
                if debug :
                    print "Motor " + `i` + " encoder: " + `curr_val[i]`
                        
                if run_stat[i]==1:
                    continue
                # Check if final value reached for each of the motors
                if(deg[i]>0 and final_val[i]<=curr_val[i]) or (deg[i]<0 and final_val[i]>=curr_val[i]) :
                    #This motor has reached its goal
                    run_stat[i]=1
                    
                    #Now let's hit the breaks by going in reverse for a VERY quick amount of time.
                    if deg[i]>0:
                        BrickPi.MotorSpeed[port[i]] = -power[i]
                    elif deg[i]<0:
                        BrickPi.MotorSpeed[port[i]] = power[i]
                    else:
                        BrickPi.MotorSpeed[port[i]] = 0            
                    BrickPiUpdateValues()
                    time.sleep(delay_when_stopping)
                    #Now let's turn the motor off all together
                    BrickPi.MotorEnable[port[i]] = 0
                    BrickPiUpdateValues()
        
        if(all(e==1 for e in run_stat)):        #If all the motors have already completed their rotation, then stop
          break
        
        #Let's use Proportional Integral Control on the Motors to keep them in Sync
        if i == 1 :
            if curr_val[0] <> 0 and curr_val[1] <>0 : 
                if last_encod[0]<>0 and last_encod[1] <>1 :
                    if abs(last_encod[0] - init_val[0]) < abs(last_encod[1] - init_val[1]) :
                        #Motor 1 is going faster
                        delta = abs(curr_val[1]-last_encod[1]) - abs(curr_val[0]-last_encod[0])
                        idelta = (abs(curr_val[1]-init_val[1]) - abs(curr_val[0]-init_val[0]))/alpha
                        if debug:
                            print "Motor 1 is faster by "  + `delta`
                            print "last_encod = " + `last_encod[0]` + " , " + `last_encod[1]`
                            print "idelta = " + `idelta`
                            print "Current Encode = " + `curr_val[0]` + " , " + `curr_val[1]`

                        if int(abs(BrickPi.MotorSpeed[port[0]])) == 255 :
                            #Motor 0 CANNOT be sped up
                            smulti=BrickPi.MotorSpeed[port[0]]*delta*gain+idelta*gain
                            #Speed Multiplier: the amount we want to slow down Motor 1
                            if int(abs(BrickPi.MotorSpeed[port[1]]-smulti)) <= 255 : 
                                #Target speed is inside the bounds of Motor speed
                                BrickPi.MotorSpeed[port[1]] = int (BrickPi.MotorSpeed[port[1]]-smulti)
                            elif int (BrickPi.MotorSpeed[port[1]]-smulti) < 0 :
                                #Target speed is outside the bounds of -255 to 255
                                BrickPi.MotorSpeed[port[1]] = -255
                            else :
                                BrickPi.MotorSpeed[port[1]] = 255
                            BrickPiUpdateValues()
                            if debug : 
                                print "Motor 1 speed : " + `BrickPi.MotorSpeed[port[1]]`
                                print "Speed Multiplier : " + `smulti`

                        else :
                            #Motor 0 CAN be sped up
                            smulti=BrickPi.MotorSpeed[port[0]]*delta*gain+idelta*gain
                            #Speed Multiplier: the amount we want to speed up Motor 0
                            if int(abs(BrickPi.MotorSpeed[port[0]]+smulti)) <= 255 :   
                                #Target speed is inside the bounds of Motor speed
                                BrickPi.MotorSpeed[port[0]] = int (BrickPi.MotorSpeed[port[0]]+smulti)
                            elif int (BrickPi.MotorSpeed[port[0]]+smulti) < 0 :
                                #Target speed is outside the bounds of -255 to 255
                                BrickPi.MotorSpeed[port[0]] = -255 
                            else :
                                BrickPi.MotorSpeed[port[0]] = 255
                            BrickPiUpdateValues()
                            if debug : 
                                print "Motor 0 speed : " + `BrickPi.MotorSpeed[port[0]]`
                                print "Speed Multiplier : " + `smulti`


                    elif (last_encod[0] - curr_val[0]) > abs(last_encod[1] - curr_val[1]) :
                        #Motor 0 is going faster
                        delta= abs(curr_val[0]-last_encod[0])- abs(curr_val[1]-last_encod[1]) 
                        idelta = (abs(curr_val[0]-init_val[0]) - abs(curr_val[1]-init_val[1]))/alpha
                        if debug :
                                print "Motor 0 is faster by "  + `delta`
                                print "last_encod = " + `last_encod[0]` + " , " + `last_encod[1]`
                                print "idelta = " + `idelta`
                                print "Current Encode = " + `curr_val[0]` + " , " + `curr_val[1]`

                        if abs(BrickPi.MotorSpeed[port[1]]) == 255 :
                            #Motor 1 CANNOT be sped up, SLOW DOWN Motor 0
                            smulti=BrickPi.MotorSpeed[port[0]]*delta*gain+idelta*gain
                            #Speed Multiplier: the amount we want to slow down Motor 0
                            if int(abs(BrickPi.MotorSpeed[port[0]]-smulti)) <= 255 :
                                #Target speed is inside the bounds of Motor
                                BrickPi.MotorSpeed[port[0]] = int (BrickPi.MotorSpeed[port[0]]-smulti)
                            elif int (BrickPi.MotorSpeed[port[0]]-smulti) < 0 :
                                #Target speed is outside the -255 to 255 bounds
                                BrickPi.MotorSpeed[port[0]] = -255
                            else : 
                                BrickPi.MotorSpeed[port[0]] = 255
                            BrickPiUpdateValues()
                            if debug : 
                                print "Motor 0 speed : " + `BrickPi.MotorSpeed[port[0]]`
                                print "Speed Multiplier : " + `smulti`

                        else :
                            #Motor 1 CAN be sped up SPEED UP Motor 1
                            smulti=BrickPi.MotorSpeed[port[0]]*delta*gain+idelta*gain
                            #Speed Multiplier: the amount we want to speed up Motor 1
                            if int(abs (BrickPi.MotorSpeed[port[1]]+smulti)) <= 255 :
                                #Target speed is inside the bounds of Motor
                                BrickPi.MotorSpeed[port[1]] = int (BrickPi.MotorSpeed[port[1]]+smulti)
                            elif int (BrickPi.MotorSpeed[port[1]]+smulti) < 0 :
                                #Target speed is outside the -255 to 255 bounds
                                BrickPi.MotorSpeed[port[1]] = -255
                            else :
                                BrickPi.MotorSpeed[port[1]] = 255
                            BrickPiUpdateValues()
                            if debug : 
                                print "Motor 1 speed : " + `BrickPi.MotorSpeed[port[1]]`
                                print "Speed Multiplier : " + `smulti`
                        
            last_encod[0] = curr_val[0]
            last_encod[1] = curr_val[1]
    BrickPi.MotorEnable[MOTOR1] = 1
    BrickPi.MotorEnable[MOTOR2] = 1
    return 0

# Checks the reading from the light sensor. If the value 
# is greater than the threshold, the robot will reverse
# and turn 45 degrees to the right.
def check_light_reading():
    BrickPiUpdateValues()
    color = BrickPi.Sensor[LIGHT_SENSOR_PORT]
	
    # While we have invalid values, call BrickPiUpdateValues()
    while color == -1:
        BrickPiUpdateValues()
        color = BrickPi.Sensor[LIGHT_SENSOR_PORT]
        time.sleep(SENSOR_DELAY)
        
    # Check the infrared light sensor has detected white
    infrared_reading = check_infrared_reading()
    
    # Check to see which sensors(s) detected the white line
    if infrared_reading and color > MAX_COLOR:
        stop()
        reverse()
        motor_rotate_deg(TURN_SPEED_ENCODER, TURN_45_ENCODER_R, PORTS_TO_ROTATE)
        forward(SLOW, SLOW)
    elif color < MAX_COLOR and not infrared_reading:
        stop()
        reverse()
        motor_rotate_deg(TURN_SPEED_ENCODER, TURN_45_ENCODER, PORTS_TO_ROTATE)
        forward(SLOW, SLOW)
    elif color < MAX_COLOR and infrared_reading:
        stop()
        reverse()
        motor_rotate_deg(TURN_SPEED_ENCODER, TURN_45_ENCODER_R, PORTS_TO_ROTATE)
        forward(SLOW, SLOW)

# Checks the reading of the digital infrared sensor which is placed on the
# left side of the robot.
def check_infrared_reading():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(LIGHT_SENSOR_PIN, GPIO.OUT)
    GPIO.output(LIGHT_SENSOR_PIN, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.setup(LIGHT_SENSOR_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    time_start = time.time()
    time_end = time.time()
    time_total = time_end - time_start
    while(GPIO.input(LIGHT_SENSOR_PIN) == GPIO.HIGH and time_total < INFRARED_TIMEOUT):
        time_end = time.time()
        time_total = time_end - time_start

    if time_total <= INFRARED_THRESHOLD:
        return True
    else:
        return False

# Sweeps in the direction specified by the input. If a limit switch
# has been reached, the opposite direction will be returned
def sweep(direction, turret_lock, left_flag, right_flag, turret_position):
    if direction == LEFT:
        with turret_lock:
            left_flag.value = True
        print('Turret position: ' + str(turret_position.value))
        if turret_position.value >= LEFT_SWEEP_LIMIT:
            return RIGHT
        else:
            return LEFT
    elif direction == RIGHT:
        with turret_lock:
            right_flag.value = True
        print('Turret position: ' + str(turret_position.value))  
        if turret_position.value <= RIGHT_SWEEP_LIMIT:
            return LEFT
        else:
            return RIGHT
            
# Checks the magnet reading from the MPU9250
# and determines if the magnetic field of the 
# z-axis has surpassed a particular threshold.
def check_magnet_reading(object_found, magnet_found, turret_lock, turret_position, turret_position_lock, left_flag, right_flag):
    while True:
        mag = mpu9250.readMagnet()
        z = mag['z'] - Z_OFFSET
        if MAGNET_FOUND_VALUE == POSITIVE_MAGNET_VALUE:
            if z >= MAGNET_FOUND_VALUE:
                print('MAGNET')
                stop()
                magnet_found.value = True
                time_start = time.time()
                time_end = time.time()
                time_total = time_end - time_start
                direction = 0
                if turret_position.value >= 0:
                    direction = RIGHT
                else:
                    direction = LEFT
                while time_total < MAGNET_TIMEOUT:
                    if not object_found.value:
                        direction = sweep(direction, turret_lock, left_flag, right_flag, turret_position)
                    time_end = time.time()
                    time_total = time_end - time_start
                with turret_lock and turret_position_lock:
                    if not turret_position.value == 0:
                        center_turret()
                        turret_position.value = 0
                reverse()
                motor_rotate_deg(TURN_SPEED_ENCODER, TURN_90_ENCODER, PORTS_TO_ROTATE)
                magnet_found.value = False
                print('Magnet phase ended!')
                time.sleep(MAGNET_CLEAR_DELAY)
                
        elif MAGNET_FOUND_VALUE == NEGATIVE_MAGNET_VALUE:
            if z <= MAGNET_FOUND_VALUE:
                print('MAGNET')
                stop()
                magnet_found.value = True
                time_start = time.time()
                time_end = time.time()
                time_total = time_end - time_start
                direction = 0
                if turret_position.value >= 0:
                    direction = RIGHT
                else:
                    direction = LEFT
                while time_total < MAGNET_TIMEOUT:
                    if not object_found.value:
                        direction = sweep(direction, turret_lock, left_flag, right_flag, turret_position)
                    time_end = time.time()
                    time_total = time_end - time_start
                with turret_lock and turret_position_lock:
                    if not turret_position.value == 0:
                        center_turret()
                        turret_position.value = 0
                reverse()
                motor_rotate_deg(TURN_SPEED_ENCODER, TURN_90_ENCODER, PORTS_TO_ROTATE)
                magnet_found.value = False
                print('Magnet phase ended!')
                time.sleep(MAGNET_CLEAR_DELAY)     
        time.sleep(PROCESS_DELAY)

# Moves the turret if any of the flags (left, right, up, down)
# are set.
def move_turret(left_flag, right_flag, up_flag, down_flag, lock, turret_position, turret_position_lock):
    while True:
        with lock:
            if left_flag.value == True:
                turn_turret_left()
                with turret_position_lock:
                    current_position = turret_position.value
                    turret_position.value = current_position + 1
                left_flag.value = False
            elif right_flag.value == True:
                turn_turret_right()
                with turret_position_lock:
                    current_position = turret_position.value
                    turret_position.value = current_position - 1
                right_flag.value = False
            if up_flag.value == True:
                turn_turret_up()
                up_flag.value = False
            elif down_flag.value == True:
                turn_turret_down()
                down_flag.value = False
                
        time.sleep(PROCESS_DELAY)
		
# Searches for the shape specified by the cascade and returns a list 
# containing coordinates of all shapes found in the image.
def search_for_shape(cascade, color, shape):
    if shape == SPHERE or shape == UP_CONE:
        shapes = cascade.detectMultiScale(color, scaleFactor = SCALE_FACTOR_SPHERE_UP_CONE, 
                                          minNeighbors = MIN_NEIGHBORS_SPHERE_UP_CONE,
                                          minSize = (MIN_WIDTH, MIN_HEIGHT), 
                                          flags = cv2.cv.CV_HAAR_SCALE_IMAGE)
    elif shape == DOWN_CONE:
        shapes = cascade.detectMultiScale(color, scaleFactor = SCALE_FACTOR_DOWN_CONE, 
                                          minNeighbors = MIN_NEIGHBORS_DOWN_CONE,
                                          minSize = (MIN_WIDTH, MIN_HEIGHT), 
                                          flags = cv2.cv.CV_HAAR_SCALE_IMAGE)
    return shapes

# Centers the turret on the detected object. If the object is centered, 
# a missile is fired from the turret or cannon at the object.
def center_on_shape(shapes, color, img, search_for_object, left_flag,
                    right_flag, up_flag, down_flag, search_lock, turret_lock,
		    missiles_remaining, shape_type, last_fire_time, c_missiles_remaining,
                    turret_position, turret_position_lock):
    last_fired = last_fire_time
    missiles_left = missiles_remaining
    c_missiles_left = c_missiles_remaining
    x = shapes[0][0]
    y = shapes[0][1]
    w = shapes[0][2]
    h = shapes[0][3]
        
    # if it's a sphere or upward pointing cone, shoot at it
    if shape_type == SPHERE or shape_type == UP_CONE:
            if w < MAX_WIDTH and h < MAX_HEIGHT:
                with search_lock:
                    search_for_object.value = True
                with turret_lock:
                    center_of_object = (x + w / 2, y + h / 2)       
                    if center_of_object[0] < X_CENTER - TOLERANCE:
                        left_flag.value = True
                    elif center_of_object[0] > X_CENTER + TOLERANCE:
                        right_flag.value = True
                    if center_of_object[1] < Y_CENTER - TOLERANCE:
                        up_flag.value = True
                    elif center_of_object[1] > Y_CENTER + TOLERANCE:
                        down_flag.value = True
                                
                time_end = time.time()
                time_total = time_end - last_fired
                # If the image is centered, fire a missile from the turret or cannon
                if (center_of_object[0] > X_CENTER - TOLERANCE and center_of_object[0] < X_CENTER + TOLERANCE
                    and center_of_object[1] > Y_CENTER - TOLERANCE and center_of_object[1] < Y_CENTER + TOLERANCE):
                    if shape_type == SPHERE:
                        print 'SPHERE centered.'
                        if time_total > MISSILE_TIMEOUT:
                            fire_turret_missile()
                            missiles_left = missiles_remaining - 1
                            print('Turret missiles remaining: ' + str(missiles_left))
                            if missiles_remaining == 0:
                                print 'Out of ammo! Need to reload turret!'
                            with turret_lock:
                                center_turret()
                            with turret_position_lock:
                                turret_position.value = 0
                            last_fired = time.time()
                        else:
                            print 'Not enough time has passed.'
                    elif shape_type == UP_CONE:
                        print 'UP_CONE centered.'
                        if time_total > MISSILE_TIMEOUT:
                            fire_cannon_missile()
                            c_missiles_left = c_missiles_remaining - 1
                            print('Cannon missiles remaining: ' + str(c_missiles_left))
                            if c_missiles_remaining == 0:
                                print 'Out of ammo! Need to reload cannon!'
                            with turret_lock:
                                center_turret()
                            with turret_position_lock:
                                turret_position.value = 0
                            last_fired = time.time()
                        else:
                            print 'Not enough time has passed.'
                            
                    with search_lock:
                        search_for_object.value = False
                else:
                    with search_lock:
                        search_for_object.value = False
                               
                cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)
                roi_gray = color[y:y+h, x:x+w]
                roi_color = img[y:y+h, x:x+w]
        
    # if it's a downward pointing cone, realign the robot and move towards it
    elif shape_type == DOWN_CONE:
        if w < MAX_WIDTH and h < MAX_HEIGHT:
            with search_lock:
                search_for_object.value = True
            print('INVERTED CONE')
            with turret_lock:
                center_of_object = (x + w / 2, y + h / 2)       
                if center_of_object[0] < X_CENTER - TOLERANCE:
                    left_flag.value = True
                elif center_of_object[0] > X_CENTER + TOLERANCE:
                    right_flag.value = True
                if center_of_object[1] < Y_CENTER - TOLERANCE:
                    up_flag.value = True
                elif center_of_object[1] > Y_CENTER + TOLERANCE:
                    down_flag.value = True
            if (center_of_object[0] > X_CENTER - TOLERANCE and center_of_object[0] < X_CENTER + TOLERANCE
                and center_of_object[1] > Y_CENTER - TOLERANCE and center_of_object[1] < Y_CENTER + TOLERANCE):
                # center the robot on the downward pointing cone
                if turret_position.value >= TURRET_LEFT_90_THRESHOLD:
                    motor_rotate_deg(TURN_SPEED_ENCODER, TURN_90_ENCODER, PORTS_TO_ROTATE)
                elif turret_position.value < TURRET_LEFT_90_THRESHOLD and turret_position.value >= TURRET_LEFT_45_THRESHOLD:
                    motor_rotate_deg(TURN_SPEED_ENCODER, TURN_45_ENCODER, PORTS_TO_ROTATE)
                elif turret_position.value > TURRET_LEFT_45_THRESHOLD and turret_position.value >= TURRET_LEFT_30_THRESHOLD:
                    motor_rotate_deg(TURN_SPEED_ENCODER, TURN_30_ENCODER, PORTS_TO_ROTATE)
                elif turret_position.value <= TURRET_RIGHT_90_THRESHOLD:
                    motor_rotate_deg(TURN_SPEED_ENCODER, TURN_90_ENCODER_R, PORTS_TO_ROTATE)
                elif turret_position.value > TURRET_RIGHT_90_THRESHOLD and turret_position.value <= TURRET_RIGHT_45_THRESHOLD:
                    motor_rotate_deg(TURN_SPEED_ENCODER, TURN_45_ENCODER_R, PORTS_TO_ROTATE)
                elif turret_position.value > TURRET_RIGHT_45_THRESHOLD and turret_position.value <= TURRET_RIGHT_30_THRESHOLD:
                    motor_rotate_deg(TURN_SPEED_ENCODER, TURN_30_ENCODER_R, PORTS_TO_ROTATE)
                            
                time_out_start = time.time()
                time_out_total = 0
                # Reposition the robot so it moves toward the downward pointing cone
                while (get_distance(FRONT) > MAX_RELOAD_OBJECT_DISTANCE and time_out_total < MAX_RELOAD_OBJECT_TIMEOUT):
                    time_end = time.time()
                    time_out_total = time_end - time_out_start
                    forward(SLOW, SLOW)
                            
                # wait for reload
                print 'Start reload (30s).'
                time.sleep(RELOAD_TIME)
                print 'Reload phase ended.'

                # Update variables
                missiles_left = MAX_TURRET_MISSILES
                c_missiles_left = MAX_CANNON_MISSILES

                with turret_lock:
                    center_turret()
                with turret_position_lock:
                    turret_position.value = 0
                    
                with search_lock:
                    search_for_object.value = False
            else:
                with search_lock:
                    search_for_object.value = False
                           
            cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)
            roi_gray = color[y:y+h, x:x+w]
            roi_color = img[y:y+h, x:x+w]
                    
    return (missiles_left, last_fired, c_missiles_left)
			
# Scans the area for a sphere or cone (upward or downward orientation).
# If a sphere or cone is located, the robot will attempt to center the
# object on the camera and fire a missile at it.
def scan_camera(search_for_object, left_flag, right_flag, up_flag, down_flag, search_lock,
                turret_lock, turret_position, turret_position_lock):
    missiles_remaining = MAX_TURRET_MISSILES
    c_missiles_remaining = MAX_CANNON_MISSILES
    last_fired_time = time.time()
    turret_data = (missiles_remaining, last_fired_time, c_missiles_remaining)
    
    while True:
        (grabbed, frame) = camera.read()
        frame = imutils.resize(frame, width = CAMERA_WIDTH)
        frame = cv2.flip(frame, -1)
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        sphere_cascade = cv2.CascadeClassifier('sphere.xml')
        up_cone_cascade = cv2.CascadeClassifier('up_cone.xml')
        down_cone_cascade = cv2.CascadeClassifier('down_cone.xml')
        img = frame
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        # search for objects in order of priority (sphere > upward cone > downward cone)
        spheres = search_for_shape(sphere_cascade, gray, SPHERE)
        up_cones = search_for_shape(up_cone_cascade, gray, UP_CONE)
        down_cones = search_for_shape(down_cone_cascade, gray, DOWN_CONE)
        
        # check for spheres
        if len(spheres) > 0 and turret_data[0] > 0:
            turret_data = center_on_shape(spheres, gray, img, search_for_object, left_flag, right_flag,
                              up_flag, down_flag, search_lock, turret_lock, turret_data[0], SPHERE,
                                                      turret_data[1], turret_data[2], turret_position,
                                                      turret_position_lock)
                            
        # check for upward pointing cones
        if len(up_cones) > 0 and turret_data[2] > 0:
            turret_data = center_on_shape(up_cones, gray, img, search_for_object, left_flag, right_flag,
                            up_flag, down_flag, search_lock, turret_lock, turret_data[0], UP_CONE,
                            turret_data[1], turret_data[2], turret_position, turret_position_lock)
                            
        # check for downward pointing cones
        if (len(down_cones) > 0 and turret_data[0] == 0 and turret_data[2] == 0):
            turret_data = center_on_shape(down_cones, gray, img, search_for_object, left_flag, 
                            right_flag, up_flag, down_flag, search_lock, turret_lock, turret_data[0],
                            DOWN_CONE, turret_data[1], turret_data[2], turret_position, turret_position_lock)
        
        cv2.waitKey(10)

# Stops the robot.
def stop():
    forward(0, 0)

# Moves the robot forward.
def forward(left_motor_speed, right_motor_speed):
    BrickPi.MotorEnable[MOTOR1] = 1
    BrickPi.MotorEnable[MOTOR2] = 1
    BrickPi.MotorSpeed[MOTOR1] = left_motor_speed
    BrickPi.MotorSpeed[MOTOR2] = right_motor_speed
    BrickPiUpdateValues()
    time.sleep(SPEED_DELAY)

# Reverses the robot for delay_time seconds and 
# then stops.
def reverse():
    stop()
    motor_rotate_deg(TURN_SPEED_ENCODER, REVERSE, PORTS_TO_ROTATE)
    stop()
    
# Checks the distance in the front, left, and right of the robot
# and adjusts the speed depending on those distances.
def check_distances():
    front_distance = get_distance(FRONT)
    left_distance = get_distance(LEFT)
    right_distance = get_distance(RIGHT)
    
    # Check to see if we're about to hit something
    if front_distance < MAX_FRONT_DISTANCE:
        reverse()
        
        if left_distance > right_distance:
            motor_rotate_deg(TURN_SPEED_ENCODER, TURN_90_ENCODER, PORTS_TO_ROTATE)
            forward(SLOW, SLOW)
        else:
            motor_rotate_deg(TURN_SPEED_ENCODER, TURN_90_ENCODER_R, PORTS_TO_ROTATE)
            forward(SLOW, SLOW)
    else:
        if left_distance < MIN_SIDE_DISTANCE:
            forward(SLOW, FAST)
        elif right_distance < MIN_SIDE_DISTANCE:
            forward(FAST, SLOW)
        else:
            forward(SLOW, SLOW)
    time.sleep(SPEED_DELAY)
    
# Returns the distance reading from the ultrasonic sensor 
# corresponding to the input side. Rounds the distance to 
# 2 decimal places.
def get_distance(side):
    trig = 0
    echo = 0
    
    # Check which ultrasonic sensor we need to read
    if side == FRONT:
        trig = TRIG_F
        echo = ECHO_F
    elif side == LEFT:
        trig = TRIG_L
        echo = ECHO_L
    elif side == RIGHT:
        trig = TRIG_R
        echo = ECHO_R
        
    # If side was valid, return the distance reading
    if trig and echo:
        GPIO.output(trig, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(trig, GPIO.LOW)

        pulse_start = time.time()
        time_end = time.time()
        time_total = time_end - pulse_start
        while GPIO.input(echo) == GPIO.LOW and time_total < MAX_PULSE_TIME:
            pulse_start = time.time()
            time_total = pulse_start - time_end
            
        pulse_end = time.time()
        time_end = time.time()
        time_total = time_end - pulse_end
        while GPIO.input(echo) == GPIO.HIGH and time_total < MAX_PULSE_TIME:
            pulse_end = time.time()
            time_total = pulse_end - time_end

        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 17150
        distance = round(distance, 2)
        if distance > MAX_US_DISTANCE:
            distance = MAX_US_DISTANCE
        elif distance < 0:
            distance = 0
            
        return distance
    
    else:
        print 'Invalid input. Side must be LEFT, RIGHT, or FRONT)'
		
# Sets up the Brick Pi sensors, motors, and pins for additional
# peripheral devices.
def set_up():
    try:
        BrickPiSetup()
        BrickPi.MotorEnable[MOTOR1] = 1
        BrickPi.MotorEnable[MOTOR2] = 1
        BrickPi.SensorType[LIGHT_SENSOR_PORT] = TYPE_SENSOR_LIGHT_ON
        BrickPiSetupSensors()
        BrickPi.Timeout = BRICK_PI_TIMEOUT
        BrickPiSetTimeout()
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(TRIG_L, GPIO.OUT)
        GPIO.setup(ECHO_L, GPIO.IN)
        GPIO.setup(TRIG_R, GPIO.OUT)
        GPIO.setup(ECHO_R, GPIO.IN)
        GPIO.setup(TRIG_F, GPIO.OUT)
        GPIO.setup(ECHO_F, GPIO.IN)
        GPIO.setup(MOTOR_1_A, GPIO.OUT)
        GPIO.setup(MOTOR_2_A, GPIO.OUT)
        GPIO.setup(MOTOR_3_A, GPIO.OUT)
        GPIO.setup(MOTOR_4_A, GPIO.OUT)
        GPIO.setup(LIGHT_SENSOR_PIN, GPIO.OUT)
        GPIO.setup(FIRE_TURRET_MISSILE_PIN, GPIO.OUT)
        GPIO.setup(CANNON_FIRE_PIN, GPIO.OUT)
        GPIO.setup(MISSILE_FIRED_TURRET_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(RIGHT_LIMIT_TURRET_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(LEFT_LIMIT_TURRET_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(UPPER_LIMIT_TURRET_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(LOWER_LIMIT_TURRET_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.output(TRIG_L, GPIO.LOW)
        GPIO.output(TRIG_R, GPIO.LOW)
        GPIO.output(TRIG_F, GPIO.LOW)
        GPIO.output(MOTOR_1_A, GPIO.LOW)
        GPIO.output(MOTOR_2_A, GPIO.LOW)
        GPIO.output(MOTOR_3_A, GPIO.LOW)
        GPIO.output(MOTOR_4_A, GPIO.LOW)
        GPIO.output(LIGHT_SENSOR_PIN, GPIO.LOW)
        GPIO.output(FIRE_TURRET_MISSILE_PIN, GPIO.LOW)
        GPIO.output(CANNON_FIRE_PIN, GPIO.LOW)

    except:
        print 'Set-up failed.'

# Turns the turret to the left for TURRET_DELAY seconds.
# The turret will not turn if the left limit switch has been engaged.
def turn_turret_left():
    if not GPIO.input(LEFT_LIMIT_TURRET_PIN):
        GPIO.output(MOTOR_2_A, GPIO.HIGH)
	time.sleep(TURRET_DELAY)
        GPIO.output(MOTOR_2_A, GPIO.LOW)

# Turns the turret to the right for TURRET_DELAY seconds.
# The turret will not turn if the right limit switch has been engaged.
def turn_turret_right():
    if not GPIO.input(RIGHT_LIMIT_TURRET_PIN):
        GPIO.output(MOTOR_1_A, GPIO.HIGH)
	time.sleep(TURRET_DELAY)
        GPIO.output(MOTOR_1_A, GPIO.LOW)
	
# Turns the turret upward for TURRET_DELAY seconds.
# The turret will not turn if the upward limit switch has been engaged.
def turn_turret_up():
    if not GPIO.input(UPPER_LIMIT_TURRET_PIN):
        GPIO.output(MOTOR_4_A, GPIO.HIGH)
	time.sleep(TURRET_DELAY)
        GPIO.output(MOTOR_4_A, GPIO.LOW)

# Turns the turret downward for TURRET_DELAY seconds.
# The turret will not turn if the downward limit switch has been engaged.
def turn_turret_down():
    delay_flag = 0
    if not GPIO.input(LOWER_LIMIT_TURRET_PIN):
        GPIO.output(MOTOR_3_A, GPIO.HIGH)
	time.sleep(TURRET_DELAY)
	delay_flag = 1
        GPIO.output(MOTOR_3_A, GPIO.LOW)

# Fires a projectile from the turret. After the missile fired pin
# has been engaged, the turret will reposition the missile launcher
# motor so that the next projectile is ready to be fired by delaying
# for FIRE_MISSILE_DELAY seconds.
def fire_turret_missile():
##    while not GPIO.input(MISSILE_FIRED_TURRET_PIN):
##        GPIO.output(FIRE_TURRET_MISSILE_PIN, GPIO.HIGH)
##    print 'Missile fired from the turret!'
    GPIO.output(FIRE_TURRET_MISSILE_PIN, GPIO.HIGH)
    print('Missile fired from the turret!')
    time.sleep(FIRE_MISSILE_DELAY)
    GPIO.output(FIRE_TURRET_MISSILE_PIN, GPIO.LOW)

# Fires a projectile from the cannon.
def fire_cannon_missile():
    GPIO.output(CANNON_FIRE_PIN, GPIO.HIGH)
    time.sleep(FIRE_CANNON_MISSILE_DELAY)
    print('Missile fired from the cannon!')
    GPIO.output(CANNON_FIRE_PIN, GPIO.LOW)
	
# Process that will check the distances and light sensor readings.
def main_process(search_for_object, magnet_found):
    stop_flag = 0
    while True:
        if search_for_object.value == True or magnet_found.value == True:
            if not stop_flag:
                stop()
                print('Stopped.')
                stop_flag = 1
                check_light_reading()
        else:
	    stop_flag = 0
            check_light_reading()
            check_distances()
        time.sleep(PROCESS_DELAY)

# Method which centers the turret by turning all the way to the left until
# the limit switch has been reached and then proceeds to turn all the way to
# the left until the right limit switch has been reached. The method then takes the
# time required to sweep from the left side to the right side and divides by 2 to center
# the turret.
def center_turret():
    # go all the way to the left
    while not GPIO.input(LEFT_LIMIT_TURRET_PIN):
        GPIO.output(MOTOR_2_A, GPIO.HIGH)
    GPIO.output(MOTOR_2_A, GPIO.LOW)

    #start timer
    time_start = time.time()

    # go all the way to the right
    while not GPIO.input(RIGHT_LIMIT_TURRET_PIN):
        GPIO.output(MOTOR_1_A, GPIO.HIGH)
    GPIO.output(MOTOR_1_A, GPIO.LOW)
    time_end = time.time()
    total_time = time_end - time_start
    turret_delay_time = total_time / 2

    # center the turret in the horizontal direction
    GPIO.output(MOTOR_2_A, GPIO.HIGH)
    time.sleep(turret_delay_time)
    GPIO.output(MOTOR_2_A, GPIO.LOW)

    # go all the way up
    while not GPIO.input(UPPER_LIMIT_TURRET_PIN):
        GPIO.output(MOTOR_4_A, GPIO.HIGH)
    GPIO.output(MOTOR_4_A, GPIO.LOW)
    time_start = time.time()

    # go all the way down
    while not GPIO.input(LOWER_LIMIT_TURRET_PIN):
        GPIO.output(MOTOR_3_A, GPIO.HIGH)
    GPIO.output(MOTOR_3_A, GPIO.LOW)
    time_end = time.time()
    total_time = time_end - time_start
    turret_delay_time = total_time / 2

    # center the turret in the vertical direction
    GPIO.output(MOTOR_4_A, GPIO.HIGH)
    time.sleep(turret_delay_time)
    GPIO.output(MOTOR_4_A, GPIO.LOW)

def main():
    set_up()
    
    # Create shared variables
    object_found = Value('i', False)
    magnet_found = Value('i', False)
    left_flag = Value('i', False)
    right_flag = Value('i', False)
    up_flag = Value('i', False)
    down_flag = Value('i', False)
    turret_position = Value('i', 0)
    search_lock = multiprocessing.Lock()
    turret_lock = multiprocessing.Lock()
    turret_position_lock = multiprocessing.Lock()

    # Center turret
    with turret_lock:
        center_turret()
    
    # Create processes
    p1 = Process(target = main_process, args = (object_found, magnet_found))
    p2 = Process(target = check_magnet_reading, args = (object_found, magnet_found, turret_lock, turret_position, turret_position_lock,
                                                        left_flag, right_flag))
    p3 = Process(target = scan_camera, args = (object_found, left_flag, right_flag,
                                               up_flag, down_flag, search_lock, turret_lock, turret_position, turret_position_lock))
    p4 = Process(target= move_turret, args = (left_flag, right_flag, up_flag,
                                              down_flag, turret_lock, turret_position, turret_position_lock))

    # Start the processes
    p1.start()
    p2.start()
    p3.start()
    p4.start()

    # Join the processes
    p1.join()
    p2.join()
    p3.join()
    p4.join()
   
if __name__ == "__main__":
    try:
        main()
        
    except KeyboardInterrupt:
        GPIO.cleanup()
        camera.release()
        sys.exit()
      
