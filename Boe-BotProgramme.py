# Assignment 4.1_0 - Boe-Bot 4
# Sam Blaauw - v1.0 - 21-4-2022

# This program must be calibrated and tuned for your Boe-bot!

# load libraries
import pyb
import math
import micropython


# define global variables
global tick_counter, pulse_left, pulse_right, counter_left, counter_right, v, rated_speed

# define debug constant
USEINTERRUPT = True  # [ True || False ] To make debugging easier. Interrupts prevent proper error messages. 
#USEINTERRUPT = False  # [ True || False ] To make debugging easier. Interrupts prevent proper error messages. 

# define constants
wheel_diameter = 67.0 # mm, new Boe-Bot wheel
wheel_holes = 32 # mumber of holes in the wheel
max_speed = 110 # [mm/s]
min_speed = 70 # [mm/s]
rated_speed = 90 # [mm/s] (desired speed)
board_length = 500 # [mm]
turn_radius = 200 # [mm]
turn_distance = math.pi * turn_radius # [mm]
track_width = 107.5 # [mm]
speedofsound = (331.5+(0.6*20))/1000 # [mm/us]

ksi = [0.0,0.0,0.0] # Initialization of ksi, representing the state of the robot, i.e., [x, y, theta]
p = [wheel_diameter/2, wheel_diameter/2, track_width]    # Robot dimensions R1, R2, W [mm] (left wheel radius, right wheel radius, wheelbase)
u = [rated_speed / p[0], rated_speed / p[1]]  # Initial control input omega1, omega2 [rad/s] (left wheel speed, right wheel speed in rad/s)
theta = None # Initialization of theta (orientation angle of the robot)

front_dist =-1 # Initial value for front ultrasonic sensor distance (to be updated during the program)
back_dist = -1 # Initial value for back ultrasonic sensor distance (to be updated during the program)
sensor_offset = 12  # Difference in length between the sensors [mm]

tick_counter = 0 # Counter for the number of ticks (increments on each timer interrupt)
time_elapsed = 0  # Elapsed time since the program started [ms]
time_start = 0    # Time at the start of a state or action [ms]
time_stop = 0     # Time at the stop of a state or action [ms]
counter_start = 0  # Counter value at the start of a state or action

pulse_left = 1490  # Pulse duration for the left servo [us]
pulse_right = 1490  # Pulse duration for the right servo [us]

toggle_now = True  # Flag to keep track of when to toggle an LED
print_now = True   # Flag to keep track of when to print debug information

counter_left = 0  # Counter for the left wheel encoder (increments on each wheel rotation)
counter_right = 0  # Counter for the right wheel encoder (increments on each wheel rotation)

distancepercount = math.pi * wheel_diameter / (2 * wheel_holes) # Distance traveled per encoder count, calculated based on the wheel diameter and number of holes
distance_left = 0 # Distance traveled by the left wheel [mm]
distance_right = 0  # Distance traveled by the right wheel [mm]
speed = 0  # Current speed of the Boe-bot [mm/s]
desired_distance = 0  # Desired distance for specific actions or movements [mm]


# define user functions:
def Tick(): # This function is called by the interrupt timer every 20ms,
    global tick_counter   # it uses the global variable inside this function,
    global pulse_left
    global pulse_right
    tick_counter += 1      # adds one to the counter,
    RedLED.toggle()        # toggles the red LED,
    LeftServo.high()       # sets the output level high
    pyb.udelay(pulse_left) # waits for the desired pulse length [us]
    LeftServo.low()        # sets output level low and 
    RightServo.high()      # repeats the previous 3 steps for the other side.
    pyb.udelay(pulse_right)
    RightServo.low()
    return

def UpdateLeft(): # This function is called by the external interrupt from the encoder
    global counter_left
    counter_left += 1    # adds one to the counter and
    YellowLED.toggle()   # toggles the yellow LED
    return

def UpdateRight(): # This function is called by the external interrupt from the encoder
    global counter_right
    counter_right += 1   # adds one to the counter and
    GreenLED.toggle()    # toggles the green LED
    return

def StartIdle(): # This function is called when the IDLE state starts,
    GreenLED.on()         # it turns the green LED on,
    now = pyb.millis()    # puts the current time - number of milliseconds since boot - in a variable,
    state = "IDLE"
    if USEINTERRUPT:
        tim.callback(lambda t:Tick()) # start the timer interrupt by telling which function to call.
                          # (the "lambda t:" thing I don't understand, but without it doesn't work :)
    return state, now
    
def StopIdle(): # This function is called when the IDLE state has to stop,
    GreenLED.off()    # it turns off the green LED, (after USR switch has been pressed)
    pyb.delay(500)    # and leaves some time to move your hand away from BoeBot
    return

def StartDrivingStraight(): # This function is called when the DRIVING state starts,
    BlueLED.on()          # it turns on the blue LED,
    Speed2Pulse( rated_speed, rated_speed)  # calls the function that sets the speed,
    now = pyb.millis()    # puts current time - number of milliseconds since boot - in variable
    state = "DRIVING"
    return state, now

def StartTurn(): # This function is called when the TURNING state starts,
    BlueLED.off()          # it turns on the blue LED,
    left_speed, right_speed = CalculateTurn(rated_speed) 
    Speed2Pulse(left_speed, right_speed)        # calls the function that adjusts the speed,
    now = pyb.millis()    # puts current time - number of milliseconds since boot - in variable
    state = "TURNING"
    return state, now

def StopRunning(): # This function is called when the RUNNING state has to stop,
    BlueLED.off()          # it turns on the blue LED,
    return

def StartFinishing(): # This function is called when the FINISH state starts,
    now = pyb.millis()    # puts current time - number of milliseconds since boot - in variable
    state = "FINISHED"
    if USEINTERRUPT:
        tim.deinit()      # and closes the timer interrupt.
    return state, now

def FullForward(): # This function sets the pulse length to the maximum length
    global speed
    speed = max_speed
    Speed2Pulse(speed, speed)
    
def CalculateTurn(velocity): # This function calculates the speed of the outer and inner wheel
    v_outer = velocity / turn_radius * (turn_radius + track_width /2)
    v_inner = velocity / turn_radius * (turn_radius - track_width /2)
    #print('Boe-Bot >> Untuned turn!')
    # _ !!! _ These are untuned values _ !!! _
    return v_outer, v_inner


def Speed2Pulse(speed_left,speed_right): # This function calculates the pulse length acording to the speed
    global pulse_left, pulse_right # Global variables to store the calculated pulse lengths
    
    offset = 10 # Offset value to fine-tune the pulse length calculation

    # Calculate pulse length for the left servo using a linear equation based on speed_left
    pulse_left = int((speed_left - offset) * 0.411929 + 1490)
    
    # Calculate pulse length for the right servo using a linear equation based on speed_right
    pulse_right = int((speed_right - offset - 8) * -0.43585 + 1490)

    #print(" left pulse :", pulse_left)
    #print(" right pulse :",pulse_right)

    
def GetDistance(side): # function to measure the distance with an ultrasonic sensor
    
    us_distance = -1  # Initialize distance to -1; if something goes wrong, return -1
    measured_time = 0  # Initialize the variable to store the measured time
    
    t_out = 5  # Time duration for the start pulse (high signal) in microseconds [us]
    t_hold_off = 750  # Hold-off time before starting to measure in microseconds [us]
    t_burst = 200  # Duration of the ultrasonic burst signal in microseconds [us]
    t_in_min = 115  # Minimum expected duration of the ultrasonic signal in microseconds [us]
    t_in_max = 18500  # Maximum expected duration of the ultrasonic signal in microseconds [us]
    
    if side == 'front': # Check which sensor to use based on the specified side
        us_pin = 'X2' # Front sensor is connected to pin X2.
        
    elif side == 'back':
        us_pin = 'X1'
    else:
        return -1 # If the specified side is incorrect, stop the function and return an error
    
    # Configure the pin connected to the ultrasonic sensor
    Ultra = pyb.Pin(us_pin, pyb.Pin.OUT_PP,pyb.Pin.PULL_DOWN) 
    # Ultrasonic sensor to be an output pin in push-pull mode with a pull-down resistor. 
    # It sets up the pin to be able to send signals to the ultrasonic sensor.
    
    # Send a start pulse (5 microseconds high)
    pyb.udelay(2)
    Ultra.high() # This sets the pin to a high (1) state. This means the voltage on the pin is raised to the logic high level.
    pyb.udelay(t_out) # Delay introduced. t_out is the duration of the high pulse. During this time, the ultrasonic sensor receives a high voltage signal.
    Ultra.low() 
    # Configure the pin as an input to read the ultrasonic sensor response
    Ultra = pyb.Pin(us_pin, pyb.Pin.IN, pyb.Pin.PULL_DOWN)
    # pyb.Pin.IN = pin is ready to receive the signal 

    now = pyb.micros() # Record the current time in microseconds
    
    # Wait for the start of the ultrasonic burst signal
    while Ultra.value() == False:
        if pyb.elapsed_micros(now) > t_hold_off + 50: 
            # 50us is a margin or offset added to the hold-off time. It provides a small buffer to account for any variations or uncertainties in the system.
            break
        continue
    
    eerstenow = pyb.micros() # Record the current time in microseconds
    
    # Measure the duration of the ultrasonic signal
    while Ultra.value() == True or pyb.elapsed_micros(eerstenow) < t_in_min:
        measured_time = pyb.elapsed_micros(eerstenow)
        if t_in_max < pyb.elapsed_micros(eerstenow) + 1000: # i.e., if (pyb.micros() - eerstenow) > t_in_max
             # If the elapsed time exceeds the maximum expected duration, break the loop
            break
        continue
    
    us_distance = measured_time*speedofsound/2 # [mm] Calculate the distance based on the measured time

    return us_distance

def CalcTheta(distance_back,distance_front):
    sensor_dist = 135 #mm distance between sensors
    sensor_offset = 0
    """
    Calculate the orientation (theta) of the Boe-bot relative to the board.

    Parameters:
    - distance_front: Distance measured by the front sensor (in millimeters).
    - distance_back: Distance measured by the back sensor (in millimeters).

    Returns:
    - theta: Orientation of the Boe-bot relative to the board (in degrees).
    """
    # Calculate the sine of the angle based on the measured distances
    sin_hyp = (distance_front + sensor_offset - distance_back) / sensor_dist
    
    # Check if valid distance readings are available, ensuring that the sine value is within the valid range [-1, 1]
    if sin_hyp > 1:
        sin_hyp = 1
    elif sin_hyp < -1:
        sin_hyp = -1
    
    # Calculate the angle (theta) using arcsine, which represents the tilt of the Boe-bot
    tilt = math.asin(sin_hyp)

    # Limit the tilt angle to be within the range of -pi/6 to pi/6 to avoid extreme values    
    tilt = min(math.pi / 6, max(-math.pi / 6, tilt))
        
    return tilt
    
    
def control(u,p,ksi):
    global rated_speed, min_speed, max_speed
    
    zeta = 1 # damping factor [-], which affects the damping or attenuation of oscillations in the system.
    K_p = -0.003 # proportional gain of the controller [-]
    
    s = (u[0]+u[1])/2 # forward speed
    K_d = -zeta * math.sqrt((-4 * K_p * s)) # Compute the derivative gain of the error and the damping term
   
    d = K_p*ksi[1] + K_d * ksi[2] # Calculate the control input for steering (i.e., control input adjustment based on proportional and derivative terms)
    
    #print('d:',d,'distance adjustment: ', K_p*ksi[1] , "angle adjustment: ", K_d * ksi[2])
    
    Vl = rated_speed - 0.5 * p[2] * d # Adjusted left wheel speed
    Vr = rated_speed + 0.5 * p[2] * d # Adjusted right wheel speed
    
    # Ensure the adjusted speeds are within the allowable range
    u = [min(max_speed, max(min_speed,Vl)),
         min(max_speed, max(min_speed,Vr))]
    
    return u
 

# assign sensors
SwitchPressed = pyb.Switch()    # Enable USR switch
LeftEncoder = pyb.ExtInt(pyb.Pin('Y1'), pyb.ExtInt.IRQ_RISING_FALLING, pyb.Pin.PULL_DOWN, lambda l: UpdateLeft())
    # assigns the pin of the left encoder to the external interrupt which calls the UpdateLeft function. The interrupt will be triggered by every rising and falling of the signal.
RightEncoder = pyb.ExtInt(pyb.Pin('Y2'), pyb.ExtInt.IRQ_RISING_FALLING, pyb.Pin.PULL_DOWN, lambda r: UpdateRight())
    # and for right.

# initialize sensors

# assign actuators
RedLED = pyb.LED(1)
GreenLED = pyb.LED(2)
YellowLED = pyb.LED(3)
BlueLED = pyb.LED(4)
LeftServo = pyb.Pin('X3', pyb.Pin.OUT_PP, pyb.Pin.PULL_DOWN)
RightServo = pyb.Pin('X4', pyb.Pin.OUT_PP, pyb.Pin.PULL_DOWN)

# initialize the timer interrupt
if USEINTERRUPT:
    micropython.alloc_emergency_exception_buf(100)  # Should help error handling using interrupts
    tim = pyb.Timer(4)    # Create a timer
    tim.init(freq=50)   # Set the frequency of the timer to 50Hz (20 ms)
    print ('Boe-Bot_04 >> Timer interrupt initialized.')

# start first state
state, time_start = StartIdle()
# Initialize the state machine by transitioning to the initial state 'IDLE'.
# Also, record the current time as the starting time.

# main loop

print("version George rip")
# Initialize variables for tracking the position of the Boe-bot on the board. These represent the initial position of the Boe-bot.
distance_y = 250
distance_x = 0

waitforprint = 0 # Initialize a variable to keep track of when to print additional debug information.
message = ""

# Initialize variables to store the previous distance readings from the front and back sensors (used to check for changes in sensor readings)
prev_front_dist = 0
prev_back_dist = 0

# Initialize lists to store recent sensor readings for smoothing and stability. The lists are used to keep a history of the last few readings for front and back sensors.
front_list = [250, 250, 250, 250, 250, 250]  # Increase the number of readings for front sensor
back_list = [250, 250, 250, 250, 250, 250]  # Increase the number of readings for back sensor
threshold = 10  # Dynamic threshold for outlier detection

while state != "FINISHED":
    # read sensors
    time_elapsed = pyb.elapsed_millis(time_start) # calculates elapsed time since start [ms]

    # Ultrasonic measurements: 4 times per second, 2 front and 2 times back.
    if time_elapsed % 250 == 25:
        # Execute every 250 milliseconds with a 25 ms offset.
        # Update the back sensor readings and smooth them using a list of recent readings.
        
        # Obtain the current distance reading from the back sensor.
        # The values is readjusted by adding 12 units to account for a specific sensor offset.
        back_dist = GetDistance('back') + 12 
        waitforprint += 1
        
    elif time_elapsed % 250 == 175:
        # Obtain the current distance reading from the front sensor.
        front_dist = GetDistance('front')
        
        if front_dist == -1 and back_dist == -1:
            continue # Skip the rest of the loop if there is an issue with sensor readings.
    else:
        # Calculate average distance and orientation based on sensor readings.
        # print("distances:",[front_dist, back_dist])
        distance_y = (front_dist + back_dist) / 2    # Calculate the average distance from front and back sensors.
        theta = CalcTheta(back_dist, front_dist)     # Calculate the orientation based on front and back sensor distances.
        ksi = [distance_x, distance_y - 250, theta]  # Update the state variables: x, y, and orientation.
        
    message = ""
       
    distance_left = counter_left * distancepercount    # Calculate the distance traveled by the left wheel.
    distance_right = counter_right * distancepercount  # Calculate the distance traveled by the right wheel.
    
    if state == "IDLE":
        # message = "Front distance: "+ str(front_dist) + " Back distance: " + str(back_dist)
        if SwitchPressed() == True: # The USR switch has been pressed,
            # Check if the absolute difference between front and back ultrasonic sensor readings is small.
            if abs(front_dist - back_dist) < 15:
                # Check if a valid orientation angle (theta) has been calculated.
                if theta is not None:
                    # Transition to the "DRIVING" state if conditions are met.
                   
                    message = "pass IDLE"
                    counter_start = tick_counter # store the number of counted ticks at the start.
                    counter_left = 0 # reset the counters for the wheels
                    counter_right = 0
                    
                    # Store the current front and back sensor readings for reference in the next iteration.
                    # These values will be used for change detection and stability analysis.
                    prev_front_dist = front_dist
                    prev_back_dist = back_dist
                    
                    StopIdle() # stop the previous state,
                    state, time_start = StartDrivingStraight() # start the new state      
                else:
                    message = "failed - theta problem"              
            else:
                message = "failed - sensor problem"

    elif state == "DRIVING":
        message = "Front distance: "+ str(front_dist) + " Back distance: " + str(back_dist)
        
        if SwitchPressed() == True:
            print("switch_pressed")
            StopRunning() 
            state, time_stop = StartFinishing() # If the switch is pressed, stop running and transition to the finishing state.
            
        u = control(u,p,ksi) # Calculate control input using the control function.
    
        # Uncomment the following line if you want to display detailed information in the 'message' variable.
        # message = "distance_y: "+str(distance_y)+" front_dist: "+str(front_dist)+" back_dist: "+str(back_dist)+" speed_left: "+str( u[1]*p[1])+ 'speed_right: '+str(u[0]*p[0])+' theta: ' +str(theta) 
        Speed2Pulse(u[0], u[1]) # Calculate and set pulse lengths based on the calculated control input.
            
        if (front_dist-back_dist) > 500 and time_elapsed > 2000:
             # If the difference between front and back distances is significant and enough time has passed, initiate a turn.
            desired_distance = (distance_left + distance_right) / 2 + turn_distance 
            state, time_start = StartTurn()
            message = "Start Turning{},{}".format(distance_y,time_elapsed)
            
        if time_elapsed > 1500:
            # Reset distance variables after 1.5 seconds.
            distance_left = 0
            distance_right = 0
            # distance_x = 0
            
    elif state == "TURNING":
        if SwitchPressed() == True:
            StopRunning()
            state, time_stop = StartFinishing() # If the switch is pressed, stop running and transition to the finishing state.
            
        # Check conditions for completing the turn or adjusting the behavior during the turn.
        if (distance_left + distance_right) / 2 >= desired_distance or front_dist <= 350:
            if back_dist > 600:
                # Keep moving straight if the front distance is low but back distance is still high.
                Speed2Pulse(rated_speed, rated_speed) # Keep moving straight if the front distance is low but back distance is still high.
        
            else:
                # Transition to driving straight state.
                state, time_start = StartDrivingStraight() # Transition to driving straight state.
            #print("Driving Straight")
        
    # compute behavior (call user functions)
        
    # actuate console, LEDs and motors
    if time_elapsed % 500 == 0  and print_now: # every 0.5 s.
        print ('#:', tick_counter, 'state:', state, 'Time elapsed:',
               time_elapsed,"theta:", "ksi: ",str(ksi), "u: ",str(u),"distances: " , str([front_dist,back_dist]), "pulses: ", str([pulse_left,pulse_right]) )
        if theta is not None:
            continue#print("theta: ", str(theta *(180/math.pi)))
        print_now = False
    elif time_elapsed % 500 != 0 and not print_now: # every 0.5 s.
        print_now = True

# Finish program
print ('#:', tick_counter, 'state:', state,  'Time elapsed:', time_elapsed, '[ms] Distance desired&driven:', desired_distance, (distance_left + distance_right) / 2, '[mm]' )
print ('Time stop:', time_stop, 'Time elapsed:', time_elapsed)
if USEINTERRUPT:
    print ('Counts:', (tick_counter - counter_start))

