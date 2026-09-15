from pymata4 import pymata4
import time
import math
import random

board = pymata4.Pymata4() #board initialisation

#---------------------Full-pin-layout---------------------
TRIG_PIN_US_1 = 2
ECHO_PIN_US_1 = 3

TRIG_PIN_US_2 = 4
ECHO_PIN_US_2 = 5

TRIG_PIN_US_3 = 6
ECHO_PIN_US_3 = 7

TRIG_PIN_US_4 = 8
ECHO_PIN_US_4 = 9

TRIG_PIN_US_5 = 10
ECHO_PIN_US_5 = 11

PB_PIN_1 = 12
PB_PIN_2 = 13

CLOCK_PIN = 14 #A0
LATCH_PIN = 15
DATA_PIN = 16

DS_PIN_1 = 3    #A3
DS_PIN_2 = 4    #A4

PA_PIN_1 = 19

#------------------------Initialising variables------------------------

# us1Value = 0
# us2Value = 0
# us3Value = 0
# us4Value = 0
# us5Value = 0

tl1TimeTracker = 0
tl2TimeTracker = 0

GROUND = 10 #ground is 10cm away from the supersonic sensors

#------------------------Timers------------------------

tl1Controller = {
    "state": 0,
    "startTime": 0.0,
    "durations": {1: 1.0, 2: 10.0}  # stage 1: 1s, stage 2: 10s
}

tl2Controller = {
    "state": 0,
    "startTime": 0.0,
    "durations": {1: 1.0, 2: 10.0}  # stage 1: 1s, stage 2: 10s
}

tl4tl5CycleController = {
    "state": 0, 
    "startTime": 0.0,
    "durations": {1: 20.0, 2: 3.0, 3: 10.0, 4: 3.0}  # stage 1: 3s, stage 2: 13s
}

buttonController = {
    "state": 0,
    "startTime": 0.0,
    "durations": {0: 0, 1: 5.0}
}

#---------------------Registering pins----------------------------
board.set_pin_mode_digital_output(CLOCK_PIN)    #Shift register
board.set_pin_mode_digital_output(LATCH_PIN)
board.set_pin_mode_digital_output(DATA_PIN)

board.set_pin_mode_sonar(TRIG_PIN_US_1, ECHO_PIN_US_1, timeout=200000)  #Ultrasonic sensors
board.set_pin_mode_sonar(TRIG_PIN_US_2, ECHO_PIN_US_2, timeout=200000)  #timeout configures listening time, longer = more distance (ms)
board.set_pin_mode_sonar(TRIG_PIN_US_3, ECHO_PIN_US_3, timeout=200000)
board.set_pin_mode_sonar(TRIG_PIN_US_4, ECHO_PIN_US_4, timeout=200000)
# board.set_pin_mode_sonar(trigPinUs5, echoPinUs5, timeout=200000) #CURRENTLY UNAVAILABLE

board.set_pin_mode_digital_input_pullup(PB_PIN_1)   #Buttons
board.set_pin_mode_digital_input_pullup(PB_PIN_2)

board.set_pin_mode_analog_input(DS_PIN_1)   #Daylight sensors
board.set_pin_mode_analog_input(DS_PIN_2)

time.sleep(0.5) #config time

#---------------------shift register bits---------------------
allOff = 0

diodeStateDict = {
    "diodes": 0b00011111111111001001101011100100
}

TL1R   = 0
TL1Y   = 1
TL1G   = 2
TL2R   = 3
TL2Y   = 4
TL2G   = 5
WL1YL  = 6
WL1YL  = 7
PL1G   = 8
PL1R   = 9
PL2G   = 10
PL2R   = 11
TL4R   = 12
TL4Y   = 13
TL4G   = 14
TL5R   = 15
TL5Y   = 16
TL5G   = 17
FL1  = 18
FL2  = 19
TL6R   = 20
TL6Y   = 21
TL6G   = 22
TL3G   = 23
TL3R   = 24
WL2R1  = 25  # left
WL2R2  = 26  # middle left
WL2R3  = 27  # middle right
WL2R4  = 28  # right

ON = 1
OFF = 0

#-------------------------------------------------

def update_bit(diodeState, ledNumber, state):
    '''
        Changes specified value in binary number 
        Parameters:
            diodeState (int): sequence (32-bit) for shift register
            ledNumber (int): corresponding led position in the diodeState sequence
            state (boolean): on or off
        Returns:
            diodeState (int): sequence (32-bit) for shift register
    '''
    if state == 1:
        return diodeState | (1 << ledNumber)    #RHS creates temp 32-bit number; compares RHS bit with LHS bit using OR operator
    else:
        return diodeState & ~(1 << ledNumber)   #RHS creates temp 32-bit number; compares RHS bit with LHS bit using NAND operator


def write_to_shift_register(value):
    '''
    Used to write to shift register activating pins by given value 
        Parameters:
            value (int): sequence (8-bit) for shift register 
        Returns:
            Does not return anything
    '''
    board.digital_write(LATCH_PIN, 0) #readies shift register to listen (initial state low for all outputs)
    for i in range(31,-1,-1):
        board.digital_write(DATA_PIN, (value >> i) & 1)  #shifts value and ensures only 0s and 1s are pushed through
        board.digital_write(CLOCK_PIN, 1)  
        board.digital_write(CLOCK_PIN, 0)  #these two lines complete one clock cycle (pushes one bit)

    board.digital_write(LATCH_PIN, 1) #executes the memory and lights the LEDs

def tl_r_off_y_off_g_on(ledNumberRed, ledNumberYellow, ledNumberGreen):
    current = diodeStateDict["diodes"]
    current = update_bit(current, ledNumberRed, OFF)
    current = update_bit(current, ledNumberYellow, OFF)
    current = update_bit(current, ledNumberGreen, ON)
    diodeStateDict["diodes"] = current
    write_to_shift_register(current)

def tl_r_off_y_on_g_off(ledNumberRed, ledNumberYellow, ledNumberGreen):
    current = diodeStateDict["diodes"]
    current = update_bit(current, ledNumberRed, OFF)
    current = update_bit(current, ledNumberYellow, ON)
    current = update_bit(current, ledNumberGreen, OFF)
    diodeStateDict["diodes"] = current
    write_to_shift_register(current)


def tl_r_on_y_off_g_off(ledNumberRed, ledNumberYellow, ledNumberGreen):
    current = diodeStateDict["diodes"]
    current = update_bit(current, ledNumberRed, ON)
    current = update_bit(current, ledNumberYellow, OFF)
    current = update_bit(current, ledNumberGreen, OFF)
    diodeStateDict["diodes"] = current
    write_to_shift_register(current)

def tl_r_off_g_on(ledNumberRed, ledNumberGreen):
    current = diodeStateDict["diodes"]
    current = update_bit(current, ledNumberRed, OFF)
    current = update_bit(current, ledNumberGreen, ON)
    diodeStateDict["diodes"] = current
    write_to_shift_register(current)

def tl_r_on_g_oFF(ledNumberRed, ledNumberGreen):
    current = diodeStateDict["diodes"]
    current = update_bit(current, ledNumberRed, ON)
    current = update_bit(current, ledNumberGreen, OFF)
    diodeStateDict["diodes"] = current
    write_to_shift_register(current)

def update_traffic(sequence, action_30s, action_default):

    currentTime = time.time()
    elapsed = currentTime - sequence["startTime"]

    if sequence["state"] == 1:
        if elapsed >= sequence["durations"][1]:
            sequence["state"] = 2
            sequence["startTime"] = currentTime
            action_30s()

    elif sequence["state"] == 2:
        if elapsed >= sequence["durations"][2]:
            sequence["state"] = 0
            action_default()

def start_sequence(sequence, action):
    if sequence["state"] == 0:  #prevents multiple timers from starting
        sequence["state"] = 1
        sequence["startTime"] = time.time()
        action()

def check_overheight(heightTime, limit, ground):
    '''
        Used to check if the height is overheight and return a boolean
            Parameters:
                heightTime (float): Values read by the supersonic sensor 
                limit(integer/float): Used to compare with distanceCm
                ground(intger/float): Distance between supersonic sensor and ground
            Returns:
                Returns a boolean dependent on if overheight was detected
    '''
    distanceCm = heightTime[0]
    if((ground-distanceCm) > limit):
        return True
    
    return False

def print_alert(heightTime,ground):
    '''
    Used to check if the height is overheight and print an alert if it is
        Parameters:
            heightTime (float): Values read by the supersonic sensor 
            ground(intger/float): Distance between supersonic sensor and ground
        Returns:
            Does not return anything
    '''
    distanceCm = ground-heightTime[0]
    timeStamp = time.localtime(heightTime[1])
    formattedDate = time.strftime("%d/%m/%Y %H:%M:%S", timeStamp)
    print(f"Overheight was detected! vehicle height: {distanceCm}m at time: {formattedDate}")

def subsystem_1():
    us1Value = board.sonar_read(TRIG_PIN_US_1)
    us2Value = board.sonar_read(TRIG_PIN_US_2)

    if check_overheight(us1Value, limit, GROUND) == True:
        if tl1Controller["state"] == 0:
            print_alert(us1Value, GROUND)
        start_sequence(tl1Controller, lambda: tl_r_off_y_on_g_off(TL1R, TL1Y, TL1G))

    update_traffic(tl1Controller, lambda: tl_r_on_y_off_g_off(TL1R, TL1Y, TL1G), lambda: tl_r_off_y_off_g_on(TL1R, TL1Y, TL1G))

    if check_overheight(us2Value, limit, GROUND) == True:
        start_sequence(tl2Controller, lambda: tl_r_off_y_on_g_off(TL2R, TL2Y, TL2G))
    update_traffic(tl2Controller, lambda: tl_r_on_y_off_g_off(TL2R, TL2Y, TL2G), lambda: tl_r_off_y_off_g_on(TL2R, TL2Y, TL2G))

    if tl1Controller["state"] == 0 and tl2Controller["state"] == 1:
        start_sequence(tl1Controller, lambda: tl_r_off_y_on_g_off(TL1R, TL1Y, TL1G))
        update_traffic(tl1Controller, lambda: tl_r_on_y_off_g_off(TL1R, TL1Y, TL1G), lambda: tl_r_off_y_off_g_on(TL1R, TL1Y, TL1G))
        start_sequence(tl2Controller, lambda: tl_r_off_y_on_g_off(TL2R, TL2Y, TL2G))
        update_traffic(tl2Controller, lambda: tl_r_on_y_off_g_off(TL2R, TL2Y, TL2G), lambda: tl_r_off_y_off_g_on(TL2R, TL2Y, TL2G))

def subsystem_2():
   currentTime = time.time()

   buttonDataOne, timeStamp = board.digital_read(PB_PIN_1)  #default 1 (up)
   buttonDataTwo, timeStamp  = board.digital_read(PB_PIN_2) #default 1 (up)
   ldr_data = board.analog_read(DS_PIN_2)

   elapsedButton = currentTime-buttonController["startTime"] 
   stateButton = buttonController["state"]
   if elapsedButton >= buttonController["durations"][stateButton]:
       if buttonController["state"] == 1:
           print("fok")
           buttonController["state"] = 0

       if (buttonDataOne == 0 or buttonDataTwo == 0) and buttonController["state"] == 0:
            buttonController["state"] = 1
            buttonController["startTime"] = currentTime
            print("The pedestrian button has been pressed")

   if buttonController["state"] == 1:
       if tl4tl5CycleController["state"] == 1 or tl4tl5CycleController["state"] == 2:
           #Force TL4 to go yellow for 3s
           print("a")
       elif
            print("a")
        
#------------------------Cycling sequence------------------------

   if tl4tl5CycleController["state"] == 0:
       tl4tl5CycleController["state"] = 1
       tl4tl5CycleController["startTime"] = currentTime
       tl_r_off_y_off_g_on(TL4R, TL4Y, TL4G)
       tl_r_on_y_off_g_off(TL5R, TL5Y, TL5G)
       return
    
   state = tl4tl5CycleController["state"]
   elapsed = currentTime-tl4tl5CycleController["startTime"]

   if elapsed >= tl4tl5CycleController["durations"][state]:
       nextState = (state+1)%4 # returns to 0
       tl4tl5CycleController["state"] = nextState
       tl4tl5CycleController["startTime"] = currentTime

       if nextState == 1:   #TL4 Green, TL5 Red (20s)
            tl_r_off_y_off_g_on(TL4R, TL4Y, TL4G)
            tl_r_on_y_off_g_off(TL5R, TL5Y, TL5G)
       elif nextState == 2: #TL4 Yellow, TL5 Red (3s)
            tl_r_off_y_on_g_off(TL4R, TL4Y, TL4G)
            tl_r_on_y_off_g_off(TL5R, TL5Y, TL5G)

       elif nextState == 3: #TL4 Red, TL5 Green (10s)
            tl_r_on_y_off_g_off(TL4R, TL4Y, TL4G)
            tl_r_off_y_off_g_on(TL5R, TL5Y, TL5G)

       elif nextState == 4: #TL4 Red, TL5 Yellow (3s)
            tl_r_on_y_off_g_off(TL4R, TL4Y, TL4G)
            tl_r_off_y_on_g_off(TL5R, TL5Y, TL5G)


#----------------------User Input---------------------

limit = 4.0 #Default value

limit = input("Input a height limit or press enter to set default: ")

while True:
    limit = limit.strip()
    if(limit == ""):
        limit = 4.0
        break
    try:
        limit = float(limit)
        if (limit<=0):
            limit = input("Enter a valid input or press enter to set default value 1 ")
            continue
        break
    except ValueError:
        limit = input("Enter a valid input or press enter to set default value: ")


print(f"The limit was set to {limit}m")

#------------------------------------------------------------------------

write_to_shift_register(diodeStateDict["diodes"])

try:
    while True:

        subsystem_1()
        subsystem_2()

        time.sleep(0.2)
except KeyboardInterrupt:
    print("Quitting the program")
    write_to_shift_register(0)
    time.sleep(0.5)
    board.shutdown()
    time.sleep(0.5)
    exit()