from pymata4 import pymata4
import time
import math
import random

board = pymata4.Pymata4() #board initialisation

#---------------------Full-pin-layout---------------------
trigPinUs1 = 2
echoPinUs1 = 3

trigPinUs2 = 4
echoPinUs2 = 5

trigPinUs3 = 6
echoPinUs3 = 7

trigPinUs4 = 8
echoPinUs4 = 9

trigPinUs5 = 10
echoPinUs5 = 11

pb1 = 12
pb2 = 13

clockPin = 14 #A0
latchPin = 15
dataPin = 16

ds1 = 17
ds2 = 18

pa1 = 19

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
    "start_time": 0.0,
    "durations": {1: 1.0, 2: 10.0}  # stage 1: 1s, stage 2: 10s
}

tl2Controller = {
    "state": 0,
    "start_time": 0.0,
    "durations": {1: 1.0, 2: 10.0}  # stage 1: 1s, stage 2: 10s
}

#---------------------Registering pins----------------------------
board.set_pin_mode_digital_output(clockPin)
board.set_pin_mode_digital_output(latchPin)
board.set_pin_mode_digital_output(dataPin)

board.set_pin_mode_sonar(trigPinUs1, echoPinUs1, timeout=200000) #timeout configures listening time, longer = more distance (ms)
board.set_pin_mode_sonar(trigPinUs2, echoPinUs2, timeout=200000)
board.set_pin_mode_sonar(trigPinUs3, echoPinUs3, timeout=200000)
board.set_pin_mode_sonar(trigPinUs4, echoPinUs4, timeout=200000)
# board.set_pin_mode_sonar(trigPinUs5, echoPinUs5, timeout=200000) #CURRENTLY UNAVAILABLE

time.sleep(0.5) #config time

#---------------------shift register bits---------------------
allOff = 0

diodeStateDict = {
    "diodes": 0b00011111111111111111111111100100
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
    board.digital_write(latchPin, 0) #readies shift register to listen (initial state low for all outputs)
    for i in range(31,-1,-1):
        board.digital_write(dataPin, (value >> i) & 1)  #shifts value and ensures only 0s and 1s are pushed through
        board.digital_write(clockPin, 1)  
        board.digital_write(clockPin, 0)  #these two lines complete one clock cycle (pushes one bit)

    board.digital_write(latchPin, 1) #executes the memory and lights the LEDs

def tl_r_off_y_off_g_on(diodeState, ledNumberRed, ledNumberYellow, ledNumberGreen):
    diodeState = update_bit(diodeState, ledNumberRed, OFF)
    diodeState = update_bit(diodeState, ledNumberYellow, OFF)
    diodeState = update_bit(diodeState, ledNumberGreen, ON)
    diodeStateDict["diodes"] = diodeState
    write_to_shift_register(diodeStateDict["diodes"])

def tl_r_off_y_on_g_off(diodeState, ledNumberRed, ledNumberYellow, ledNumberGreen):
    diodeState = update_bit(diodeState, ledNumberRed, OFF)
    diodeState = update_bit(diodeState, ledNumberYellow, ON)
    diodeState = update_bit(diodeState, ledNumberGreen, OFF)
    diodeStateDict["diodes"] = diodeState
    write_to_shift_register(diodeStateDict["diodes"])


def tl_r_on_y_off_g_off(diodeState, ledNumberRed, ledNumberYellow, ledNumberGreen):
    diodeState = update_bit(diodeState, ledNumberRed, ON)
    diodeState = update_bit(diodeState, ledNumberYellow, OFF)
    diodeState = update_bit(diodeState, ledNumberGreen, OFF)
    diodeStateDict["diodes"] = diodeState
    write_to_shift_register(diodeStateDict["diodes"])

def update_traffic(sequence, action_30s, action_default):

    currentTime = time.time()
    elapsed = currentTime - sequence["start_time"]

    if sequence["state"] == 1:
        if elapsed >= sequence["durations"][1]:
            sequence["state"] = 2
            sequence["start_time"] = currentTime
            action_30s()

    elif sequence["state"] == 2:
        if elapsed >= sequence["durations"][2]:
            sequence["state"] = 0
            action_default()

def start_sequence(sequence, action_1s):
    if sequence["state"] == 0:  #prevents multiple timers from starting
        sequence["state"] = 1
        sequence["start_time"] = time.time()
        action_1s()

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
    us1Value = board.sonar_read(trigPinUs1)
    us2Value = board.sonar_read(trigPinUs2)

    if check_overheight(us1Value, limit, GROUND) == True:
        print_alert(us1Value, GROUND)
        start_sequence(tl1Controller, lambda: tl_r_off_y_on_g_off(diodeStateDict["diodes"], TL1R, TL1Y, TL1G))
    update_traffic(tl1Controller, lambda: tl_r_on_y_off_g_off(diodeStateDict["diodes"], TL1R, TL1Y, TL1G), lambda: tl_r_off_y_off_g_on(diodeStateDict["diodes"], TL1R, TL1Y, TL1G))

    if check_overheight(us2Value, limit, GROUND) == True:
        start_sequence(tl2Controller, lambda: tl_r_off_y_on_g_off(diodeStateDict["diodes"], TL2R, TL2Y, TL2G))
    update_traffic(tl2Controller, lambda: tl_r_on_y_off_g_off(diodeStateDict["diodes"], TL2R, TL2Y, TL2G), lambda: tl_r_off_y_off_g_on(diodeStateDict["diodes"], TL2R, TL2Y, TL2G))



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

        time.sleep(0.2)
except KeyboardInterrupt:
    print("Quitting the program")
    write_to_shift_register(0)
    time.sleep(0.5)
    board.shutdown()
    time.sleep(0.5)
    exit()