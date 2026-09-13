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
testDiodes = 0b00011111111111111111111111111111 #0b00011111111111111111111111111111

T1R   = 0
T1Y   = 1
T1G   = 2
T2R   = 3
T2Y   = 4
T2G   = 5
W1Yl  = 6
W1Yr  = 7
P1G   = 8
P1R   = 9
P2G   = 10
P2R   = 11
T4R   = 12
T4Y   = 13
T4G   = 14
T5R   = 15
T5Y   = 16
T5G   = 17
F1  = 18
F2  = 19
T6R   = 20
T6Y   = 21
T6G   = 22
T3G   = 23
T3R   = 24
W2R1  = 25  # left
W2R2  = 26  # middle left
W2R3  = 27  # middle right
W2R4  = 28  # right


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

try:
    while True: 
        testDiodes = update_bit(testDiodes, T2R, 0)
        write_to_shift_register(testDiodes)
        time.sleep(0.2)

except KeyboardInterrupt:
    print("Quitting the program")
    write_to_shift_register(0)
    time.sleep(0.5)
    board.shutdown()
    time.sleep(0.5)
    exit()