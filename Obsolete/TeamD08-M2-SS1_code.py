from pymata4 import pymata4
import time
import math
import random
board = pymata4.Pymata4() #board initialisation

#------------------------------

pollingRate = 0.1 #Seconds

lastPollTime = time.time() 

tl1State = "green" #used to track tl current states
tl2State = "green"

groundDistance = 15 #15cm = 15m

timerOneSecondTl1 = {
    "timeStart": time.time(),
    "duration":  1,
    "triggered":  False
}

timerThirtySecondTl1 = {
    "timeStart":time.time(),
    "duration": 10,
    "triggered": False
}

timerOneSecondTl2 = {
    "timeStart": time.time(),
    "duration":  1,
    "triggered":  False
}

timerThirtySecondTl2 = {
    "timeStart":time.time(),
    "duration": 10,
    "triggered": False
}

#-----traffic-light_states------
#in binary so it can be used by shift register

redGreen  = 0b01000010  
redYellow = 0b00100010  
redRed    = 0b00010010  

yellowGreen = 0b01000100
yellowYellow = 0b00100100
yellowRed = 0b00010100

greenGreen = 0b01001000
greenYellow = 0b00101000   
greenRed  = 0b00011000

# Troubleshooting
allOn     = 0b01111110
allOff    = 0b00000000

#----------pin-register---------

trigPin_1 = 2
echoPin_1 = 3   #SN1
trigPin_2 = 4
echoPin_2 = 5   #SN2

clockPin = 14  # Pins for shift registers
latchPin = 15  #A1, analogue pins are used as full integrations requires degital pins 2-11 for supersonic sensors
dataPin = 16   #A2

board.set_pin_mode_sonar(trigPin_1, echoPin_1, timeout=200000) #timeout configures listening time, longer = more distance (ms)
board.set_pin_mode_sonar(trigPin_2, echoPin_2, timeout=200000)

board.set_pin_mode_digital_output(clockPin)
board.set_pin_mode_digital_output(latchPin)
board.set_pin_mode_digital_output(dataPin)

time.sleep(0.5) #give time for configuration

#--------functions--------------
 

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

def write_to_shift_register(value):
    '''
    Used to write to shift register activating pins by given value 
        Parameters:
            value (int): sequence (8-bit)to be set for shift register 
        Returns:
            Does not return anything
    '''
    board.digital_write(latchPin, 0) #readies shift register to listen (initial state low for all outputs)
    for i in range(7,-1,-1):
        board.digital_write(dataPin, (value >> i) & 1)  #shifts value and ensures only 0s and 1s are pushed through
        board.digital_write(clockPin, 1)  
        board.digital_write(clockPin, 0)  #these two lines complete one clock cycle (pushes one bit)

    board.digital_write(latchPin, 1) #executes the memory and lights the LEDs

def check_timer_elapsed(timerType, currentTime):
    '''
    Used to check if time elapsed 
        Parameters:
            timerType (dict): dictionary containing timer information
            currentTime (int): Current time in seconds
        Returns:
            Returns a boolean dependent on time elasped
    '''
    if(timerType["triggered"]==True):
        elapsed = currentTime-timerType["timeStart"] 
        if (elapsed > timerType["duration"]):
            return True
        # print(elapsed)  #debugging
        # print(timerType["timeStart"])
    return False

def map_traffic_light_by_states(tl1State, tl2State):
    '''
    Used map LED colours by the current state of the traffic light 
        Parameters:
            tl1State (string): The current state of trafficlight 1
            tl2state (string): The current state of trafficlight 2
        Returns:
            Does not return anything
    '''
    if(tl1State == "green" and tl2State == "green"):
        write_to_shift_register(greenGreen)
    elif(tl1State == "yellow" and tl2State == "green"):
        write_to_shift_register(yellowGreen)
    elif(tl1State == "red" and tl2State == "green"):
        write_to_shift_register(redGreen)
    elif(tl1State == "green" and tl2State == "yellow"):
        write_to_shift_register(greenYellow)
    elif(tl1State == "yellow" and tl2State == "yellow"):
        write_to_shift_register(yellowYellow)
    elif(tl1State == "red" and tl2State == "red"):
        write_to_shift_register(redRed)
    elif(tl1State == "red" and tl2State == "yellow"):
        write_to_shift_register(redYellow)
    elif(tl1State == "yellow" and tl2State == "red"):
        write_to_shift_register(yellowRed)
    elif(tl1State == "green" and tl2State == "red"):
        write_to_shift_register(greenRed)

#--------ask-user-for-limit--------------

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

#--------ultrasonic sensor polling-------

write_to_shift_register(greenGreen) #inital state of tl1 and tl2

try:
    while True: 
        currentTime = time.time()

        timerElaspedOneTl1 = check_timer_elapsed(timerOneSecondTl1, currentTime)
        timerElaspedThirtyTl1 = check_timer_elapsed(timerThirtySecondTl1, currentTime)

        timerElaspedOneTl2 = check_timer_elapsed(timerOneSecondTl2, currentTime)
        timerElaspedThirtyTl2 = check_timer_elapsed(timerThirtySecondTl2, currentTime)

        if(timerElaspedOneTl1 == True):    #Checks if 1s timer has past, sets 30s timer to start
            timerOneSecondTl1["triggered"] = False
            timerThirtySecondTl1["triggered"] = True
            timerThirtySecondTl1["timeStart"] = currentTime
            tl1State = "red"
            
        if(timerElaspedThirtyTl1 == True):
            timerThirtySecondTl1["triggered"] = False
            tl1State = "green"

        if(timerElaspedOneTl2 == True):
            timerOneSecondTl2["triggered"] = False
            timerThirtySecondTl2["triggered"] = True
            timerThirtySecondTl2["timeStart"] = currentTime
            tl2State = "red"
            
        if(timerElaspedThirtyTl2 == True):
            timerThirtySecondTl2["triggered"] = False
            tl2State = "green"

        if((currentTime-lastPollTime)>=pollingRate):
            lastPollTime=currentTime
            us1Result = board.sonar_read(trigPin_1) #returns tuple distance (cm), timestamp
            us2Result = board.sonar_read(trigPin_2)

            if(check_overheight(us1Result, limit, groundDistance) == True and timerOneSecondTl1["triggered"] == False and tl1State == "green"): 
                print_alert(us1Result, groundDistance)  #Checks first supersonic for overheight
                timerOneSecondTl1["triggered"] = True
                timerOneSecondTl1["timeStart"] = currentTime
                tl1State = "yellow"

            if(check_overheight(us2Result, limit, groundDistance) == True and timerOneSecondTl2["triggered"] == False and tl2State == "green"): 
                if(tl1State in ["yellow", "red"]): #Checks second supersonic for overheight and if tl1 is active or not 
                    timerOneSecondTl2["triggered"] = True
                    timerOneSecondTl2["timeStart"] = currentTime
                    tl2State = "yellow"
                else:
                    timerOneSecondTl1["triggered"] = True
                    timerOneSecondTl1["timeStart"] = currentTime
                    tl1State = "yellow"
                    
                    timerOneSecondTl2["triggered"] = True
                    timerOneSecondTl2["timeStart"] = currentTime
                    tl2State = "yellow"            

        map_traffic_light_by_states(tl1State, tl2State)
        time.sleep(0.2)
        
except KeyboardInterrupt:
    print("Quitting the program")
    write_to_shift_register(allOff)
    time.sleep(0.5)
    board.shutdown()
    time.sleep(0.5)
    exit()