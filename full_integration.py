from pymata4 import pymata4
import time
import math
import random

#----Temp stuff (delete when done)
import queue
import threading
#----

board = pymata4.Pymata4()  # board initialisation

# ---------------------Full-pin-layout---------------------
TRIG_PIN_US_1 = 2
ECHO_PIN_US_1 = 3

TRIG_PIN_US_2 = 4
ECHO_PIN_US_2 = 5

TRIG_PIN_US_3 = 6
ECHO_PIN_US_3 = 7

TRIG_PIN_US_4 = 8
ECHO_PIN_US_4 = 9

TRIG_PIN_US_5 = 10
ECHO_PIN_US_5 = 12

PB_1_2_PIN = 13

CLOCK_PIN = 14  # A0
LATCH_PIN = 15
DATA_PIN = 16

DS_PIN_1 = 3  # A3
DS_PIN_2 = 4  # A4

# ------------------------Constants------------------------

GROUND = 20  # ground is 10cm away from the supersonic sensors

UP = 1 #button states
DOWN = 0

ON = 1  
OFF = 0

FILTER_WINDOW = 5  #avg last 5 readings 

NIGHT_THRESHOLD = 200   #Daylight sensor values

DAY_THRESHOLD = 300 #100 deadzone to prevent random fluctuations

ERROR_MARGIN = 5 #5cm

# ------------------------Controllers------------------------

diodeStateDict = {"diodes": 0b0000000100100001001101000100100}


sensor_history = {
    "US1": [],
    "US2": []
}

tl1Controller = {
    "state": 0,
    "startTime": 0.0,
    "durations": {1: 1.0, 2: 30.0},
}  # stage 1: 1s, stage 2: 10s


tl2Controller = {
    "state": 0,
    "startTime": 0.0,
    "durations": {1: 1.0, 2: 30.0},
}  # stage 1: 1s, stage 2: 10s


tl4tl5CycleController = {   
    "state": 0,
    "startTime": 0.0,
    "durations": {1: 20.0, 2: 3.0, 3: 10.0, 4: 3.0, 5: 30.0, 6: 3.0, 7: 5.0, 8: 3.0, 9: 50},    #Stage 9, is for interruption cycle
}  #Day cyle: stage 1: 20s, stage 2: 3s, stage 3: 10s, stage 4: 3s. Night cycle: stage 5: 30, stage 6: 3, stage 7: 5, stage 8: 3 

tl6Controller = {
    "state": 0,
    "startTime": 0.0,
    "durations": {1: 5.0, 2: 3.0, 3: 1.0, 4: 10.0, 5: 3.0, 6: 60.0},   #Stage 5 for interruption 
    "freeze": 0, #to stop at red
}   

ds1EnvironmentState = {
    "isNight": False,
}

ds2EnvironmentState = {
    "isNight": False,
}

buttonController = {
    "state": 1,
    "startTime": 0.0,
    "durations": {0: 30.0, 1: 0},
}

pedestrianInterrupt = {
    "state": 0,
    "startTime": 0.0,
    "durations": {1: 2.0, 2: 3.0, 3: 3.0, 4: 2.0},
    "freezeState": 4, #Used to check the state after the 2s wait later
}

us45Controller = {
    "us4Detect": False,
}

wl1Controller = {
    "hz": 2,
    "startTime": 0.0,
    "state": 0,
}

wl2Controller = {
    "hz": 2,
    "startTime": 0.0,
    "state": 0,
}

# ---------------------Registering pins----------------------------
board.set_pin_mode_digital_output(CLOCK_PIN)  # Shift register
board.set_pin_mode_digital_output(LATCH_PIN)
board.set_pin_mode_digital_output(DATA_PIN)

board.set_pin_mode_sonar(TRIG_PIN_US_1, ECHO_PIN_US_1, timeout=2500)  # Ultrasonic sensors
board.set_pin_mode_sonar(TRIG_PIN_US_2, ECHO_PIN_US_2, timeout=2500)  # timeout configures listening time, longer = more distance (ms)
board.set_pin_mode_sonar(TRIG_PIN_US_3, ECHO_PIN_US_3, timeout=2500)  #5000 - 50-70cm range
board.set_pin_mode_sonar(TRIG_PIN_US_4, ECHO_PIN_US_4, timeout=2500)  #2500 - 30cm max range
board.set_pin_mode_sonar(TRIG_PIN_US_5, ECHO_PIN_US_5, timeout=2500)

board.set_pin_mode_digital_input_pullup(PB_1_2_PIN)  # Buttons

board.set_pin_mode_analog_input(DS_PIN_1)  # Daylight sensors
board.set_pin_mode_analog_input(DS_PIN_2)

time.sleep(0.5)  # config time

# ---------------------Shift Register Constants---------------------
ALLOFF = 0

TL1R = 0
TL1Y = 1
TL1G = 2
TL2R = 3
TL2Y = 4
TL2G = 5
WL1L = 6
WL1R = 7
PL1G = 8
PL1R = 9
PL2G = 10
PL2R = 11
TL4R = 12
TL4Y = 13
TL4G = 14
TL5R = 15
TL5Y = 16
TL5G = 17
FL1 = 18
FL2 = 19
TL6R = 20
TL6Y = 21
TL6G = 22
TL3G = 23
TL3R = 24
WL2R1 = 25  # left
WL2R2 = 26  # middle left
WL2R3 = 27  # middle right
WL2R4 = 28  # right

# -------------------------------------------------


def update_bit(diodeState, ledNumber, state):
    """
    Changes specified value in binary number
    Parameters:
        diodeState (int): sequence (32-bit) for shift register
        ledNumber (int): corresponding led position in the diodeState sequence
        state (boolean): on or off
    Returns:
        diodeState (int): sequence (32-bit) for shift register
    """
    if state == 1:
        return diodeState | (
            1 << ledNumber
        )  # RHS creates temp 32-bit number; compares RHS bit with LHS bit using OR operator
    else:
        return diodeState & ~(
            1 << ledNumber
        )  # RHS creates temp 32-bit number; compares RHS bit with LHS bit using NAND operator

def write_to_shift_register(value):
    """
    Used to write to shift register activating pins by given value
        Parameters:
            value (int): sequence (8-bit) for shift register
        Returns:
            Does not return anything
    """
    board.digital_write(
        LATCH_PIN, 0
    )  # readies shift register to listen (initial state low for all outputs)
    for i in range(31, -1, -1):
        board.digital_write(DATA_PIN, (value >> i) & 1)  # shifts value and ensures only 0s and 1s are pushed through
        board.digital_write(CLOCK_PIN, 1)
        board.digital_write(CLOCK_PIN, 0)  # these two lines complete one clock cycle (pushes one bit)
        time.sleep(0.001)

    board.digital_write(LATCH_PIN, 1)  # executes the memory and lights the LEDs

def tl_on_off_on_off(ledNumber1, ledNumber2, ledNumber3, ledNumber4):
    current = diodeStateDict["diodes"]
    current = update_bit(current, ledNumber1, ON)
    current = update_bit(current, ledNumber2, OFF)
    current = update_bit(current, ledNumber3, ON)
    current = update_bit(current, ledNumber4, OFF)
    diodeStateDict["diodes"] = current

def tl_off_on_off_on(ledNumber1, ledNumber2, ledNumber3, ledNumber4):
    current = diodeStateDict["diodes"]
    current = update_bit(current, ledNumber1, OFF)
    current = update_bit(current, ledNumber2, ON)
    current = update_bit(current, ledNumber3, OFF)
    current = update_bit(current, ledNumber4, ON)
    diodeStateDict["diodes"] = current

def tl_off_off_off_off(ledNumber1, ledNumber2, ledNumber3, ledNumber4):
    current = diodeStateDict["diodes"]
    current = update_bit(current, ledNumber1, OFF)
    current = update_bit(current, ledNumber2, OFF)
    current = update_bit(current, ledNumber3, OFF)
    current = update_bit(current, ledNumber4, OFF)
    diodeStateDict["diodes"] = current

def tl_r_off_y_off_g_on(ledNumberRed, ledNumberYellow, ledNumberGreen):
    current = diodeStateDict["diodes"]
    current = update_bit(current, ledNumberRed, OFF)
    current = update_bit(current, ledNumberYellow, OFF)
    current = update_bit(current, ledNumberGreen, ON)
    diodeStateDict["diodes"] = current
    # write_to_shift_register(current)

def tl_r_off_y_on_g_off(ledNumberRed, ledNumberYellow, ledNumberGreen):
    current = diodeStateDict["diodes"]
    current = update_bit(current, ledNumberRed, OFF)
    current = update_bit(current, ledNumberYellow, ON)
    current = update_bit(current, ledNumberGreen, OFF)
    diodeStateDict["diodes"] = current
    # write_to_shift_register(current)

def tl_r_on_y_off_g_off(ledNumberRed, ledNumberYellow, ledNumberGreen):
    current = diodeStateDict["diodes"]
    current = update_bit(current, ledNumberRed, ON)
    current = update_bit(current, ledNumberYellow, OFF)
    current = update_bit(current, ledNumberGreen, OFF)
    diodeStateDict["diodes"] = current
    # write_to_shift_register(current)

def tl_r_off_g_on(ledNumberRed, ledNumberGreen):
    current = diodeStateDict["diodes"]
    current = update_bit(current, ledNumberRed, OFF)
    current = update_bit(current, ledNumberGreen, ON)
    diodeStateDict["diodes"] = current
    # write_to_shift_register(current)

def tl_r_on_g_off(ledNumberRed, ledNumberGreen):
    current = diodeStateDict["diodes"]
    current = update_bit(current, ledNumberRed, ON)
    current = update_bit(current, ledNumberGreen, OFF)
    diodeStateDict["diodes"] = current
    # write_to_shift_register(current)

def tl_r_off_g_off(ledNumberRed, ledNumberGreen):
    current = diodeStateDict["diodes"]
    current = update_bit(current, ledNumberRed, OFF)
    current = update_bit(current, ledNumberGreen, OFF)
    diodeStateDict["diodes"] = current

def tl_r_on_g_on(ledNumberRed, ledNumberGreen):
    current = diodeStateDict["diodes"]
    current = update_bit(current, ledNumberRed, ON)
    current = update_bit(current, ledNumberGreen, ON)
    diodeStateDict["diodes"] = current

def update_traffic(sequence, action, action_default):

    currentTime = time.time()
    elapsed = currentTime - sequence["startTime"]

    if sequence["state"] == 1:
        if elapsed >= sequence["durations"][1]:
            sequence["state"] = 2
            sequence["startTime"] = currentTime
            action()

    elif sequence["state"] == 2:
        if elapsed >= sequence["durations"][2]:
            sequence["state"] = 0
            action_default()

def start_sequence(sequence, action):
    if sequence["state"] == 0:  # prevents multiple timers from starting
        sequence["state"] = 1
        sequence["startTime"] = time.time()
        action()

def check_overheight(heightTime, limit, ground):
    """
    Used to check if the height is overheight and return a boolean
        Parameters:
            heightTime (float): Values read by the supersonic sensor
            limit(integer/float): Used to compare with distanceCm
            ground(intger/float): Distance between supersonic sensor and ground
        Returns:
            Returns a boolean dependent on if overheight was detected
    """
    distanceCm = heightTime[0]
    if (ground - distanceCm) > limit:
        return True

    return False

def print_alert(heightTime, ground):
    """
    Used to check if the height is overheight and print an alert if it is
        Parameters:
            heightTime (float): Values read by the supersonic sensor
            ground(intger/float): Distance between supersonic sensor and ground
        Returns:
            Does not return anything
    """
    distanceCm = ground - heightTime[0]
    timeStamp = time.localtime(heightTime[1])
    formattedDate = time.strftime("%d/%m/%Y %H:%M:%S", timeStamp)
    print(
        f"Overheight was detected! vehicle height: {distanceCm}m at time: {formattedDate}"
    )

def apply_moving_average(raw_reading, history_list, window_size):

    if (raw_reading or raw_reading[0]) == None or raw_reading[0] == 0:
        return raw_reading 
        
    distance = raw_reading[0]
    timestamp = raw_reading[1]
    
    history_list.append(distance)
    
    if len(history_list) > window_size:
        history_list.pop(0) #removes last (oldest item)

    avg_distance = sum(history_list) / len(history_list)
    
    return [avg_distance, timestamp]

#---delete when done----------------
mock_input_queue = queue.Queue()

def keyboard_listener():
    """Background thread to read terminal input without blocking the main loop."""
    while True:
        try:
            line = input()
            val = float(line.strip())
            mock_input_queue.put(val)
            print(f"[MANUAL OVERRIDE] Set US1 distance to: {val} cm")
        except ValueError:
            pass
        except EOFError:
            break

# Start the listener thread as a daemon (closes automatically when script stops)
input_thread = threading.Thread(target=keyboard_listener, daemon=True)
input_thread.start()

# Default fallback value: distance = 15 cm (below threshold), current timestamp
last_us1_value = [15.0, time.time()]
#----------------------------------

def subsystem_1(us1History, us2History):

    #--------delete when done--------------
    global last_us1_value

    try:
        new_dist = mock_input_queue.get_nowait()
        last_us1_value = [new_dist, time.time()]
    except queue.Empty:
        # Keep previous distance, update timestamp to now
        last_us1_value[1] = time.time()

    us1Value = last_us1_value

    #-------------------------------------------
    #us1ValueRaw = board.sonar_read(TRIG_PIN_US_1)
    us2ValueRaw = board.sonar_read(TRIG_PIN_US_2)

    #us1Value = apply_moving_average(us1ValueRaw, us1History, 5)
    us2Value = apply_moving_average(us2ValueRaw, us2History, 5)

    currentTime = time.time()
    elapsed = currentTime - wl1Controller["startTime"] 

    if check_overheight(us1Value, limit, GROUND) == True:
        if tl1Controller["state"] == 0:
            print_alert(us1Value, GROUND)
        start_sequence(tl1Controller, lambda: tl_r_off_y_on_g_off(TL1R, TL1Y, TL1G))

    update_traffic(
        tl1Controller,
        lambda: tl_r_on_y_off_g_off(TL1R, TL1Y, TL1G),
        lambda: tl_r_off_y_off_g_on(TL1R, TL1Y, TL1G),
    )

    if check_overheight(us2Value, limit, GROUND) == True:

        start_sequence(tl2Controller, lambda: tl_r_off_y_on_g_off(TL2R, TL2Y, TL2G))
    update_traffic(
        tl2Controller,
        lambda: tl_r_on_y_off_g_off(TL2R, TL2Y, TL2G),
        lambda: tl_r_off_y_off_g_on(TL2R, TL2Y, TL2G),
    )
    
    if tl1Controller["state"] == 0 and tl2Controller["state"] == 1:
        start_sequence(tl1Controller, lambda: tl_r_off_y_on_g_off(TL1R, TL1Y, TL1G))
        update_traffic(
            tl1Controller,
            lambda: tl_r_on_y_off_g_off(TL1R, TL1Y, TL1G),
            lambda: tl_r_off_y_off_g_on(TL1R, TL1Y, TL1G),
        )
        start_sequence(tl2Controller, lambda: tl_r_off_y_on_g_off(TL2R, TL2Y, TL2G))
        update_traffic(
            tl2Controller,
            lambda: tl_r_on_y_off_g_off(TL2R, TL2Y, TL2G),
            lambda: tl_r_off_y_off_g_on(TL2R, TL2Y, TL2G),
        )
    
    if tl1Controller["state"] != 0 or tl2Controller["state"] != 0:
        elapsed = currentTime - wl1Controller["startTime"]

        if wl1Controller["state"] == 0:
            wl1Controller["startTime"] = currentTime
            wl1Controller["state"] = 1
            elapsed = 0.0
            
        if wl1Controller["state"] == 1:
            if int(elapsed * (wl1Controller["hz"] * 2)) % 2 == 0:
                tl_r_off_g_on(WL1L, WL1R)
            else:
                tl_r_on_g_off(WL1L, WL1R)

        else:
            tl_r_off_g_off(WL1L, WL1R)
            wl1Controller["state"] = 0

def subsystem_2():
    currentTime = time.time()
    buttonData, timeStampPb12 = board.digital_read(PB_1_2_PIN)  # default 1 (up)
    ldr_data2, timeStampDs2 = board.analog_read(DS_PIN_2)
    # print(ldr_data2)

    if ldr_data2 < NIGHT_THRESHOLD:
        ds2EnvironmentState["isNight"] = True
    elif ldr_data2 > DAY_THRESHOLD:
        ds2EnvironmentState["isNight"] = False

    elapsedButton = currentTime - buttonController["startTime"]
    stateButton = buttonController["state"]
    elapsedInterrupt = currentTime - pedestrianInterrupt["startTime"]
    stateInterrupt = pedestrianInterrupt["state"]

    if (buttonData == DOWN) and buttonController["state"] == UP:
        
        buttonController["state"] = DOWN
        buttonController["startTime"] = currentTime
        
        elapsedButton = 0.0 
        stateButton = DOWN

        if tl4tl5CycleController["state"] in (1, 2, 5, 6):
            pedestrianInterrupt["freezeState"] = 4
        else:
            pedestrianInterrupt["freezeState"] = 5


        tl4tl5CycleController["state"] = 9 #stops the original cycle
        
        pedestrianInterrupt["state"] = 1
        pedestrianInterrupt["startTime"] = currentTime
        stateInterrupt = pedestrianInterrupt["state"]
        
        elapsedInterrupt = 0.0
        
        print("The pedestrian button has been pressed")

    if buttonController["state"] == DOWN: #Checks the state, so it only runs when it has been registered to been pressed
        if stateInterrupt != 0:
            if (elapsedInterrupt >= pedestrianInterrupt["durations"][stateInterrupt]) and pedestrianInterrupt["state"] == 1:
                if pedestrianInterrupt["freezeState"]==4:
                    tl_r_off_y_on_g_off(TL4R, TL4Y, TL4G)   # Force TL4 to go yellow for 3s, TL5 stays red
                    tl_r_on_y_off_g_off(TL5R, TL5Y, TL5G)            
                else:
                    tl_r_on_y_off_g_off(TL4R, TL4Y, TL4G)   # Force TL5 to go yellow for 3s, TL4 stays red
                    tl_r_off_y_on_g_off(TL5R, TL5Y, TL5G)
                pedestrianInterrupt["state"] = 2
                pedestrianInterrupt["startTime"] = currentTime

            elif (elapsedInterrupt >= pedestrianInterrupt["durations"][stateInterrupt]) and pedestrianInterrupt["state"] == 2:
                tl_r_on_y_off_g_off(TL4R, TL4Y, TL4G)   
                tl_r_on_y_off_g_off(TL5R, TL5Y, TL5G)
                tl_r_off_g_on(PL1R, PL1G)
                tl_r_off_g_on(PL2R, PL2G)
                pedestrianInterrupt["state"] = 3
                pedestrianInterrupt["startTime"] = currentTime
                        
            elif (elapsedInterrupt >= pedestrianInterrupt["durations"][stateInterrupt]) and pedestrianInterrupt["state"] == 3:
                tl_r_on_g_off(PL1R, PL1G)
                tl_r_on_g_off(PL2R, PL2G) 
                pedestrianInterrupt["state"] = 4
                pedestrianInterrupt["startTime"] = currentTime

            elif pedestrianInterrupt["state"] == 4:
                if elapsedInterrupt >= pedestrianInterrupt["durations"][stateInterrupt]:    #Return to solid red after flashing is done
                    tl_r_on_g_off(PL1R, PL1G)
                    tl_r_on_g_off(PL2R, PL2G)
                    
                    pedestrianInterrupt["state"] = 0

                    
                    tl4tl5CycleController["state"] = 0

                else:
                    if int(elapsedInterrupt * 5) % 2 == 0:
                        tl_r_on_g_off(PL1R, PL1G)
                        tl_r_on_g_off(PL2R, PL2G)
                    else:
                        tl_r_off_g_off(PL1R, PL1G)
                        tl_r_off_g_off(PL2R, PL2G)

        if (elapsedButton >= buttonController["durations"][stateButton]) and buttonController["state"] == DOWN:  #button has been pressed & only allows when 30s has passed
            buttonController["state"] = UP
                


    # ------------------------Cycling sequence------------------------
    
    if tl4tl5CycleController["state"] == 9:
        # print("this is happening")
        return

    
    if tl4tl5CycleController["state"] == 0:
        if ds2EnvironmentState["isNight"] == True:
            tl4tl5CycleController["state"] = 5
        else:
            tl4tl5CycleController["state"] = 1

        tl4tl5CycleController["startTime"] = currentTime
        tl_r_off_y_off_g_on(TL4R, TL4Y, TL4G)
        tl_r_on_y_off_g_off(TL5R, TL5Y, TL5G)
        return

    state = tl4tl5CycleController["state"]
    elapsed = currentTime - tl4tl5CycleController["startTime"]

    if elapsed >= tl4tl5CycleController["durations"][state]:
        if ds2EnvironmentState["isNight"] == True:
            nextState = (state % 4) + 5  #Add 5 to start from 5 when modulo returns 0 to cycle different times
        else:
            nextState = (state % 4) + 1
        tl4tl5CycleController["state"] = nextState
        tl4tl5CycleController["startTime"] = currentTime

        if nextState == 1 or nextState == 5:  # TL4 Green, TL5 Red (20s or 30s if night)
            tl_r_off_y_off_g_on(TL4R, TL4Y, TL4G)
            tl_r_on_y_off_g_off(TL5R, TL5Y, TL5G)
        elif nextState == 2 or nextState == 6:  # TL4 Yellow, TL5 Red (3s)
            tl_r_off_y_on_g_off(TL4R, TL4Y, TL4G)
            tl_r_on_y_off_g_off(TL5R, TL5Y, TL5G)

        elif nextState == 3 or nextState == 7:  # TL4 Red, TL5 Green (10s or 5s if night)
            tl_r_on_y_off_g_off(TL4R, TL4Y, TL4G)
            tl_r_off_y_off_g_on(TL5R, TL5Y, TL5G)

        elif nextState == 4 or nextState == 8:  # TL4 Red, TL5 Yellow (3s)
            tl_r_on_y_off_g_off(TL4R, TL4Y, TL4G)
            tl_r_off_y_on_g_off(TL5R, TL5Y, TL5G)

def subsystem_3():
    us5Value = board.sonar_read(TRIG_PIN_US_5)  #[distance, timestamp]
    ldr_data1, timeStampDs2 = board.analog_read(DS_PIN_1)
    currentTime = time.time()

    if ldr_data1 < NIGHT_THRESHOLD:
        ds1EnvironmentState["isNight"] = True
    elif ldr_data1 > DAY_THRESHOLD:
        ds1EnvironmentState["isNight"] = False

    if check_overheight(us5Value, limit, GROUND) == True:
        if ds1EnvironmentState["isNight"] == True:
            tl6Controller["state"] = 4
        else:    
            tl6Controller["state"] = 1
            
        tl_r_off_y_off_g_on(TL6R, TL6Y, TL6G)
        tl6Controller["startTime"] = currentTime

    state = tl6Controller["state"]
    elapsed = currentTime - tl6Controller["startTime"]

    if check_overheight(us5Value, limit, GROUND) == True and tl6Controller["state"] in (1,2):
        if elapsed >= tl6Controller["durations"][state]:
            tl6Controller["freeze"] = 1

    if check_overheight(us5Value, limit, GROUND) == True and ds1EnvironmentState["isNight"] == True:
        tl_r_on_g_on(FL1, FL2)
    else:
        tl_r_off_g_off(FL1, FL2)

    if tl6Controller["state"] not in (0,6):
        if elapsed >= tl6Controller["durations"][state]:
            if ds1EnvironmentState["isNight"] == True:
                nextState = (state % 3) + 4
            else:
                nextState = (state % 3) + 1

            if tl6Controller["freeze"] == 1:

                tl6Controller["state"] = 0
                tl6Controller["freeze"] = 0
                return
            
            elif nextState in (2,5):
                tl6Controller["freeze"] = 0 #remove freeze if there was previously a freeze
                tl_r_off_y_on_g_off(TL6R, TL6Y, TL6G)
                tl6Controller["state"] = 2
                tl6Controller["startTime"] = currentTime

            elif nextState in (3,6):
                tl6Controller["freeze"] = 0
                tl_r_on_y_off_g_off(TL6R, TL6Y, TL6G)
                tl6Controller["state"] = 3  #freeze state

def subsystem_4():
    #2hz flashing - 1s per XOXO/OXOX
    us3Value = board.sonar_read(TRIG_PIN_US_3)  #[distance, timestamp]
    us4Value = board.sonar_read(TRIG_PIN_US_4)  #[distance, timestamp]
    #first flash off on off on

    currentTime = time.time()
    elapsed = currentTime - wl2Controller["startTime"] 

    ValueCheck = abs(us3Value[0] - us4Value[0]) <= ERROR_MARGIN

    if check_overheight(us3Value, limit, GROUND) == True and check_overheight(us4Value, limit, GROUND) == True and ValueCheck == True:
        tl_r_on_g_off(TL3R, TL3G)

        if wl2Controller["state"] == 0:
            wl2Controller["startTime"] = currentTime
            wl2Controller["state"] = 1
            elapsed = 0.0
        
        if wl2Controller["state"] == 1:
            
            if int(elapsed*(wl2Controller["hz"]*2))%2 == 0: #times 2 cause we are doing 2hz/3hz, so 4/6 state changes int truncates so we can use modulo properly
                tl_off_on_off_on(WL2R1, WL2R2, WL2R3, WL2R4)
            else:
                tl_on_off_on_off(WL2R1, WL2R2, WL2R3, WL2R4)
    else:
        tl_r_off_g_on(TL3R, TL3G)
        tl_off_off_off_off(WL2R1, WL2R2, WL2R3, WL2R4)
        wl2Controller["state"] = 0
     

    
            







            


# ----------------------User Input---------------------

limit = 4.0  # Default value

limit = input("Input a height limit or press enter to set default: ")

while True:
    limit = limit.strip()
    if limit == "":
        limit = 4.0
        break
    try:
        limit = float(limit)
        if limit <= 0:
            limit = input("Enter a valid input or press enter to set default value 1 ")
            continue
        break
    except ValueError:
        limit = input("Enter a valid input or press enter to set default value: ")


print(f"The limit was set to {limit}m")

# ------------------------------------------------------------------------

write_to_shift_register(diodeStateDict["diodes"])
previous_diodes = diodeStateDict["diodes"]

try:
    while True:
        us1_moving_average_list = []
        us2_moving_average_list = []

        subsystem_1(us1_moving_average_list, us2_moving_average_list)
        subsystem_2()
        subsystem_3()
        subsystem_4()

        if diodeStateDict["diodes"] != previous_diodes: #Lowers the amount of bits goings to the shift register, decreases component load
            write_to_shift_register(diodeStateDict["diodes"])
            previous_diodes = diodeStateDict["diodes"]  

        time.sleep(0.2)
except KeyboardInterrupt:
    print("Quitting the program")
    write_to_shift_register(ALLOFF)
    time.sleep(0.5)
    board.shutdown()
    time.sleep(0.5)
    exit()
