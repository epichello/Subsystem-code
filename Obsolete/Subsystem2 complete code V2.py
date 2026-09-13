import time
from pymata4 import pymata4

# Pin assignments
tl4Red, tl4Yellow, tl4Green = 2, 3, 4
tl5Red, tl5Yellow, tl5Green = 5, 6, 7
plGreen, plRedSolid, plFlashEnable = 8, 9, 10
pb1Pin, pb2Pin = 11, 12
ldrPin = 0  # A0

# Timing constants (seconds)
yellowTime = 3
pedGreenTime = 3
pedFlashTime = 2
pedWaitBefore = 2
pedCooldown = 30

dayTl4Green, dayTl5Green = 20, 10
nightTl4Green, nightTl5Green = 30, 5

buttonDebounce = 0.25
ldrNightThreshold = 350
ldrDayThreshold = 450
ldrPollInterval = 0.5

# Global state 
board = None

tl4Colour = "RED"
tl5Colour = "RED"

isNight = False
lastLdrValue = 1023
lastLdrPoll = 0.0

pedRequested = False
pedPrinted = False
lastPedComplete = -pedCooldown
lastButtonTime = 0.0

state = "TL4_GREEN"
stateStart = 0.0

pedStopTarget = None
nightNextGreen = None

# Pin setup
def configurePins():
    for pin in (tl4Red, tl4Yellow, tl4Green,
                tl5Red, tl5Yellow, tl5Green,
                plGreen, plRedSolid, plFlashEnable):
        board.set_pin_mode_digital_output(pin)

    board.set_pin_mode_digital_input_pullup(pb1Pin, callback=onButton)
    board.set_pin_mode_digital_input_pullup(pb2Pin, callback=onButton)
    board.set_pin_mode_analog_input(ldrPin, callback=onLdr)

# Low level output helpers
def setTl(which, colour):
    global tl4Colour, tl5Colour
    pins = (tl4Red, tl4Yellow, tl4Green) if which == "TL4" \
        else (tl5Red, tl5Yellow, tl5Green)
    r, y, g = pins
    board.digital_write(r, 1 if colour == "RED" else 0)
    board.digital_write(y, 1 if colour == "YELLOW" else 0)
    board.digital_write(g, 1 if colour == "GREEN" else 0)
    if which == "TL4":
        tl4Colour = colour
    else:
        tl5Colour = colour

def plWalk():
    board.digital_write(plGreen, 1)
    board.digital_write(plRedSolid, 0)
    board.digital_write(plFlashEnable, 0)

def plFlash():
    board.digital_write(plGreen, 0)
    board.digital_write(plRedSolid, 0)
    board.digital_write(plFlashEnable, 1)

def plSolidRed():
    board.digital_write(plGreen, 0)
    board.digital_write(plFlashEnable, 0)
    board.digital_write(plRedSolid, 1)

# Callbacks
def onButton(data):
    global pedRequested, pedPrinted, lastButtonTime
    value = data[2]
    now = time.monotonic()
    if value == 0 and (now - lastButtonTime) > buttonDebounce:
        lastButtonTime = now
        if not pedPrinted:
            print("Pedestrian crossing requested (PB1/PB2 pressed)")
            pedPrinted = True
        pedRequested = True

def onLdr(data):
    global lastLdrValue
    lastLdrValue = data[2]

# Helpers
def tl4GreenTime():
    return nightTl4Green if isNight else dayTl4Green

def tl5GreenTime():
    return nightTl5Green if isNight else dayTl5Green

def cooldownElapsed():
    return (time.monotonic() - lastPedComplete) >= pedCooldown

def completePedSequence():
    global lastPedComplete, pedRequested, pedPrinted
    lastPedComplete = time.monotonic()
    pedRequested = False
    pedPrinted = False

def enterState(newState):
    global state, stateStart
    state = newState
    stateStart = time.monotonic()

    if newState == "TL4_GREEN":
        setTl("TL4", "GREEN")
        setTl("TL5", "RED")
    elif newState == "TL4_YELLOW":
        setTl("TL4", "YELLOW")
    elif newState == "TL5_GREEN":
        setTl("TL5", "GREEN")
        setTl("TL4", "RED")
    elif newState == "TL5_YELLOW":
        setTl("TL5", "YELLOW")

    elif newState == "PED_WAIT":
        pass
    elif newState == "PED_STOP_YELLOW":
        setTl(pedStopTarget, "YELLOW")
    elif newState == "PED_WALK":
        setTl(pedStopTarget, "RED")
        plWalk()
    elif newState == "PED_FLASH":
        plFlash()

    elif newState == "NIGHT_RED_HOLD_WALK":
        plWalk()
    elif newState == "NIGHT_RED_HOLD_FLASH":
        plFlash()

def elapsed():
    return time.monotonic() - stateStart

def nightBoundaryHit():
    return isNight and pedRequested and cooldownElapsed()

def pollLdr():
    global isNight, lastLdrPoll
    now = time.monotonic()
    if now - lastLdrPoll < ldrPollInterval:
        return
    lastLdrPoll = now
    if lastLdrValue < ldrNightThreshold:
        isNight = True
    elif lastLdrValue > ldrDayThreshold:
        isNight = False

# Main tick - call this in a tight loop
def tick():
    global pedStopTarget, nightNextGreen

    pollLdr()

    if (not isNight and pedRequested and cooldownElapsed()
            and state in ("TL4_GREEN", "TL4_YELLOW", "TL5_GREEN", "TL5_YELLOW")):
        enterState("PED_WAIT")
        return

    e = elapsed()

    if state == "TL4_GREEN":
        if e >= tl4GreenTime():
            enterState("TL4_YELLOW")

    elif state == "TL4_YELLOW":
        if e >= yellowTime:
            setTl("TL4", "RED")
            if nightBoundaryHit():
                nightNextGreen = "TL5"
                enterState("NIGHT_RED_HOLD_WALK")
            else:
                enterState("TL5_GREEN")

    elif state == "TL5_GREEN":
        if e >= tl5GreenTime():
            enterState("TL5_YELLOW")

    elif state == "TL5_YELLOW":
        if e >= yellowTime:
            setTl("TL5", "RED")
            if nightBoundaryHit():
                nightNextGreen = "TL4"
                enterState("NIGHT_RED_HOLD_WALK")
            else:
                enterState("TL4_GREEN")

    elif state == "PED_WAIT":
        if e >= pedWaitBefore:
            pedStopTarget = "TL5" if tl5Colour != "RED" else "TL4"
            enterState("PED_STOP_YELLOW")

    elif state == "PED_STOP_YELLOW":
        if e >= yellowTime:
            enterState("PED_WALK")

    elif state == "PED_WALK":
        if e >= pedGreenTime:
            enterState("PED_FLASH")

    elif state == "PED_FLASH":
        if e >= pedFlashTime:
            plSolidRed()
            completePedSequence()
            enterState("TL4_GREEN")

    elif state == "NIGHT_RED_HOLD_WALK":
        if e >= pedGreenTime:
            enterState("NIGHT_RED_HOLD_FLASH")

    elif state == "NIGHT_RED_HOLD_FLASH":
        if e >= pedFlashTime:
            plSolidRed()
            completePedSequence()
            enterState(f"{nightNextGreen}_GREEN")

def main():
    global board
    board = pymata4.Pymata4()
    configurePins()
    plSolidRed()
    enterState("TL4_GREEN")
    try:
        while True:
            tick()
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        board.shutdown()

if __name__ == "__main__":
    main()