from pymata4 import pymata4
import time

board = pymata4.Pymata4() 

d3 = 3
d4 = 4 
d5 = 5
d6 = 6
d7 = 7

board.set_pin_mode_digital_output(8) 
board.set_pin_mode_digital_input(7) 
board.set_pin_mode_digital_input(d3) 
board.set_pin_mode_digital_input(d4) 
board.set_pin_mode_digital_input(d5) 
board.set_pin_mode_digital_input(d6) 
board.set_pin_mode_digital_input(d7) 


try:
    while True:

        state_1 = board.digital_read(d3)
        state_2 = board.digital_read(d4)
        state_3 = board.digital_read(d5)
        state_4 = board.digital_read(d6)
        state_5 = board.digital_read(d7)

        if state_1 == 1:
            print("Button is pressed (S1)")
        time.sleep(1)
        board.digital_write(8, 1) 
        time.sleep(1)
        board.digital_write(8, 0) 
except KeyboardInterrupt:
    board.digital_write(8, 0) 
    board.shutdown()
    quit()
