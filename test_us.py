import time
from pymata4 import pymata4

# Define the analog pins
# In pymata4, analog pins A0-A5 are simply referenced by integers 0-5
PIN_A3 = 3
PIN_A4 = 4

def main():
    # Initialize the board connection
    print("Connecting to Arduino...")
    board = pymata4.Pymata4()
    
    try:
        # Configure the pins as analog inputs
        board.set_pin_mode_analog_input(PIN_A3)
        board.set_pin_mode_analog_input(PIN_A4)
        
        print("Starting sensor readings. Press Ctrl+C to stop.")
        print("-" * 45)
        
        # Polling loop to read and display values
        while True:
            # board.analog_read() returns a tuple: (pin_value, timestamp)
            a3_value, _ = board.analog_read(PIN_A3)
            a4_value, _ = board.analog_read(PIN_A4)
            
            # Print the readings side-by-side
            print(f"Daylight A3: {a3_value:4} | Daylight A4: {a4_value:4}")
            
            # Wait half a second before reading again
            time.sleep(0.5)
            
    except KeyboardInterrupt:
        print("\nTest stopped by user.")
    finally:
        # Cleanly close the serial connection to the board
        board.shutdown()

if __name__ == '__main__':
    main()