import time
from pymata4 import pymata4

TRIG_PIN = 2
ECHO_PIN = 3

def sonar_callback(data):
    """
    Triggered automatically when the sensor calculates a new distance.
    data[1] = trigger pin number
    data[2] = distance in centimeters
    """
    distance_cm = data[2]
    print(f"Distance: {distance_cm} cm")

def main():
    board = pymata4.Pymata4()

    try:
        print(f"Configuring sensor (Trig: {TRIG_PIN}, Echo: {ECHO_PIN})...")
        
        # Initialize the sensor and assign the callback function
        board.set_pin_mode_sonar(TRIG_PIN, ECHO_PIN, callback=sonar_callback)
        
        print("Monitoring distance. Press Ctrl+C to exit.")
        
        # Keep the script running to listen for callbacks
        while True:
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nExiting and cleaning up...")
        board.shutdown()

if __name__ == '__main__':
    main()