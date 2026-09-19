import time
from pymata4 import pymata4

TRIG_PIN = 10
ECHO_PIN = 12

board = pymata4.Pymata4()

board.set_pin_mode_sonar(TRIG_PIN, ECHO_PIN, timeout=35000)

time.sleep(0.5)

print("Starting sensor test. Move an object in front of the sensor. Press Ctrl+C to stop.")

try:
    while True:
        reading = board.sonar_read(TRIG_PIN)
        
        if reading and reading[0] is not None:
            distance, timestamp = reading
            
            if distance == 0:
                print("Distance: 0 cm (Out of range or timeout)")
            else:
                print(f"Distance: {distance} cm | Timestamp: {timestamp}")
        else:
            print("Waiting for valid reading...")

        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nStopping test...")
    board.shutdown()
    print("Done.")