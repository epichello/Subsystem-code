"""
Tunnel Ave Junction Controller
Controls the minor-junction traffic lights TL4/TL5, the pedestrian
lights PL1/PL2 (mirrored, single controlled signal), the two
pedestrian push buttons PB1/PB2, and reads the LDR DS2 for day/night
mode.

Hardware notes
---------------
* PL1 and PL2 are wired to the SAME three Arduino output lines
  (plGreen, plRedSolid, plFlashEnable) because they must always
  show an identical state - the kit only supplies one 4-pin RGB LED,
  and a second one (if you want a real PL2) is simply wired in
parallel to the same three lines with its own resistors.
* The RED channel of the PL RGB LED is driven by TWO isolated sources
  OR'd together through two 1N4007 diodes:
    - plRedSolid  -> 220R -> diode -> LED red anode   (solid red)
    - 555 pin 3 out -> 220R -> diode -> LED red anode   (flashing red)
  The 555's RESET pin (pin 4) is tied to plFlashEnable. RESET low
  forces the 555 output permanently low (no flash); RESET high lets
  the 555 free-run and flash the LED at ~1.5 Hz. This means the
  ACTUAL FLASH RATE IS GENERATED IN HARDWARE by the 555, not by
  software toggling a pin - Arduino only turns the oscillator on/off.
* PB1/PB2 use the UNO's internal pull-up resistors: one leg of each
  button to GND, the other to its digital pin. No external resistor
  needed. A press pulls the pin LOW.
* DS2 (LDR) forms a voltage divider with a 10k resistor between 5V
  and GND: 5V -> LDR -> (node -> A0) -> 10k -> GND. Bright light =
  low LDR resistance = HIGH reading at A0. Dark = LOW reading.
 
Pin map
-------
D2  tl4Red         D3  tl4Yellow      D4  tl4Green
D5  tl5Red         D6  tl5Yellow      D7  tl5Green
D8  plGreen        D9  plRedSolid    D10 plFlashEnable (555 RESET)
D11 PB1 (pull-up)   D12 PB2 (pull-up)
A0  DS2 LDR divider node
"""
 
import time
from pymata4 import pymata4
 
# Pin assignments
tl4Red, tl4Yellow, tl4Green = 2, 3, 4
tl5Red, tl5Yellow, tl5Green = 5, 6, 7
plGreen, plRedSolid, plFlashEnable = 8, 9, 10
pb1, pb2 = 11, 12
ldr = 0  # A0
 
# assignments of timing constants (seconds)
yellowTime = 3
pedGreenTime = 3
pedFlashTime = 2
pedWaitBefore = 2          # day-mode "2 second wait" from Rule 1
pedCooldown = 30            # Rule 4
nightRedHold = pedGreenTime + pedFlashTime  # = 5s, Rule 5
 
DAY_tl4Green, DAY_tl5Green = 20, 10
NIGHT_tl4Green, NIGHT_tl5Green = 30, 5
 
# allowing time for button to bounce
buttonBounce = 0.25

#light boundaries
ldrNightThreshold = 350    # below this reading -> dark
ldr = 450      # above this reading -> bright
ldrPollInterval = 0.5
 
 
class Intersection:
    def __init__(self, board):
        self.board = board

        # --- current colour of each traffic light ---
        self.tl4_colour = "RED"
        self.tl5_colour = "RED"

        # --- day/night flag, defaults to day until first LDR sample ---
        self.is_night = False
        self.last_ldr_value = 1023
        self.last_ldr_poll = 0.0
 
        # --- pedestrian request bookkeeping ---
        self.ped_requested = False
        self.ped_printed = False
        self.last_ped_complete = -pedCooldown  # allow immediate first use
        self.last_button_time = 0.0
 
        # --- main state machine ---
        # 'tl4Green' / 'tl4Yellow' / 'tl5Green' / 'tl5Yellow'
        # 'PED_WAIT' / 'PED_STOP_YELLOW' / 'PED_WALK' / 'PED_FLASH'
        # 'nightRedHold_WALK' / 'nightRedHold_FLASH'
        self.state = "tl4Green"
        self.state_start = time.monotonic()
 
        # Remembers which TL is being stopped for the *day* pedestrian
        # sequence, and which TL should go green next after a *night*
        # red hold (the one that was naturally due).
        self.ped_stop_target = None
        self.night_next_green = None
 
        self._configure_pins()
        self._enter_state("tl4Green")
 
    # -----------------------------------------------------------------
    # Pin setup
    # -----------------------------------------------------------------
    def _configure_pins(self):
        b = self.board
        for pin in (tl4Red, tl4Yellow, tl4Green,
                    tl5Red, tl5Yellow, tl5Green,
                    plGreen, plRedSolid, plFlashEnable):
            b.set_pin_mode_digital_output(pin)
 
        # Buttons: internal pull-up, callback-driven (edge detection)
        b.set_pin_mode_digital_input_pullup(pb1, callback=self._on_button)
        b.set_pin_mode_digital_input_pullup(pb2, callback=self._on_button)
 
        # LDR: analog input, callback keeps the latest reading fresh
        b.set_pin_mode_analog_input(ldr, callback=self._on_ldr)
 
    # -----------------------------------------------------------------
    # Low level output helpers
    # -----------------------------------------------------------------
    def _set_tl(self, which, colour):
        """Drive one traffic light's three LEDs so only `colour` is lit."""
        pins = (tl4Red, tl4Yellow, tl4Green) if which == "TL4" \
            else (tl5Red, tl5Yellow, tl5Green)
        r, y, g = pins
        self.board.digital_write(r, 1 if colour == "RED" else 0)
        self.board.digital_write(y, 1 if colour == "YELLOW" else 0)
        self.board.digital_write(g, 1 if colour == "GREEN" else 0)
        if which == "TL4":
            self.tl4_colour = colour
        else:
            self.tl5_colour = colour
 
    def _pl_walk(self):
        self.board.digital_write(plGreen, 1)
        self.board.digital_write(plRedSolid, 0)
        self.board.digital_write(plFlashEnable, 0)
 
    def _pl_flash(self):
        self.board.digital_write(plGreen, 0)
        self.board.digital_write(plRedSolid, 0)
        self.board.digital_write(plFlashEnable, 1)   # lets the 555 free-run
 
    def _pl_solid_red(self):
        self.board.digital_write(plGreen, 0)
        self.board.digital_write(plFlashEnable, 0)
        self.board.digital_write(plRedSolid, 1)
 
    # -----------------------------------------------------------------
    # Callbacks
    # -----------------------------------------------------------------
    def _on_button(self, data):
        # data = [pin_type, pin_number, value, timestamp]
        value = data[2]
        now = time.monotonic()
        if value == 0 and (now - self.last_button_time) > buttonBounce:
            self.last_button_time = now
            if not self.ped_printed:
                print("Pedestrian crossing requested (PB1/PB2 pressed)")
                self.ped_printed = True
            self.ped_requested = True
 
    def _on_ldr(self, data):
        # data = [pin_type, pin_number, value, timestamp]
        self.last_ldr_value = data[2]
 
    # -----------------------------------------------------------------
    # Helpers for cycle timing
    # -----------------------------------------------------------------
    def _tl4Green_time(self):
        return NIGHT_tl4Green if self.is_night else DAY_tl4Green
 
    def _tl5Green_time(self):
        return NIGHT_tl5Green if self.is_night else DAY_tl5Green
 
    def _cooldown_elapsed(self):
        return (time.monotonic() - self.last_ped_complete) >= pedCooldown
 
    def _complete_ped_sequence(self):
        """Called at the exact moment Rule 2's '2.R1' finishes."""
        self.last_ped_complete = time.monotonic()
        self.ped_requested = False
        self.ped_printed = False   # allows the console message again next time
 
    def _enter_state(self, new_state):
        self.state = new_state
        self.state_start = time.monotonic()
 
        if new_state == "tl4Green":
            self._set_tl("TL4", "GREEN")
            self._set_tl("TL5", "RED")
        elif new_state == "tl4Yellow":
            self._set_tl("TL4", "YELLOW")
        elif new_state == "tl5Green":
            self._set_tl("TL5", "GREEN")
            self._set_tl("TL4", "RED")
        elif new_state == "tl5Yellow":
            self._set_tl("TL5", "YELLOW")
 
        elif new_state == "PED_WAIT":
            pass  # lights stay exactly as they were - just wait 2s
        elif new_state == "PED_STOP_YELLOW":
            self._set_tl(self.ped_stop_target, "YELLOW")
        elif new_state == "PED_WALK":
            self._set_tl(self.ped_stop_target, "RED")
            self._pl_walk()
        elif new_state == "PED_FLASH":
            self._pl_flash()
 
        elif new_state == "nightRedHold_WALK":
            self._pl_walk()
        elif new_state == "nightRedHold_FLASH":
            self._pl_flash()
 
    def _elapsed(self):
        return time.monotonic() - self.state_start
 
    # -----------------------------------------------------------------
    # Main tick - call this in a tight loop
    # -----------------------------------------------------------------
    def tick(self):
        self._poll_ldr()
 
        # ---- DAY MODE pre-emption: a press can interrupt mid-green ----
        if (not self.is_night and self.ped_requested and self._cooldown_elapsed()
                and self.state in ("tl4Green", "tl4Yellow",
                                    "tl5Green", "tl5Yellow")):
            self._enter_state("PED_WAIT")
            return

        elapsed = self._elapsed()

        # ---------------- Normal 20/10 (or 30/5 at night) cycle --------
        if self.state == "tl4Green":
            if elapsed >= self._tl4Green_time():
                self._enter_state("tl4Yellow")

        elif self.state == "tl4Yellow":
            if elapsed >= yellowTime:
                self._set_tl("TL4", "RED")
                if self._night_boundary_hit():
                    self.night_next_green = "TL5"
                    self._enter_state("nightRedHold_WALK")
                else:
                    self._enter_state("tl5Green")

        elif self.state == "tl5Green":
            if elapsed >= self._tl5Green_time():
                self._enter_state("tl5Yellow")

        elif self.state == "tl5Yellow":
            if elapsed >= yellowTime:
                self._set_tl("TL5", "RED")
                if self._night_boundary_hit():
                    self.night_next_green = "TL4"
                    self._enter_state("nightRedHold_WALK")
                else:
                    self._enter_state("tl4Green")

        # ---------------- Day-mode pedestrian sequence (Rule 1) --------
        elif self.state == "PED_WAIT":
            if elapsed >= pedWaitBefore:
                # "If TL5 is currently not red, stop TL5; otherwise stop TL4"
                self.ped_stop_target = "TL5" if self.tl5_colour != "RED" else "TL4"
                self._enter_state("PED_STOP_YELLOW")

        elif self.state == "PED_STOP_YELLOW":
            if elapsed >= yellowTime:
                self._enter_state("PED_WALK")

        elif self.state == "PED_WALK":
            if elapsed >= pedGreenTime:
                self._enter_state("PED_FLASH")

        elif self.state == "PED_FLASH":
            if elapsed >= pedFlashTime:
                self._pl_solid_red()
                self._complete_ped_sequence()   # Rule 2/4: 2.R1 has now run
                self._enter_state("tl4Green")  # Rule 1: "TL4 turns back to green"

        # ---------------- Night-mode pedestrian hold (Rule 5) ----------
        elif self.state == "nightRedHold_WALK":
            if elapsed >= pedGreenTime:
                self._enter_state("nightRedHold_FLASH")

        elif self.state == "nightRedHold_FLASH":
            if elapsed >= pedFlashTime:
                self._pl_solid_red()
                self._complete_ped_sequence()
                self._enter_state(f"{self.night_next_green}_GREEN")

    def _night_boundary_hit(self):
        """True exactly once, right when both TLs are red at night with a
        pending, un-cooled-down pedestrian request."""
        return self.is_night and self.ped_requested and self._cooldown_elapsed()

    def _poll_ldr(self):
        now = time.monotonic()
        if now - self.last_ldr_poll < ldrPollInterval:
            return
        self.last_ldr_poll = now
        # hysteresis band prevents flicker around dusk/dawn
        if self.last_ldr_value < ldrNightThreshold:
            self.is_night = True
        elif self.last_ldr_value > ldr:
            self.is_night = False

def main():
    board = pymata4.Pymata4()
    intersection = Intersection(board)
    try:
        while True:
            intersection.tick()
            time.sleep(0.02)   # ~50 Hz - fast enough for responsive buttons
    except KeyboardInterrupt:
        pass
    finally:
        board.shutdown()

if __name__ == "__main__":
    main()
