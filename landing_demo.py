#!/usr/bin/env python3
"""Landing console self-test: runs a made-up landing through the Arduino with NO camera and NO distance sensor.
It sends fake terrain results and fake hand heights, and prints what the Arduino answers.
You should see (and hear) this story:
   1. SEARCH  (yellow blinks)           2. LOCKED  (green steady, double chirp)      3. DESCEND (green blinks, buzzer ticks faster and faster)
   4. TOUCHDOWN (green + yellow, jingle)   5. a second attempt where a ROCK lands on the target -> ABORT (red, alarm)
Before you run it:  upload landing_console.ino and CLOSE the Serial Monitor.
Run:   py landing_demo.py COM5            (your Arduino's port; Mac/Linux: python3 landing_demo.py /dev/ttyACM0)
"""
import sys
import time

import serial

if len(sys.argv) < 2:
    sys.exit("Usage: py landing_demo.py COM5")
try:
    ser = serial.Serial(sys.argv[1], 9600, timeout=0)
except serial.SerialException as e:
    sys.exit(f"Could not open {sys.argv[1]}: {e}\nClose the Serial Monitor and check the port name.")
print("opened", sys.argv[1], "- waiting 2 s for the Arduino to restart...")
time.sleep(2.0)
CELL = 0x11                                   # the safe zone: column 1, row 1
CLEAR = "0" * 36
ROCK_ON_TARGET = "0" * 31 + "2" + "0" * 4       # cell 17 becomes a hazard
buf = b""
heard = False


def listen():
    global buf, heard
    n = ser.in_waiting
    if n:
        buf += ser.read(n)
    while b"\n" in buf:
        line, buf = buf.split(b"\n", 1)
        text = line.decode(errors="ignore").strip()
        if text.startswith("ST "):
            heard = True
            print("   Arduino:", text[3:])


def phase(title, seconds, hazards, mask, hand=None):
    print(f"\n== {title}")
    t_end = time.time() + seconds
    last_d = 0.0
    while time.time() < t_end:
        ser.write(("Z%02X,%02X,%s\r\n" % (CELL, hazards, mask)).encode())
        if hand is not None and time.time() - last_d > 0.5:
            last_d = time.time()
            ser.write(("D%d\n" % hand).encode())
        listen()
        time.sleep(0.1)


try:
    phase("1. Smooth ground, safe zone steady: the console searches, then locks on", 2.6, 5, CLEAR)
    phase("2. Hand lowered to 25 cm: DESCEND begins", 2.0, 5, CLEAR, 25)
    phase("3. Hand at 12 cm: ticks get faster", 2.0, 5, CLEAR, 12)
    phase("4. Hand at 6 cm: TOUCHDOWN (fanfare + celebration lights)", 5.0, 5, CLEAR, 6)
    phase("5. Hand raised to 33 cm: console re-arms", 4.5, 5, CLEAR, 33)
    phase("6. Second attempt: descend again", 2.0, 5, CLEAR, 18)
    phase("7. A ROCK lands on the target while descending: ABORT (siren)", 3.5, 6, ROCK_ON_TARGET, 14)
    phase("8. Rock removed, hand raised: back to searching", 3.0, 5, CLEAR, 33)
    print("\nDone.")
    if not heard:
        print("*** The Arduino never answered. Is landing_console.ino (the newest) uploaded, the Serial Monitor closed, and the port right?")
except KeyboardInterrupt:
    print("\nstopped")
