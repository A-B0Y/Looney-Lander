#!/usr/bin/env python3
"""Distance sensor check for the landing console: keeps the Arduino "awake" with fake terrain data and prints the hand height
it measures, so you can see the HC-SR04 working.
Before you run it:  upload landing_console.ino and CLOSE the Serial Monitor.
Run:   py sensor_check.py COM5
Then wave your hand above the sensor (5 to 35 cm). After about 1.5 seconds the Arduino locks on and prints lines like
   LOCKED target=1,1 alt=22        <- alt is your hand height in cm
If you never see "alt=" the sensor is not wired right (VCC 5V, GND, TRIG D12, ECHO D13) or it is not pointing up at your hand.
Stop with Ctrl+C.
"""
import sys
import time

import serial

if len(sys.argv) < 2:
    sys.exit("Usage: py sensor_check.py COM5")
try:
    ser = serial.Serial(sys.argv[1], 9600, timeout=0)
except serial.SerialException as e:
    sys.exit(f"Could not open {sys.argv[1]}: {e}\nClose the Serial Monitor and check the port name.")
print("opened", sys.argv[1], "- waiting 2 s for the Arduino to restart...")
time.sleep(2.0)
buf = b""
seen_alt = False
t0 = time.time()
warned = False
try:
    while True:
        ser.write(("Z11,05,%s\r\n" % ("0" * 36)).encode())
        n = ser.in_waiting
        if n:
            buf += ser.read(n)
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            text = line.decode(errors="ignore").strip()
            if text.startswith("ST "):
                print("  ", text[3:])
                if "alt=" in text:
                    seen_alt = True
        if not seen_alt and not warned and time.time() - t0 > 8:
            warned = True
            print("\n*** No hand height yet. Hold your hand 10-30 cm above the sensor and wait. If there is still no 'alt=', check the wiring:")
            print("    VCC -> 5V, GND -> GND, TRIG -> D12, ECHO -> D13, sensor pointing UP.\n")
        time.sleep(0.1)
except KeyboardInterrupt:
    print("\nstopped")
