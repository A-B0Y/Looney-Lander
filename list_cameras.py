#!/usr/bin/env python3
"""Finds which camera number is which, so you know what to pass to --camera (for example, the Camo phone camera).
It tries camera numbers 0 to 5, prints the ones that give a picture, and saves a snapshot of each as camera_<n>.jpg.
Open the snapshots to see which one is your phone.

Run:  py list_cameras.py          (Mac / Linux: python3 list_cameras.py)
Needs Camo Studio running and showing your phone's picture. Close other programs that use the camera first.
"""
import sys

import cv2

backends = [("default", None)]
if sys.platform.startswith("win"):
    backends.append(("dshow", cv2.CAP_DSHOW))
found = False
for name, api in backends:
    for i in range(6):
        cap = cv2.VideoCapture(i) if api is None else cv2.VideoCapture(i, api)
        if not cap.isOpened():
            cap.release()
            continue
        ok, frame = cap.read()
        cap.release()
        if ok and frame is not None:
            found = True
            fn = f"camera_{i}.jpg" if name == "default" else f"camera_{i}_dshow.jpg"
            cv2.imwrite(fn, frame)
            h, w = frame.shape[:2]
            print(f"camera {i} ({name} driver): picture {w}x{h}, saved {fn}")
if not found:
    print("No camera gave a picture. Is Camo Studio open and showing your phone? Close other apps that use the camera.")
else:
    print("Open the saved .jpg files; the one that shows your phone's view is the number to use: --camera NUMBER")
    print("If a camera only appears with the dshow driver, add --dshow when you run camera_head.py.")
