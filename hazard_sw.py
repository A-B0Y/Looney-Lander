#!/usr/bin/env python3
"""Hazard Vision: finds ANYTHING that is not smooth, empty ground in the camera picture, outlines each object with its true shape
(round things stay round), picks the safest round landing zone, and (optionally) drives the Arduino landing console.

Install:  py -m pip install opencv-python numpy pyserial
Run:      py hazard_sw.py --camera 1                        camera 1 (Camo), shown in a window
          py hazard_sw.py --camera 1 --arduino COM5          ALSO drive the Arduino landing console over USB (close the Serial Monitor first)
          py hazard_sw.py --image rocks.jpg                  process one picture and save out.png
          py hazard_sw.py --selftest                         no camera needed: checks the detector on made-up scenes

BEST RESULTS (do this once, takes 20 seconds):
   1. Y  then click the corners of your yard, then ENTER       everything outside the yard is ignored
   2. clear the yard completely, then press C                  teaches it what EMPTY ground looks like (12 frames)
   Both are remembered next time (yard.json, calib.npz). Press C again if the lighting or the camera moved.

Keys:     c calibrate   x forget calibration   y set the yard (click corners, ENTER)   u clear the yard
          m view: detection / clean video / heat map / mask      g cell grid + servo scale     space pause
          + / -  sensitivity (+ = fewer hazards)     j / k  pretend your hand is lower / higher over the sensor, 0 = stop pretending
          h hide the text    f full screen    s save a screenshot    q or Esc quit
"""
import argparse
import math
import re
import sys
import time

import cv2
import numpy as np

from detector import YardDetector

VIEWS = ["detection", "clean video", "heat map", "mask"]
GREEN, AMBER, RED, BLUE, WHITE = (140, 224, 90), (70, 195, 255), (80, 80, 255), (255, 190, 90), (245, 245, 245)
STATE_COL = {"SEARCH": AMBER, "LOCKED": GREEN, "DESCEND": GREEN, "TOUCHDOWN": GREEN, "ABORT": RED}
BANNERS = {"LOCKED": ("TARGET LOCKED", GREEN), "TOUCHDOWN": ("TOUCHDOWN", GREEN), "ABORT": ("ABORT - HAZARD AT TARGET", RED)}
FONT = cv2.FONT_HERSHEY_DUPLEX
_instance_socket = None


def single_instance():
    """Only one Hazard Vision at a time (two copies would fight over the camera and the Arduino)."""
    global _instance_socket
    import socket
    sk = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sk.bind(("127.0.0.1", 47831))
    except OSError:
        sys.exit("Another Hazard Vision is already running. Close its window (and its black window) first.")
    _instance_socket = sk


# ------------------------------------------------------------------ drawing helpers
def shade(view, x0, y0, x1, y1, alpha=0.6):
    h, w = view.shape[:2]
    x0, y0, x1, y1 = max(0, int(x0)), max(0, int(y0)), min(w, int(x1)), min(h, int(y1))
    if x1 > x0 and y1 > y0:
        roi = view[y0:y1, x0:x1]
        view[y0:y1, x0:x1] = cv2.addWeighted(roi, 1 - alpha, np.zeros_like(roi), alpha, 0)


def text_w(t, scale, thick=1):
    return cv2.getTextSize(t, FONT, scale, thick)[0][0]


def put(view, t, x, y, scale=0.55, col=WHITE, thick=1):
    cv2.putText(view, t, (int(x), int(y)), FONT, scale, col, thick, cv2.LINE_AA)


def dashed_circle(view, c, r, col, thick=1, dashes=36):
    for i in range(dashes):
        if i % 2 == 0:
            a0, a1 = 360.0 * i / dashes, 360.0 * (i + 1) / dashes
            cv2.ellipse(view, (int(c[0]), int(c[1])), (int(r), int(r)), 0, a0, a1, col, thick, cv2.LINE_AA)


def draw_objects(view, res, sx, sy, now):
    """Fill + outline every object with its TRUE shape: circles, ellipses, or a smooth contour. Never a box."""
    fill = view.copy()
    for o in res.objects:
        k, s = o["kind"], o["shape"]
        if k == "circle":
            cv2.circle(fill, (int(s[0] * sx), int(s[1] * sy)), int(s[2] * sx), RED, -1, cv2.LINE_AA)
        elif k == "ellipse":
            (cx, cy), (a, b), ang = s
            cv2.ellipse(fill, ((cx * sx, cy * sy), (a * sx, b * sy), ang), RED, -1, cv2.LINE_AA)
        else:
            cv2.fillPoly(fill, [(s * [sx, sy]).astype(np.int32)], RED, cv2.LINE_AA)
    cv2.addWeighted(fill, 0.34, view, 0.66, 0, view)
    for o in res.objects:
        k, s = o["kind"], o["shape"]
        if k == "circle":
            cv2.circle(view, (int(s[0] * sx), int(s[1] * sy)), int(s[2] * sx), (190, 190, 255), 2, cv2.LINE_AA)
        elif k == "ellipse":
            (cx, cy), (a, b), ang = s
            cv2.ellipse(view, ((cx * sx, cy * sy), (a * sx, b * sy), ang), (190, 190, 255), 2, cv2.LINE_AA)
        else:
            cv2.polylines(view, [(s * [sx, sy]).astype(np.int32)], True, (190, 190, 255), 2, cv2.LINE_AA)


def draw_landing(view, det, res, sx, sy, now, descending, state):
    tx, ty = res.target[0] * sx, res.target[1] * sy
    R = res.radius * sx
    secured = state in ("LOCKED", "DESCEND", "TOUCHDOWN") or res.secure_s >= 0.3
    col = RED if res.touched else (GREEN if (res.safe and secured) else AMBER)
    pulse = 0.5 + 0.5 * math.sin(now * (3 if secured else 9))
    if res.max_clearance > det.R and not descending:
        dashed_circle(view, (tx, ty), min(res.clearance, 4 * det.R) * sx, col, 1)       # how much empty space there is around it
    glow = view.copy()
    cv2.circle(glow, (int(tx), int(ty)), int(R), col, 9, cv2.LINE_AA)
    cv2.addWeighted(glow, 0.18 + 0.12 * pulse, view, 0.82 - 0.12 * pulse, 0, view)
    cv2.circle(view, (int(tx), int(ty)), int(R), col, 3, cv2.LINE_AA)
    spin = 0 if secured else now * 2.5                                                 # the ticks spin while it is still acquiring, then lock still
    for a in range(4):
        ca, sa = math.cos(a * math.pi / 2 + spin), math.sin(a * math.pi / 2 + spin)
        cv2.line(view, (int(tx + ca * (R - 4)), int(ty + sa * (R - 4))), (int(tx + ca * (R + 10)), int(ty + sa * (R + 10))), col, 3, cv2.LINE_AA)
    cv2.circle(view, (int(tx), int(ty)), 3, col, -1, cv2.LINE_AA)
    label = "BLOCKED" if res.touched else ("NO SAFE SITE" if not res.safe else ("SECURED" if secured else "ACQUIRING"))
    tw = text_w(label, 0.5)
    ly = ty - R - 16 if ty - R - 16 > 80 else ty + R + 30
    shade(view, tx - tw / 2 - 8, ly - 17, tx + tw / 2 + 8, ly + 7, 0.7)
    put(view, label, tx - tw / 2, ly, 0.5, col)


def draw_yard(view, det, sx, sy, editing_pts=None):
    if det.roi_poly is not None:
        pts = (np.array(det.roi_poly) * [det.w * sx, det.h * sy]).astype(np.int32)
        mask = np.zeros(view.shape[:2], np.uint8)
        cv2.fillPoly(mask, [pts], 255)
        dim = (view * 0.45).astype(np.uint8)
        view[mask == 0] = dim[mask == 0]
        cv2.polylines(view, [pts], True, (255, 255, 255), 1, cv2.LINE_AA)
    if editing_pts:
        p = np.array([[x * det.w * sx, y * det.h * sy] for x, y in editing_pts], np.int32)
        if len(p) > 1:
            cv2.polylines(view, [p], len(p) >= 3, BLUE, 2, cv2.LINE_AA)
        for x, y in p:
            cv2.circle(view, (int(x), int(y)), 6, BLUE, -1, cv2.LINE_AA)


def draw_grid(view, det, res, sx, sy):
    h, w = view.shape[:2]
    g = view.copy()
    for c in range(1, det.cols):
        cv2.line(g, (int(c * det.cw * sx), 0), (int(c * det.cw * sx), h), (255, 255, 255), 1)
    for r in range(1, det.rows):
        cv2.line(g, (0, int(r * det.ch * sy)), (w, int(r * det.ch * sy)), (255, 255, 255), 1)
    cv2.addWeighted(g, 0.14, view, 0.86, 0, view)
    x0, y0 = int(res.col * det.cw * sx), int(res.row * det.ch * sy)
    cv2.rectangle(view, (x0, y0), (int(x0 + det.cw * sx), int(y0 + det.ch * sy)), WHITE, 1, cv2.LINE_AA)
    # the servo scale: which column the Arduino arrow points at
    bx0, bx1, by = 60, w - 60, h - 52
    shade(view, bx0 - 10, by - 14, bx1 + 10, by + 10, 0.5)
    for c in range(det.cols):
        x = bx0 + (bx1 - bx0) * (c + 0.5) / det.cols
        cv2.line(view, (int(x), by - 4), (int(x), by + 4), (200, 200, 200), 1, cv2.LINE_AA)
    mx = bx0 + (bx1 - bx0) * (res.col + 0.5) / det.cols
    cv2.fillPoly(view, [np.array([[mx, by - 12], [mx - 7, by - 2], [mx + 7, by - 2]], np.int32)], GREEN, cv2.LINE_AA)


def draw_hud(view, det, res, state, alt, fps, note, banner, now, show_text, view_name, paused, signal_lost=False):
    h, w = view.shape[:2]
    if state == "ABORT":
        a = 0.35 + 0.35 * (int(now * 6) % 2 == 0)
        fr = view.copy()
        cv2.rectangle(fr, (0, 0), (w - 1, h - 1), RED, 24)
        cv2.addWeighted(fr, a, view, 1 - a, 0, view)
    elif state == "TOUCHDOWN":
        fr = view.copy()
        cv2.rectangle(fr, (0, 0), (w - 1, h - 1), GREEN, 18)
        cv2.addWeighted(fr, 0.45, view, 0.55, 0, view)
    if not show_text:
        return
    # ---- top bar
    shade(view, 0, 0, w, 58, 0.62)
    col = STATE_COL.get(state, (200, 200, 200))
    cv2.circle(view, (30, 29), 11, col, -1, cv2.LINE_AA)
    put(view, state or "VISION ONLY", 52, 40, 1.0, col, 2)
    if res.calibrating:
        chip, ccol = ("AUTO-CALIBRATING" if det.calibrating_auto else "CALIBRATING") + f" {res.calibrating[0]}/{res.calibrating[1]}", AMBER
    elif det.calibrated:
        chip, ccol = "CALIBRATED", GREEN
    else:
        chip, ccol = ("COULD NOT CALIBRATE - too much of the yard is covered" if det.cal_failed else "NOT CALIBRATED"), AMBER
    tw = text_w(chip, 0.55, 1)
    cv2.rectangle(view, (w // 2 - tw // 2 - 14, 15), (w // 2 + tw // 2 + 14, 44), ccol, 1, cv2.LINE_AA)
    put(view, chip, w // 2 - tw // 2, 35, 0.55, ccol)
    if alt is not None:
        t = f"ALT {alt} cm"
        put(view, t, w - text_w(t, 0.9, 2) - 22, 40, 0.9, BLUE, 2)
        top, bot = 86, h - 120
        shade(view, w - 46, top - 8, w - 14, bot + 8, 0.55)
        cv2.rectangle(view, (w - 38, top), (w - 22, bot), (200, 200, 200), 1, cv2.LINE_AA)
        y = int(bot - max(0.0, min(1.0, alt / 36.0)) * (bot - top))
        cv2.rectangle(view, (w - 37, y), (w - 23, bot - 1), BLUE, -1)
        for cm, name in ((28, "START"), (7, "TOUCH")):
            yy = int(bot - cm / 36.0 * (bot - top))
            cv2.line(view, (w - 46, yy), (w - 38, yy), WHITE, 2, cv2.LINE_AA)
            put(view, name, w - 108, yy + 5, 0.4)
    # ---- bottom bar
    shade(view, 0, h - 40, w, h, 0.65)
    cov = res.coverage * 100
    stats = [("OBJECTS", f"{len(res.objects)}"), ("COVERAGE", f"{cov:.0f}%"), ("CLEAR", f"{res.clearance / det.R:.1f}x" if res.safe else "none"), ("FPS", f"{fps:.0f}"), ("VIEW", view_name.upper())]
    x = 16
    for name, val in stats:
        put(view, name, x, h - 14, 0.38, (170, 170, 170))
        put(view, val, x + text_w(name, 0.38) + 8, h - 14, 0.55, WHITE)
        x += text_w(name, 0.38) + text_w(val, 0.55) + 34
    q = res.quality
    qx = x + 6
    put(view, "SITE", qx, h - 14, 0.38, (170, 170, 170))
    cv2.rectangle(view, (qx + 40, h - 25), (qx + 140, h - 13), (170, 170, 170), 1)
    cv2.rectangle(view, (qx + 41, h - 24), (qx + 41 + int(98 * q), h - 14), GREEN if q > 0.5 else AMBER if q > 0.15 else RED, -1)
    hint = "C re-calibrate   Y yard   M view   G grid   H hide"
    put(view, hint, w - text_w(hint, 0.4) - 70, h - 14, 0.4, (150, 150, 150))
    if paused:
        put(view, "PAUSED", w // 2 - 40, 92, 0.8, AMBER, 2)
    if signal_lost:
        t = "CAMERA SIGNAL LOST - waiting for it to come back"
        shade(view, w // 2 - text_w(t, 0.7) // 2 - 16, h // 2 - 24, w // 2 + text_w(t, 0.7) // 2 + 16, h // 2 + 16, 0.8)
        put(view, t, w // 2 - text_w(t, 0.7) // 2, h // 2 + 4, 0.7, RED)
    if res.calibrating:
        k, n = res.calibrating
        bw, cy = 360, h - 150                                          # low on the screen so it does not cover the yard
        shade(view, w // 2 - bw // 2 - 24, cy - 40, w // 2 + bw // 2 + 24, cy + 36, 0.75)
        t = "Learning the empty ground. Objects already there stay objects."
        put(view, t, w // 2 - text_w(t, 0.5) // 2, cy - 12, 0.5, WHITE)
        cv2.rectangle(view, (w // 2 - bw // 2, cy + 4), (w // 2 + bw // 2, cy + 20), (200, 200, 200), 1)
        cv2.rectangle(view, (w // 2 - bw // 2 + 1, cy + 5), (w // 2 - bw // 2 + int((bw - 2) * k / n), cy + 19), AMBER, -1)
    if banner:
        text, bcol = banner
        tw = text_w(text, 1.3, 2)
        shade(view, w // 2 - tw // 2 - 24, 74, w // 2 + tw // 2 + 24, 138, 0.7)
        put(view, text, w // 2 - tw // 2, 120, 1.3, bcol, 2)
    if note:
        tw = text_w(note, 0.65, 1)
        shade(view, w // 2 - tw // 2 - 14, 150, w // 2 + tw // 2 + 14, 184, 0.65)
        put(view, note, w // 2 - tw // 2, 174, 0.65, WHITE)


def render(view, det, res, sx, sy, now, view_idx, grid, editing_pts, descending, state=""):
    """Everything drawn on the picture except the text bars."""
    if view_idx == 3:                                                  # mask only
        view[:] = 0
        m = cv2.resize(res.hazard.astype(np.uint8) * 255, (view.shape[1], view.shape[0]), interpolation=cv2.INTER_NEAREST)
        view[m > 0] = (210, 210, 210)
    elif view_idx == 2:                                                # heat map of how un-ground-like each pixel is
        sc = np.clip(res.score / 2.0, 0, 1)
        heat = cv2.applyColorMap((cv2.resize(sc, (view.shape[1], view.shape[0])) * 255).astype(np.uint8), cv2.COLORMAP_INFERNO)
        cv2.addWeighted(heat, 0.65, view, 0.35, 0, view)
    draw_yard(view, det, sx, sy, editing_pts)
    if view_idx != 1:
        draw_objects(view, res, sx, sy, now)
        if grid:
            draw_grid(view, det, res, sx, sy)
        draw_landing(view, det, res, sx, sy, now, descending, state)


# ------------------------------------------------------------------ self test
def selftest():
    rng = np.random.default_rng(5)
    W, H = 640, 360
    yy, xx = np.mgrid[0:H, 0:W]
    light = (1 + 0.08 * (xx / W - 0.5) - 0.06 * (yy / H - 0.5))[..., None]
    grain = cv2.GaussianBlur(rng.normal(0, 3, (H, W, 3)).astype(np.float32), (7, 7), 0)

    def ground(seed):
        g = np.random.default_rng(seed)
        return np.clip(np.array([150, 135, 110], np.float32) * light + grain + g.normal(0, 2, (H, W, 3)), 0, 255).astype(np.uint8)

    det = YardDetector(W, H, calib_file="selftest_calib.npz", yard_file="selftest_yard.json")
    det.forget_calibration()
    det.clear_yard()
    det.start_calibration(12)
    for i in range(20):
        det.analyse(ground(100 + i))
    for f in ("selftest_calib.npz", "selftest_yard.json"):
        try:
            import os
            os.remove(f)
        except OSError:
            pass
    empty = None
    for _ in range(5):
        empty = det.analyse(ground(7))
    assert empty.coverage < 0.003 and len(empty.objects) == 0, "an empty calibrated yard must show nothing"
    assert empty.safe and not (empty.col <= 2 and empty.row <= 2), "the landing zone must not sit in a corner"
    img = ground(8)
    m = ((xx - 430) ** 2 + (yy - 110) ** 2) <= 24 ** 2
    img[m] = (200, 60, 55)                                             # a smooth round ball
    img[220:262, 90:150] = (118, 116, 110)                             # a smooth grey box with almost no texture
    img[250:290, 480:540] = rng.integers(60, 200, (40, 60, 3))         # a rough rock
    t0 = time.time()
    res = None
    for _ in range(20):
        res = det.analyse(img)
    ms = (time.time() - t0) / 20 * 1000
    kinds = sorted(o["kind"] for o in res.objects)
    assert len(res.objects) == 3, f"expected 3 objects, found {len(res.objects)}"
    ball = [o for o in res.objects if abs(o["centre"][0] - 430) < 15]
    assert ball and ball[0]["kind"] == "circle", "the ball must be outlined as a circle"
    assert res.safe and not res.touched, "there is still a safe landing zone"
    view = cv2.resize(cv2.cvtColor(img, cv2.COLOR_RGB2BGR), (1280, 720))
    render(view, det, res, 2.0, 2.0, time.time(), 0, True, None, False, "LOCKED")
    draw_hud(view, det, res, "DESCEND", 18, 25.0, "note", ("TARGET LOCKED", GREEN), time.time(), True, "detection", False, True)
    print(f"self-test OK: 3 objects ({', '.join(kinds)}), landing zone column {res.col + 1} row {res.row + 1}, {ms:.1f} ms per frame at 640x360")


# ------------------------------------------------------------------ main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", type=int, default=0, help="which camera: run list_cameras.py to find the number for Camo")
    ap.add_argument("--dshow", action="store_true", help="Windows: use the DirectShow driver (try it if Camo will not open or looks laggy)")
    ap.add_argument("--proc", default="640x360", help="processing size (default 640x360; 960x540 is more precise but slower)")
    ap.add_argument("--show", default="1280x720", help="size of the picture on screen")
    ap.add_argument("--fullscreen", action="store_true")
    ap.add_argument("--image", default=None)
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--sens", type=int, default=6)
    ap.add_argument("--arduino", default=None, help="serial port of the Arduino landing console, e.g. COM5 or /dev/ttyACM0")
    ap.add_argument("--log", default=None, help="record the run to this .csv file (for mission_report.py)")
    a = ap.parse_args()
    if a.selftest:
        selftest()
        return
    pw, ph = (int(v) for v in a.proc.lower().split("x"))
    sw_, sh_ = (int(v) for v in a.show.lower().split("x"))
    sx, sy = sw_ / pw, sh_ / ph
    sens = a.sens
    det = YardDetector(pw, ph)

    if a.image:
        bgr = cv2.imread(a.image)
        if bgr is None:
            sys.exit(f"could not read {a.image}")
        rgb = cv2.cvtColor(cv2.resize(bgr, (pw, ph)), cv2.COLOR_BGR2RGB)
        res = None
        for _ in range(5):
            res = det.analyse(rgb, sens)
        view = cv2.resize(bgr, (sw_, sh_))
        render(view, det, res, sx, sy, time.time(), 0, False, None, False)
        cv2.imwrite("out.png", view)
        print(f"saved out.png  objects: {len(res.objects)}  landing zone: column {res.col + 1} row {res.row + 1}")
        return

    single_instance()
    cap = cv2.VideoCapture(a.camera, cv2.CAP_DSHOW) if a.dshow else cv2.VideoCapture(a.camera)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    if not cap.isOpened():
        sys.exit(f"could not open camera {a.camera}. Try --camera 1, and close other programs that use the webcam.")
    win = "Hazard Vision"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, sw_, sh_)
    fs = a.fullscreen
    if fs:
        cv2.setWindowProperty(win, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    edit = {"on": False, "pts": []}

    def on_mouse(event, x, y, flags, param):                           # clicking the corners of the yard
        if event == cv2.EVENT_LBUTTONDOWN and edit["on"] and len(edit["pts"]) < 8:
            wx, wy = cv2.getWindowImageRect(win)[2:4] if hasattr(cv2, "getWindowImageRect") else (sw_, sh_)
            wx, wy = (wx or sw_), (wy or sh_)
            edit["pts"].append((min(1.0, max(0.0, x / wx)), min(1.0, max(0.0, y / wy))))
    cv2.setMouseCallback(win, on_mouse)

    ser, last_send = None, 0.0
    ser_ready_at, last_reconnect, arduino_lost = 0.0, 0.0, False
    last_idx, moves, last_print = None, 0, 0.0
    rxbuf, ar_status, fake_alt, last_fake, note, note_until = b"", "", None, 0.0, "", 0.0
    t_start, last_log, csv_file, last_flush = time.time(), 0.0, None, time.time()
    if a.log:
        csv_file = open(a.log, "w", newline="")
        csv_file.write("t,fps,hazards,safe_col,safe_row,moves,arduino\n")
    serial_mod = None
    if a.arduino:
        import serial as serial_mod
        try:
            ser = serial_mod.Serial(a.arduino, 9600, timeout=0)
            print(f"opened {a.arduino}; waiting 2 s for the Arduino to restart...")
            time.sleep(2.0)
        except Exception as e:
            ser = None
            print(f"could not open {a.arduino} ({e}). Close the Arduino Serial Monitor and check the port in settings.bat. Still trying every 2 seconds.")
    if det.calibrated:
        note, note_until = "saved calibration loaded (press C to redo it)", time.time() + 4
    else:
        det.start_calibration(12, auto=True)                           # first run: learn the ground by itself, objects already there stay objects
        note, note_until = "auto-calibrating: hold still for a moment", time.time() + 4
    view_idx, grid, show_text, paused = 0, False, True, False
    prev_state, state_since, fps, last = "", time.time(), 0.0, time.time()
    bgr, res = None, None
    bad_reads, high_cov_since, frames_seen = 0, None, 0
    while True:
        if not paused or bgr is None:
            ok, frame = cap.read()
            if ok:
                bgr, bad_reads = frame, 0
            else:                                                      # Camo hiccup / cable wiggle: keep the last picture and keep trying
                bad_reads += 1
                time.sleep(0.03)
                if bgr is None or bad_reads > 400:                     # about 12 s without a picture: give up
                    print("camera stopped sending pictures")
                    break
        frames_seen += 1
        now = time.time()
        rgb = cv2.cvtColor(cv2.resize(bgr, (pw, ph)), cv2.COLOR_BGR2RGB)
        state_word = ar_status.split(" ")[0] if ar_status else ""
        if (not paused or res is None) and bad_reads == 0:
            res = det.analyse(rgb, sens, hold=(state_word == "DESCEND"))
        elif res is None:
            res = det.analyse(rgb, sens)
        if last_idx is not None and res.idx != last_idx:
            moves += 1
        last_idx = res.idx
        # ---------------- the whole scene changed (camera bumped, lighting changed a lot): re-learn the ground by itself
        if res.coverage > 0.6 and not res.calibrating:
            high_cov_since = high_cov_since or now
            if now - high_cov_since > 2.5:
                det.start_calibration(12, auto=True)
                note, note_until = "the scene changed a lot: re-calibrating", now + 4
                high_cov_since = None
        else:
            high_cov_since = None
        # ---------------- Arduino link (survives a pulled USB cable)
        if a.arduino and ser is None and now - last_reconnect > 2.0:
            last_reconnect = now
            try:
                ser = serial_mod.Serial(a.arduino, 9600, timeout=0)
                ser_ready_at, arduino_lost, rxbuf = now + 2.0, False, b""
                note, note_until = "Arduino reconnected", now + 3
            except Exception:
                pass
        if ser is not None and now >= ser_ready_at:
            try:
                if now - last_send >= 0.1:                             # Z<target cell>,<hazard count>,<hazard map>
                    last_send = now
                    mask = 0
                    for i, v in enumerate(res.cells.reshape(-1)):
                        if v:
                            mask |= 1 << i
                    ser.write(("Z%02X,%02X,%036X\r\n" % (res.idx & 0xFF, min(res.count, 255), mask)).encode())
                    if now - last_print >= 1.0:
                        last_print = now
                        print(f"sent to Arduino: objects {len(res.objects)}, landing column {res.col + 1} row {res.row + 1}"
                              f"{'' if res.safe else ' (NO SAFE SITE)'}, moves {moves}  |  Arduino: {ar_status or '(nothing yet)'}")
                if fake_alt is not None and now - last_fake >= 0.4:    # pretend hand height for testing without the distance sensor
                    last_fake = now
                    ser.write(("D%d\n" % fake_alt).encode())
                n_in = ser.in_waiting                                  # what the Arduino says back
                if n_in:
                    rxbuf += ser.read(n_in)
                    while b"\n" in rxbuf:
                        line, rxbuf = rxbuf.split(b"\n", 1)
                        text = line.decode(errors="ignore").strip()
                        if text.startswith("ST "):
                            ar_status = text[3:]
                if len(rxbuf) > 4096:
                    rxbuf = b""
            except Exception as e:                                     # cable pulled / board reset: show it and keep reconnecting
                print("Arduino connection lost:", e)
                try:
                    ser.close()
                except Exception:
                    pass
                ser, ar_status, arduino_lost, last_reconnect = None, "", True, now
        state_word = ar_status.split(" ")[0] if ar_status else ""
        m_alt = re.search(r"alt=(\d+)", ar_status)
        alt_now = int(m_alt.group(1)) if m_alt else None
        if state_word and state_word != prev_state:
            state_since = now
            prev_state = state_word
        # ---------------- the picture
        view = cv2.resize(bgr, (sw_, sh_))
        render(view, det, res, sx, sy, now, view_idx, grid, edit["pts"] if edit["on"] else None, state_word == "DESCEND", state_word)
        if edit["on"]:
            note, note_until = f"Click the corners of the yard ({len(edit['pts'])} so far), then press ENTER. ESC cancels.", now + 0.5
        shown_alt = fake_alt if fake_alt is not None else alt_now
        banner = BANNERS.get(state_word) if (state_word and now - state_since < 2.5) else None
        fps = 0.9 * fps + 0.1 / max(1e-6, now - last)
        last = now
        hud_state = state_word if ser is not None else ("ARDUINO OFFLINE" if a.arduino else "")
        draw_hud(view, det, res, hud_state, shown_alt, fps, note if now < note_until else "", banner, now, show_text,
                 VIEWS[view_idx], paused, bad_reads > 5)
        cv2.imshow(win, view)
        if frames_seen > 30:
            try:
                if cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:          # the window was closed with the X button
                    break
            except cv2.error:
                pass
        if csv_file is not None and now - last_flush >= 2.0:
            csv_file.flush()
            last_flush = now
        if csv_file is not None and now - last_log >= 0.2:
            last_log = now
            csv_file.write("%.2f,%.1f,%d,%d,%d,%d,%s\n" % (now - t_start, fps, int(res.cells.sum()), res.col + 1, res.row + 1, moves, ar_status.replace(",", ";")))
        k = cv2.waitKey(1) & 0xFF
        if edit["on"]:
            if k == 13 or k == ord("y"):
                if len(edit["pts"]) >= 3:
                    det.set_yard(edit["pts"])
                    note, note_until = "yard saved: everything outside it is ignored. Now clear it and press C.", now + 5
                else:
                    note, note_until = "need at least 3 corners", now + 3
                edit["on"] = False
            elif k == 27:
                edit["on"], note, note_until = False, "yard setup cancelled", now + 2
            continue
        if k in (ord("q"), 27):
            break
        elif k in (ord("+"), ord("=")):
            sens = min(15, sens + 1)
            note, note_until = f"sensitivity {sens} (higher = fewer hazards)", now + 2
        elif k in (ord("-"), ord("_")):
            sens = max(0, sens - 1)
            note, note_until = f"sensitivity {sens} (lower = more hazards)", now + 2
        elif k == ord("m"):
            view_idx = (view_idx + 1) % len(VIEWS)
        elif k == ord("g"):
            grid = not grid
        elif k == ord("c"):
            det.start_calibration(12)
            note, note_until = "calibrating: keep the yard EMPTY", now + 3
        elif k == ord("x"):
            det.forget_calibration()
            note, note_until = "calibration forgotten", now + 3
        elif k == ord("y"):
            edit["on"], edit["pts"] = True, []
        elif k == ord("u"):
            det.clear_yard()
            note, note_until = "yard cleared: the whole picture is used", now + 3
        elif k == ord(" "):
            paused = not paused
        elif k == ord("j"):
            fake_alt = max(3, (35 if fake_alt is None else fake_alt) - 2)
        elif k == ord("k"):
            fake_alt = min(36, (35 if fake_alt is None else fake_alt) + 2)
        elif k == ord("0"):
            fake_alt = None
        elif k == ord("h"):
            show_text = not show_text
        elif k == ord("s"):
            cv2.imwrite(f"hazard_{int(now)}.png", view)
            note, note_until = "screenshot saved", now + 2
        elif k == ord("f"):
            fs = not fs
            cv2.setWindowProperty(win, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN if fs else cv2.WINDOW_NORMAL)
    cap.release()
    if csv_file is not None:
        csv_file.close()
        print("mission log saved to", a.log)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
