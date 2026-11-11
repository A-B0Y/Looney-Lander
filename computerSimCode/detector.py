"""Yard detector: finds ANYTHING that is not smooth, empty ground, at pixel level, and picks the best round landing zone.

How it works (every frame):
  1. colour (Lab), edges and local texture are measured for every pixel
  2. each is compared with what the EMPTY yard looked like when you pressed  c  (a calibration of 12 frames: per-pixel average and noise).
     Without a calibration it assumes the most common calm colour is the ground. Exposure and white-balance drift are measured on the
     ground itself and removed, so a camera that changes brightness does not cause false alarms.
  3. a pixel is a hazard if it differs in colour, or has more edge / texture than the empty yard, by more than the noise allows
     (a strong hit, plus everything weakly connected to it, so whole objects are found, not just their outlines)
  4. the mask is cleaned (gaps closed, specks removed, holes filled) and smoothed over a few frames: a new object shows after 2 frames,
     and only disappears after 3 calm frames, so nothing flickers
  5. every object is outlined with its TRUE shape: round things become exact circles or ellipses, other things get a smooth contour
  6. the landing zone is the centre of the largest empty circle in the yard (distance transform). It stays put unless a clearly better
     place appears, and it is never closer to an object or to the edge of the yard than its own radius
  7. the 16 x 9 cell map that the Arduino expects is made from the pixel mask, and the landing cell is flagged the moment ANY object touches
     the landing circle, so the abort works for objects that only partly overlap
"""
import json
import os
import time

import cv2
import numpy as np

K5 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
K3 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))


class Result:
    pass


BLUR_K, BLUR_S, TEX_W = 5, 1.0, 5


def smooth_contour(pts, k=7):
    """Round off a contour by averaging neighbouring points (circular moving average)."""
    n = len(pts)
    if n < k * 2:
        return pts.astype(np.float32)
    ker = np.ones(k, np.float32) / k
    p = pts.astype(np.float32)
    ext = np.concatenate([p[-k:], p, p[:k]])
    x = np.convolve(ext[:, 0], ker, mode="same")[k:-k]
    y = np.convolve(ext[:, 1], ker, mode="same")[k:-k]
    return np.stack([x, y], 1)


class YardDetector:
    def __init__(self, w=640, h=360, cols=16, rows=9, calib_file="calib.npz", yard_file="yard.json"):
        self.w, self.h, self.cols, self.rows = w, h, cols, rows
        self.cw, self.ch = w / cols, h / rows
        self.R = 0.42 * min(self.cw, self.ch)                         # radius of the landing circle in pixels
        self.min_area = max(30, int(0.00025 * w * h))
        self.calib_file, self.yard_file = calib_file, yard_file
        self.roi = np.full((h, w), 255, np.uint8)
        self.roi_poly = None
        self._inner = np.zeros((h, w), np.uint8)
        self._inner[int(self.ch):int(h - self.ch), int(self.cw):int(w - self.cw)] = 255     # the outer ring of cells is never a landing site
        self.ref = None                                               # calibration of the empty yard
        self.cal = None                                               # calibration in progress
        self.mem = np.zeros((h, w), np.float32)
        self.haz = np.zeros((h, w), bool)
        self._flash = 0
        self.raw_coverage = 0.0
        self._sec_pos, self._sec_t = None, 0.0
        self.target = None
        self.cell = None
        self.cal_failed = False
        self._yy, self._xx = np.mgrid[0:h, 0:w]
        self.load_yard()
        self.load_calibration()

    # ------------------------------------------------------------------ the yard (region of interest)
    def set_yard(self, pts_norm):
        """pts_norm: list of (x, y) in 0..1. Everything outside the polygon is ignored."""
        poly = np.array([[x * self.w, y * self.h] for x, y in pts_norm], np.int32)
        m = np.zeros((self.h, self.w), np.uint8)
        cv2.fillPoly(m, [poly], 255)
        self.roi, self.roi_poly = m, [list(map(float, p)) for p in pts_norm]
        self.target = None
        try:
            json.dump({"poly": self.roi_poly}, open(self.yard_file, "w"))
        except OSError:
            pass

    def clear_yard(self):
        self.roi = np.full((self.h, self.w), 255, np.uint8)
        self.roi_poly = None
        self.target = None
        try:
            if os.path.exists(self.yard_file):
                os.remove(self.yard_file)
        except OSError:
            pass

    def load_yard(self):
        try:
            if os.path.exists(self.yard_file):
                p = json.load(open(self.yard_file))["poly"]
                if len(p) >= 3:
                    self.set_yard(p)
        except (OSError, ValueError, KeyError):
            pass

    # ------------------------------------------------------------------ calibration
    def start_calibration(self, n=12, auto=False):
        """Learn what the EMPTY yard looks like. Objects that are already on the yard are recognised (by the plain detector) and are NOT
        learned as ground, so they stay detected even if you forgot to clear the yard."""
        self.ref = None                                                # while learning, the plain detector decides what is an object
        self.mem[:] = 0
        self.haz[:] = False
        self._flash = 0
        self.cal_failed = False
        self.cal = {"n": n, "k": 0, "warm": 3, "auto": auto, "s": np.zeros((self.h, self.w, 3), np.float32), "ss": np.zeros((self.h, self.w, 3), np.float32),
                    "g": np.zeros((self.h, self.w), np.float32), "gg": np.zeros((self.h, self.w), np.float32),
                    "t": np.zeros((self.h, self.w), np.float32), "tt": np.zeros((self.h, self.w), np.float32),
                    "cnt": np.zeros((self.h, self.w), np.float32)}

    def forget_calibration(self):
        self.ref = None
        self.cal = None
        try:
            if os.path.exists(self.calib_file):
                os.remove(self.calib_file)
        except OSError:
            pass

    def _cal_feed(self, lab, grad, tex):
        c = self.cal
        if c["warm"] > 0:                                              # let the detector settle on the first frames
            c["warm"] -= 1
            return
        valid = ((cv2.dilate(self.haz.astype(np.uint8), K5, iterations=2) == 0) & (self.roi > 0)).astype(np.float32)   # not on or next to an object
        v3 = valid[..., None]
        c["s"] += lab * v3
        c["ss"] += lab * lab * v3
        c["g"] += grad * valid
        c["gg"] += grad * grad * valid
        c["t"] += tex * valid
        c["tt"] += tex * tex * valid
        c["cnt"] += valid
        c["k"] += 1
        if c["k"] >= c["n"]:
            self._finish_calibration()

    def _finish_calibration(self):
        c = self.cal
        n = float(c["n"])
        cnt = np.maximum(c["cnt"], 1.0)
        good = (c["cnt"] >= max(3.0, n / 2)) & (self.roi > 0)                # pixels seen often enough as bare ground
        if good.sum() < 0.1 * self.w * self.h:                           # nearly everything is covered: cannot learn the ground
            self.cal = None
            self.cal_failed = True
            return
        mean = c["s"] / cnt[..., None]
        var = np.maximum(c["ss"] / cnt[..., None] - mean * mean, 0)
        g_mean, t_mean = c["g"] / cnt, c["t"] / cnt
        g_noise = np.sqrt(np.maximum(c["gg"] / cnt - g_mean ** 2, 0))
        t_noise = np.sqrt(np.maximum(c["tt"] / cnt - t_mean ** 2, 0))
        noise_e = np.sqrt(var.sum(axis=2))
        bad = ~good                                                       # pixels that were always under an object: use typical ground values
        if bad.any():
            for ch in range(3):
                mean[..., ch][bad] = np.median(mean[..., ch][good])
            g_mean[bad], t_mean[bad], noise_e[bad] = np.median(g_mean[good]), np.median(t_mean[good]), np.median(noise_e[good])
        noise_e = cv2.GaussianBlur(noise_e, (15, 15), 0)
        self.ref = {
            "lab": mean.astype(np.float32), "grad": g_mean.astype(np.float32), "tex": t_mean.astype(np.float32),
            "tol_e": np.clip(3.0 * noise_e + 4.0, 5.0, 25.0).astype(np.float32),
            "tol_g": float(max(6.0, 4.0 * np.percentile(g_noise[good], 99))),
            "tol_t": float(max(3.0, 4.0 * np.percentile(t_noise[good], 99))),
        }
        self.cal = None
        try:
            np.savez_compressed(self.calib_file, lab=self.ref["lab"].astype(np.float16), grad=self.ref["grad"].astype(np.float16),
                                tex=self.ref["tex"].astype(np.float16), tol_e=self.ref["tol_e"].astype(np.float16),
                                tol_g=self.ref["tol_g"], tol_t=self.ref["tol_t"], size=np.array([self.w, self.h]))
        except OSError:
            pass

    def load_calibration(self):
        try:
            if os.path.exists(self.calib_file):
                d = np.load(self.calib_file)
                if tuple(d["size"]) == (self.w, self.h):
                    self.ref = {"lab": d["lab"].astype(np.float32), "grad": d["grad"].astype(np.float32), "tex": d["tex"].astype(np.float32),
                                "tol_e": d["tol_e"].astype(np.float32), "tol_g": float(d["tol_g"]), "tol_t": float(d["tol_t"])}
        except Exception:
            self.ref = None

    @property
    def calibrated(self):
        return self.ref is not None

    @property
    def calibrating(self):
        return None if self.cal is None else (self.cal["k"], self.cal["n"])

    @property
    def calibrating_auto(self):
        return bool(self.cal and self.cal.get("auto"))

    # ------------------------------------------------------------------ features
    @staticmethod
    def _features(rgb):
        lab = cv2.cvtColor(rgb, cv2.COLOR_RGB2LAB).astype(np.float32)
        lab[..., 0] *= 100.0 / 255.0
        lab[..., 1:] -= 128.0
        lab = cv2.GaussianBlur(lab, (BLUR_K, BLUR_K), BLUR_S)
        gx = cv2.Sobel(lab, cv2.CV_32F, 1, 0, ksize=3)
        gy = cv2.Sobel(lab, cv2.CV_32F, 0, 1, ksize=3)
        grad = (np.hypot(gx[..., 0], gy[..., 0]) + 1.5 * np.sqrt(gx[..., 1] ** 2 + gy[..., 1] ** 2 + gx[..., 2] ** 2 + gy[..., 2] ** 2)) / 8.0
        L = lab[..., 0]
        mean = cv2.boxFilter(L, -1, (TEX_W, TEX_W))
        tex = np.sqrt(np.maximum(cv2.boxFilter(L * L, -1, (TEX_W, TEX_W)) - mean * mean, 0))
        return lab, grad, tex

    def score_map(self, lab, grad, tex, sens):
        """How far beyond 'normal ground' every pixel is. 1.0 = hazard threshold."""
        k = 1.12 ** (sens - 6)
        calm = (self.roi > 0) & ~self.haz
        if self.ref is not None:
            d = lab - self.ref["lab"]
            idx = calm[::3, ::3]
            if idx.sum() > 200:                                       # measure global brightness / white-balance drift on the ground only
                d = d - np.median(d[::3, ::3][idx], axis=0)
            tol_e, refg, reft, tol_g, tol_t = self.ref["tol_e"], self.ref["grad"], self.ref["tex"], self.ref["tol_g"], self.ref["tol_t"]
        else:
            sub = lab[::3, ::3]
            quiet = tex[::3, ::3] <= np.percentile(tex[::3, ::3][calm[::3, ::3]], 50) if calm[::3, ::3].any() else np.ones(sub.shape[:2], bool)
            sel = quiet & calm[::3, ::3]
            ground = np.median(sub[sel], axis=0) if sel.any() else np.median(sub.reshape(-1, 3), axis=0)
            d = lab - ground
            tol_e, refg, reft, tol_g, tol_t = 11.0, 0.0, 0.0, 7.0, 4.5
        de = np.sqrt((0.5 * d[..., 0]) ** 2 + d[..., 1] ** 2 + d[..., 2] ** 2)
        return np.maximum.reduce([de / (tol_e * k), np.maximum(grad - refg, 0) / (tol_g * k), np.maximum(tex - reft, 0) / (tol_t * k)])

    # ------------------------------------------------------------------ one frame
    def analyse(self, rgb, sens=6, hold=False):
        """hold=True freezes the landing zone (use it during the descent so a rock dropped on it triggers the abort)."""
        lab, grad, tex = self._features(rgb)
        if self.cal is not None:
            self._cal_feed(lab, grad, tex)
        score = self.score_map(lab, grad, tex, sens)
        roi = self.roi > 0
        strong = (score >= 1.0) & roi
        weak = (score >= 0.6) & roi
        if strong.any():
            n, labels = cv2.connectedComponents(weak.astype(np.uint8), connectivity=8)
            keep = np.unique(labels[strong])
            raw = np.isin(labels, keep[keep > 0])
        else:
            raw = np.zeros((self.h, self.w), bool)
        m8 = cv2.morphologyEx(raw.astype(np.uint8) * 255, cv2.MORPH_CLOSE, K5)
        cnts, _ = cv2.findContours(m8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        filled = np.zeros((self.h, self.w), np.uint8)
        big = [c for c in cnts if cv2.contourArea(c) >= self.min_area]
        if big:
            cv2.drawContours(filled, big, -1, 255, -1)
        roi_area = max(1, int(roi.sum()))
        cov_now = float((filled > 0).sum()) / roi_area
        self.raw_coverage = cov_now
        if cov_now > 0.55 and self.haz.sum() < 0.3 * roi_area and self._flash < 8:
            self._flash += 1                                             # the whole picture changed in one frame (camera exposure jump, a hand
        else:                                                            # waved over the yard): keep the previous map for a few frames
            self._flash = self._flash if cov_now > 0.55 else 0
            present = filled > 0
            self.mem = np.where(present, 0.5 * self.mem + 0.5, self.mem * 0.88)          # appears in 2 frames, clears only after ~13 calm frames
            self.haz = (self.mem >= 0.55) | (self.haz & (self.mem >= 0.2))
        hazard = cv2.GaussianBlur(self.haz.astype(np.float32), (0, 0), 1.6) >= 0.5          # smooth outlines
        hazard &= roi
        return self._finish(hazard, score, hold)

    # ------------------------------------------------------------------ objects, cells, landing zone
    def _objects(self, hazard):
        h8 = hazard.astype(np.uint8)
        n, lab, stats, _ = cv2.connectedComponentsWithStats(h8, connectivity=8)
        big = (stats[:, cv2.CC_STAT_AREA] >= 350)[lab]                  # big objects: trim the 2 px safety halo so the outline hugs the object
        shown = np.where(big, cv2.erode(h8, K3, iterations=2), h8)
        cnts, _ = cv2.findContours(shown, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        objs = []
        for c in cnts:
            area = cv2.contourArea(c)
            if area < self.min_area:
                continue
            M = cv2.moments(c)
            cx, cy = (M["m10"] / M["m00"], M["m01"] / M["m00"]) if M["m00"] else (float(c[0][0][0]), float(c[0][0][1]))
            (ex, ey), er = cv2.minEnclosingCircle(c)
            kind, shape = "blob", smooth_contour(c[:, 0, :])
            pts = c[:, 0, :].astype(np.float32)
            if area >= 120:                                            # tiny things are just drawn as they are
                # round only if the contour really follows a circle / ellipse (a rectangle or a lumpy rock fails this test)
                fit_c = cv2.fitEllipse(c) if len(c) >= 5 else None
                rd = np.hypot(pts[:, 0] - ex, pts[:, 1] - ey) / max(er, 1e-6)
                circle_ok = (area / (np.pi * er * er + 1e-6) >= 0.80 and abs(rd.mean() - 1) < 0.07 and np.abs(rd - rd.mean()).max() < 0.16)
                if circle_ok:
                    kind, shape = "circle", (ex, ey, er)
                elif fit_c is not None:
                    (fx, fy), (fa, fb), ang = fit_c
                    if min(fa, fb) > 4 and max(fa, fb) / min(fa, fb) <= 3.0:
                        t = np.deg2rad(ang)
                        dx, dy = pts[:, 0] - fx, pts[:, 1] - fy
                        u = dx * np.cos(t) + dy * np.sin(t)
                        v = -dx * np.sin(t) + dy * np.cos(t)
                        e = np.sqrt((u / (fa / 2)) ** 2 + (v / (fb / 2)) ** 2)
                        if abs(e.mean() - 1) < 0.07 and np.abs(e - e.mean()).max() < 0.16:
                            kind, shape = "ellipse", ((fx, fy), (fa, fb), ang)
            objs.append({"kind": kind, "shape": shape, "area": float(area), "centre": (cx, cy)})
        return objs

    def _finish(self, hazard, score, hold=False):
        r = Result()
        r.hazard, r.score = hazard, score
        r.objects = self._objects(hazard)
        roi_area = max(1, int((self.roi > 0).sum()))
        r.coverage = float(hazard.sum()) / roi_area
        # --- the 16 x 9 cell map for the Arduino
        frac = cv2.resize(hazard.astype(np.float32), (self.cols, self.rows), interpolation=cv2.INTER_AREA)
        cells = frac >= 0.10
        # --- the landing zone: centre of the largest empty circle
        blocked = cv2.dilate(hazard.astype(np.uint8), K3)
        free = ((blocked == 0) & (self.roi > 0) & (self._inner > 0)).astype(np.uint8)
        dist = cv2.distanceTransform(free, cv2.DIST_L2, 5)
        by, bx = np.unravel_index(np.argmax(dist), dist.shape)
        best = float(dist[by, bx])
        if self.target is None:
            self.target = (float(bx), float(by))
        elif hold:
            pass
        else:
            px, py = self.target
            r0 = int(self.R * 2.5)
            y0, y1, x0, x1 = max(0, int(py) - r0), min(self.h, int(py) + r0 + 1), max(0, int(px) - r0), min(self.w, int(px) + r0 + 1)
            sub = dist[y0:y1, x0:x1]
            ly, lx = np.unravel_index(np.argmax(sub), sub.shape)
            local_best = float(sub[ly, lx])
            cur = float(dist[int(round(py)), int(round(px))])
            if best > local_best * 1.35 + 0.6 * self.R:                # a clearly better place exists elsewhere: move there
                self.target = (float(bx), float(by))
            elif cur >= 0.92 * local_best:                              # already (almost) the best spot around here: stay exactly put
                pass
            else:                                                       # otherwise glide to the best spot nearby
                self.target = (px + 0.4 * (lx + x0 - px), py + 0.4 * (ly + y0 - py))
        tx, ty = self.target
        r.target = (tx, ty)
        r.clearance = float(dist[int(round(ty)), int(round(tx))])
        r.max_clearance = best
        r.radius = float(np.clip(r.clearance - 1.0, 0.5 * self.R, self.R))     # the ring shrinks to fit a tight spot instead of giving up
        r.safe = r.clearance >= 0.5 * self.R + 1                                # only a yard that is completely full has no spot
        r.quality = float(np.clip((r.clearance - 0.5 * self.R) / (1.5 * self.R), 0, 1))
        now = time.time()
        if self._sec_pos is None or np.hypot(tx - self._sec_pos[0], ty - self._sec_pos[1]) > 3.0:
            self._sec_pos, self._sec_t = (tx, ty), now
        r.secure_s = now - self._sec_t                                          # how long the spot has stayed put
        # --- landing cell with a little hysteresis so it does not flicker on a cell border
        c = int(np.clip(tx / self.cw, 1, self.cols - 2))
        rr = int(np.clip(ty / self.ch, 1, self.rows - 2))
        if hold and self.cell is not None:
            c, rr = self.cell
        elif self.cell is not None:
            pc, pr = self.cell
            if abs(tx - (pc + 0.5) * self.cw) < 0.62 * self.cw and abs(ty - (pr + 0.5) * self.ch) < 0.62 * self.ch and 1 <= pc <= self.cols - 2 and 1 <= pr <= self.rows - 2:
                c, rr = pc, pr
        self.cell = (c, rr)
        # --- the landing cell is flagged the moment anything touches the landing circle
        circ = ((self._xx - tx) ** 2 + (self._yy - ty) ** 2) <= r.radius ** 2
        touched = int(hazard[circ].sum()) >= 6
        cells[rr, c] = touched
        r.touched = touched
        r.cells = cells
        r.col, r.row, r.idx = c, rr, rr * self.cols + c
        r.count = 255 if not r.safe else int(cells.sum())              # "no safe site" is reported to the Arduino as very rough ground
        r.flags = (cells, c, rr, r.idx)
        r.calibrating = self.calibrating
        return r
