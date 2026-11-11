#!/usr/bin/env python3
"""Mission report: turns the log from  py hazard_sw.py --arduino COM5 --log mission.csv  into one HTML page
(open mission_report.html in any browser) with the numbers and a chart.

Run:  py mission_report.py mission.csv           (Mac / Linux: python3 mission_report.py mission.csv)
"""
import csv
import html
import sys

COLORS = {"SEARCH": "#f2c94c", "LOCKED": "#6fcf97", "DESCEND": "#56ccf2", "TOUCHDOWN": "#27ae60", "ABORT": "#eb5757", "": "#555555"}


def state_of(text):
    word = (text or "").replace(";", ",").split(" ")[0]
    return word if word in COLORS else ""


def main(path):
    rows = []
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            try:
                rows.append({"t": float(r["t"]), "fps": float(r["fps"]), "haz": int(r["hazards"]), "col": int(r["safe_col"]),
                             "row": int(r["safe_row"]), "moves": int(r["moves"]), "state": state_of(r["arduino"]), "raw": r["arduino"]})
            except (ValueError, KeyError):
                continue
    if len(rows) < 3:
        sys.exit("The log has too few rows. Run hazard_sw.py with --log for at least a few seconds.")
    dur = rows[-1]["t"] - rows[0]["t"]
    # state changes
    events, last = [], None
    for r in rows:
        if r["state"] and r["state"] != last:
            events.append((r["t"], r["state"], r["raw"]))
            last = r["state"]
    count = {k: sum(1 for e in events if e[1] == k) for k in COLORS if k}
    time_in = {k: 0.0 for k in COLORS if k}
    for a, b in zip(rows, rows[1:]):
        if a["state"]:
            time_in[a["state"]] += b["t"] - a["t"]
    haz = [r["haz"] for r in rows]
    fps = [r["fps"] for r in rows if r["fps"] > 0]
    stats = [("Run length", f"{dur:.1f} s"), ("Frames logged", str(len(rows))), ("Average picture rate", f"{sum(fps) / max(1, len(fps)):.1f} fps"),
             ("Hazard cells: average / most", f"{sum(haz) / len(haz):.1f} / {max(haz)} of 144"), ("Safe zone moves", str(rows[-1]["moves"] - rows[0]["moves"])),
             ("Landing attempts (descents)", str(count["DESCEND"])), ("Touchdowns", str(count["TOUCHDOWN"])), ("Aborts", str(count["ABORT"]))]
    # chart: hazard count over time, with the landing state as a coloured band
    W, H, pad = 900, 260, 40
    tmax = max(rows[-1]["t"], 1e-6)
    hmax = max(max(haz), 30)
    def X(t): return pad + (t / tmax) * (W - 2 * pad)
    def Y(v): return H - pad - (v / hmax) * (H - 2 * pad)
    svg = [f'<svg viewBox="0 0 {W} {H + 40}" width="100%" role="img" aria-label="hazard cells over time with landing states">']
    for a, b in zip(rows, rows[1:]):
        if a["state"]:
            svg.append(f'<rect x="{X(a["t"]):.1f}" y="{pad}" width="{max(0.5, X(b["t"]) - X(a["t"])):.1f}" height="{H - 2 * pad}" fill="{COLORS[a["state"]]}" opacity="0.28"/>')
    for v in range(0, hmax + 1, max(1, hmax // 5)):
        svg.append(f'<line x1="{pad}" x2="{W - pad}" y1="{Y(v):.1f}" y2="{Y(v):.1f}" stroke="#ccc" stroke-width="0.5"/><text x="4" y="{Y(v) + 4:.1f}" font-size="11" fill="#666">{v}</text>')
    pts = " ".join(f"{X(r['t']):.1f},{Y(r['haz']):.1f}" for r in rows)
    svg.append(f'<polyline points="{pts}" fill="none" stroke="#2d2d2d" stroke-width="1.6"/>')
    for t, st, _ in events:
        svg.append(f'<text x="{X(t) + 2:.1f}" y="{pad - 6}" font-size="10" fill="{COLORS[st]}" stroke="#444" stroke-width="0.2">{st}</text>')
    svg.append(f'<text x="{W / 2:.0f}" y="{H + 20}" font-size="12" text-anchor="middle" fill="#444">time (seconds); line = hazard cells, background colour = landing state</text></svg>')
    legend = " ".join(f'<span style="background:{c};opacity:.6;padding:2px 8px;border-radius:4px;margin-right:6px">{k}</span>' for k, c in COLORS.items() if k)
    ev_rows = "".join(f"<tr><td>{t:.1f} s</td><td>{html.escape(st)}</td><td>{html.escape(raw)}</td></tr>" for t, st, raw in events)
    st_rows = "".join(f"<tr><td>{k}</td><td>{time_in[k]:.1f} s</td><td>{count[k]}</td></tr>" for k in time_in)
    srows = "".join(f"<tr><td>{k}</td><td><b>{v}</b></td></tr>" for k, v in stats)
    page = f"""<!DOCTYPE html><html lang="en"><head><meta charset="utf-8"><title>Mission report</title>
<style>body{{font-family:system-ui,sans-serif;max-width:960px;margin:24px auto;padding:0 16px;color:#222}}h1{{margin-bottom:4px}}
table{{border-collapse:collapse;margin:8px 0 18px}}td,th{{border-bottom:1px solid #ddd;padding:5px 14px;text-align:left}}small{{color:#666}}</style></head><body>
<h1>Landing mission report</h1><small>Made from {html.escape(path)}</small>
<h2>Summary</h2><table>{srows}</table>
<h2>Terrain and landing states</h2>{legend}{''.join(svg)}
<h2>Time in each state</h2><table><tr><th>State</th><th>Time</th><th>Times entered</th></tr>{st_rows}</table>
<h2>Event log</h2><table><tr><th>When</th><th>State</th><th>Arduino said</th></tr>{ev_rows}</table>
<small>Hazard cells = grid cells (of 144) with a lot of edges, from the same algorithm the FPGA runs.</small></body></html>"""
    open("mission_report.html", "w", encoding="utf-8").write(page)
    print("wrote mission_report.html:", len(events), "state changes,", count["DESCEND"], "descents,", count["TOUCHDOWN"], "touchdowns,", count["ABORT"], "aborts")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit("Usage: py mission_report.py mission.csv")
    main(sys.argv[1])
