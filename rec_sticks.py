"""
rec_sticks.py — pull the pilot's stick movements out of a RealFlight
.recording file. For StickMover-style hardware: no pixel tracking, no
screen scraping — the recording file itself stores all four stick
channels every frame (~21 Hz), and this decodes them directly.

    python rec_sticks.py "<flight.recording>"                 # summary
    python rec_sticks.py "<flight.recording>" --out sticks.csv
    python rec_sticks.py "<flight.recording>" --stream --hz 50

--stream prints one line per tick, real-time paced, to stdout:
    t,ail,ele,thr,rud        (t seconds; channels 0.000-1.000)
Pipe that into whatever feeds your servo hardware. --hz interpolates
between the recording's native ~21 Hz frames (default 50).

Format notes (reverse-engineered against paired live-telemetry captures,
Fly RC-X project, 2026): the flight body is a flat run of frames
[u16 LE size][6-byte constant][f32 LE t][...]; within each frame, at
offsets +44..+47 from the constant field, the four sticks sit as single
bytes 0-255 in AETR order (aileron, elevator, throttle, rudder).
Verified r=0.93-0.97 per channel against ground truth. Elevator: LOW
values = stick pulled (RealFlight convention).

Standalone: Python 3.8+, standard library only.
"""

from __future__ import annotations

import argparse
import csv
import struct
import sys
import time

STICK_OFFS = {"ail": 44, "ele": 45, "thr": 46, "rud": 47}


def find_body(data: bytes) -> int:
    """Start of the longest valid [u16 size][...] frame chain, found by
    size-chaining with a sane, near-monotone f32 timestamp at +8."""
    def walk(start: int, limit: int = 10 ** 9) -> int:
        count, off, last_t = 0, start, -1.0
        while off + 12 < len(data) and count < limit:
            sz = struct.unpack_from("<H", data, off)[0]
            if sz < 20 or sz > 4096 or off + sz > len(data):
                break
            t = struct.unpack_from("<f", data, off + 8)[0]
            if not (0.0 <= t < 3600.0) or t < last_t - 0.5:
                break
            count += 1
            last_t = t
            off += sz
        return count

    best, off = (-1, 0), 0
    while off < len(data) - 12:
        if walk(off, limit=12) >= 12:
            full = walk(off)
            if full > best[1]:
                best = (off, full)
            if full > 500:
                o, steps = off, 0
                while steps < full:
                    o += struct.unpack_from("<H", data, o)[0]
                    steps += 1
                off = o
                continue
        off += 1
    return best[0]


def stick_frames(path: str):
    """-> list of (t, ail, ele, thr, rud), channels normalized 0..1."""
    data = open(path, "rb").read()
    body = find_body(data)
    if body < 0:
        raise SystemExit("no flight body found — is this a RealFlight "
                         ".recording file?")
    out = []
    off, last_t = body, -1.0
    while off + 12 < len(data):
        sz = struct.unpack_from("<H", data, off)[0]
        if sz < 20 or sz > 4096 or off + sz > len(data):
            break
        t = struct.unpack_from("<f", data, off + 8)[0]
        if not (0.0 <= t < 3600.0) or t < last_t - 0.5:
            break
        last_t = t
        mp = off + 2
        if sz >= 65:                      # physics-bearing frames only
            out.append((t,
                        data[mp + STICK_OFFS["ail"]] / 255.0,
                        data[mp + STICK_OFFS["ele"]] / 255.0,
                        data[mp + STICK_OFFS["thr"]] / 255.0,
                        data[mp + STICK_OFFS["rud"]] / 255.0))
        off += sz
    if not out:
        raise SystemExit("no stick frames decoded")
    t0 = out[0][0]
    return [(t - t0, a, e, th, r) for t, a, e, th, r in out]


def interp(frames, hz: float):
    """Resample to a fixed rate with linear interpolation."""
    out = []
    dur = frames[-1][0]
    j = 0
    t = 0.0
    step = 1.0 / hz
    while t <= dur:
        while j + 1 < len(frames) and frames[j + 1][0] < t:
            j += 1
        if j + 1 >= len(frames):
            out.append((t,) + frames[-1][1:])
        else:
            ta, tb = frames[j][0], frames[j + 1][0]
            w = (t - ta) / (tb - ta) if tb > ta else 0.0
            out.append(tuple([t] + [frames[j][k] + (frames[j + 1][k] -
                                                    frames[j][k]) * w
                                    for k in (1, 2, 3, 4)]))
        t += step
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("recording")
    ap.add_argument("--out", help="write CSV here")
    ap.add_argument("--stream", action="store_true",
                    help="print real-time paced t,ail,ele,thr,rud lines")
    ap.add_argument("--hz", type=float, default=50.0,
                    help="resample rate for --stream/--out (0 = native ~21)")
    args = ap.parse_args()

    frames = stick_frames(args.recording)
    dur = frames[-1][0]
    print(f"{args.recording}", file=sys.stderr)
    print(f"  {len(frames)} stick frames, {dur:.1f}s "
          f"(~{len(frames)/max(dur, 0.01):.1f} Hz native)", file=sys.stderr)

    ticks = interp(frames, args.hz) if args.hz > 0 else frames

    if args.out:
        with open(args.out, "w", newline="", encoding="utf-8") as fh:
            w = csv.writer(fh)
            w.writerow(["t", "ail", "ele", "thr", "rud"])
            for t, a, e, th, r in ticks:
                w.writerow([f"{t:.3f}", f"{a:.3f}", f"{e:.3f}",
                            f"{th:.3f}", f"{r:.3f}"])
        print(f"  wrote {args.out}", file=sys.stderr)

    if args.stream:
        start = time.perf_counter()
        for t, a, e, th, r in ticks:
            lag = t - (time.perf_counter() - start)
            if lag > 0:
                time.sleep(lag)
            print(f"{t:.3f},{a:.3f},{e:.3f},{th:.3f},{r:.3f}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
