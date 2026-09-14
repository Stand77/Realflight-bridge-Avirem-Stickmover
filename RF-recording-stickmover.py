# -*- coding: utf-8 -*-
"""
RF-recording-stickmover.py - Live RealFlight stick capture + recording playback
to StickMover hardware. 09/13/2026

Primary mode  : Live from RealFlight via FlightAxis SOAP (pilot stick positions)
Secondary mode: Play a .recording file (decoder is built in; rec_sticks.py not needed)
Sync mode     : Click RealFlight Play + stream recording sticks together

Mode 2 layout (display + hardware mapping):
  Left stick  -> Rudder (axis1) + Throttle (axis2)
  Right stick -> Aileron (axis4) + Elevator (axis3)

RealFlight channel order assumed (AETR):
  ch0 = Aileron, ch1 = Elevator, ch2 = Throttle, ch3 = Rudder
"""

from __future__ import annotations

import sys
import time
import threading
import queue
import re
import json
import os
import struct
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
try:
    import serial.tools.list_ports
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False

try:
    import urllib.request
except ImportError:
    pass

try:
    import ctypes
    from ctypes import wintypes
except ImportError:
    ctypes = None

# Optional StickMover library. Script still runs in monitor-only mode if missing.
pystickmover = None
HAS_STICKMOVER_LIB = False
STICKMOVER_SEARCH_PATHS = [
    r"C:\Users\Stand\Downloads\pystickmover-0.1\pystickmover-0.1",
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "pystickmover-0.1"),
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "pystickmover"),
]
for _p in STICKMOVER_SEARCH_PATHS:
    if _p and os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)
try:
    from pystickmover import pystickmover as _psm
    pystickmover = _psm
    HAS_STICKMOVER_LIB = True
except ImportError:
    HAS_STICKMOVER_LIB = False
    pystickmover = None

TARGET_VID = 0x0403
TARGET_PID = 0x6015

FLIGHTAXIS_URL = "http://127.0.0.1:18083"
DEFAULT_HZ = 60.0
LIVE_POLL_HZ = 40.0

CHANNEL_MAP = {
    "ail": 0,
    "ele": 1,
    "thr": 2,
    "rud": 3,
}

# Dual-rate / expo (same as Radio-gadget-red-stickmover.py).
# FlightAxis and .recording values are 0..1. They are converted to -1..+1,
# Rate then Expo are applied, then converted back to 0..1 for hardware.
DEFAULT_RATE_PCT = 100.0
DEFAULT_EXPO_PCT = 0.0
RATE_MIN_PCT = 50.0
RATE_MAX_PCT = 400.0
EXPO_MIN_PCT = -100.0
EXPO_MAX_PCT = 100.0


def hw_to_norm(hw):
    """Map StickMover / FlightAxis 0..1 to bipolar -1..+1 (0.5 = center)."""
    return max(-1.0, min(1.0, float(hw) * 2.0 - 1.0))


def norm_to_hw(norm):
    """Map bipolar -1..+1 back to hardware 0..1."""
    return max(0.0, min(1.0, (float(norm) + 1.0) / 2.0))


def apply_rate(val_norm, rate_pct):
    """Scale a -1..+1 stick by dual-rate percent, then clamp.

    100% = incoming value unchanged.
    >100% enlarges small FlightAxis / recording travel toward full throw.
    """
    scaled = val_norm * (float(rate_pct) / 100.0)
    return max(-1.0, min(1.0, scaled))


def apply_expo(val_norm, expo_pct):
    """RC-transmitter expo on a -1..+1 axis.

    y = x * (1 - e + e * x^2)  with e = expo/100.
      expo = 0    linear
      expo > 0    softer around center
      expo < 0    more throw around center
    """
    x = max(-1.0, min(1.0, float(val_norm)))
    e = max(EXPO_MIN_PCT, min(EXPO_MAX_PCT, float(expo_pct))) / 100.0
    y = x * (1.0 - e + e * x * x)
    return max(-1.0, min(1.0, y))


def shape_axis(val_norm, rate_pct, expo_pct):
    """Apply Rate then Expo on -1..+1."""
    return apply_expo(apply_rate(val_norm, rate_pct), expo_pct)


def shape_hw(hw, rate_pct, expo_pct):
    """Shape a 0..1 channel: to bipolar, Rate, Expo, back to 0..1."""
    return norm_to_hw(shape_axis(hw_to_norm(hw), rate_pct, expo_pct))


# ---------------------------------------------------------------------------
# Built-in .recording decoder (formerly rec_sticks.py)
# Frames: [u16 LE size][6-byte constant][f32 LE t][...]; sticks at +44..+47
# as bytes 0-255 in AETR order.
# ---------------------------------------------------------------------------
STICK_OFFS = {"ail": 44, "ele": 45, "thr": 46, "rud": 47}


def find_body(data):
    def walk(start, limit=10 ** 9):
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


def stick_frames(path):
    data = open(path, "rb").read()
    body = find_body(data)
    if body < 0:
        raise RuntimeError("no flight body found - is this a RealFlight .recording file?")
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
        if sz >= 65:
            out.append((t,
                        data[mp + STICK_OFFS["ail"]] / 255.0,
                        data[mp + STICK_OFFS["ele"]] / 255.0,
                        data[mp + STICK_OFFS["thr"]] / 255.0,
                        data[mp + STICK_OFFS["rud"]] / 255.0))
        off += sz
    if not out:
        raise RuntimeError("no stick frames decoded")
    t0 = out[0][0]
    return [(t - t0, a, e, th, r) for t, a, e, th, r in out]


def interp(frames, hz):
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


# Config file next to this script (Play button coords + offset)
CONFIG_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "stickmover_sync_config.json",
)


# ---------------------------------------------------------------------------
# Windows mouse click (for RealFlight Play button)
# ---------------------------------------------------------------------------
_user32 = ctypes.windll.user32 if ctypes else None

# DPI-aware so fullscreen RF coords match on high-DPI displays
if _user32:
    try:
        _user32.SetProcessDPIAware()
    except Exception:
        pass


def get_cursor_pos():
    """Return (x, y) of current mouse cursor (screen coords)."""
    if ctypes is None or _user32 is None:
        raise RuntimeError("ctypes not available")
    class POINT(ctypes.Structure):
        _fields_ = [("x", ctypes.c_long), ("y", ctypes.c_long)]
    pt = POINT()
    _user32.GetCursorPos(ctypes.byref(pt))
    return int(pt.x), int(pt.y)


def _find_realflight_hwnd():
    """Find a RealFlight top-level window by title substring."""
    if not _user32:
        return None
    found = []

    @ctypes.WINFUNCTYPE(ctypes.c_bool, wintypes.HWND, wintypes.LPARAM)
    def enum_proc(hwnd, _lparam):
        if not _user32.IsWindowVisible(hwnd):
            return True
        length = _user32.GetWindowTextLengthW(hwnd)
        if length <= 0:
            return True
        buf = ctypes.create_unicode_buffer(length + 1)
        _user32.GetWindowTextW(hwnd, buf, length + 1)
        title = buf.value.lower()
        if "realflight" in title or "real flight" in title:
            found.append(hwnd)
        return True

    _user32.EnumWindows(enum_proc, 0)
    return found[0] if found else None


def _sendinput_click(x, y):
    """Absolute-position left click via SendInput."""
    MOUSEEVENTF_MOVE = 0x0001
    MOUSEEVENTF_LEFTDOWN = 0x0002
    MOUSEEVENTF_LEFTUP = 0x0004
    MOUSEEVENTF_ABSOLUTE = 0x8000

    screen_w = _user32.GetSystemMetrics(0)
    screen_h = _user32.GetSystemMetrics(1)
    ax = int(x * 65535 / max(screen_w - 1, 1))
    ay = int(y * 65535 / max(screen_h - 1, 1))

    class MOUSEINPUT(ctypes.Structure):
        _fields_ = [
            ("dx", wintypes.LONG),
            ("dy", wintypes.LONG),
            ("mouseData", wintypes.DWORD),
            ("dwFlags", wintypes.DWORD),
            ("time", wintypes.DWORD),
            ("dwExtraInfo", ctypes.POINTER(ctypes.c_ulong)),
        ]

    class INPUT_UNION(ctypes.Union):
        _fields_ = [("mi", MOUSEINPUT)]

    class INPUT(ctypes.Structure):
        _fields_ = [("type", wintypes.DWORD), ("union", INPUT_UNION)]

    def make_input(flags, dx=0, dy=0):
        inp = INPUT()
        inp.type = 0  # INPUT_MOUSE
        inp.union.mi = MOUSEINPUT(dx, dy, 0, flags, 0, None)
        return inp

    inputs = (INPUT * 3)(
        make_input(MOUSEEVENTF_MOVE | MOUSEEVENTF_ABSOLUTE, ax, ay),
        make_input(MOUSEEVENTF_LEFTDOWN),
        make_input(MOUSEEVENTF_LEFTUP),
    )
    _user32.SendInput(3, ctypes.byref(inputs), ctypes.sizeof(INPUT))


def click_at(x, y, settle=0.10):
    """Focus RealFlight if found, move cursor, left-click."""
    if ctypes is None or _user32 is None:
        raise RuntimeError("ctypes not available")

    hwnd = _find_realflight_hwnd()
    if hwnd:
        _user32.ShowWindow(hwnd, 9)  # SW_RESTORE
        _user32.SetForegroundWindow(hwnd)
        time.sleep(0.20)

    _user32.SetCursorPos(int(x), int(y))
    time.sleep(settle)

    try:
        _sendinput_click(int(x), int(y))
    except Exception:
        pass
    # Fallback / second pulse
    _user32.mouse_event(0x0002, 0, 0, 0, 0)
    time.sleep(0.05)
    _user32.mouse_event(0x0004, 0, 0, 0, 0)
    time.sleep(0.05)


def load_sync_config():
    defaults = {"play_x": None, "play_y": None, "offset_sec": 0.0}
    try:
        with open(CONFIG_PATH, "r", encoding="utf-8") as f:
            data = json.load(f)
        defaults.update(data)
    except Exception:
        pass
    return defaults


def save_sync_config(cfg):
    try:
        with open(CONFIG_PATH, "w", encoding="utf-8") as f:
            json.dump(cfg, f, indent=2)
    except Exception as e:
        print("Could not save config:", e)


def discover_port():
    if not HAS_SERIAL:
        return None
    for port in serial.tools.list_ports.comports():
        if port.vid == TARGET_VID and port.pid == TARGET_PID:
            return port.device
    return None


# ---------------------------------------------------------------------------
# FlightAxis SOAP helper
# ---------------------------------------------------------------------------
class FlightAxisClient:
    def __init__(self, url=FLIGHTAXIS_URL, timeout=1.0):
        self.url = url
        self.timeout = timeout
        self._injected = False
        self._channel_re = re.compile(
            r"<m-channelValues-0to1[^>]*>\s*(.*?)\s*</m-channelValues-0to1>",
            re.DOTALL | re.IGNORECASE,
        )
        self._item_re = re.compile(r"<item>([^<]+)</item>", re.IGNORECASE)
        self._speed_re = re.compile(
            r"<m-currentPhysicsSpeedMultiplier[^>]*>([^<]+)</m-currentPhysicsSpeedMultiplier>",
            re.IGNORECASE,
        )
        self._phys_re = re.compile(
            r"<m-currentPhysicsTime-SEC[^>]*>([^<]+)</m-currentPhysicsTime-SEC>",
            re.IGNORECASE,
        )

    def _soap(self, action, body_inner):
        body = (
            "<?xml version='1.0' encoding='UTF-8'?>"
            "<soap:Envelope xmlns:soap='http://schemas.xmlsoap.org/soap/envelope/' "
            "xmlns:xsd='http://www.w3.org/2001/XMLSchema' "
            "xmlns:xsi='http://www.w3.org/2001/XMLSchema-instance'>"
            "<soap:Body>" + body_inner + "</soap:Body></soap:Envelope>"
        )
        data = body.encode("utf-8")
        req = urllib.request.Request(self.url, data=data, method="POST")
        req.add_header("Content-Type", "text/xml; charset=utf-8")
        req.add_header("SOAPAction", action)
        req.add_header("Content-Length", str(len(data)))
        with urllib.request.urlopen(req, timeout=self.timeout) as resp:
            return resp.read().decode("utf-8", errors="replace")

    def inject(self):
        self._soap(
            "InjectUAVControllerInterface",
            "<InjectUAVControllerInterface><a>1</a><b>2</b></InjectUAVControllerInterface>",
        )
        self._injected = True

    def restore(self):
        try:
            self._soap(
                "RestoreOriginalControllerDevice",
                "<RestoreOriginalControllerDevice><a>1</a><b>2</b></RestoreOriginalControllerDevice>",
            )
        except Exception:
            pass
        self._injected = False

    def ping(self):
        try:
            if not self._injected:
                self.inject()
            self.exchange()
            return True
        except Exception:
            self._injected = False
            return False

    def exchange(self):
        if not self._injected:
            self.inject()
        xml = self._soap(
            "ExchangeData",
            "<ExchangeData><pControlInputs>"
            "<m-selectedChannels>0</m-selectedChannels>"
            "</pControlInputs></ExchangeData>",
        )
        m = self._channel_re.search(xml)
        if not m:
            raise RuntimeError("No m-channelValues-0to1 in reply")
        items = [float(x) for x in self._item_re.findall(m.group(1))]
        if len(items) < 4:
            raise RuntimeError("Expected >=4 channels, got %d" % len(items))
        return items

    def exchange_clock(self):
        """Return (physics_speed_multiplier, physics_time_sec) from FlightAxis."""
        if not self._injected:
            self.inject()
        xml = self._soap(
            "ExchangeData",
            "<ExchangeData><pControlInputs>"
            "<m-selectedChannels>0</m-selectedChannels>"
            "</pControlInputs></ExchangeData>",
        )
        sm = self._speed_re.search(xml)
        pm = self._phys_re.search(xml)
        speed = float(sm.group(1)) if sm else 1.0
        phys = float(pm.group(1)) if pm else 0.0
        if speed < 0.01:
            speed = 1.0
        return speed, phys


# ---------------------------------------------------------------------------
# GUI
# ---------------------------------------------------------------------------
class StickMoverBridgeGUI:
    def __init__(self, root, recording_path=None):
        self.root = root
        self.root.title("RealFlight -> StickMover")
        self.root.geometry("560x920")
        self.root.resizable(False, False)

        self.device = None
        self.port = None
        self.fa = FlightAxisClient()

        self.mode = "live"
        self.recording_path = recording_path
        self.ticks = []
        self.playing = False
        self.live_running = False

        self.stop_event = threading.Event()
        self.worker_thread = None
        self.gui_queue = queue.Queue()

        self.sync_cfg = load_sync_config()
        self.play_offset = float(self.sync_cfg.get("offset_sec") or 0.0)

        # Live slider vars plus plain floats the worker threads can read safely.
        self.rate_vars = {}
        self.expo_vars = {}
        self.rate_vals = {"rud": DEFAULT_RATE_PCT, "thr": DEFAULT_RATE_PCT,
                          "ele": DEFAULT_RATE_PCT, "ail": DEFAULT_RATE_PCT}
        self.expo_vals = {"rud": DEFAULT_EXPO_PCT, "thr": DEFAULT_EXPO_PCT,
                          "ele": DEFAULT_EXPO_PCT, "ail": DEFAULT_EXPO_PCT}

        if not self._ensure_hardware():
            self.root.destroy()
            return

        self._build_ui()
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.after(33, self._poll_queue)

        if recording_path:
            self._set_mode("play")
            self._load_recording(recording_path)
        else:
            self._set_mode("live")

    def _ensure_hardware(self):
        """Open StickMover if library + USB are present.
        Always allows monitor-only if the user chooses No / library is missing."""
        self.device = None
        self.port = None

        if not HAS_STICKMOVER_LIB:
            choice = messagebox.askyesnocancel(
                "StickMover library not installed",
                "pystickmover is not installed on this PC.\n\n"
                "The GUI can still run in monitor-only mode\n"
                "(bars follow Live / Play / Sync; no servos).\n\n"
                "Yes = Retry after installing pystickmover\n"
                "No = Continue without hardware\n"
                "Cancel = Exit",
                icon=messagebox.WARNING,
            )
            if choice is True:
                return self._ensure_hardware()
            if choice is False:
                return True
            return False

        while True:
            self.port = discover_port()
            if self.port:
                try:
                    self.device = pystickmover.StickMover(self.port, debug=False)
                    return True
                except Exception as e:
                    detail = "Found on %s but failed to open:\n%s" % (self.port, e)
            else:
                detail = (
                    "StickMover USB device not found.\n\n"
                    "Plug it in if you have it.\n"
                    "(VID=0x%04X  PID=0x%04X)" % (TARGET_VID, TARGET_PID)
                )
            choice = messagebox.askyesnocancel(
                "StickMover not found",
                detail + "\n\nYes = Retry\nNo = Continue without hardware (monitor only)\nCancel = Exit",
                icon=messagebox.WARNING,
            )
            if choice is True:
                continue
            if choice is False:
                self.device = None
                self.port = None
                return True
            return False

    def _build_ui(self):
        status = ttk.LabelFrame(self.root, text=" Hardware ", padding=8)
        status.pack(fill="x", padx=12, pady=(10, 4))
        if self.device and self.port:
            hw_text, hw_color = "StickMover: %s" % self.port, "green"
        else:
            hw_text, hw_color = "StickMover: not connected (monitor only)", "orange"
        self.lbl_hw = tk.Label(status, text=hw_text, fg=hw_color, font=("Consolas", 10, "bold"))
        self.lbl_hw.pack(side="left")

        mode_fr = ttk.LabelFrame(self.root, text=" Mode ", padding=8)
        mode_fr.pack(fill="x", padx=12, pady=4)
        self.mode_var = tk.StringVar(value="live")
        ttk.Radiobutton(
            mode_fr, text="Live from RealFlight (FlightAxis)",
            variable=self.mode_var, value="live",
            command=lambda: self._set_mode("live"),
        ).pack(anchor="w")
        ttk.Radiobutton(
            mode_fr, text="Play .recording file  (Load a file, then Play or Sync Play)",
            variable=self.mode_var, value="play",
            command=lambda: self._set_mode("play"),
        ).pack(anchor="w")

        # Live
        self.live_fr = ttk.LabelFrame(self.root, text=" Live FlightAxis ", padding=8)
        self.live_fr.pack(fill="x", padx=12, pady=4)
        self.lbl_fa = ttk.Label(self.live_fr, text="FlightAxis: not checked", font=("Consolas", 9))
        self.lbl_fa.pack(fill="x", pady=(0, 4))
        btn_row = ttk.Frame(self.live_fr)
        btn_row.pack(fill="x")
        self.btn_live_start = ttk.Button(btn_row, text="Start Live", command=self._start_live)
        self.btn_live_start.pack(side="left", padx=(0, 6))
        self.btn_live_stop = ttk.Button(btn_row, text="Stop", command=self._stop_live, state="disabled")
        self.btn_live_stop.pack(side="left")

        # Recording playback
        self.play_fr = ttk.LabelFrame(self.root, text=" Recording Playback ", padding=8)
        self.play_fr.pack(fill="x", padx=12, pady=4)
        self.lbl_rec = ttk.Label(self.play_fr, text="No file loaded", font=("Consolas", 9), wraplength=500)
        self.lbl_rec.pack(fill="x", pady=(0, 4))
        prow = ttk.Frame(self.play_fr)
        prow.pack(fill="x")
        self.btn_load = ttk.Button(prow, text="Load .recording...", command=self._ask_recording)
        self.btn_load.pack(side="left", padx=(0, 6))
        self.btn_play = ttk.Button(prow, text="Play (local only)", command=self._start_play, state="disabled")
        self.btn_play.pack(side="left", padx=6)
        self.btn_stop_play = ttk.Button(prow, text="Stop", command=self._stop_play, state="disabled")
        self.btn_stop_play.pack(side="left")

        # Sync with RealFlight on-screen playback
        self.sync_fr = ttk.LabelFrame(self.root, text=" Sync with RealFlight Playback ", padding=8)
        self.sync_fr.pack(fill="x", padx=12, pady=4)

        self.lbl_play_pos = ttk.Label(self.sync_fr, text=self._play_pos_text(), font=("Consolas", 9))
        self.lbl_play_pos.pack(fill="x", pady=(0, 4))

        srow = ttk.Frame(self.sync_fr)
        srow.pack(fill="x")
        self.btn_learn = ttk.Button(srow, text="Learn Play Button", command=self._learn_play_button)
        self.btn_learn.pack(side="left", padx=(0, 6))
        self.btn_test_click = ttk.Button(srow, text="Test Click", command=self._test_click)
        self.btn_test_click.pack(side="left", padx=6)
        self.btn_sync = ttk.Button(
            srow, text="Sync Play (click RF + stream)", command=self._sync_play, state="disabled",
        )
        self.btn_sync.pack(side="left", padx=6)

        orow = ttk.Frame(self.sync_fr)
        orow.pack(fill="x", pady=(6, 0))
        ttk.Label(orow, text="Offset (sec):").pack(side="left")
        self.offset_var = tk.StringVar(value="%.2f" % self.play_offset)
        self.offset_spin = ttk.Spinbox(
            orow, from_=-1.0, to=2.0, increment=0.05, width=7,
            textvariable=self.offset_var, command=self._save_offset,
        )
        self.offset_spin.pack(side="left", padx=6)
        self.offset_spin.bind("<Return>", lambda e: self._save_offset())
        self.offset_spin.bind("<FocusOut>", lambda e: self._save_offset())
        ttk.Label(
            orow, text="(+ = delay sticks after click)", font=("Segoe UI", 8),
        ).pack(side="left", padx=4)

        ttk.Label(
            self.sync_fr,
            text="How to learn: hover mouse on RealFlight Play, then click Learn Play Button here.",
            font=("Segoe UI", 8), foreground="gray",
        ).pack(anchor="w", pady=(4, 0))

        # Stick monitor
        tele = ttk.LabelFrame(self.root, text=" Live Stick Data  (Mode 2) ", padding=12)
        tele.pack(fill="both", expand=True, padx=12, pady=4)
        tele.columnconfigure(0, weight=1)
        tele.columnconfigure(1, weight=1)
        ttk.Label(tele, text="--- LEFT STICK ---", font=("Segoe UI", 9, "bold")).grid(row=0, column=0, pady=(0, 6))
        ttk.Label(tele, text="--- RIGHT STICK ---", font=("Segoe UI", 9, "bold")).grid(row=0, column=1, pady=(0, 6))

        self.progress_bars = {}
        self.val_labels = {}
        axes = [
            ("Rudder  (Axis 1)", "rud", 1, 0),
            ("Throttle (Axis 2)", "thr", 2, 0),
            ("Aileron  (Axis 4)", "ail", 1, 1),
            ("Elevator (Axis 3)", "ele", 2, 1),
        ]
        for title, key, r, c in axes:
            sub = ttk.Frame(tele, padding=3)
            sub.grid(row=r, column=c, sticky="ew", pady=3, padx=4)
            hdr = ttk.Frame(sub)
            hdr.pack(fill="x")
            ttk.Label(hdr, text=title, font=("Segoe UI", 8, "bold")).pack(side="left")
            vl = ttk.Label(hdr, text="0.50", font=("Consolas", 9))
            vl.pack(side="right")
            self.val_labels[key] = vl
            pb = ttk.Progressbar(sub, orient="horizontal", mode="determinate", maximum=1.0, value=0.5)
            pb.pack(fill="x", pady=(3, 0))
            self.progress_bars[key] = pb

        # Rate / Expo panel — Live, Play, and Sync all use these values.
        shape_frame = ttk.LabelFrame(
            self.root,
            text=" Rate %  /  Expo %  (per axis, like a transmitter) ",
            padding=10
        )
        shape_frame.pack(fill="x", padx=12, pady=4)

        hint = ttk.Label(
            shape_frame,
            text="Rate >100 enlarges small FlightAxis / recording travel.  "
                 "Expo + softens center, Expo - adds throw near center.  "
                 "Elevator often wants Rate 200–300 and Expo -20 to -40.",
            font=("Segoe UI", 8),
            wraplength=520,
            justify="left"
        )
        hint.pack(fill="x", pady=(0, 8))

        shape_axes = [
            ("rud", "Rudder"),
            ("thr", "Throttle"),
            ("ele", "Elevator"),
            ("ail", "Aileron"),
        ]
        for key, title in shape_axes:
            row = ttk.Frame(shape_frame)
            row.pack(fill="x", pady=3)

            ttk.Label(row, text=title, width=10, font=("Segoe UI", 8, "bold")).pack(side="left")

            ttk.Label(row, text="Rate", width=5, font=("Segoe UI", 8)).pack(side="left")
            rate_var = tk.DoubleVar(value=DEFAULT_RATE_PCT)
            self.rate_vars[key] = rate_var
            rate_lbl = ttk.Label(row, text="100", width=4, font=("Consolas", 8))
            ttk.Scale(
                row, from_=RATE_MIN_PCT, to=RATE_MAX_PCT,
                variable=rate_var, orient="horizontal"
            ).pack(side="left", fill="x", expand=True, padx=4)

            def _sync_rate(var=rate_var, lbl=rate_lbl, k=key):
                # Copy to rate_vals so Live/Play worker threads do not call Tk.
                v = float(var.get())
                self.rate_vals[k] = v
                lbl.config(text=str(int(round(v))))

            rate_var.trace_add("write", lambda *_a, fn=_sync_rate: fn())
            rate_lbl.pack(side="left")

            ttk.Label(row, text="Expo", width=5, font=("Segoe UI", 8)).pack(side="left", padx=(8, 0))
            expo_var = tk.DoubleVar(value=DEFAULT_EXPO_PCT)
            self.expo_vars[key] = expo_var
            expo_lbl = ttk.Label(row, text="0", width=4, font=("Consolas", 8))
            ttk.Scale(
                row, from_=EXPO_MIN_PCT, to=EXPO_MAX_PCT,
                variable=expo_var, orient="horizontal"
            ).pack(side="left", fill="x", expand=True, padx=4)

            def _sync_expo(var=expo_var, lbl=expo_lbl, k=key):
                v = float(var.get())
                self.expo_vals[k] = v
                lbl.config(text=str(int(round(v))))

            expo_var.trace_add("write", lambda *_a, fn=_sync_expo: fn())
            expo_lbl.pack(side="left")

        ttk.Button(
            shape_frame,
            text="Reset Rate/Expo to 100 / 0",
            command=self.reset_rate_expo,
        ).pack(anchor="e", pady=(6, 0))

        self.lbl_time = ttk.Label(self.root, text="idle", font=("Consolas", 9), foreground="gray")
        self.lbl_time.pack(pady=(2, 8))
        self._update_bars(0.5, 0.0, 0.5, 0.5)
        self._update_sync_buttons()

    def reset_rate_expo(self):
        """Restore linear 100% rate and 0 expo on all four axes."""
        for key in ("rud", "thr", "ele", "ail"):
            self.rate_vars[key].set(DEFAULT_RATE_PCT)
            self.expo_vars[key].set(DEFAULT_EXPO_PCT)

    def _shape_channels(self, rud, thr, ele, ail):
        """Apply current Rate/Expo to four 0..1 channels (thread-safe floats)."""
        rud = shape_hw(rud, self.rate_vals["rud"], self.expo_vals["rud"])
        thr = shape_hw(thr, self.rate_vals["thr"], self.expo_vals["thr"])
        ele = shape_hw(ele, self.rate_vals["ele"], self.expo_vals["ele"])
        ail = shape_hw(ail, self.rate_vals["ail"], self.expo_vals["ail"])
        return rud, thr, ele, ail

    def _play_pos_text(self):
        x, y = self.sync_cfg.get("play_x"), self.sync_cfg.get("play_y")
        if x is not None and y is not None:
            return "Play button: (%d, %d)" % (int(x), int(y))
        return "Play button: not learned yet"

    def _save_offset(self):
        try:
            self.play_offset = float(self.offset_var.get())
        except ValueError:
            self.play_offset = 0.0
            self.offset_var.set("0.00")
        self.sync_cfg["offset_sec"] = self.play_offset
        save_sync_config(self.sync_cfg)

    def _update_sync_buttons(self):
        has_file = bool(self.ticks)
        has_pos = self.sync_cfg.get("play_x") is not None and self.sync_cfg.get("play_y") is not None
        self.btn_sync.config(state="normal" if (has_file and has_pos and not self.playing) else "disabled")
        self.btn_play.config(state="normal" if (has_file and not self.playing) else "disabled")

    def _learn_play_button(self):
        messagebox.showinfo(
            "Learn Play Button",
            "After you click OK, a 3-second countdown starts.\n\n"
            "During the countdown:\n"
            "1. Alt+Tab to RealFlight (fullscreen OK).\n"
            "2. Put the mouse tip exactly on the Play control.\n"
            "3. Hold still until the countdown finishes.\n\n"
            "The cursor position at the end of the countdown is saved.",
        )
        self.lbl_time.config(text="learn: switch to RF, aim at Play...")
        self.root.update()

        def finish_learn():
            try:
                x, y = get_cursor_pos()
            except Exception as e:
                messagebox.showerror("Learn failed", str(e))
                return
            self.sync_cfg["play_x"] = x
            self.sync_cfg["play_y"] = y
            save_sync_config(self.sync_cfg)
            self.lbl_play_pos.config(text=self._play_pos_text())
            self._update_sync_buttons()
            self.lbl_time.config(text="learn saved (%d, %d)" % (x, y))
            messagebox.showinfo("Saved", "Play button position saved: (%d, %d)" % (x, y))

        # 3 second countdown so user can Alt+Tab and aim
        def tick(n):
            if n <= 0:
                finish_learn()
                return
            self.lbl_time.config(text="learn: %d - aim at Play in RF..." % n)
            self.root.after(1000, lambda: tick(n - 1))

        self.root.after(200, lambda: tick(3))

    def _test_click(self):
        """Click the learned position only (no stick stream) so you can verify RF starts."""
        x = self.sync_cfg.get("play_x")
        y = self.sync_cfg.get("play_y")
        if x is None or y is None:
            messagebox.showerror("Not learned", "Learn the Play button position first.")
            return
        self.lbl_time.config(text="test click at (%d, %d)..." % (int(x), int(y)))
        self.root.update()
        try:
            click_at(int(x), int(y))
            self.lbl_time.config(text="test click done - did RF start?")
        except Exception as e:
            messagebox.showerror("Click failed", str(e))

    def _sync_play(self):
        """Click RealFlight Play, wait offset, then stream sticks from the file."""
        if not self.ticks or self.playing:
            return
        x = self.sync_cfg.get("play_x")
        y = self.sync_cfg.get("play_y")
        if x is None or y is None:
            messagebox.showerror("Not learned", "Learn the Play button position first.")
            return
        self._save_offset()

        self.playing = True
        self.stop_event.clear()
        self.btn_play.config(state="disabled")
        self.btn_sync.config(state="disabled")
        self.btn_stop_play.config(state="normal")
        self.btn_load.config(state="disabled")
        self.btn_live_start.config(state="disabled")
        self.lbl_time.config(text="sync: clicking RF Play...")

        def run():
            try:
                click_at(int(x), int(y))
                offset = self.play_offset
                if offset > 0:
                    self.gui_queue.put(("status", "sync: waiting %.2fs..." % offset))
                    end = time.perf_counter() + offset
                    while time.perf_counter() < end:
                        if self.stop_event.is_set():
                            return
                        time.sleep(0.01)
                elif offset < 0:
                    # Negative: start sticks early, then click
                    # (rare; usually offset is small positive)
                    pass
                self.gui_queue.put(("status", "sync playing..."))
                self._play_worker_body(follow_rf=True)
            except Exception as e:
                self.gui_queue.put(("error", "Sync click/play: %s" % e))
            finally:
                self.gui_queue.put(("play_done",))

        self.worker_thread = threading.Thread(target=run, daemon=True)
        self.worker_thread.start()

    def _update_bars(self, rud, thr, ele, ail):
        for k, v in (("rud", rud), ("thr", thr), ("ele", ele), ("ail", ail)):
            self.progress_bars[k]["value"] = v
            self.val_labels[k].config(text="%.2f" % v)

    def _set_mode(self, mode):
        self._stop_all()
        self.mode = mode
        self.mode_var.set(mode)
        if mode == "live":
            self.btn_live_start.config(state="normal")
            self._check_flightaxis()
        else:
            self.btn_live_start.config(state="disabled")

    def _check_flightaxis(self):
        ok = self.fa.ping()
        if ok:
            self.lbl_fa.config(text="FlightAxis: connected  (127.0.0.1:18083)", foreground="green")
        else:
            self.lbl_fa.config(
                text="FlightAxis: not reachable - enable RealFlight Link / FlightAxis",
                foreground="red",
            )

    def _start_live(self):
        if self.live_running:
            return
        self._check_flightaxis()
        if not self.fa.ping():
            messagebox.showerror(
                "FlightAxis offline",
                "Cannot reach RealFlight Link on 127.0.0.1:18083.\n\n"
                "Restart RealFlight, enable RealFlight Link, start a flight, then try again.",
            )
            return
        self.live_running = True
        self.stop_event.clear()
        self.btn_live_start.config(state="disabled")
        self.btn_live_stop.config(state="normal")
        self.btn_play.config(state="disabled")
        self.btn_sync.config(state="disabled")
        self.btn_load.config(state="disabled")
        self.lbl_time.config(text="live streaming...")
        self.worker_thread = threading.Thread(target=self._live_worker, daemon=True)
        self.worker_thread.start()

    def _stop_live(self):
        self.stop_event.set()
        self.live_running = False
        if self.worker_thread and self.worker_thread.is_alive():
            self.worker_thread.join(timeout=1.5)
        self.worker_thread = None
        self.fa.restore()
        self._reset_hardware()
        self.btn_live_start.config(state="normal")
        self.btn_live_stop.config(state="disabled")
        self.btn_load.config(state="normal")
        self._update_sync_buttons()
        self.lbl_time.config(text="live stopped")

    def _live_worker(self):
        interval = 1.0 / LIVE_POLL_HZ
        try:
            while not self.stop_event.is_set():
                t0 = time.perf_counter()
                try:
                    ch = self.fa.exchange()
                    ail = max(0.0, min(1.0, ch[CHANNEL_MAP["ail"]]))
                    ele = max(0.0, min(1.0, ch[CHANNEL_MAP["ele"]]))
                    thr = max(0.0, min(1.0, ch[CHANNEL_MAP["thr"]]))
                    rud = max(0.0, min(1.0, ch[CHANNEL_MAP["rud"]]))
                    # Shape after FlightAxis read so bars match servos
                    rud, thr, ele, ail = self._shape_channels(rud, thr, ele, ail)
                    if self.device:
                        self.device.axis1 = rud
                        self.device.axis2 = thr
                        self.device.axis3 = ele
                        self.device.axis4 = ail
                        self.device.update()
                    self.gui_queue.put(("values", 0.0, rud, thr, ele, ail))
                except Exception as e:
                    self.gui_queue.put(("error", "FlightAxis: %s" % e))
                    break
                elapsed = time.perf_counter() - t0
                sleep = interval - elapsed
                if sleep > 0:
                    end = time.perf_counter() + sleep
                    while time.perf_counter() < end and not self.stop_event.is_set():
                        time.sleep(max(0.0, min(0.01, end - time.perf_counter())))
        finally:
            self.gui_queue.put(("live_done",))

    def _ask_recording(self):
        path = filedialog.askopenfilename(
            title="Select RealFlight .recording",
            filetypes=[("RealFlight recording", "*.recording"), ("All files", "*.*")],
        )
        if path:
            self._load_recording(path)

    def _load_recording(self, path):
        try:
            raw = stick_frames(path)
            self.ticks = interp(raw, hz=DEFAULT_HZ)
            dur = self.ticks[-1][0] if self.ticks else 0.0
            self.recording_path = path
            short = path.replace("\\", "/").split("/")[-1]
            self.lbl_rec.config(
                text="%s\n%d frames @ %.0f Hz  (%.1f s)" % (short, len(self.ticks), DEFAULT_HZ, dur),
            )
            self.lbl_time.config(text="ready  (%.1f s)" % dur)
            self._update_sync_buttons()
        except Exception as e:
            messagebox.showerror("Load failed", "Could not decode recording:\n%s" % e)
            self.ticks = []
            self._update_sync_buttons()

    def _start_play(self):
        if not self.ticks or self.playing:
            return
        self.playing = True
        self.stop_event.clear()
        self.btn_play.config(state="disabled")
        self.btn_sync.config(state="disabled")
        self.btn_stop_play.config(state="normal")
        self.btn_load.config(state="disabled")
        self.btn_live_start.config(state="disabled")
        self.lbl_time.config(text="playing...")
        self.worker_thread = threading.Thread(target=self._play_worker, daemon=True)
        self.worker_thread.start()

    def _stop_play(self):
        self.stop_event.set()
        self.playing = False
        if self.worker_thread and self.worker_thread.is_alive():
            self.worker_thread.join(timeout=1.5)
        self.worker_thread = None
        self._reset_hardware()
        self.btn_stop_play.config(state="disabled")
        self.btn_load.config(state="normal")
        self.btn_live_start.config(state="normal")
        self._update_sync_buttons()
        self.lbl_time.config(text="playback stopped")

    def _play_worker_body(self, follow_rf=False):
        if follow_rf:
            self._play_follow_rf_clock()
            return
        start = time.perf_counter()
        for t, ail, ele, thr, rud in self.ticks:
            if self.stop_event.is_set():
                break
            lag = t - (time.perf_counter() - start)
            if lag > 0:
                end = time.perf_counter() + lag
                while time.perf_counter() < end:
                    if self.stop_event.is_set():
                        return
                    time.sleep(max(0.0, min(0.01, end - time.perf_counter())))
            self._emit_play_frame(t, ail, ele, thr, rud)

    def _play_follow_rf_clock(self):
        """Pace recording sticks to RealFlight physics time / speed multiplier.
        Loops when RF restarts the recording; Stop ends it."""
        try:
            speed, phys0 = self.fa.exchange_clock()
        except Exception:
            self.gui_queue.put(("status", "RF clock unavailable - playing at 100%"))
            self._play_worker_body(follow_rf=False)
            return

        last_speed = speed
        last_poll = time.perf_counter()
        last_phys = phys0
        i = 0
        n = len(self.ticks)
        if n == 0:
            return
        dur = self.ticks[-1][0]
        if dur <= 0:
            return
        step = 1.0 / DEFAULT_HZ
        self.gui_queue.put(("status", "sync playing @ %.0f%%" % (last_speed * 100.0)))

        while not self.stop_event.is_set():
            now = time.perf_counter()
            rec_t = None
            if now - last_poll >= 0.15:
                try:
                    speed, phys = self.fa.exchange_clock()
                    last_speed = speed
                    # Recording looped: physics clock jumped backward
                    if phys < last_phys - 0.25:
                        phys0 = phys
                        i = 0
                        self.gui_queue.put(("status", "sync loop @ %.0f%%" % (last_speed * 100.0)))
                    last_phys = phys
                    last_poll = now
                    rec_t = max(0.0, phys - phys0)
                except Exception:
                    rec_t = max(0.0, (last_phys - phys0) + (now - last_poll) * last_speed)
            if rec_t is None:
                rec_t = max(0.0, (last_phys - phys0) + (now - last_poll) * last_speed)

            if rec_t >= dur:
                # Continuous physics clock + RF looping the file
                extra = rec_t - dur
                if extra >= 0.25:
                    loops = int(rec_t / dur)
                    rec_t = rec_t - loops * dur
                    i = 0
                    self.gui_queue.put(("status", "sync loop @ %.0f%%" % (last_speed * 100.0)))
                else:
                    t, ail, ele, thr, rud = self.ticks[-1]
                    if not self._emit_play_frame(t, ail, ele, thr, rud, last_speed):
                        return
                    time.sleep(max(0.0, step))
                    continue

            while i + 1 < n and self.ticks[i + 1][0] <= rec_t:
                i += 1
            t, ail, ele, thr, rud = self.ticks[i]
            if not self._emit_play_frame(t, ail, ele, thr, rud, last_speed):
                return
            time.sleep(max(0.0, step))

    def _emit_play_frame(self, t, ail, ele, thr, rud, speed=None):
        # Shape Play / Sync frames the same way as Live
        rud, thr, ele, ail = self._shape_channels(rud, thr, ele, ail)
        if self.device:
            self.device.axis1 = rud
            self.device.axis2 = thr
            self.device.axis3 = ele
            self.device.axis4 = ail
            try:
                self.device.update()
            except Exception as e:
                self.gui_queue.put(("error", str(e)))
                return False
        self.gui_queue.put(("values", t, rud, thr, ele, ail, speed))
        return True

    def _play_worker(self):
        try:
            self._play_worker_body()
        except Exception as e:
            self.gui_queue.put(("error", str(e)))
        finally:
            self.gui_queue.put(("play_done",))

    def _stop_all(self):
        if self.live_running:
            self._stop_live()
        if self.playing:
            self._stop_play()

    def _reset_hardware(self):
        if self.device:
            self.device.axis1 = 0.5
            self.device.axis2 = 0.0
            self.device.axis3 = 0.5
            self.device.axis4 = 0.5
            try:
                self.device.update()
            except Exception:
                pass
        self._update_bars(0.5, 0.0, 0.5, 0.5)

    def _poll_queue(self):
        try:
            while True:
                msg = self.gui_queue.get_nowait()
                kind = msg[0]
                if kind == "values":
                    _, t, rud, thr, ele, ail = msg[:6]
                    speed = msg[6] if len(msg) > 6 else None
                    self._update_bars(rud, thr, ele, ail)
                    if self.live_running:
                        self.lbl_time.config(text="live streaming...")
                    elif speed:
                        self.lbl_time.config(
                            text="t = %06.2f s   |   playing  |  RF %.0f%%" % (t, speed * 100.0)
                        )
                    else:
                        self.lbl_time.config(text="t = %06.2f s   |   playing" % t)
                elif kind == "status":
                    self.lbl_time.config(text=msg[1])
                elif kind == "error":
                    messagebox.showerror("Error", msg[1])
                    self._stop_all()
                elif kind == "live_done":
                    self.live_running = False
                    self._reset_hardware()
                    self.btn_live_start.config(state="normal")
                    self.btn_live_stop.config(state="disabled")
                    self.btn_load.config(state="normal")
                    self._update_sync_buttons()
                    self.lbl_time.config(text="live stopped")
                elif kind == "play_done":
                    self.playing = False
                    self._reset_hardware()
                    self.btn_stop_play.config(state="disabled")
                    self.btn_load.config(state="normal")
                    self.btn_live_start.config(state="normal")
                    self._update_sync_buttons()
                    self.lbl_time.config(text="playback finished")
        except queue.Empty:
            pass
        self.root.after(33, self._poll_queue)

    def on_close(self):
        self.stop_event.set()
        if self.worker_thread and self.worker_thread.is_alive():
            self.worker_thread.join(timeout=1.0)
        self.fa.restore()
        self._reset_hardware()
        self.root.destroy()


def main():
    recording = sys.argv[1] if len(sys.argv) > 1 else None
    root = tk.Tk()
    root.withdraw()
    app = StickMoverBridgeGUI(root, recording_path=recording)
    try:
        root.deiconify()
        root.mainloop()
    except tk.TclError:
        pass


if __name__ == "__main__":
    main()