# -*- coding: utf-8 -*-
import sys
import time
import ctypes
from ctypes import wintypes
import tkinter as tk
from tkinter import ttk, messagebox
import serial.tools.list_ports

# Add local package path for pystickmover
true_package_path = r"C:\Users\Stand\Downloads\pystickmover-0.1\pystickmover-0.1"
if true_package_path not in sys.path:
    sys.path.insert(0, true_package_path)

try:
    from pystickmover import pystickmover
except ImportError:
    sys.path.append(true_package_path)
    from pystickmover import pystickmover

# Target FTDI IDs for StickMover
TARGET_VID = 0x0403
TARGET_PID = 0x6015

# Windows API setup
user32 = ctypes.WinDLL("user32")
gdi32 = ctypes.WinDLL("gdi32")

VK_SPACE = 0x20
SRCCOPY = 0x00CC0020
BI_RGB = 0

LAST_KNOWN_POS = {
    'left': (0.0, -1.0),
    'right': (0.0, 0.0)
}

# Default dual-rate / expo (RC-transmitter style).
# Rate 100 and Expo 0 leave the captured gadget values unchanged.
DEFAULT_RATE_PCT = 100.0
DEFAULT_EXPO_PCT = 0.0
# Rate above 100% stretches the small Radio Gadget red-dot travel
# so the StickMover can reach nearer the physical endpoints.
RATE_MIN_PCT = 50.0
RATE_MAX_PCT = 400.0
# Expo percent: +softens center, -sharpens center (helps slight elevator).
EXPO_MIN_PCT = -100.0
EXPO_MAX_PCT = 100.0


class BITMAPINFOHEADER(ctypes.Structure):
    _fields_ = [
        ("biSize", wintypes.DWORD),
        ("biWidth", ctypes.c_long),
        ("biHeight", ctypes.c_long),
        ("biPlanes", wintypes.WORD),
        ("biBitCount", wintypes.WORD),
        ("biCompression", wintypes.DWORD),
        ("biSizeImage", wintypes.DWORD),
        ("biXPelsPerMeter", ctypes.c_long),
        ("biYPelsPerMeter", ctypes.c_long),
        ("biClrUsed", wintypes.DWORD),
        ("biClrImportant", wintypes.DWORD),
    ]

def discover_port():
    for port in serial.tools.list_ports.comports():
        if port.vid == TARGET_VID and port.pid == TARGET_PID:
            return port.device
    return None

def get_mouse_pos():
    pt = wintypes.POINT()
    user32.GetCursorPos(ctypes.byref(pt))
    return pt.x, pt.y

def capture_screen_block(hdc_screen, center_x, center_y, stick_key, box_radius=45):
    width = box_radius * 2
    height = box_radius * 2
    top_left_x = center_x - box_radius
    top_left_y = center_y - box_radius

    hdc_mem = gdi32.CreateCompatibleDC(hdc_screen)
    hbmp = gdi32.CreateCompatibleBitmap(hdc_screen, width, height)
    gdi32.SelectObject(hdc_mem, hbmp)

    gdi32.BitBlt(hdc_mem, 0, 0, width, height, hdc_screen, top_left_x, top_left_y, SRCCOPY)

    bmi = BITMAPINFOHEADER()
    bmi.biSize = ctypes.sizeof(BITMAPINFOHEADER)
    bmi.biWidth = width
    bmi.biHeight = -height
    bmi.biPlanes = 1
    bmi.biBitCount = 32
    bmi.biCompression = BI_RGB

    buffer_size = width * height * 4
    pixel_buffer = (ctypes.c_ubyte * buffer_size)()

    gdi32.GetDIBits(hdc_mem, hbmp, 0, height, ctypes.byref(pixel_buffer), ctypes.byref(bmi), 0)

    gdi32.DeleteObject(hbmp)
    gdi32.DeleteDC(hdc_mem)

    red_x_sum = 0
    red_y_sum = 0
    count = 0

    for y in range(0, height, 2):
        for x in range(0, width, 2):
            idx = (y * width + x) * 4
            b = pixel_buffer[idx]
            g = pixel_buffer[idx + 1]
            r = pixel_buffer[idx + 2]

            if r > 150 and g < 60 and b < 60:
                red_x_sum += x
                red_y_sum += y
                count += 1

    if count == 0:
        return LAST_KNOWN_POS[stick_key]

    avg_x = red_x_sum / count
    avg_y = red_y_sum / count

    norm_x = (avg_x - box_radius) / 30.0
    norm_y = (avg_y - box_radius) / 30.0

    norm_x = round(max(-1.0, min(1.0, norm_x)), 2)
    norm_y = round(max(-1.0, min(1.0, norm_y)), 2)

    LAST_KNOWN_POS[stick_key] = (norm_x, norm_y)

    return norm_x, norm_y

def to_hardware_range(val_norm):
    hw_val = (val_norm + 1.0) / 2.0
    return round(max(0.0, min(1.0, hw_val)), 2)


def apply_rate(val_norm, rate_pct):
    """Scale a -1..+1 stick by dual-rate percent, then clamp.

    100% = captured gadget motion unchanged.
    >100% amplifies small Radio Gadget red-dot travel toward full throw.
    """
    scaled = val_norm * (float(rate_pct) / 100.0)
    return max(-1.0, min(1.0, scaled))


def apply_expo(val_norm, expo_pct):
    """RC-transmitter expo on a -1..+1 axis.

    Formula: y = x * (1 - e + e * x^2)  with e = expo/100.
      expo = 0    linear
      expo > 0    softer around center (classic radio expo)
      expo < 0    more throw around center (helps slight elevator / rudder)
    Endpoints stay at +/-1 after clamp. Does not create travel the gadget
    never produced — use Rate % for that.
    """
    x = max(-1.0, min(1.0, float(val_norm)))
    e = max(EXPO_MIN_PCT, min(EXPO_MAX_PCT, float(expo_pct))) / 100.0
    y = x * (1.0 - e + e * x * x)
    return max(-1.0, min(1.0, y))


def shape_axis(val_norm, rate_pct, expo_pct):
    """Apply Rate then Expo, same order many radios use after raw stick."""
    return apply_expo(apply_rate(val_norm, rate_pct), expo_pct)


class StickMoverBridgeGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("RealFlight StickMover Telemetry Bridge")
        self.root.geometry("560x860")
        self.root.resizable(False, False)

        self.device = None
        self.port = None
        self.hdc_screen = None

        self.cal_step = 1
        self.space_latched = False
        self.lx, self.ly = 0, 0
        self.rx, self.ry = 0, 0

        # Per-axis Rate % and Expo % (Rudder / Throttle / Elevator / Aileron)
        self.rate_vars = {}
        self.expo_vars = {}

        self.connect_hardware()
        self.create_widgets()

        self.hdc_screen = user32.GetDC(0)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        # Force focus on root frame so buttons don't steal spacebar presses
        self.root.focus_set()

        self.listen_for_spacebar()

    def connect_hardware(self):
        self.port = discover_port()
        if self.port:
            try:
                self.device = pystickmover.StickMover(self.port, debug=False)
            except Exception as e:
                print(f"[X] Hardware Init Error: {e}")

    def create_widgets(self):
        status_frame = ttk.LabelFrame(self.root, text=" Hardware Connection ", padding=10)
        status_frame.pack(fill="x", padx=15, pady=10)

        port_str = self.port if self.port else "Not Detected"
        status_color = "green" if self.device else "red"

        self.lbl_status = tk.Label(
            status_frame,
            text=f"StickMover Port: {port_str}",
            fg=status_color,
            font=("Consolas", 10, "bold")
        )
        self.lbl_status.pack(side="left")

        self.cal_frame = ttk.LabelFrame(self.root, text=" Calibration Steps ", padding=15)
        self.cal_frame.pack(fill="x", padx=15, pady=5)

        self.lbl_cal_instruct = ttk.Label(
            self.cal_frame,
            text="[Step 1] Leave Throttle FULL DOWN (Idle) & Rudder CENTERED.\n"
                 "Hover mouse over LEFT RED TIP and press SPACEBAR.",
            font=("Segoe UI", 9, "bold"),
            justify="left"
        )
        self.lbl_cal_instruct.pack(fill="x", pady=5)

        self.lbl_cal_readout = ttk.Label(
            self.cal_frame,
            text="Left Stick: Pending... | Right Stick: Pending...",
            font=("Consolas", 9),
            foreground="gray"
        )
        self.lbl_cal_readout.pack(fill="x", pady=5)

        telemetry_frame = ttk.LabelFrame(self.root, text=" Live Telemetry Stream ", padding=15)
        telemetry_frame.pack(fill="both", expand=True, padx=15, pady=10)

        telemetry_frame.columnconfigure(0, weight=1, pad=10)
        telemetry_frame.columnconfigure(1, weight=1, pad=10)

        self.progress_bars = {}
        self.val_labels = {}

        axes_config = [
            ("Rudder (Axis 1)", "rud", 0, 0),
            ("Throttle (Axis 2)", "thr", 1, 0),
            ("Aileron (Axis 4)", "ail", 0, 1),
            ("Elevator (Axis 3)", "ele", 1, 1),
        ]

        ttk.Label(telemetry_frame, text="--- LEFT STICK ---", font=("Segoe UI", 9, "bold")).grid(row=0, column=0, pady=(0, 5))
        ttk.Label(telemetry_frame, text="--- RIGHT STICK ---", font=("Segoe UI", 9, "bold")).grid(row=0, column=1, pady=(0, 5))

        for title, key, r, c in axes_config:
            sub = ttk.Frame(telemetry_frame, padding=5)
            grid_row = (r * 2) + 1
            sub.grid(row=grid_row, column=c, sticky="ew", pady=5)

            hdr = ttk.Frame(sub)
            hdr.pack(fill="x")

            ttk.Label(hdr, text=title, font=("Segoe UI", 8, "bold")).pack(side="left")
            v_lbl = ttk.Label(hdr, text="0.50", font=("Consolas", 9))
            v_lbl.pack(side="right")
            self.val_labels[key] = v_lbl

            pbar = ttk.Progressbar(sub, orient="horizontal", mode="determinate", maximum=1.0, value=0.5)
            pbar.pack(fill="x", pady=(5, 0))
            self.progress_bars[key] = pbar

        # Rate / Expo panel — live sliders, no restart needed
        shape_frame = ttk.LabelFrame(
            self.root,
            text=" Rate %  /  Expo %  (per axis, like a transmitter) ",
            padding=10
        )
        shape_frame.pack(fill="x", padx=15, pady=5)

        hint = ttk.Label(
            shape_frame,
            text="Rate >100 enlarges small Radio Gadget travel.  "
                 "Expo + softens center, Expo - adds throw near center.  "
                 "Elevator often wants Rate 200–300 and Expo -20 to -40.",
            font=("Segoe UI", 8),
            wraplength=510,
            justify="left"
        )
        hint.pack(fill="x", pady=(0, 8))

        # Slider rows: Rud, Thr, Ele, Ail
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
            rate_scale = ttk.Scale(
                row, from_=RATE_MIN_PCT, to=RATE_MAX_PCT,
                variable=rate_var, orient="horizontal"
            )
            rate_scale.pack(side="left", fill="x", expand=True, padx=4)

            def _sync_rate(var=rate_var, lbl=rate_lbl):
                # Integer display; value is read live in update_telemetry
                lbl.config(text=str(int(round(var.get()))))

            rate_var.trace_add("write", lambda *_a, fn=_sync_rate: fn())
            rate_lbl.pack(side="left")

            ttk.Label(row, text="Expo", width=5, font=("Segoe UI", 8)).pack(side="left", padx=(8, 0))
            expo_var = tk.DoubleVar(value=DEFAULT_EXPO_PCT)
            self.expo_vars[key] = expo_var
            expo_lbl = ttk.Label(row, text="0", width=4, font=("Consolas", 8))
            expo_scale = ttk.Scale(
                row, from_=EXPO_MIN_PCT, to=EXPO_MAX_PCT,
                variable=expo_var, orient="horizontal"
            )
            expo_scale.pack(side="left", fill="x", expand=True, padx=4)

            def _sync_expo(var=expo_var, lbl=expo_lbl):
                lbl.config(text=str(int(round(var.get()))))

            expo_var.trace_add("write", lambda *_a, fn=_sync_expo: fn())
            expo_lbl.pack(side="left")

        reset_rates_btn = ttk.Button(
            shape_frame,
            text="Reset Rate/Expo to 100 / 0",
            command=self.reset_rate_expo,
            takefocus=0
        )
        reset_rates_btn.pack(anchor="e", pady=(6, 0))

        btn_frame = ttk.Frame(self.root, padding=15)
        btn_frame.pack(fill="x")

        # takefocus=0 PREVENTS Spacebar from triggering the buttons
        self.recal_btn = ttk.Button(btn_frame, text="Recalibrate (Reset Spacebar Setup)", command=self.reset_calibration, takefocus=0)
        self.recal_btn.pack(side="left", expand=True, fill="x", padx=5)

        stop_btn = ttk.Button(btn_frame, text="Stop & Close Hardware Bridge", command=self.on_close, takefocus=0)
        stop_btn.pack(side="right", expand=True, fill="x", padx=5)

    def reset_rate_expo(self):
        """Restore linear 100% rate and 0 expo on all four axes."""
        for key in ("rud", "thr", "ele", "ail"):
            self.rate_vars[key].set(DEFAULT_RATE_PCT)
            self.expo_vars[key].set(DEFAULT_EXPO_PCT)

    def _axis_rate(self, key):
        return float(self.rate_vars[key].get())

    def _axis_expo(self, key):
        return float(self.expo_vars[key].get())

    def listen_for_spacebar(self):
        """Global key polling loop with strict latch protection."""
        if self.cal_step in (1, 2):
            space_pressed = bool(user32.GetAsyncKeyState(VK_SPACE) & 0x8000)

            if space_pressed and not self.space_latched:
                self.space_latched = True
                self.advance_step()
            elif not space_pressed:
                self.space_latched = False

            self.root.after(30, self.listen_for_spacebar)

    def advance_step(self):
        """Handles step progression explicitly."""
        if self.cal_step == 1:
            x, y = get_mouse_pos()
            self.lx = x
            self.ly = y - 30
            self.cal_step = 2

            self.lbl_cal_instruct.config(
                text="[Step 2] Hover mouse over RIGHT STICK CENTER.\n"
                     "Press SPACEBAR to lock right gimbal and start telemetry stream."
            )
            self.lbl_cal_readout.config(text=f"Left Stick: Locked (X:{self.lx}, Y:{self.ly}) | Right Stick: Pending...")

        elif self.cal_step == 2:
            x, y = get_mouse_pos()
            self.rx = x
            self.ry = y
            self.cal_step = 3  # Mark state as complete FIRST

            self.lbl_cal_instruct.config(text="[✓] Telemetry Active! Streaming to StickMover at 60 FPS.")
            self.lbl_cal_readout.config(
                text=f"Left: Locked (X:{self.lx}, Y:{self.ly}) | Right: Locked (X:{self.rx}, Y:{self.ry})"
            )
            self.root.update_idletasks()

            # Launch main tracking loop after UI repaints
            self.root.after(20, self.update_telemetry)

    def update_telemetry(self):
        if self.cal_step != 3:
            return

        try:
            rud_norm, thr_norm = capture_screen_block(self.hdc_screen, self.lx, self.ly, 'left')
            thr_norm = -thr_norm

            ail_norm, ele_norm = capture_screen_block(self.hdc_screen, self.rx, self.ry, 'right')
            ele_norm = -ele_norm

            # Shape each axis: Rate (enlarge small gadget travel) then Expo (center feel)
            rud_norm = shape_axis(rud_norm, self._axis_rate("rud"), self._axis_expo("rud"))
            thr_norm = shape_axis(thr_norm, self._axis_rate("thr"), self._axis_expo("thr"))
            ele_norm = shape_axis(ele_norm, self._axis_rate("ele"), self._axis_expo("ele"))
            ail_norm = shape_axis(ail_norm, self._axis_rate("ail"), self._axis_expo("ail"))

            hw_rud = to_hardware_range(rud_norm)
            hw_thr = to_hardware_range(thr_norm)
            hw_ele = to_hardware_range(ele_norm)
            hw_ail = to_hardware_range(ail_norm)

            if self.device:
                self.device.axis1 = hw_rud
                self.device.axis2 = hw_thr
                self.device.axis3 = hw_ele
                self.device.axis4 = hw_ail
                try:
                    self.device.update()
                except Exception as e:
                    print(f"[X] Transmit Error: {e}")

            self.progress_bars['rud']['value'] = hw_rud
            self.val_labels['rud'].config(text=f"{hw_rud:.2f}")

            self.progress_bars['thr']['value'] = hw_thr
            self.val_labels['thr'].config(text=f"{hw_thr:.2f}")

            self.progress_bars['ail']['value'] = hw_ail
            self.val_labels['ail'].config(text=f"{hw_ail:.2f}")

            self.progress_bars['ele']['value'] = hw_ele
            self.val_labels['ele'].config(text=f"{hw_ele:.2f}")

        except Exception as err:
            print(f"[X] Telemetry Frame Error: {err}")

        if self.cal_step == 3:
            self.root.after(16, self.update_telemetry)

    def reset_calibration(self):
        global LAST_KNOWN_POS
        
        self.cal_step = 0
        self.space_latched = True  # Prevent key-up glitch on reset
        self.lx, self.ly = 0, 0
        self.rx, self.ry = 0, 0

        if self.device:
            self.device.axis1 = 0.5
            self.device.axis2 = 0.0
            self.device.axis3 = 0.5
            self.device.axis4 = 0.5
            try:
                self.device.update()
            except Exception:
                pass

        LAST_KNOWN_POS = {
            'left': (0.0, -1.0),
            'right': (0.0, 0.0)
        }

        for key in ['rud', 'ail', 'ele']:
            self.progress_bars[key]['value'] = 0.5
            self.val_labels[key].config(text="0.50")
        self.progress_bars['thr']['value'] = 0.0
        self.val_labels['thr'].config(text="0.00")

        self.cal_step = 1
        self.lbl_cal_instruct.config(
            text="[Step 1] Leave Throttle FULL DOWN (Idle) & Rudder CENTERED.\n"
                 "Hover mouse over LEFT RED TIP and press SPACEBAR."
        )
        self.lbl_cal_readout.config(text="Left Stick: Pending... | Right Stick: Pending...")
        
        self.root.focus_set()
        
        # Delay turning key listener back on until spacebar is fully released
        self.root.after(200, self.listen_for_spacebar)

    def on_close(self):
        self.cal_step = 0
        if self.hdc_screen:
            user32.ReleaseDC(0, self.hdc_screen)
        if self.device:
            self.device.axis1 = 0.5
            self.device.axis2 = 0.0
            self.device.axis3 = 0.5
            self.device.axis4 = 0.5
            try:
                self.device.update()
            except Exception:
                pass

        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = StickMoverBridgeGUI(root)
    root.mainloop()