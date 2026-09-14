# RealFlight StickMover

Two distinct Windows Tkinter GUIs that drive **AVIrem StickMover** hardware. They share the same Mode 2 axis map and the same Rate / Expo shaping. They do **not** share a data source.

StickMover hardware is an AVIrem product ([avirem.de](https://www.avirem.de/)). These GUIs are a RealFlight / Radio Gadget front end on top of Robinson’s driver, not a replacement for it. AVIrem discontinued support, so the original software no longer makes the hardware usable. Without `pystickmover` the GUIs can still open in monitor-only mode.

| Script | 64-bit Windows exe | Stick source | Typical use |
| --- | --- | --- | --- |
| `RF-recording-stickmover.py` | `RF-recording-stickmover.exe` | RealFlight **data** (FlightAxis SOAP and/or a `.recording` file) | Live InterLink, file playback, or Sync Play with the sim clock |
| `Radio-gadget-red-stickmover.py` | `Radio-gadget-red-stickmover.exe` | **Red-tip pixel tracking** on screen | Radio Gadget overlay during RealFlight recording playback, or a YouTube / video window of the same sim view |

Hardware is optional on both. If USB or `pystickmover` is missing, the GUI can run monitor-only and you can retry later.

Copy either `.py` next to RealFlight or keep it anywhere on the PC. The scripts look for Robinson’s **0.1** driver beside themselves or under the current user’s `Downloads` folder.

---

## 1. FlightAxis / recording — `RF-recording-stickmover.py`

Reads stick positions from RealFlight itself. No camera, no screen scrape.

### Modes

| Mode | Source | What happens |
| --- | --- | --- |
| **Live** | FlightAxis at `http://127.0.0.1:18083` | Injects `InjectUAVControllerInterface`, polls `ExchangeData` (`selectedChannels=0`), follows InterLink sticks |
| **Play** | Built-in `.recording` decoder | Loads a RealFlight recording; decoder is inside this file (`rec_sticks.py` is not required) |
| **Sync Play** | Same file + sim clock | Clicks the on-screen RealFlight Play button, then streams frames using `m-currentPhysicsSpeedMultiplier` and `m-currentPhysicsTime-SEC`. Loops until Stop |

Sync Play button coordinates and time offset are stored next to the script in `stickmover_sync_config.json`.

### FlightAxis

- URL: `http://127.0.0.1:18083`
- Inject: `InjectUAVControllerInterface`
- Data: `ExchangeData`, `selectedChannels=0`

### Run

```bat
python RF-recording-stickmover.py
```

1. Start RealFlight. Confirm FlightAxis on port **18083**.
2. Plug in StickMover, or skip for monitor-only.
3. Live — move InterLink. Play — Load a `.recording`, then Play. Sync Play — Load a file, capture the RF Play button once, then Sync Play.

---

## 2. Red-tip tracker — `Radio-gadget-red-stickmover.py`

Does **not** talk to FlightAxis. It samples the screen around two calibrated points and finds the **red tips** of the Radio Gadget sticks.

That picture can be:

- RealFlight playing a recording with the Radio Gadget (or stick overlay) visible
- A YouTube (or other) video of the same sim / gadget view, sized so the red tips stay in the capture boxes

### Calibrate then track

1. Leave throttle full down (idle) and rudder centered. Hover the mouse over the **left red tip** and press **Space**.
2. Repeat for the **right red tip**.
3. The loop grabs a small block around each point, averages red pixels, maps them to −1…+1, applies Rate then Expo, and sends axes to hardware.

Rate above 100% is meant to stretch the small on-screen red-dot travel toward full StickMover throw.

### Run

```bat
python Radio-gadget-red-stickmover.py
```

Keep the gadget (or video) visible and unobscured. Recalibrate if the window moves.

---

## Stick layout (both scripts, Mode 2)

| Stick | Function | Hardware axis |
| --- | --- | --- |
| Left | Rudder | axis1 |
| Left | Throttle | axis2 |
| Right | Elevator | axis3 |
| Right | Aileron | axis4 |

FlightAxis / `.recording` channel order assumed (**AETR**):

- ch0 aileron  
- ch1 elevator  
- ch2 throttle  
- ch3 rudder  

---

## Rate and Expo (both scripts)

Per axis: **Rate%** 50–400 (default 100), **Expo%** −100…100 (default 0). Order is **Rate then Expo**.

Values from FlightAxis, `.recording`, or the red-tip tracker are treated as 0…1 (or mapped to −1…+1 for the gadget), shaped, then sent to hardware as 0…1.

- Rate 100% = incoming travel unchanged  
- Rate > 100% enlarges small travel toward full throw  
- Expo 0 = linear  
- Expo > 0 = softer around center  
- Expo < 0 = more throw around center  

---

## 64-bit Windows executables

This repo also includes two **64-bit Windows `.exe`** builds so you do not have to install Python to run the GUIs:

| Executable | Same as |
| --- | --- |
| `RF-recording-stickmover.exe` | `RF-recording-stickmover.py` |
| `Radio-gadget-red-stickmover.exe` | `Radio-gadget-red-stickmover.py` |

Double-click the exe, or run it from a Command Prompt. Behavior matches the matching `.py` file (Live / Play / Sync Play vs red-tip tracking).

Python 3 is only needed if you run the scripts yourself. The executables still need Windows. FlightAxis is needed for the data-path exe. A visible red tip is needed for the tracker exe. Hardware motion is optional; without USB or Robinson 0.1 the windows still open in monitor-only mode.

## Requirements

- Windows (64-bit for the provided executables)
- Python 3 + Tkinter — only if you run the `.py` scripts instead of the `.exe` files
- Optional: `pyserial` (COM listing; needed when running from source)
- **Optional to drive hardware:** [Python AVIrem StickMover Driver](https://github.com/nicholasrobinson/pystickmover) by **Nicholas Robinson**. These GUIs talk to the transmitter through that driver when it is present. They are a RealFlight / Radio Gadget front end on top of it, **not** a replacement for it. Current scripts expect the **0.1** API (`StickMover` on the FTDI serial port, **VID 0x0403 / PID 0x6015**). GitHub **1.0** is not required and is not wired in. The `.exe` files run without it.

Search paths are the `STICKMOVER_SEARCH_PATHS` list near the top of **both** scripts (generic; no machine-specific user folder):

```python
STICKMOVER_SEARCH_PATHS = [
    os.path.join(_SCRIPT_DIR, "pystickmover-0.1"),
    os.path.join(_SCRIPT_DIR, "pystickmover"),
    os.path.join(os.path.expanduser("~"), "Downloads", "pystickmover-0.1", "pystickmover-0.1"),
]
```

Unpack Robinson’s **0.1** package next to the script, or to `%USERPROFILE%\Downloads\pystickmover-0.1\pystickmover-0.1` (the folder Python can `import pystickmover` from). Do not drop a **1.0** zip into this repo and expect the GUIs to use it.

Private override (not committed): copy `stickmover_local.json.example` to `stickmover_local.json` next to the scripts and set `pystickmover_dir`. Or set environment variable `STICKMOVER_DIR` to that folder.

**FlightAxis script only:** RealFlight with FlightAxis on port 18083; Sync Play uses Win32 mouse / window APIs.

**Red-tip script only:** visible red stick tips on screen (sim gadget or video). Uses GDI screen capture.

---

## Files

| File | Role |
| --- | --- |
| `RF-recording-stickmover.py` | Data path: Live FlightAxis, Play `.recording`, Sync Play |
| `RF-recording-stickmover.exe` | 64-bit Windows build of the data-path GUI |
| `Radio-gadget-red-stickmover.py` | Vision path: red-tip tracking from RF playback or a YouTube / video window |
| `Radio-gadget-red-stickmover.exe` | 64-bit Windows build of the red-tip GUI |

`rec_sticks.py` is unused by the current FlightAxis GUI; that decoder is built in.

---

## Credits

StickMover hardware is an AVIrem product ([avirem.de](https://www.avirem.de/)). These GUIs are a RealFlight / Radio Gadget front end on top of **[Robinson’s driver](https://github.com/nicholasrobinson/pystickmover)**, not a replacement for it. AVIrem discontinued support; the original vendor software no longer makes the hardware usable.

## License

MIT. See `LICENSE`.

StickMover hardware, RealFlight, and Robinson’s `pystickmover` remain their owners’ products. This repo does **not** vendor `pystickmover`.
