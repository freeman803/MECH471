# MECH471 HIL RC Car Controller

Hardware-in-the-Loop control systems project for Arduino Uno. Implements speed/cruise control, traction/launch control, and braking control for a virtual RC car via a HIL simulator board.

---

## Prerequisites

- [VS Code](https://code.visualstudio.com/) installed
- USB cable for Arduino Uno

---

## Install PlatformIO

1. Open VS Code
2. Click the **Extensions** icon in the left sidebar (or press `Ctrl+Shift+X`)
3. Search for **PlatformIO IDE**
4. Click **Install** on the result by *PlatformIO*
5. Wait for the install to finish, then **restart VS Code** when prompted

> PlatformIO will automatically install the AVR toolchain and Arduino framework the first time you build — no separate Arduino IDE install needed.

---

## Open the Project

1. In VS Code, go to **File → Open Folder**
2. Navigate to and select the `Mech 471` folder (the one containing `platformio.ini`)
3. VS Code will detect the PlatformIO project automatically

---

## Build & Upload

Use the PlatformIO toolbar at the bottom of VS Code or the **PlatformIO** sidebar tab.

| Action | Button | Keyboard |
|--------|--------|----------|
| Build | ✓ (checkmark) | `Ctrl+Alt+B` |
| Upload | → (arrow) | `Ctrl+Alt+U` |
| Serial Monitor | plug icon | `Ctrl+Alt+S` |

### Build environments

There are four build targets defined in `platformio.ini`:

| Environment | What it builds | Upload to |
|-------------|---------------|-----------|
| `uno` (default) | Main controller | Controller board |
| `simulator1` | Fixed-voltage test harness | Simulator board |
| `simulator2` | DC motor dynamics model | Simulator board |
| `simulator3` | Full vehicle + tyre model | Simulator board |

To switch environment, click the environment name in the blue status bar at the bottom of VS Code and select from the list, or run from the PlatformIO terminal:

```
pio run -e simulator2 --target upload
```

---

## Serial Monitor Baud Rates

| Simulator | Baud rate |
|-----------|-----------|
| sim1 | 115200 |
| sim2 / sim3 | 1000000 |

Make sure `Serial.begin()` in `main.cpp` matches the simulator you are running.

---

## Wiring (controller board ↔ simulator board)

| Controller pin | Simulator pin | Signal |
|----------------|---------------|--------|
| D7 | D2 | u1 — drive motor servo pulse |
| D8 | D3 | u2 — steering servo pulse |
| A1 | D5 | y1 — rear wheel speed |
| A3 | D6 | y2 — right front wheel speed |
| A5 | D11 | y3 — left front wheel speed |
| GND | GND | Common ground (required) |

**Always upload to both boards before connecting any wires.**
