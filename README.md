# vr-mc-diagnostic

Qt6 diagnostic / tuning desktop tool for the motor-control stack in
[`vr-mc-sdk`](../vr-mc-sdk). Talks to one or more CiA 402 drive nodes
through the in-tree `master_mgr` + `motor_drive_interface` APIs and any
of the supported transports (UDP-multicast loopback, ZLG USB-CANFD,
Feetech / Dynamixel RS-485).

```
┌─ Picker bar ────────────────────────────────────────────────────────┐
│ Slave: [5 — jnt_0 ▼]  ●OP_EN  q +0.52  ω +0.04  τ +0.12  err 0x0000 │
├─ Top split: slave list │ MotorView dial │ Motor profile ─────────── │
│                                                                       │
├─ Bottom split: Control / Gains / SigGen tabs │ Telemetry chart ───── │
│                                                                       │
└─ Log dock ──────────────────────────────────────────────────────────┘
```

## Features

- **Connection dialog** — UDP multicast (loopback sim), ZLG USB-CANFD,
  Feetech, Dynamixel. ESI + SRDF pickers. Sensible defaults.
- **Slave picker bar** — single-row combobox + live state / `q` / `ω`
  / `τ` / `err` strip; replaces a wide multi-column table for the
  common single-slave-at-a-time workflow.
- **MotorView dial** — at-a-glance rotational position with arrow
  colour-coded by velocity direction (green = CW, blue = CCW).
- **Control tab** — bring-up walker, Enable / Disable / Quick Stop /
  Fault Reset, state-aware button gating, decoded CiA-402 state +
  controlword readout, mode-aware setpoint with rad↔deg toggle,
  jog +/-, preset buttons (`0` / `±π/4` / `Home`), live-stream slider,
  named save/recall presets, manual CiA-402 walk debug aid.
- **Gains tab** — per-loop Kp/Ki editor (Current / Velocity /
  Position).
- **Signal generator** — Constant / Step / Ramp / Sine / Chirp
  channels.
- **Configure-drive dialog** (Tools → Configure drive) — Homing,
  Motion profile, Protection, Encoder, Manufacturer, and a Custom-SDO
  free-form poke tab. Fault-thresholds expanded into 10 sub-fields
  (over-current, over-load, current loss phase, unbalance, stall,
  over/under-voltage, over-temperature, …).
- **Telemetry chart** — single chart with grouped show/hide
  checkboxes (Position / Velocity / Torque / Tracking / Electrical).
- **Record to CSV** — toolbar **Record** button prompts for a path,
  writes timestamped snapshot rows; auto-stops on Disconnect.
- **Help → Start demo** — spawns N copies of the in-tree
  simulator (`vrmc_sim`) and auto-connects. No SDK build needed.

## Build

```bash
# from the parent workspace
cmake -S vr-mc-diagnostic -B vr-mc-diagnostic/build
cmake --build vr-mc-diagnostic/build -j

# produces TWO binaries in vr-mc-diagnostic/build/:
#   vr_mc_diagnostic     — the GUI
#   vrmc_sim             — in-tree CiA-402 simulator (used by Help → Start demo)

./vr-mc-diagnostic/build/vr_mc_diagnostic
```

Requires:
- **Qt6** — Core, Gui, Widgets, Charts. On Debian/Ubuntu:
  `sudo apt install qt6-base-dev qt6-base-dev-tools libqt6charts6-dev`
- **vr-mc-sdk** — pulled in as a CMake subdirectory at `../vr-mc-sdk`;
  override with `-DVR_MC_SDK_DIR=/abs/path`. SDK tests / examples /
  apps are disabled in this build.

## Try it without hardware (Help → Start demo)

The simplest way to see the GUI talking to "drives":

1. Launch `vr_mc_diagnostic`.
2. **Help → Start demo…** — pick how many simulated slaves
   (1–32, default 3).
3. The diagnostic spawns that many `vrmc_sim` processes on UDP
   multicast `239.192.0.42:23400` (sequential node IDs from 5) and
   auto-connects. The slave list populates, MotorView dials light up,
   telemetry starts flowing.
4. **Help → Stop demo** terminates the simulator processes. Pressing
   the toolbar **Disconnect** also stops the demo.

No SDK binaries needed — `vrmc_sim` is built from `tools/vrmc_sim.c` +
`tools/vrmc_sim_od.{c,h}` as part of the diagnostic.

## Real hardware

Pick the matching backend in the Connection dialog:

| Backend | Default endpoint | Notes |
|---|---|---|
| UDP multicast | `239.192.0.42:23400` | Loopback against any CiA-402 sim on the same group |
| ZLG USB-CANFD | channel 0, 1 M/4 M | Real CAN-FD; vendor `libcontrolcanfd.so` ships with the build |
| Feetech RS-485 | `/dev/ttyUSB0` @ 1 M | Add user to the `dialout` group |
| Dynamixel RS-485 | `/dev/ttyUSB0` @ 1 M | Protocol 2.0, choose at Connect |

## Tools

Everything operational lives under `tools/`:

| File | Purpose |
|---|---|
| `tools/vrmc_sim.c` + `vrmc_sim_od.{c,h}` | In-tree CiA-402 simulator — built as the `vrmc_sim` executable by the main CMake project. |
| `tools/build_docs.py` | Re-render `docs/*.md` → `*.html` + `*.pdf`. Self-contained. Needs Python `markdown` and `google-chrome` / `chromium` on PATH. `--no-pdf` skips the PDF step. |
| `tools/build_appimage.sh` | Package the diagnostic + simulator + Qt runtime + docs into a portable AppImage. Output: `dist/VinRobotics_MotorControl_Diagnostic_Tool-<version>-<arch>.AppImage`. |
| `tools/.cache/` | Downloaded `linuxdeploy*.AppImage` blobs reused across AppImage rebuilds. |

```bash
# Re-render docs after editing user_guide.md
python3 tools/build_docs.py

# Build a release AppImage (after cmake --build)
tools/build_appimage.sh 1.0.0
# → dist/VinRobotics_MotorControl_Diagnostic_Tool-1.0.0-x86_64.AppImage
```
