# vr-mc-diagnostic

Qt6 diagnostic / tuning desktop tool for the motor-control stack in
[`vr-mc-sdk`](../vr-mc-sdk). Talks to one or more CiA 402 drive nodes
through the in-tree `master_mgr` + `motor_drive_interface` APIs and any
of the supported transports. Default Connect dialog opens on
**ZLG USB-CANFD**; UDP loopback + RS-485 serial buses are also
available.

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
  Fault Reset / **Brake** (CiA-402 Halt bit toggle), state-aware
  button gating, decoded CiA-402 state + controlword readout,
  mode-aware setpoint with rad↔deg toggle, jog +/-, preset buttons
  (`0` / `±π/4` / `Home`), live-stream slider, named save/recall
  presets, manual CiA-402 walk debug aid.
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

Pick the matching backend in the Connection dialog (default = ZLG):

| Backend | Default endpoint | Notes |
|---|---|---|
| **ZLG USB-CANFD** | channel 0, **500 k arb / 2 M data** | Real CAN-FD; matches `motor_drive_master` SDK default. Vendor `libcontrolcanfd.so` ships with the build |
| UDP multicast | `239.192.0.42:23400` | Loopback against `vrmc_sim` or any CiA-402 sim on the same group |
| Feetech RS-485 | `/dev/ttyUSB0` @ 1 M | Add user to the `dialout` group |
| Dynamixel RS-485 | `/dev/ttyUSB0` @ 1 M | Protocol 2.0, choose at Connect |

## Architecture & cycle times

Worker thread owns the C SDK stack and runs two independent timers
(both lock the same mutex as bringup/disable so SDO walks never race
the cached-controlword RPDOs):

| Timer | Rate | Job |
|---|---|---|
| `m_cycle` | **200 Hz** (5 ms) | `pump()` + `cycleStep()` — drain RX + send 1 RPDO per slot |
| `m_tick`  | **30 Hz** (33 ms) | Build SlaveSnapshot vector + emit `snapshots()` to UI |
| `m_genTimer` | user-config (default 50 Hz) | Push waveform setpoint via `setTarget` |

**RPDO** (master → slave, 12 B at COB-ID `0x200+id`):
- Streamed every cycle period (5 ms) for every registered slot.
- Payload = `[cached controlword | tgt_pos | tgt_vel | tgt_torque]`.
- Cached controlword per slot, updated by:
  - `0x0000` Disable Voltage — default after Connect, after Disable / Fault Reset / E-STOP.
  - `0x0006` Shutdown — transient during the SDK bringup walk.
  - `0x000F` Enable Operation — after successful bringup.
  - `0x0002` Quick Stop — after Quick Stop button.
  - `0x0080` Fault Reset — pulsed by the SDK's SDO walk.
  - `| 0x0100` Halt bit OR'd in when Brake is engaged (preserves state bits).

**TPDO** (slave → master, 14 B at COB-ID `0x180+id`):
- Diagnostic only subscribes; rate is determined by the firmware.
- Paired (1:1 with each RPDO) on `cia402_drive_sim` / `vrmc_sim` and most
  MCU ports → effectively ~200 Hz.
- Cached per slot in `PdoSlot`; read at 30 Hz by `onTick`.

**SDO** (sync R/W, 100 ms timeout per request):
- Bringup / Enable / Disable / Fault Reset walks (~5 ops, ~500 ms).
- Drive-config tab (~30 fields, ~3 s).
- Custom-SDO tab (single request).
- PDO mapping apply (~N+3 ops).
- All synchronous on the worker thread; `pumpStub` keeps the transport
  draining during the wait.

**Wire load** (1 slave, 500 k arb / 2 M data):

| Direction | Rate | Bytes/s |
|---|---|---|
| RPDO out | 200 Hz × 12 B | 2 400 |
| TPDO in (paired) | 200 Hz × 14 B | 2 800 |
| **Total** | | **~5 KB/s** |

Well under 1 % bus utilisation — plenty of headroom for 4–8 slaves +
bursty SDO traffic.

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
