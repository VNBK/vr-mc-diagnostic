# vr-mc-diagnostic

Qt6 diagnostic / tuning desktop tool for the motor-control stack in
[`vr-mc-sdk`](../vr-mc-sdk). Talks to one or more CiA 402 drive nodes
through the in-tree `master_mgr` + `motor_drive_interface` APIs. Default
Connect dialog opens on **ZLG USB-CANFD**; a UDP-multicast loopback
transport is available for the SDK's simulator.

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

- **Connection dialog** — UDP multicast (loopback sim) or ZLG USB-CANFD.
  ESI + SRDF pickers. Sensible defaults.
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
  over/under-voltage, over-temperature, …). Sensor calibration exposes
  the **Angle-offsets record `0x2060`** for direct writes:
  `:1` commutation offset (rad, electrical align — Hall / abs-encoder /
  resolver) and `:2` position offset (writes the TMAG sin offset;
  applied to the live sensor + persisted on the board).
- **Telemetry chart** — single chart with grouped show/hide
  checkboxes (Position / Velocity / Torque / Tracking / Electrical).
- **Record to CSV** — toolbar **Record** button prompts for a path,
  writes timestamped snapshot rows; auto-stops on Disconnect.
- **Help → Start demo** — spawns N copies of the in-tree
  simulator (`vrmc_sim`) and auto-connects. No SDK build needed.

## Roadmap — required features still missing

Gaps a production-grade motor-control diagnostic is expected to cover
but this tool does not yet (or only partially) implement. Ordered by
priority. OD indices reference the drive object dictionary.

### Must-have

1. **Fault / EMCY diagnostics.** Today: raw `errorCode` field +
   *Fault reset* button only. Missing: decode of **0x1001** (error
   register bits — over-current / -voltage / -temperature / encoder…),
   **0x1003** (predefined error field = *fault history*), live **EMCY**
   frame capture, and a human-readable fault-code table. *(Highest-value
   gap — SDO-only, no board change.)*
2. **CiA-402 homing (mode 6).** The current *Home* button is only a
   position preset of `0`. Missing the real procedure: **0x6098** method,
   **0x607C** offset, **0x6099** speeds, **0x609A** accel;
   home-to-index / limit-switch.
3. **Object-Dictionary browser.** Only a manual "type the index" Custom-SDO
   poke exists (Configure-drive tab). Missing: a named OD tree with
   type/unit metadata, read-all, and a live watch list.
4. **Parameter backup / restore to file + diff.** Only flash-persist
   (**0x1010**) and default-restore (**0x1011**) exist. Missing: dump the
   full parameter set to a file, reload onto another drive, and diff
   against a golden reference (production / board-swap workflow).

### Should-have

5. **Triggered oscilloscope.** Telemetry is free-running + record only.
   Missing: threshold / statusword-bit trigger with pre/post capture. The
   high-rate path already exists in Auto-Tune (**0x2080**) and can be reused.
6. **Auto-identification / commissioning wizard** — see below.
7. **Live protection dashboard.** Configure-drive only *sets* limits.
   Missing: actual-vs-threshold view (current / temperature / bus-V /
   following-error) with a warning banner before a trip.
8. **CAN bus health + NMT.** Heartbeat exists only at transport level.
   Missing: heartbeat-timeout / bus-off / error-counter view,
   node-guarding, and NMT start/stop/reset-node control.

### Auto-identification / commissioning wizard (feature 6, detailed)

Much of the engine already lives **on the board** (`0x2030` V/f open-loop,
`0x2031` calibration control, `0x2080` model-based PI tuner). The missing
piece is a diagnostic-side wizard that **sequences** these primitives,
**gates** each step pass/fail, and **writes results back** to the motor
profile (`0x2070`). Markers: ✅ = board primitive exists (orchestrate now),
⚠️ = needs new firmware.

- **Phase 0 — Prep / safety.** Load name-plate into profile **0x2070**
  (pole pairs, rated V/I/speed/torque, motor type); set current/torque
  limits (0x6073/0x6072); check bus-V; confirm the shaft can spin. ✅
- **Phase 1 — Current-sense offset calibration** (drive disabled). Zero
  the phase-current ADC offsets so every later current reading is
  trustworthy. Board: **`0x2030:1 = 1`**, poll done. ✅ Optional analog-
  encoder sin/cos min-max calibration (TMAG6180): **`0x2031:1 = 2`**
  (begin) → spin ≥1 electrical rev → **`0x2031:1 = 3`** (finalize +
  persist). ✅ The single sin offset can also be written directly via
  **`0x2060:2`** (Configure-drive → Sensor calibration). ✅
- **Phase 2 — Electrical parameter identification** (the real "auto-ID").
  Currently entered **by hand from the name-plate**; true measurement
  needs firmware ⚠️:
  - **Rs** — inject d-axis DC current (rotor held), V/I → Rs → **0x2070:04**.
  - **Ls_d / Ls_q** — inject HF AC voltage on d then q, |I|/phase →
    `L = V/(2πf·I)` → **0x2070:05/06**.
  - **λ_m (PM flux / BEMF constant)** — spin open-loop via V/f
    (**0x2030**, ✅ can spin) at constant speed, measure back-EMF →
    `λ = V_bemf/ω_e` (⇒ Kt) → **0x2070:07/09** (estimator ⚠️).
- **Phase 3 — Encoder / commutation alignment.** Find electrical zero
  (inject d-axis current → rotor locks to d → encoder reading = offset),
  store `alignment_offset_rad`; check direction/polarity and pole-pair
  sanity; seek index/home. Board: **`0x2031:1 = 5`** (3FL_2 align), poll
  **0x2031:2**. ✅ The resulting commutation offset can also be read /
  written directly via **`0x2060:1`** (rad). ✅
- **Phase 4 — Current loop: seed + verify.** Model-based seed from
  profile: `0x2080:01=0` (current), `:02=<bw>`, `:03=1` (trigger),
  poll `:04==0`, read **Kp/Ki = 0x60F6:01/02**; then verify with a step
  capture (Auto-Tune → *Capture Step*) and refine BW. ✅
- **Phase 5 — Velocity loop (+ inertia).** **J** identification —
  known torque step, `J = T/(dω/dt)` → **0x2070:08** ⚠️. Tune velocity PI
  (`0x2080:01=1`) + verify step. ✅
- **Phase 6 — Position loop.** Tune (`0x2080:01=2`) + verify step. ✅
- **Phase 7 — Persist.** Flash-save via **0x1010:01** (all) or **:05**
  (motor). ✅

**Wizard responsibilities (diagnostic side):** run Phases 1→7 as guided
pages with progress + status polling; gate pass/fail between steps (offset
drift, align `status < 0`, step overshoot over threshold); write results
into **0x2070** with a before/after table; and provide a safe abort
(disable + quick-stop) at every step. A semi-automatic version can ship
immediately using the ✅ primitives (offset → align → seed all three loops
→ verify → save), leaving the ⚠️ R/L/λ/J fields as name-plate entry until
the identification routines land in firmware.

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
