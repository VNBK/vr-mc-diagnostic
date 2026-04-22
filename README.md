# vr-mc-diagnostic

Qt6 diagnostic / tuning desktop tool for the motor-control stack in
[`vr-mc-sdk`](../vr-mc-sdk). Talks to one or more CiA 402 drive nodes
through the in-tree `master_mgr` + `motor_drive_interface` APIs and the
`hal_can_udp` transport (loopback-friendly UDP-multicast CAN bus).

## Features (MVP)

- **Connect** to a UDP-multicast CAN endpoint (default `239.0.0.123:20000`),
  scan a configurable slave-ID range, register slaves through the CiA 402
  backend (`motor_drive_cia402_intf_create`).
- **Overview** table: id, name, state, online, position / velocity / torque.
- **Per-slave control**: enable / disable / fault-reset, mode switch,
  torque / velocity / position setpoint.
- **Live telemetry** plot (QtCharts) of position / velocity / torque.
- **Log dock** tailing spdlog output with severity colouring.

Later milestones (V1+) add the Feetech and Dynamixel backends, gain /
limit editors, trajectory capture, and config persistence.

## Build

```bash
# From parent workspace root:
cmake -S vr-mc-diagnostic -B vr-mc-diagnostic/build
cmake --build vr-mc-diagnostic/build -j
./vr-mc-diagnostic/build/vr_mc_diagnostic
```

Requires Qt6 (Core, Gui, Widgets, Charts). The build pulls vr-mc-sdk in
as a subdirectory at `../vr-mc-sdk`; override with
`-DVR_MC_SDK_DIR=/abs/path`. SDK tests / examples / apps are disabled
in this build.

## Loopback demo

Start the vr-mc-sdk drive simulator in another terminal:

```bash
cd vr-mc-sdk/build && cmake --build . --target motor_drive_cia402
./app/motor_drive_cia402 --udp
```

Then launch `vr_mc_diagnostic`, keep the default endpoint + slave-id 5,
and click *Connect*.
