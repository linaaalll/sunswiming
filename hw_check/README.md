# hw_check

> is everything actually plugged in and talking, BEFORE we blame the code

Plain python scripts, not a ros package, so they still work when colcon / MAVROS is broken.
Every check prints green / yellow / red:

- **PASS** = there and working
- **WARN** = kinda works, something's off (or the check isn't written yet)
- **FAIL** = not there / not talking

## The layers (check bottom up, the first FAIL is your problem)

| layer | what | check |
|---|---|---|
| 1 | cable / switch | `link/check_link.py` |
| 2 | ip, jetson can ping the pi | `link/check_link.py` |
| 3 | ardusub mavlink heartbeats arriving | `link/check_link.py` |
| 4 | MAVROS bridge connected | `link/check_mavros.py` |
| 5 | ros topics at the right rate | `ros2_ws/src/output/monitor/basic_monitor.py` |

Usual ways the bridge breaks: the jetson's ip isn't the one the pi sends to (layer 2),
or something else is already on the mavlink port (layer 4).

## Folder

```
hw_check/
  config.py            every ip / port / number, change them HERE
  common.py            PASS/WARN/FAIL printing, run(), finish()
  run_all.sh           everything except motor_test, in order
  jetson/
    check_system.py    disk, temperature, zed driver
    check_zed.py       zed x mini opens and gives frames
  link/
    check_link.py      layers 1-3
    check_mavros.py    layer 4
    check_dvl.py       dvl on the network + bottom lock (skips until DVL_IP set)
  pi/
    check_fcu.py       ardusub sensor health, depth sensor, battery
    check_outputs.py   8 thrusters mapped to outputs, sitting at neutral
    motor_test.py      spins each thruster, compares current -> finds loose crimps
```

## Running

```bash
./hw_check/run_all.sh                       # MAVROS off, it hogs the mavlink port
python3 hw_check/link/check_mavros.py       # MAVROS on
python3 hw_check/pi/motor_test.py           # PROPS SPIN, on purpose not in run_all
```

## What's written vs what's yours

Written: `common.py`, `config.py`, `run_all.sh`, layers 1-2 in `check_link.py` (the worked
example, copy its pattern), and the safety bits of `motor_test.py` (confirm prompt + always disarm).

Everything else is a `TODO (you)` with step by step hints. Suggested order, easy -> hard:

1. `jetson/check_system.py`  plain python, files, no robot needed
2. `link/check_link.py` heartbeat  first pymavlink
3. `link/check_dvl.py`  sockets + json
4. `pi/check_fcu.py`  bitmasks
5. `pi/check_outputs.py`  params
6. `jetson/check_zed.py`  ZED SDK
7. `link/check_mavros.py`  rclpy
8. `pi/motor_test.py`  sending commands, putting it all together

Needs: `pip install pymavlink` (and the ZED SDK's `pyzed` on the jetson).
