# Stand-By Mode for ArduPlane — SITL Testing Guide

A complete walkthrough for getting the code, building SITL, and verifying the fixed-wing
Stand-By feature. Written so someone with no prior context can reproduce every result.

---

## 1. What the feature is, in one paragraph

Stand-By lets two or more flight controllers run in parallel on one airframe, where only one is
the *controller in command* (CIC) and the others "ride along" behind an external PWM multiplexer.
A ride-along controller's output has no effect on the aircraft, so any disagreement between it and
the CIC integrates without ever being nulled — its integrators wind up to `IMAX`, and it would
dump that into the servos the instant it took command. Stand-By is a single flag, set by RC aux
function **76**, that continuously flushes accumulated state while leaving every control loop
running at full rate. It is **not a flight mode**: the aircraft stays in FBWA, LOITER, AUTO or
whatever else, and the flag sits alongside.

The key property to verify: **standby flushes state, it must not stop the control loops.**

---

## 2. Prerequisites (one time)

### 2.1 WSL

SITL is Linux software. On Windows, use WSL2 with Ubuntu 22.04:

```powershell
wsl --install -d Ubuntu-22.04
```

Everything from here runs *inside* WSL. Open it by typing `wsl` in a terminal.

> Building on `/mnt/c` (the Windows filesystem) is extremely slow. Keep the checkout in the Linux
> home directory, e.g. `~/ardupilot`.

### 2.2 Packages

Build tools and Python libraries:

```bash
sudo apt-get update
sudo apt-get install -y --no-install-recommends \
    build-essential ccache g++ gawk git make wget rsync pkg-config \
    python3-dev python3-pip python3-setuptools python3-pexpect \
    libtool libtool-bin libxml2-dev libxslt1-dev \
    python3-numpy python3-pyparsing python3-psutil

python3 -m pip install --user \
    "empy==3.3.4" future lxml pymavlink pyserial MAVProxy \
    ptyprocess pexpect junitparser wsproto tabulate
```

For the **console, map and graph windows** (needed for the visual checks below):

```bash
sudo apt-get install -y --no-install-recommends \
    python3-wxgtk4.0 xterm python3-matplotlib python3-opencv
```

WSLg provides the X display automatically, so GUI windows appear on your Windows desktop. Confirm:

```bash
echo $DISPLAY          # should print :0
python3 -c "import wx, matplotlib, cv2; print('GUI deps OK')"
```

### 2.3 Put the tools on PATH

`pip --user` installs `mavproxy.py` and `MAVExplorer.py` into `~/.local/bin`, which a
non-login shell does **not** pick up. Add it permanently:

```bash
echo 'export PATH="$PATH:$HOME/.local/bin"' >> ~/.bashrc
source ~/.bashrc
which mavproxy.py MAVExplorer.py     # both must resolve
```

ArduPilot also ships an official prereq script (`Tools/environment_install/install-prereqs-ubuntu.sh -y`)
which does all of the above plus the ARM cross-compiler. It is heavier but works.

---

## 3. Get the code

### 3.1 Fresh clone (recommended for a new machine)

```bash
cd ~
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot

# add the fork holding the Stand-By work and check the branch out
git remote add fork https://github.com/crew-12-to-moon/harsh12-ardupilot.git
git fetch fork plane-standby-mode
git checkout -b plane-standby-mode fork/plane-standby-mode
git submodule update --init --recursive
```

### 3.2 Getting later updates

The branch is periodically rebased onto upstream master, which rewrites its commits. A plain
`git pull` will make a mess of that. Use:

```bash
git fetch fork plane-standby-mode
git reset --hard fork/plane-standby-mode
git submodule update --init --recursive
```

To rebase the work onto the newest upstream master yourself:

```bash
git fetch origin master
git rebase origin/master
git submodule update --init --recursive
```

### 3.3 Verify you have the feature

```bash
ls ArduPlane/standby.cpp                                   # must exist
git log --oneline origin/master..HEAD                      # 5 commits, no merge commits
```

Expected series (SHAs change after each rebase):

```
AP_Scripting: add plane_standby_guard.lua example
autotest: add Plane Standby tests
Plane: add Stand-By mode for parallel flight controllers
AP_L1_Control: flush the cross-track integrator on standby_reset
AP_Navigation: add standby_reset to flush accumulated navigation state
```

---

## 4. Build SITL

```bash
cd ~/ardupilot
./waf configure --board sitl
./waf plane -j$(nproc)
```

First build takes 5–15 minutes; later builds are incremental and take seconds.

> **If you used a shallow clone** (`--depth`), waf's submodule fast-forward check cannot compute a
> merge-base and the build aborts. Add `--no-submodule-update` to both commands:
> ```bash
> ./waf configure --board sitl --no-submodule-update
> ./waf plane --no-submodule-update -j$(nproc)
> ```

Confirm the feature is compiled in:

```bash
strings build/sitl/bin/arduplane | grep "Stand By"
# expect: Stand By Enabled / Stand By Disabled
```

---

## 5. Run a SITL instance

Logs and parameters are written to the directory you launch from, so use a dedicated one:

```bash
mkdir -p ~/sitl && cd ~/sitl
~/ardupilot/Tools/autotest/sim_vehicle.py -v ArduPlane -f plane --console --map
```

You get a `MAV>` prompt in the terminal, plus a Console and Map window. **Wait for the console to
show `Ready to FLY`** (GPS lock + EKF settled, roughly 30 s) before arming.

Useful flags:

| flag | effect |
|---|---|
| `-w` | wipe parameters to defaults |
| `--speedup 5` | run 5× real time |
| `--no-rebuild` | skip the build step (faster startup) |
| `--no-mavproxy` | headless; connect your own tool to `tcp:127.0.0.1:5760` |
| `-I 1` | second instance on ports 5770+ (dual-FC rehearsal) |
| `-L <name>` | start at a named location from `Tools/autotest/locations.txt` |

Stop with `Ctrl-C`, or type `exit` at the `MAV>` prompt.

---

## 6. Testing the feature by hand

All commands below go at the `MAV>` prompt.

### 6.1 Assign the switch — do this first

```
param set RC7_OPTION 76
```

**76 is the STANDBY aux function.** Without this, channel 7 does nothing at all and the toggle
below will appear to do nothing. It takes effect immediately — no reboot needed. Any channel
works; just keep the number consistent.

Add some wind so the controllers have something to fight:

```
param set SIM_WIND_SPD 8
param set SIM_WIND_DIR 45
```

### 6.2 Get airborne

```
mode TAKEOFF
arm throttle
```

Wait until it levels off (default `TKOFF_ALT`), then circle and let things settle for ~30 s:

```
mode LOITER
```

### 6.3 Load the graph module — required

**`graph` is not a built-in command.** It lives in a MAVProxy module that is not loaded by
default, so typing `graph ...` straight away gives *unknown command*. Load it first:

```
module load graph
```

Then graph the aileron and elevator outputs:

```
graph SERVO_OUTPUT_RAW.servo1_raw SERVO_OUTPUT_RAW.servo2_raw
```

A matplotlib window opens showing both traces live. Related commands:

```
module list          # what is currently loaded
module load graph    # add graphing
graph <field> ...    # add a graph (repeatable)
help                 # every command your MAVProxy build has
```

### 6.4 Toggle standby

```
rc 7 2000        # ENABLE   -> console prints "Stand By Enabled"
rc 7 1000        # DISABLE  -> console prints "Stand By Disabled"
```

`rc` *is* a built-in command, so it needs no module.

### 6.5 What you should see — the checks that matter

| # | Check | Pass condition |
|---|---|---|
| 1 | **Toggle plumbing** | `Stand By Enabled` / `Stand By Disabled` appear in the Console on each transition |
| 2 | **Loops still running** | With standby asserted, the `servo1_raw`/`servo2_raw` traces **keep moving**. If they flatline, the feature is broken — standby must flush state, not freeze control |
| 3 | **Still flyable** | The aircraft keeps circling normally in LOITER, stays armed, does not disarm or deploy anything |
| 4 | **Altitude target follows** | See 6.6 |
| 5 | **Integrators flushed** | Post-flight, see section 8 |

Check 2 is the one that catches a bad implementation. It is easy to write a "standby" that quietly
stops flying the aircraft.

### 6.6 Altitude target follows the aircraft

While standing by, the altitude target tracks the current altitude (through a clamped lag), so a
commanded climb is ignored until standby is released:

```
module load graph
graph NAV_CONTROLLER_OUTPUT.alt_error

rc 7 2000                                  # standby ON
long DO_CHANGE_ALTITUDE 200 6 0 0 0 0 0    # ask for 200 m
```

The aircraft should **not** chase 200 m, and `alt_error` stays small. Then:

```
rc 7 1000                                  # standby OFF
long DO_CHANGE_ALTITUDE 200 6 0 0 0 0 0
```

Now it climbs normally.

### 6.7 The MAVLink path (what a companion computer uses)

Standby does not need RC at all — this is how a supervisor board or companion computer drives it:

```
long DO_AUX_FUNCTION 76 2 0 0 0 0 0     # 2 = HIGH = enable
long DO_AUX_FUNCTION 76 0 0 0 0 0 0     # 0 = LOW  = disable
```

If the name is not recognised, use the numeric command id (`MAV_CMD_DO_AUX_FUNCTION` = 218):

```
long 218 76 2 0 0 0 0 0
long 218 76 0 0 0 0 0 0
```

Worth exercising at least once, since the real dual-FC system will use this path rather than RC.

---

## 7. Automated tests

Faster and stricter than testing by hand:

```bash
cd ~/ardupilot
python3 Tools/autotest/autotest.py --no-configure \
    test.Plane.Standby test.Plane.StandbyParachute
```

- **`Standby`** — flies a loiter in wind, asserts standby, and checks from the onboard log that the
  rate-controller I terms stay flushed, that the `STANDBY_ENABLE` event is written, and that the
  servo outputs keep moving.
- **`StandbyParachute`** — dives until the sink rate exceeds `CHUTE_CRT_SINK` while standing by,
  releases standby *while still sinking fast*, and requires the release to wait out the parachute's
  one-second debounce instead of firing immediately.

Wider regression set:

```bash
python3 Tools/autotest/autotest.py --no-configure \
    test.Plane.Standby test.Plane.StandbyParachute test.Plane.MainFlight \
    test.Plane.AuxModeSwitch test.Plane.MAV_CMD_DO_AUX_FUNCTION \
    test.Plane.Parachute test.Plane.ParachuteSinkRate \
    test.Plane.LOITER test.Plane.TakeoffTakeoff1
```

---

## 8. Reading the dataflash log

The rate-controller I terms are the direct evidence that standby works, and they are **only in the
dataflash log** — they are not sent over MAVLink, so `graph` cannot show them live.

```bash
cd ~/sitl
ls logs/                      # newest .BIN is your flight
MAVExplorer.py logs/00000001.BIN
```

In MAVExplorer:

```
graph PIDR.I PIDP.I           # roll and pitch rate-controller I terms
graph EV.Id                   # 74 = standby enabled, 75 = standby disabled
graph CTUN.NavPitch CTUN.ThO  # pitch demand and throttle
graph RCOU.C1 RCOU.C2         # servo outputs
```

Between `EV` 74 and `EV` 75 both I terms should sit near zero, then resume normal behaviour after 75.

---

## 9. Reference numbers

Measured in SITL on this branch, so you know what "working" looks like:

| Measurement | Normal flight | During standby |
|---|---|---|
| max \|`PIDR.I`\| | 0.440 | **0.008** |
| max \|`PIDP.I`\| | 4.837 | **0.025** |

Servo outputs move *more* during standby, not less — with the integrators flushed, the P and
feed-forward terms carry the load the I term was carrying. That is expected.

Altitude drift while standing by, straight and level with no wind, over a 90 s window:

| Implementation | Drift |
|---|---|
| target pinned exactly to current altitude | 0.905 m/s |
| **clamped lagged pin (this branch)** | **0.25–0.27 m/s** |

Parachute sink-rate debounce, time from releasing standby to release firing while still sinking:

| Implementation | Time |
|---|---|
| naive (skip the check during standby) | 0.6 s — debounce skipped, fires immediately |
| **this branch** | **1.7–1.8 s** — debounce correctly re-armed |

---

## 10. Troubleshooting

| Symptom | Cause and fix |
|---|---|
| `graph` → *unknown command* | The graph module is not loaded by default. Run `module load graph` first |
| `rc 7 2000` does nothing, no statustext | `RC7_OPTION` is not set to 76. Check with `param show RC7_OPTION` |
| Flood of `Warning, time moved backwards. Restarting timer.` | Harmless MAVProxy noise when SITL's clock jumps (speedup, reboot, or a heavily loaded host). Safe to ignore |
| Build aborts in `git_submodule.py` on `git merge-base` | Shallow clone. Add `--no-submodule-update` to the `waf` commands |
| `MAVExplorer.py: command not found` | `~/.local/bin` not on PATH — see section 2.3 |
| No Console/Map/graph window appears | Missing `python3-wxgtk4.0` / `python3-matplotlib` / `python3-opencv`, or `$DISPLAY` is unset |
| `Failed to load module: No module named 'map'` | Install `python3-opencv` and `python3-matplotlib` |
| Cannot arm | Wait for `Ready to FLY` in the console; SITL needs GPS lock and a settled EKF |
| Parachute fires unexpectedly in your own testing | Only relevant if `CHUTE_ENABLED=1`; the default SITL plane has it off |

---

## 11. What the feature does and does not do

**Does:**

- flushes roll/pitch/yaw/steering rate-controller I terms and the ground-steering locked course
- flushes the TECS throttle and energy-balance integrators
- flushes the L1 cross-track integrator
- keeps the altitude target following the current altitude through a clamped lag
- re-anchors navigation references a mode captured on entry — the LOITER and CIRCLE centres and the
  accumulated loiter angle, the CRUISE locked heading, the RTL cross-track origin
- resets the VTOL attitude and position controllers on quadplanes
- suppresses crash detection, the parachute sink-rate check, hover-throttle learning and servo auto-trim

**Does not:**

- switch servo outputs — that is an external PWM multiplexer's job, driven by your supervisor
- arbitrate between controllers, or exchange any data between them
- synchronise flight modes or mission state between controllers — both must be fed the same RC and
  configured with matching `FLTMODE*` parameters
- hold altitude precisely while asserted; a residual drift of roughly 0.25 m/s remains, which is
  inherent to flushing the energy-controller integrators

**Deliberately left active:** battery, RC and geofence failsafes, since those are based on reality
that both controllers share.

---

## 12. Two-instance rehearsal

Single-instance SITL cannot model a shared airframe, but two instances rehearse the state machine
and switchover logic:

```bash
# terminal 1
cd ~/sitl && ~/ardupilot/Tools/autotest/sim_vehicle.py -v ArduPlane -I 0 --console
# terminal 2
mkdir -p ~/sitl2 && cd ~/sitl2 && ~/ardupilot/Tools/autotest/sim_vehicle.py -v ArduPlane -I 1 --console
```

`libraries/AP_Scripting/examples/sitl_standby_sim.lua` shows how to drive standby on two simulated
controllers from one RC switch. Each instance flies its own physics, so this exercises the switch
handling and mode mirroring, not the aerodynamic takeover.
