# Water Pump Remote Control

ESP32-based IoT retrofit that adds remote **ON / OFF / RESTART** control to an existing
residential water pump over MQTT, **without replacing the pump's existing automatic
controller**.

> **Status as of 2026-09-13: transport layer RESOLVED. Remaining work is electrical.**
> Option A is implemented: a ground-floor access point is installed and the ESP32 holds an
> MQTT-over-TLS session with HiveMQ Cloud from the enclosure position at **-58 dBm**,
> verified end to end with a command round trip. The open items are now mechanical and
> electrical — DMAX topology, contactor coil voltage, and the bench test of the control
> chain, which has still never been run. See [Open Problems](#6-open-problems).

---

## Table of Contents

1. [Objective](#1-objective)
2. [Architecture](#2-architecture)
3. [Control Design and Fail-Safe Behaviour](#3-control-design-and-fail-safe-behaviour)
4. [Firmware](#4-firmware)
5. [Hardware](#5-hardware)
6. [Open Problems](#6-open-problems)
7. [Site Survey Results](#7-site-survey-results)
8. [Options Under Evaluation](#8-options-under-evaluation)
9. [Current Status](#9-current-status)
10. [Decision Log](#10-decision-log)
11. [Repository Layout](#11-repository-layout)
12. [Building](#12-building)
13. [Safety](#13-safety)
14. [Next Steps](#14-next-steps)
15. [Changelog](#15-changelog)
16. [Status Tag Convention](#16-status-tag-convention)

---

## 1. Objective

Add a remote command layer to an existing pump installation while preserving everything
that already works.

**In scope**

- Remote ON
- Remote OFF
- Remote RESTART
- Status reporting back to a mobile app

**Explicitly out of scope**

- Replacing the existing DMAX Plus PX-4 automatic controller
- Making the ESP32 responsible for pump protection or automatic operation

**Design philosophy**

> The ESP32 issues the command. The relay interfaces the command. The contactor switches
> the motor circuit. The DMAX keeps doing its existing automatic job.

A consequence worth stating: if the network link dies, the DMAX still runs the pump. The
remote layer is additive, never load-bearing for basic operation.

---

## 2. Architecture

```text
        Mobile app  (Claude Design front end)
              |
              |  MQTT over TLS, over the internet
              v
      HiveMQ Cloud broker
              |
              |  MQTT
              v
      +----------------+
      |     ESP32      |   <-- transport layer UNRESOLVED (see Open Problems)
      +----------------+
         |          ^
   GPIO 26          | GPIO 27  (relay feedback contact, INPUT_PULLUP)
         v          |
      +--------------------------+
      | Optocoupler relay module |   1-channel, 5 V, opto-isolated
      +--------------------------+
              |  COM / NC  (fail-safe, normally closed)
              |  coil drive (A1 / A2)
              v
      +--------------------------+
      | Schneider LC1E0910       |
      | contactor                |
      +--------------------------+
              |
              |  220 VAC switching
              v
         DMAX Plus PX-4
              |
              v
      JET NS3 pump, 1 HP, ~4.5 A
```

### MQTT topics

| Topic | Direction | Payload | Retained |
|---|---|---|---|
| `home/pump/cmd` | in | `CMD` or `CMD:user` — see below | no |
| `home/pump/state` | out | `ON` \| `OFF` — the real contactor reading | yes |
| `home/pump/avail` | out | `online` \| `offline` — published by the broker via MQTT's last will | yes |
| `home/pump/hb` | out | uptime in seconds, every `HEARTBEAT_MS` (60 s) | no |
| `home/pump/status` | out | JSON reply to a `STATUS` command | no |
| `home/pump/log` | out | one system-log line per message | no |

**Commands:** `ON`, `OFF`, `RESTART`, `STATUS`.

`RESTART` cuts power, waits `RESTART_DELAY_MS` (6 s), then restores it. A second
`RESTART` arriving while one is already running is **refused by the firmware** and logged.
This is enforced on the board rather than in the UI: a greyed-out button is a convention
that any other MQTT client ignores, and without the guard a second `RESTART` would simply
push the timer out by another 6 s, so repeated presses could hold the pump off
indefinitely. A UI lock is still worth having on top — the app reads `restarting` from a
`STATUS` reply so that an app opened mid-cycle knows to lock its button.

**Command payload format.** The payload is either a bare command (`ON`) or a command with
the person who issued it (`ON:Mohamed`). The name is optional, so a plain MQTT client can
still drive the pump by hand during testing, and when present it is recorded in the log
and returned as `last_user` in the next `STATUS` reply. Both halves travel in **one**
message deliberately: sending the actor on a separate topic could not be correlated
reliably once two people act at nearly the same moment.

**`STATUS` reply shape:**

```json
{"ssid":"Damasy","rssi":-58,"ip":"192.168.100.57","uptime_s":3600,
 "state":"ON","restarting":false,"last_user":"Mohamed","cpu_mhz":240}
```

### Judging whether the reported state is current

A retained message carries no timestamp, so `home/pump/state` on its own cannot say
whether it is current or was left behind hours ago by a board that has since died. Three
mechanisms answer that, in increasing order of confidence:

- **`avail`** — the broker's own view, free and retained, but it only fires once the MQTT
  keepalive expires (~22 s with PubSubClient's 15 s default). It can also show a brief
  `offline` → `online` flap on a fast reconnect, because a new session taking over the
  same client ID causes the broker to publish the old session's will.
- **`hb`** — deliberately **not** retained, so it can only ever arrive from a board that
  is alive at that moment. Hear nothing for ~2.5 intervals and treat the state as stale
  whatever `avail` says. The uptime payload also makes a crash loop obvious.
- **`STATUS`** — on demand and immediate. Publish it, hear nothing back within a second
  or two, and the controller is not there. This is what the app should use on open and
  behind a "check controller" button.

---

## 3. Control Design and Fail-Safe Behaviour

`DECIDED` — designed and implemented in `Code/pump_minimal`. **Not yet bench tested.**

### Fail-safe switching philosophy

The pump circuit is normally energised around the clock, and the DMAX controller is what
decides when the motor actually runs. The retrofit must therefore never leave the pump
dead just because the ESP32 crashed, lost power, or is mid-reboot.

That is achieved by wiring the relay through its **normally-closed** contact, so the
logical pump command and the physical relay state are deliberately inverted:

| MQTT command | Relay coil | COM–NC contact | Result |
|---|---|---|---|
| `ON` (normal state) | de-energised | closed | power reaches the motor controller |
| `OFF` (rare) | energised | open | power to the motor controller is cut |

With the relay idle in the common case, the opto-coupler LED and relay coil draw no
current for the vast majority of the time — the power cost is only paid during the rare
`OFF` state.

Because `PIN_RELAY` floats during the ESP32's own boot/reset, before any firmware runs, a
**physical 10 kΩ pull-up from GPIO 26 to 3.3 V** is required so the pin reads HIGH (relay
de-energised, power flowing) during that window too. Without it the fail-safe guarantee
only holds once `setup()` has executed.

### State feedback from the contactor

`GPIO 27` reads the contactor's **NO auxiliary contact — terminals 13/14** on the
LC1E0910, so what gets reported is what the contactor actually did, not merely what the
firmware asked for. The reading is debounced (150 ms) before it is published retained on
`home/pump/state`.

Wiring: `ESP32 GND → terminal 13`, `terminal 14 → GPIO 27`.

An auxiliary contact is a **dry contact** — it carries no voltage of its own, only
whatever is fed into terminal 13. Feeding it the ESP32's own ground keeps the whole loop
at 3.3 V and means no isolation is needed, **provided 13/14 are wired to nothing else**.
The terminals still sit on a body carrying 220 VAC, so the pair is routed and insulated
as mains-adjacent wiring.

**Use an external 1 kΩ pull-up from GPIO 27 to 3.3 V**, not the internal one. Schneider
rates this auxiliary contact for a minimum switching capacity of **17 V / 5 mA**; the
ESP32's ~45 kΩ internal pull-up passes only ~70 µA, far too little to break through the
oxide film that forms on the contact surface, which shows up as intermittent false
readings. 1 kΩ gives ~3.3 mA.

Until 13/14 are physically wired, GPIO 27 sits at the pull-up and reports `OFF`
permanently — expected, not a fault.

*Superseded:* an earlier bench-test loopback read a spare contact on the relay module
itself (`ESP32 GND → relay COM`, `relay NO → GPIO 27`). That verified only that the relay
clicked, and a 1-channel relay has a single COM terminal, so it cannot coexist with the
production COM/NC wiring. It remains usable for testing the control chain **before** the
contactor is in circuit.

### Still to be specified

The default state after each of the following events is inherited from the NC wiring
above (power flows), but the firmware-level behaviour has not been written into a test
matrix yet: ESP32 reboot, WiFi loss, MQTT broker loss, mains power restoration. These are
covered by the functional test matrix in [Next Steps](#14-next-steps).

---

## 4. Firmware

### Networking

The firmware holds two sets of WiFi credentials and cycles between them — primary first,
backup second, 15 s per attempt — until one answers. This runs at boot and again from
`loop()` whenever the connection drops; the ESP32's built-in auto-reconnect is disabled
because it only ever retries the last SSID.

### Logging

Everything printed to the serial monitor is also published to `home/pump/log`, so the
mobile app can subscribe and show the same system log for live debugging. Lines produced
before WiFi/MQTT are available (for example `failed to connect to internet`) are held in a
**4 KB RAM ring buffer** and flushed in order the moment MQTT connects. If the buffer
fills first, the oldest lines are dropped. The buffer is RAM, not flash: it bridges a
network outage, it does not survive a power loss.

### CPU throttling — thermal, then power

The enclosure lives **under the stairs**: poor ventilation, and Egyptian summer ambient
temperatures on top of that. The board is mains-powered, so this is primarily about not
cooking the processor over years of continuous running; the power saving is a secondary
benefit and a small one.

The firmware runs at **240 MHz while the system is in use** and drops to **80 MHz after
`CPU_IDLE_MS` (1 hour) with no command on `home/pump/cmd`**, returning to full speed the
instant a command arrives. The idle countdown restarts from every command, so the board
spends the long idle stretches — which is most of its life, since the pump may go a week
untouched — running cooler.

**80 MHz is the floor, and the reason is not arbitrary.** At 240, 160 and 80 MHz the CPU
is clocked from the PLL and the **APB bus stays at 80 MHz**, so UART baud rates and
peripheral timing are unaffected. Below 80 MHz the CPU switches to the crystal, the APB
bus follows it down, and serial output turns to garbage. WiFi also needs the PLL domain.
So 80 MHz is the lowest setting that changes nothing functionally.

**What it does not change:** WiFi, the MQTT session, the feedback pin, command latency
and the fail-safe behaviour all carry on identically. The only observable difference is
`cpu_mhz` in a `STATUS` reply, and two lines in the log when it switches.

**What it realistically saves.** The CPU core is not the dominant heat source on a dev
board — the LDO regulator and the WiFi radio during transmit are comparable or larger, so
this reduces the board's thermal load, it does not solve a ventilation problem. Improving
airflow in the enclosure matters more than this setting does.

### Command attribution

Commands may name the person who issued them (`ON:Mohamed`), and the firmware records
that name in the log line for the command and returns it as `last_user` in the next
`STATUS` reply. The intent is that the mobile app always sends its signed-in user, so the
log answers "who turned the motor off" without ambiguity, and so the app can notify other
users that someone changed the state.

The board only records the name it is given — it does not authenticate it. Anyone holding
the MQTT credentials can publish any name. Treat `last_user` as an audit convenience, not
as an access control mechanism.

### Sketches

| Sketch | Purpose |
|---|---|
| `Code/pump_minimal` | minimal firmware: WiFi + MQTT + relay + feedback + logging |
| `Code/pump_control` | fuller firmware: lockout, runtime limits, JSON state, LWT |
| `Code/wifi_survey` | RSSI site survey tool used for the measurements in section 7 |

---

## 5. Hardware

| Item | Part | Status |
|---|---|---|
| Controller | ESP32-D0WD-V3 rev 301 dev board | DECIDED |
| Main switching device | Schneider LC1E0910 (EasyPact TVS) | DECIDED — purchased |
| Control interface | RAM KIT.M2 1-channel 5 V optocoupler relay module | DECIDED |
| ESP32 supply | MicroOhm 5 V / 1 A adapter | DECIDED |
| Pump | JET NS3, 1 HP, single phase, ~4.5 A | EXISTING |
| Automatic controller | DMAX Plus PX-4 | EXISTING — stays in circuit |
| Relay pull-up | 10 kΩ, GPIO 26 to 3.3 V | DECIDED — see section 3 |
| RC snubber | across the switched inductive load | PENDING — spec not finalised |
| Fuse | protecting the control branch | PENDING — rating and location not fixed |
| Enclosure | not specified | PENDING |
| Network bridge | Huawei HG633-12 as ground-floor AP, SSID `Damasy` | DECIDED — installed and verified |

### Superseded selections

| Part | Replaced by | Reason |
|---|---|---|
| Himel HDC3-0911M7 contactor | Schneider LC1E0910 | availability / preference at purchase time |

### Why a contactor rather than a relay

The pump's ~4.5 A figure is running current, not starting current. A single-phase
induction motor draws several times that at startup, and interrupting an inductive load
produces a voltage transient across the opening contacts. A general-purpose PCB relay
rated "10 A resistive" is not qualified for repeated switching of this load. The
contactor handles the motor circuit; the small relay only drives the contactor coil.

**A1 and A2 are coil terminals, not power contacts.** The coil must be energised at its
rated coil voltage. Shorting A1 to A2 is not a method of operating the contactor.

---

## 6. Open Problems

### 6.1 Mains topology not established — HIGH

The exact point at which the contactor is inserted into the existing DMAX/pump circuit
has not been determined from the physical installation. Must be resolved from the actual
DMAX terminals, not from assumptions about generic pump controllers. Risk if guessed: the
DMAX gets electrically bypassed or behaves unexpectedly.

### 6.2 Contactor coil voltage not confirmed — HIGH

The coil supply voltage of the purchased LC1E0910 and the method by which the relay
module drives it must be verified before wiring. This determines whether the control run
is mains-level or SELV, which in turn affects enclosure design and any long cable run.

### 6.3 Protection components not specified — MEDIUM

RC snubber model and connection point, fuse rating and location, protective earth
implementation, wire gauges, terminal blocks.

### 6.4 Access point disturbs the rest of the LAN — LOW

**Discovered:** 2026-09-13. **Does not affect the pump link.**

While the LAN cable from Nabil is connected to the HG633, Windows and Android clients on
**both** house networks display "Action needed / Connected without internet" and Windows
auto-opens a browser. Unplugging that one cable clears it instantly on every client.

The warning is false. Measured while it was showing: `ping 8.8.8.8` returned **0% loss at
44 ms**, `ping 192.168.100.1` 0% loss, DHCP lease, gateway and DNS all correct and served
by Nabil. The OS connectivity probe (`msftconnecttest.com`) fails while real traffic flows,
so TCP survives — it retries — and the short-timeout probe does not.

**The ESP32 is unaffected** and holds its MQTT session throughout, so this is cosmetic for
this project. Accepted as a known defect rather than fixed; see the diagnosis record below
for why, and [section 8](#8-options-under-evaluation) for the replacement option.

### Resolved

| Problem | Resolution |
|---|---|
| Fail-safe behaviour undefined | Resolved 2026-09-12. NC-contact wiring plus a 10 kΩ hardware pull-up, implemented in `pump_minimal`. See [section 3](#3-control-design-and-fail-safe-behaviour). Bench verification still outstanding. |
| No WiFi coverage at the pump location (was the project BLOCKER) | Resolved 2026-09-13 by Option A. A spare Huawei HG633-12 was installed as a ground-floor access point (`Damasy`, ch 11, cabled LAN-to-LAN2 from the Nabil router). Signal at the enclosure position went from **-92 dBm, 10/16 detection** to **-58 dBm**, and the ESP32 now opens and holds MQTT over TLS on port 8883. See [section 8](#8-options-under-evaluation). |
| ESP32 would not associate with the new AP | Resolved 2026-09-13. **A typo, not a network fault.** `secrets.h` held the `Damasy` PSK with its leading `#` missing (10 chars instead of 11), so the 4-way handshake failed, `WiFi.status()` never reached `WL_CONNECTED`, and the firmware's 15 s timeout handed the connection to the backup SSID every time. The symptom looked exactly like an AP rejecting the client. See the lesson in the diagnosis record below. |

### Network diagnosis record — 2026-09-13

Kept so none of this is repeated. The investigation started from a captive-portal
hypothesis, which was wrong, and cost several rounds before the evidence overturned it.

**Ruled out, with the evidence that ruled it out**

| Hypothesis | Evidence against |
|---|---|
| Captive portal / HTTP interception on the AP | The ESP32 never reached layer 3 at all — the serial log showed a 15 s association timeout, not a failed TLS session. A portal operates above the layer that was failing. |
| Weak signal at the pump | -58 dBm measured by the ESP32 itself at the enclosure position |
| Rogue DHCP server on the AP | With the cable out and a laptop on the AP's own WiFi, no lease was offered at all — no DHCP server line, no APIPA address |
| Duplicate IP / gateway address conflict | `arp -a` showed exactly one MAC for `192.168.100.1` while the fault was present |
| IPv6 RA with an RDNSS option from the AP | `netsh interface ipv6 show neighbors` resolved `fe80::1` to the **Nabil** router's MAC, not the AP's |
| `fe80::1` being the cause at all | It is present and first in the DNS list in the **healthy** state too. Every layer-3 parameter is byte-identical between the working and broken states |
| The AP's IPv6-enabled WAN profile | Setting `INTERNET_TR069_ETH` to IPv4-only changed nothing |

**What that leaves.** Every layer-3 setting is identical in both states, and a client on
the Nabil network does not route through the HG633 at all, yet is still affected. That
points to layer-2 behaviour of the HG633 — frame echo, MAC-table flapping, or broadcast
flooding from its always-online PPPoE dialer, which carries the placeholder account
`00000@tedata.net.eg` and can never authenticate. **This is unverified**; confirming it
needs port monitoring or a packet capture that was judged not worth the time, since the
pump link is unaffected.

**Why it was not fixed in the device.** The firmware is TEData-locked: `Maintain → Remote
Management` is greyed out, so CWMP cannot be disabled, and the TR-069 log shows a
`parameter change` from `hdm.tedata.net.eg` every 30 minutes — the ISP can revert any
setting. A layer-2 behaviour is not a settings-page item in any case. Replacing the HG633
with a plain unmanaged router or AP is the remaining option.

**Changes left applied to the HG633**

- `INTERNET_TR069_ETH` → `IP protocol version` set to IPv4 only (correct for an AP)
- `INTERNET_TR069_ETH` → `Enable connection` unchecked, stopping the permanent failing
  PPPoE dial loop and cutting the device's path to the TEData ACS

**Lesson recorded.** Before investigating any network for a device that will not connect,
read the serial log and establish **which layer is failing**. An association timeout and a
broker timeout look identical from the outside and lead to completely different
investigations. Diffing the credentials in `secrets.h` against the working copy in
`wifi_survey.ino` would have found this in one minute.

---

## 7. Site Survey Results

**Date:** 2026-09-11
**Instrument:** ESP32-D0WD-V3, `Code/wifi_survey/wifi_survey.ino`
**Method:** reset at each point, discard first 2 rounds, record 8+ rounds

| Location | Nabil avg | Nabil worst | Detect | JOY avg | JOY worst | Detect |
|---|---|---|---|---|---|---|
| Next to JOY router (2F) | **-70** | -83 | 17/17 | **-46** | -50 | 17/17 |
| Mid-stairs | -88 | -91 | 13/13 | never seen | — | 0/13 |
| Pump room door (GF) | -93 | -93 | 2/9 | -98 | -98 | 1/9 |
| **Enclosure position (GF)** | **-92** | -96 | 10/16 | **-94** | -97 | 10/16 |

Unidentified network `Y?` at the enclosure position: **~-71 dBm**, channel 5, not visible
from the 2nd floor.

**Reference scale used**

| RSSI | Verdict |
|---|---|
| >= -70 dBm | good, MQTT should hold |
| -70 to -80 | marginal, will drop |
| -80 to -88 | very weak, unreliable |
| < -88 dBm | dead |

**Channel occupancy observed:** 1, 3, 5, 6, 7, 9, 10, 13 all in use by neighbouring
networks. Nabil on ch 1, JOY on ch 10. **Channel 11 is clear** and is the recommended
channel for any new AP.

### Post-installation measurement — 2026-09-13

After the ground-floor AP was installed, measured by the ESP32 itself at the enclosure
position and reported on `home/pump/status`:

| Network | Before (2026-09-11) | After (2026-09-13) |
|---|---|---|
| Nabil (1F) | -92 dBm, 10/16 detection | unchanged |
| **Damasy (GF AP)** | did not exist | **-58 dBm, MQTT session holds** |

That is a 34 dB improvement and roughly 13 dB of margin above the -70 dBm threshold. It
confirms the vertical-asymmetry analysis: the problem was transmitter placement, not the
site.

**Not yet measured**

- Reading directly at the Nabil router (1F) — no baseline for the downward path
- Reading with the ESP32 inside a **closed** enclosure on the Damasy network. The -58 dBm
  figure has margin for the expected enclosure loss, but it has not been measured.
- Long-run stability: no multi-hour count of `home/pump/hb` gaps or `uptime_s` resets has
  been taken, so "holds the session" is verified over minutes, not days

---

## 8. Options Under Evaluation

Option A was committed and implemented on 2026-09-13. B, C and D are retained as recorded
alternatives, and B remains the planned Phase 2.

### Option A — LAN cable to a ground-floor access point ★ IMPLEMENTED 2026-09-13

Run Cat5e from the 1st floor down to the ground floor, terminate in a router configured
as an access point on channel 11.

- **Reliability:** highest of the options
- **Cost:** cable plus a spare router
- **Precedent:** a LAN cable is already run between the 1st and 2nd floors for extender
  duty, so this is a known-feasible operation, just in the other direction
- **Dependency:** requires mains power at the ground-floor AP location — **confirmed
  available; the AP is installed and powered in the ground-floor reception**
- **Architecture impact:** none — HiveMQ, app, and firmware unchanged

**As built.** Huawei HG633-12 (firmware `V100R001C105B022 TEDATA`), SSID `Damasy`,
channel 11, LAN IP `192.168.100.200`, DHCP and IPv6 services off, cabled from a LAN port
on the Nabil router to LAN2. Clients bridge onto Nabil's `192.168.100.0/24` and take
their leases from Nabil at `192.168.100.1`.

**Known defect.** This particular router disturbs the rest of the LAN while cabled in —
see [6.4](#64-access-point-disturbs-the-rest-of-the-lan--low). It does not affect the pump
link. Replacing it with a plain unmanaged router or access point is the open follow-up,
and is the preferred fix rather than further configuration work, because the HG633's
firmware is ISP-locked.

### Option B — LoRa link, 433 MHz

ESP32 gateway upstairs (WiFi + LoRa), ESP32 node at the pump (LoRa only).

- 433 MHz penetrates reinforced concrete far better than 2.4 GHz; link budget has large
  margin for this geometry
- Cheap modules (SX1278 class)
- **Cost:** a second ESP32 and a custom protocol to write
- **Architecture impact:** MQTT client moves to the gateway. App and broker unchanged.
- Becomes first choice if no mains power is available at the pump for an AP

### Option C — Powerline (PLC) adapters

Uses existing mains wiring as the data path.

- **Hard dependency:** the pump circuit must share a meter and phase with the apartment.
  In a multi-unit Egyptian building the pump is commonly on a separate meter, in which
  case this fails outright.
- Must be verified before purchase, not after

### Option D — 4G / cellular module at the pump

- Removes all dependence on house networks
- **Cost:** module, SIM, recurring monthly charge
- **Unverified:** cellular signal strength under the stairs has not been measured and may
  be poor for the same structural reasons

### Rejected

- External antenna as a standalone fix
- Repeater in the stairwell
- Any access point on the 1st floor

---

## 9. Current Status

| Layer | State |
|---|---|
| Electrical architecture | Settled |
| Main components | Purchased |
| Mobile app front end | In development (Claude Design) |
| MQTT broker | HiveMQ Cloud cluster provisioned |
| Transport layer | **Resolved** — ground-floor AP installed, Option A as built |
| Firmware — connectivity | Written and **verified at the pump position**, -58 dBm |
| MQTT over TLS on 8883 | **Verified end to end** — command sent, status returned |
| Firmware — ON/OFF/RESTART logic | Written in `pump_minimal` |
| Fail-safe behaviour | Designed and implemented — not bench tested |
| Relay feedback path | Implemented — not bench tested |
| Long-run link stability | Not measured beyond a few minutes |
| Mains wiring topology | Not frozen |
| Bench test of control chain | Not performed |
| Mains integration | Not started |

---

## 10. Decision Log

| Date | Decision | Status |
|---|---|---|
| 2026-08-22 (approx.) | Contactor rather than PCB relay as main switching device | DECIDED |
| 2026-08-22 (approx.) | Schneider LC1E0910M5 identified as lead candidate | DECIDED |
| 2026-09-05 | Master project reference consolidated | — |
| 2026-09-05 | DMAX Plus PX-4 stays in circuit; ESP32 is additive only | DECIDED |
| 2026-09-05 | Low-voltage and mains sections physically separated in enclosure | DECIDED |
| (date TBC) | HiveMQ Cloud chosen over a local broker, for control from outside the house | DECIDED |
| (date TBC) | Schneider LC1E0910 purchased, superseding Himel HDC3-0911M7 | DECIDED |
| (date TBC) | Credentials split into a gitignored `secrets.h` | DECIDED |
| (date TBC) | Fail-safe via relay COM/NC wiring plus a 10 kΩ hardware pull-up on GPIO 26 | DECIDED |
| (date TBC) | Relay feedback read back on GPIO 27, debounced 150 ms, published retained on `home/pump/state` | DECIDED |
| (date TBC) | Dual WiFi credential sets cycled at 15 s; built-in auto-reconnect disabled | DECIDED |
| (date TBC) | Serial log mirrored to `home/pump/log` with a 4 KB RAM ring buffer | DECIDED |
| 2026-09-11 | Site survey performed; both house networks unusable at the pump | FINDING |
| 2026-09-12 | Antenna upgrade, JOY targeting, stairwell repeater, 1F AP all ruled out | DECIDED |
| 2026-09-12 | Ground-floor AP established as a hard requirement for any WiFi solution | DECIDED |
| 2026-09-12 | `README.md` and `README2.md` merged into this single reference | — |
| 2026-09-13 | Option A committed and built: Huawei HG633-12 installed as ground-floor AP `Damasy`, ch 11, `192.168.100.200`, cabled LAN-to-LAN2 from Nabil | DECIDED |
| 2026-09-13 | ESP32 association failure traced to a missing leading `#` in the `Damasy` PSK in `secrets.h`, not to any router setting | FINDING |
| 2026-09-13 | MQTT over TLS on 8883 verified from the enclosure position at -58 dBm with a full command round trip; transport layer closed | DECIDED |
| 2026-09-13 | Captive portal, rogue DHCP, IP conflict and IPv6 RDNSS all ruled out by measurement as causes of the LAN-wide "no internet" indicator | DECIDED |
| 2026-09-13 | HG633 LAN disturbance accepted as a known low-severity defect; replacing the router preferred over further configuration, since the firmware is ISP-locked | DECIDED |
| 2026-09-14 | State feedback moved from the relay loopback to the contactor's 13/14 auxiliary contact, with an external 1 kΩ pull-up to meet the 17 V / 5 mA minimum switching spec | DECIDED |
| 2026-09-14 | `avail` (LWT), `hb` (non-retained heartbeat) and `STATUS` (on-demand) added as three independent ways to judge whether the reported state is current | DECIDED |
| 2026-09-14 | Heartbeat slowed from 10 s to 60 s once `STATUS` covered on-demand checks; measured cost was 0.22% of the HiveMQ free-tier allowance, so message volume was never the real argument | DECIDED |
| 2026-09-14 | `cmd` payload extended to `CMD:user`, in one message rather than a separate actor topic | DECIDED |
| 2026-09-14 | Repeat `RESTART` refused in firmware while one is in progress; UI lock treated as advisory only | DECIDED |
| 2026-09-14 | CPU throttled to 80 MHz after 1 h idle, back to 240 MHz on any command, for thermal headroom under the stairs | DECIDED |

---

## 11. Repository Layout

```text
water-pump-control/
├── Code/
│   ├── pump_minimal/        minimal connectivity + relay + feedback + logging sketch
│   │   ├── pump_minimal.ino
│   │   ├── secrets_example.h   template — copy to secrets.h
│   │   └── secrets.h           gitignored, never committed
│   ├── pump_control/        fuller firmware: lockout, runtime limits, JSON state, LWT
│   ├── wifi_survey/         RSSI site survey sketch
│   └── isrgrootx1.crt       ISRG Root X1 certificate for HiveMQ Cloud TLS
├── Docs/                    reference material, survey logs
├── Icons/                   app icons for ON / OFF / RESTART
├── Prompts/                 working notes
├── .gitignore               excludes secrets.h and build output
└── README.md                this file
```

`secrets.h` holds WiFi and MQTT credentials and is **never committed**. Clone the repo,
copy `secrets_example.h` to `secrets.h`, and fill in your own values.

---

## 12. Building

1. Install the ESP32 board package in the Arduino IDE.
2. Install the **PubSubClient** library (Nick O'Leary) via the Library Manager.
3. Create your credentials file:

   ```sh
   cd Code/pump_minimal
   cp secrets_example.h secrets.h
   ```

   Then fill in your WiFi and MQTT values.
4. Select your ESP32 board and flash.

---

## 13. Safety

This project combines low-voltage electronics with a 220 VAC mains circuit and an
inductive motor load. The following are requirements, not suggestions.

- The ESP32 never switches the pump directly
- The optocoupler relay module never carries motor current — it only drives the
  contactor coil
- Physical and electrical separation between the low-voltage zone (ESP32, relay module,
  5 V wiring) and the mains zone (contactor power contacts, pump and DMAX wiring) inside
  the enclosure
- No exposed mains conductors; insulated terminals and strain relief throughout
- Appropriate conductor sizing and proper protective earthing
- Verify the contactor coil voltage and the relay module's contact rating against your own
  hardware before energising anything
- The control chain (ESP32 → relay → contactor) is bench tested **before** any mains
  integration, and before the pump is connected
- A fuse on the control branch is not a substitute for proper motor protection

---

## 14. Next Steps

The transport layer is closed. Everything below is electrical and mechanical.

### Immediate — the highest-risk untested item

1. **Bench test ESP32 → relay → contactor with no pump connected.** The fail-safe design is
   written and implemented but has **never been run once**. Include the GPIO 27 feedback
   loopback and, specifically, the 10 kΩ pull-up behaviour through a full ESP32 reset — the
   whole fail-safe guarantee during boot rests on that resistor.
2. **Verify LC1E0910 coil voltage**, contact rating, load category and terminal
   identification. This decides whether the control run is mains-level or SELV, which
   drives the enclosure design.
3. **Document the existing DMAX installation** from the physical hardware: input, output,
   pump connection, L/N/PE, existing protection, current behaviour. Tagged `UNKNOWN` —
   must not be guessed.

### After those three

4. Verify relay module input requirements against ESP32 GPIO, and output rating against
   the coil
5. Specify protection: fuse, snubber, earthing, enclosure, wire gauges
6. Mains integration
7. Functional test matrix: local ON/OFF, remote ON/OFF, restart, ESP32 reboot, WiFi loss,
   MQTT loss, power restoration, DMAX automatic operation, repeated motor starts

### Network follow-ups — optional, not blocking

8. Measure long-run link stability: let the ESP32 run and count gaps in `home/pump/hb` and
   resets of `uptime_s`. Converts "holds the session" from minutes to a real number.
9. Replace the HG633 with a plain router or access point, which is expected to clear
   [6.4](#64-access-point-disturbs-the-rest-of-the-lan--low)
10. Measure RSSI with the ESP32 inside the closed enclosure on `Damasy`

---

## 15. Changelog

### 2026-09-14

- State feedback rewired in the design from the relay loopback to the contactor's 13/14
  auxiliary contact, so `home/pump/state` reports what the contactor did rather than what
  the relay was told to do. Added the external 1 kΩ pull-up requirement and the reasoning
  behind it (17 V / 5 mA minimum switching capacity vs ~70 µA from the internal pull-up)
- Added `home/pump/avail` (retained, via MQTT last will), `home/pump/hb` (non-retained
  heartbeat carrying uptime) and a `STATUS` command replying on `home/pump/status`, and
  documented how a subscriber should combine the three to judge staleness
- Heartbeat interval moved 10 s → 60 s after measuring that message volume was never the
  constraint: HiveMQ's free tier caps data, not messages, and 10 s cost ~21 MB/month
  against a 10 GB allowance
- `cmd` payload extended to the optional `CMD:user` form; the name is logged and returned
  as `last_user`. Recorded explicitly that it is not authenticated
- Repeat `RESTART` now refused by the firmware while one is running, with the reasoning
  for enforcing it on the board rather than in the UI
- CPU throttling added: 80 MHz after 1 h idle, 240 MHz on any command, with the APB/UART
  reasoning for why 80 MHz is the floor. Motivated by the under-stairs enclosure

### 2026-09-13

- **Project unblocked.** Option A built and verified: Huawei HG633-12 installed as the
  ground-floor access point `Damasy`; ESP32 measures -58 dBm at the enclosure position and
  holds an MQTT-over-TLS session on port 8883 with HiveMQ Cloud, confirmed by a command
  round trip on `home/pump/cmd` → `home/pump/status`
- Root cause of the ESP32 association failure identified as a missing leading `#` in the
  `Damasy` PSK in `secrets.h` — a typo, not a router setting; corrected
- Status changed from `BLOCKED` to electrical/mechanical work; section 6.1 (no WiFi
  coverage) moved to Resolved and the remaining problems renumbered
- Added section 6.4 for the HG633 LAN disturbance, and a network diagnosis record listing
  every hypothesis ruled out with the evidence that ruled it out, so it is not repeated
- Added the post-installation RSSI measurement to section 7
- Next Steps reordered around the bench test of the control chain, which has never been run

### 2026-09-12

- `README.md` and `README2.md` merged into this single reference; contactor selection,
  repository layout and firmware status reconciled between the two
- Fail-safe behaviour reclassified from "not defined" to DECIDED and implemented,
  pending bench verification
- Ruled out: external antenna as standalone fix, JOY as target network, stairwell
  repeater, any 1st-floor access point
- Established ground-floor AP as a hard requirement for the WiFi path
- Ranked transport options A–D; Option A (LAN to ground-floor AP) set as favourite
- Defined the hotspot validation test as the gate before any purchase

### 2026-09-11

- Site survey performed at 4 locations with `wifi_survey.ino`
- Confirmed both house networks unusable at the pump (~-92 dBm, 10/16 detection)
- Identified the 22 dB up/down vertical asymmetry from the Nabil router
- Identified a strong unknown ground-level network at -71 dBm as evidence that the
  location is not an RF dead zone
- Project status changed to BLOCKED on connectivity

### 2026-09-05

- Master project reference consolidated
- Architecture confirmed: ESP32 → relay → contactor → DMAX → pump
- Enclosure separation philosophy confirmed

---

## 16. Status Tag Convention

| Tag | Meaning |
|---|---|
| `DECIDED` | Selected and committed |
| `RECOMMENDED` | Engineering preference, not frozen |
| `PENDING` | Requires verification before final design |
| `BLOCKED` | Cannot proceed until a dependency is resolved |
| `REJECTED` | Considered and ruled out, with reason recorded |
| `UNKNOWN` | Not established. Must not be guessed. |
