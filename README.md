# Water Pump Remote Control

ESP32-based IoT retrofit that adds remote **ON / OFF / RESTART** control to an existing
residential water pump over MQTT, **without replacing the pump's existing automatic
controller**.

> **Status as of 2026-09-12: BLOCKED on network connectivity at the pump location.**
> The electrical design and the control/fail-safe design are settled. The pump site has
> no usable WiFi coverage, so the transport layer is under redesign.
> See [Open Problems](#6-open-problems).

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
| `home/pump/cmd` | in | `ON` \| `OFF` \| `RESTART` | no |
| `home/pump/state` | out | `ON` \| `OFF` (real relay feedback) | yes |
| `home/pump/log` | out | one system-log line per message | no |

`RESTART` cuts power, waits `RESTART_DELAY_MS` (6 s), then restores it.

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

### Relay feedback

`GPIO 27` is configured as `INPUT_PULLUP` and reads back a spare relay contact, so the
firmware can confirm the relay physically actuated rather than merely trusting the
command it sent. The reading is debounced (150 ms) before it is reported on
`home/pump/state`.

Bench-test wiring: `ESP32 GND → relay COM`, `relay NO → GPIO 27`. This loop carries only
the ESP32's own ground through a dry contact — no coil voltage or mains — so it needs no
isolation. Note that a 1-channel relay has a single COM terminal, so the test loopback
and the production COM/NC wiring **cannot be connected at the same time**.

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
| Network bridge | not selected | **BLOCKED** |

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

### 6.1 No WiFi coverage at the pump location — BLOCKER

**Discovered:** 2026-09-11

**Site layout**

```text
2nd floor   JOY router
1st floor   Nabil router
Ground      pump, under the stairs, enclosed space
```

**Symptom.** The ESP32 cannot associate with either house network at the pump. Both
networks are visible on a phone at that location but the ESP32 radio has roughly
6–10 dB less receive sensitivity than a phone, and the margin is not there.

**Measured.** Both networks sit around **-92 dBm** at the intended enclosure position,
with the scan itself failing roughly 37% of the time. A stable ESP32 MQTT session needs
about **-75 dBm or better**; -80 dBm is the practical floor. The deficit is 12–17 dB,
i.e. the received power is 16 to 50 times below what is needed. This is not a margin that
an antenna upgrade can close.

**Key finding — vertical asymmetry.** The Nabil router is on the 1st floor.

- One floor **up** (at the JOY router): **-70 dBm**, detected 17/17
- One floor **down** (at the pump): **-92 dBm**, detected 10/16

A 22 dB asymmetry over the same one-floor distance, in opposite directions. Plausible
mechanism: the upward path runs through the open stairwell void, while the downward path
crosses a reinforced concrete slab and then enters an enclosed under-stair space. Rebar
in the slab acts as a reflective mesh, and the enclosed space removes any indirect path.

**Why this matters for the fix.** Nabil is already on the 1st floor and delivers -92 dBm
at the pump. Therefore *any* access point placed on the 1st floor will perform similarly.
Best case, positioned at the stairwell door, optimistically -80 dBm — still below the
operating threshold.

> **Conclusion: the access point must be on the ground floor. No 1st-floor placement and
> no antenna change solves this.**

**Counter-evidence that the site itself is fine.** An unidentified neighbouring network
(rendered as `Y?` in the scan output, likely a non-ASCII SSID) measures **-71 dBm** at the
enclosure position and is *not visible at all* from the 2nd floor. Its source is therefore
at ground level, roughly one wall away. The pump location is not an RF dead zone — it
simply has no transmitter near it. A ground-floor AP should give good coverage. The
network is unusable directly since its owner is unknown.

**Ruled out**

| Approach | Why rejected |
|---|---|
| External antenna on the ESP32 | +3 to 5 dB. Deficit is 12–17 dB. |
| Targeting JOY instead of Nabil | JOY is worse: -94 dBm, invisible mid-stairs. |
| WiFi repeater mid-stairs | Repeater input there is -88 dBm. A repeater rebroadcasts noise along with signal; it needs -70 dBm or better at its input. |
| Any AP on the 1st floor | See vertical asymmetry above. |

### 6.2 Mains topology not established — HIGH

The exact point at which the contactor is inserted into the existing DMAX/pump circuit
has not been determined from the physical installation. Must be resolved from the actual
DMAX terminals, not from assumptions about generic pump controllers. Risk if guessed: the
DMAX gets electrically bypassed or behaves unexpectedly.

### 6.3 Contactor coil voltage not confirmed — HIGH

The coil supply voltage of the purchased LC1E0910 and the method by which the relay
module drives it must be verified before wiring. This determines whether the control run
is mains-level or SELV, which in turn affects enclosure design and any long cable run.

### 6.4 Protection components not specified — MEDIUM

RC snubber model and connection point, fuse rating and location, protective earth
implementation, wire gauges, terminal blocks.

### Resolved

| Problem | Resolution |
|---|---|
| Fail-safe behaviour undefined | Resolved 2026-09-12. NC-contact wiring plus a 10 kΩ hardware pull-up, implemented in `pump_minimal`. See [section 3](#3-control-design-and-fail-safe-behaviour). Bench verification still outstanding. |

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

**Not yet measured**

- Reading directly at the Nabil router (1F) — no baseline for the downward path
- Reading with the ESP32 inside a closed enclosure — deferred, the figure is already
  below threshold before enclosure loss is applied

---

## 8. Options Under Evaluation

Ranked by current assessment. None committed.

### Option A — LAN cable to a ground-floor access point ★ current favourite

Run Cat5e from the 1st floor down to the ground floor, terminate in a router configured
as an access point on channel 11.

- **Reliability:** highest of the options
- **Cost:** cable plus a spare router
- **Precedent:** a LAN cable is already run between the 1st and 2nd floors for extender
  duty, so this is a known-feasible operation, just in the other direction
- **Dependency:** requires mains power at the ground-floor AP location
- **Architecture impact:** none — HiveMQ, app, and firmware unchanged

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
| Firmware — connectivity | Written, **cannot connect at site** |
| Firmware — ON/OFF/RESTART logic | Written in `pump_minimal` |
| Fail-safe behaviour | Designed and implemented — not bench tested |
| Relay feedback path | Implemented — not bench tested |
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

### Immediate — validate before spending

1. Determine whether mains power is available at the candidate ground-floor AP location,
   and which meter it is on. This single answer decides between Option A and Option B.
2. Run the hotspot test: place a phone hotspot (2.4 GHz, SSID `test24`) at the candidate
   AP location, set `TARGETS` in the survey sketch to `test24`, and measure 8 rounds at
   the enclosure position.
   **Pass criterion: avg better than -70 dBm and 8/8 detection.**
   This converts the Option A plan from expectation to measurement before any cable is
   bought or run.

### After the transport layer is resolved

3. Document the existing DMAX installation: input, output, pump connection, L/N/PE,
   existing protection, current behaviour
4. Verify LC1E0910 coil voltage, contact rating, load category, terminal identification
5. Verify relay module input requirements against ESP32 GPIO, and output rating against
   the coil
6. Bench test ESP32 → relay → contactor with no pump connected, including the GPIO 27
   feedback loopback and the 10 kΩ pull-up behaviour through a full ESP32 reset
7. Specify protection: fuse, snubber, earthing, enclosure, wire gauges
8. Mains integration
9. Functional test matrix: local ON/OFF, remote ON/OFF, restart, ESP32 reboot, WiFi loss,
   MQTT loss, power restoration, DMAX automatic operation, repeated motor starts

---

## 15. Changelog

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
