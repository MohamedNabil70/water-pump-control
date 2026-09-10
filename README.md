# Water Pump Remote Control

An ESP32-based retrofit that adds remote ON / OFF / RESTART control to an existing
residential water pump over MQTT, **without replacing the pump's existing automatic
controller**.

The design principle throughout is that the ESP32 issues commands, a relay interfaces
those commands, a contactor does the actual mains switching, and the original DMAX
controller keeps performing its normal automatic function.

```
   Remote user (mobile app)
            │
        MQTT / TLS  (HiveMQ Cloud)
            │
            ▼
        ┌────────┐
        │ ESP32  │  GPIO 26 ─────► relay IN
        └────────┘  GPIO 27 ◄───── relay feedback contact
            │
            ▼
   Opto-isolated relay module
            │  (COM / NC, fail-safe)
            ▼
      Contactor coil (A1/A2)
            │
            ▼
        Contactor
            │  220 VAC
            ▼
      DMAX Plus PX-4  ──►  JET NS3 pump (1 HP, ~4.5 A)
```

## Fail-safe switching philosophy

The pump circuit is normally energised around the clock, and the DMAX controller is
what decides when the motor actually runs. The retrofit must therefore never leave the
pump dead just because the ESP32 crashed, lost power, or is mid-reboot.

That is achieved by wiring the relay through its **normally-closed** contact, so the
logical pump command and the physical relay state are deliberately inverted:

| MQTT command | Relay coil | COM–NC contact | Result |
|---|---|---|---|
| `ON` (normal state) | de-energised | closed | power reaches the motor controller |
| `OFF` (rare) | energised | open | power to the motor controller is cut |

With the relay idle in the common case, the opto-coupler LED and relay coil draw no
current for the vast majority of the time — the power cost is only paid during the rare
`OFF` state.

Because `PIN_RELAY` floats during the ESP32's own boot/reset, before any firmware runs,
a **physical 10 kΩ pull-up from GPIO 26 to 3.3 V** is required so the pin reads HIGH
(relay de-energised, power flowing) during that window too. Without it the fail-safe
guarantee only holds once `setup()` has executed.

## Relay feedback

`GPIO 27` is configured as `INPUT_PULLUP` and reads back a spare relay contact, so the
firmware can confirm the relay physically actuated rather than merely trusting the
command it sent. The reading is debounced (150 ms) before it is reported.

Bench-test wiring: `ESP32 GND → relay COM`, `relay NO → GPIO 27`. This loop carries only
the ESP32's own ground through a dry contact — no coil voltage or mains — so it needs no
isolation. Note that a 1-channel relay has a single COM terminal, so the test loopback
and the production COM/NC wiring cannot be connected at the same time.

## MQTT topics

| Topic | Direction | Payload | Retained |
|---|---|---|---|
| `home/pump/cmd` | in | `ON` \| `OFF` \| `RESTART` | no |
| `home/pump/state` | out | `ON` \| `OFF` (real relay feedback) | yes |
| `home/pump/log` | out | one system-log line per message | no |

`RESTART` cuts power, waits `RESTART_DELAY_MS` (6 s), then restores it.

## Networking and logging

The firmware holds two sets of WiFi credentials and cycles between them — primary first,
backup second, 15 s per attempt — until one answers. This runs at boot and again from
`loop()` whenever the connection drops; the ESP32's built-in auto-reconnect is disabled
because it only ever retries the last SSID.

Everything printed to the serial monitor is also published to `home/pump/log`, so the
mobile app can subscribe and show the same system log for live debugging. Lines produced
before WiFi/MQTT are available (for example `failed to connect to internet`) are held in
a 4 KB RAM ring buffer and flushed in order the moment MQTT connects. If the buffer fills
first, the oldest lines are dropped. The buffer is RAM, not flash: it bridges a network
outage, it does not survive a power loss.

## Hardware

| Component | Part | Status |
|---|---|---|
| Controller | ESP32 dev board | selected |
| Pump | JET NS3, 1 HP, ~4.5 A, single phase | existing |
| Automatic controller | DMAX Plus PX-4 | existing, kept in circuit |
| Mains switching | Himel HDC3-0911M7 contactor | selected |
| Control interface | RAM KIT.M2 1-channel 5 V opto relay module | selected |
| ESP32 supply | MicroOhm 5 V / 1 A adapter | selected |
| RC snubber | across the switched inductive load | recommended, spec not finalised |
| Fuse | protecting the control branch | rating and placement not finalised |

The contactor — not the small relay — is the element rated for the motor's inrush
current. The relay only drives the contactor coil.

## Repository layout

```
Code/
  pump_minimal/     minimal firmware: WiFi + MQTT + relay + feedback + logging
  pump_control/     fuller firmware: lockout, runtime limits, JSON state, LWT
  isrgrootx1.crt    ISRG Root X1 certificate for HiveMQ Cloud TLS
Icons/              app icons for ON / OFF / RESTART
Docs/               project documentation
Prompts/            working notes
```

## Building

1. Install the ESP32 board package in the Arduino IDE.
2. Install the **PubSubClient** library (Nick O'Leary) via the Library Manager.
3. Create your credentials file:

   ```
   cd Code/pump_minimal
   cp secrets_example.h secrets.h
   ```

   Then fill in your WiFi and MQTT values. `secrets.h` is gitignored and stays on your
   machine.
4. Select your ESP32 board and flash.

## Safety

This project interfaces low-voltage electronics with a 220 VAC motor circuit. The
low-voltage section (ESP32, relay module, 5 V wiring) must be physically and
electrically separated from the mains section (contactor power contacts, pump and DMAX
wiring), with insulated terminals, strain relief, no exposed mains conductors,
appropriate conductor sizing, and proper protective earthing.

Verify the contactor coil voltage and the relay module's contact rating against your own
hardware before energising anything, and bench-test the ESP32 → relay → contactor chain
before connecting the pump.
