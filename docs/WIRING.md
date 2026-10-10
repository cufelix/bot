# Wiring

## Power tree
```
3S LiPo (9.6–12.6 V) ──XT60──► kill switch 30 A ──► 20 A ATC fuse ──► INA219 shunt ──► V+ strip  ─┬─► L leg feed   (20 AWG)
                                                                                     GND strip ─┤─► R leg feed   (20 AWG)
                                                                                                ├─► L arm feed   (22 AWG)
                                                                                                ├─► R arm feed   (22 AWG)
                                                                                                ├─► neck feed    (22 AWG)
                                                                                                └─► Pololu D24V50F5 (5 V 5 A) ─► Pi 5 (GPIO pins 2/4 + 6, PSU_MAX_CURRENT=5000)
```
- Trunk (battery → strips): 14 AWG silicone, ≤ 25 cm. Everything on **one common ground star** (the GND strip); the USB adapters and the Pi share it through their cables only.
- INA219: replace the stock 0.1 Ω shunt by 2 × 20 mΩ in parallel (or a 10 mΩ shunt) for a 20 A range, set `config` accordingly; address 0x40.
- Kill switch is the only way the pack is isolated; the software e-stop only drops servo torque.
- Add the LiPo alarm buzzer to the pack balance lead; it screams below 3.3 V/cell. Never run below 3.5 V/cell under load (firmware cuts at 9.6 V).

## Signal
```
Pi 5 USB ─► Waveshare Bus Servo Adapter A ─► data hub A ─► L leg chain (IDs 1-6) and R leg chain (IDs 7-12)
Pi 5 USB ─► Waveshare Bus Servo Adapter B ─► data hub B ─► L arm (13-15), R arm (16-18), neck (19-20)
Pi 5 USB ─► camera (head)       Pi 5 USB ─► microphone (optional)
Pi 5 I2C-1 (GPIO2/3, 400 kHz) ─► BNO085 0x4A, ADS1115 0x48 (L foot), ADS1115 0x49 (R foot), INA219 0x40
```
- Servo bus is half-duplex TTL at 1 Mbps; keep data stubs < 40 cm. Each adapter is powered from its USB port only. Make the adapter-to-hub cable **2-wire (GND + DATA)** — cut/insulate its V+ lead — so the 12 V rail never back-feeds the adapter; servos get power only from the strips.
- Daisy-chain order and cable lengths per hop: [HARNESS](HARNESS.md). Cross-joint hops need a strain-relief loop; leave the 40 mm slack that table includes.
- Pins (servo cable): V+ / GND / DATA — check polarity on your cable before plugging the first servo, it is not keyed against the adapter.
- Data hub = a perfboard strip with 5 × 3-pin headers: one common DATA rail (all headers) and one connector per limb chain; power does **not** run through the hub.

## Foot sensors (per foot)
4 × FSR 402 in the sole pockets: heel-L, heel-R, toe-L, toe-R. Each FSR: one side to 3.3 V, other side to an ADS1115 input **and** a 10 kΩ resistor to GND (voltage divider). The ADS1115 for the left foot is at 0x48 (ADDR→GND), right foot 0x49 (ADDR→VDD). Route the 4-wire flex up the shank inside the spiral wrap; leave a loop at the ankle for ±50°.

## IMU
BNO085 breakout on the torso floor at ≈ (−20, 0, 395 mm) — its X arrow **forward**, Z up, 3 mm foam tape. Keep it away from the servo-bus cables and the buck converter.

## Connector summary
| Link | Connector |
|---|---|
| Battery | XT60 |
| Servo bus | 3-pin Feetech cable (match the connector family of the servos you receive) |
| Adapters | USB-C |
| Sensors | 4-pin 2.54 mm Dupont/JST-SH, strain-relieved with hot glue |
