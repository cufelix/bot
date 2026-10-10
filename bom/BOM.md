# Bill of Materials

Prices are retail estimates (Oct 2026), USD, shipping/tax/contingency are explicit lines. Re-verify before ordering.

## Actuation  ($1,055.00)

| Ref | Item | Spec | Qty | Unit $ | Line $ | Notes |
|---|---|---|--:|--:|--:|---|
| A1 | Feetech STS3250 servo | 12V 50kg.cm stall (4.9 N.m) 1/345 magnetic encoder TTL bus dual-shaft | 8 | 75.00 | 600.00 | hip roll x2 + hip pitch x2 + knee x2 + ankle pitch x2. Order the 12V C001/C002 variant; price swings $60-97 |
| A2 | Feetech STS3215 servo (ST-3215-C018) | 12V 30kg.cm stall (2.9 N.m) 1/345 magnetic encoder TTL bus dual-shaft 55 g | 13 | 35.00 | 455.00 | 12 joints + 1 spare. Must be the 12V C018 version not 6V |

## Compute  ($266.50)

| Ref | Item | Spec | Qty | Unit $ | Line $ | Notes |
|---|---|---|--:|--:|--:|---|
| B1 | Raspberry Pi 5 4GB | quad A76 2.4GHz | 1 | 110.00 | 110.00 | runs control loop + vision. 2GB (~$80) also works if the price is still elevated (2026 RAM-shortage pricing: $60 -> $110) |
| B2 | Raspberry Pi 5 active cooler | heatsink + fan | 1 | 5.00 | 5.00 |  |
| B3 | microSD card 64GB A2 | UHS-I | 1 | 10.00 | 10.00 |  |
| B4 | USB camera module 1080p wide-angle | UVC board 32x32 mm with 28 mm mounting-hole square; lens FOV 90-120deg | 1 | 30.00 | 30.00 | lens fits the 20 mm head window; M2 self-tap posts in head; USB2 |
| B5 | USB mini microphone | UVC audio class | 1 | 8.00 | 8.00 | optional voice input |
| B6 | Waveshare Bus Servo Adapter (A) | USB-C to half-duplex TTL for Feetech STS | 2 | 10.00 | 20.00 | bus A legs (12 servos) bus B arms+neck (8 servos) |
| B7 | BNO085 9-DoF IMU breakout | I2C fused orientation | 1 | 27.00 | 27.00 | on torso floor near CoM |
| B8 | ADS1115 16-bit ADC breakout | 4ch I2C | 2 | 8.00 | 16.00 | addresses 0x48 and 0x49 for 8 foot sensors |
| B9 | FSR 402 force sensor (12.7 mm) | round | 8 | 5.00 | 40.00 | 4 per foot under TPU sole |
| B10 | Resistor 10k 1/4W | voltage divider for FSR | 10 | 0.05 | 0.50 |  |

## Power  ($163.20)

| Ref | Item | Spec | Qty | Unit $ | Line $ | Notes |
|---|---|---|--:|--:|--:|---|
| C1 | LiPo 3S 11.1V 3000mAh 50C XT60 | ~105x34x24 mm ~190 g | 2 | 28.00 | 56.00 | one in robot one charging |
| C2 | LiPo balance charger 6A (iMAX B6 class) | 3S balance charge + storage mode | 1 | 35.00 | 35.00 |  |
| C3 | LiPo safe charging bag | fire retardant | 1 | 8.00 | 8.00 |  |
| C4 | Pololu D24V50F5 buck regulator | 5V 5A | 1 | 22.00 | 22.00 | Pi 5 supply via GPIO 5V pins. Set PSU_MAX_CURRENT=5000 |
| C5 | INA219 current/voltage sensor | I2C 26V 3.2A shunt | 1 | 8.00 | 8.00 | replace shunt with 10 mOhm for 20A range (see docs) |
| C6 | Inline blade fuse holder + 20A ATC fuse | 14AWG | 2 | 3.00 | 6.00 | main fuse + spare. Walking peaks ~10 A (see docs/power.md); a fall can stall several servos |
| C7 | Latching kill switch 30A (XT60 inline) | panel mount | 1 | 9.00 | 9.00 | hard e-stop at the back |
| C8 | XT60 connector pair | male+female | 6 | 1.20 | 7.20 |  |
| C9 | Terminal barrier strip 12pos 25A | 5.08 mm pitch | 2 | 4.00 | 8.00 | star power distribution (V+ and GND) |
| C10 | LiPo low-voltage alarm buzzer 1-8S |  | 1 | 4.00 | 4.00 | connect to balance lead |

## Wiring  ($93.80)

| Ref | Item | Spec | Qty | Unit $ | Line $ | Notes |
|---|---|---|--:|--:|--:|---|
| D1 | Servo extension cables 3-pin | assorted 100-300 mm matching Feetech connector | 24 | 1.20 | 28.80 | verify connector family against servos before ordering |
| D2 | Silicone wire 14AWG red/black 2 m each | main trunk | 1 | 8.00 | 8.00 |  |
| D3 | Silicone wire 22AWG 5 colours 3 m each | limb power feeds + sensors | 1 | 15.00 | 15.00 |  |
| D4 | Heat shrink + ferrules + crimp terminal assortment |  | 1 | 18.00 | 18.00 |  |
| D5 | Spiral cable wrap + zip ties + Kapton tape |  | 1 | 10.00 | 10.00 |  |
| D6 | USB-C cable 15 cm | right-angle preferred | 2 | 4.00 | 8.00 | adapter to Pi |
| D7 | Dupont jumper wires + 2.54 mm headers |  | 1 | 6.00 | 6.00 |  |

## Print  ($62.00)

| Ref | Item | Spec | Qty | Unit $ | Line $ | Notes |
|---|---|---|--:|--:|--:|---|
| E1 | PETG-CF filament 1kg | structural links + pelvis + torso | 1 | 30.00 | 30.00 | ~700 g used |
| E2 | PETG filament 1kg | head + covers | 1 | 18.00 | 18.00 | ~150 g used |
| E3 | TPU 95A filament 0.5kg | foot soles | 1 | 14.00 | 14.00 |  |

## Hardware  ($63.00)

| Ref | Item | Spec | Qty | Unit $ | Line $ | Notes |
|---|---|---|--:|--:|--:|---|
| F1 | M3 socket cap screw assortment A2 (6-30 mm) ~600 pcs |  | 1 | 14.00 | 14.00 |  |
| F2 | M3 brass heat-set inserts 4.6mm | 200 pcs | 1 | 9.00 | 9.00 |  |
| F3 | M3 nylock nuts + washers |  | 1 | 8.00 | 8.00 |  |
| F4 | M2.5 brass standoff + screw kit | Pi 5 + sensor boards | 1 | 9.00 | 9.00 |  |
| F5 | 623ZZ bearing 3x10x4 | neck pitch rear support | 4 | 1.00 | 4.00 |  |
| F6 | Threadlocker medium (blue) + silicone grease |  | 1 | 10.00 | 10.00 |  |
| F7 | Velcro strap 20 mm 300 mm | battery retention | 2 | 2.00 | 4.00 |  |
| F8 | Double-sided foam tape + felt pads | board mounting | 1 | 5.00 | 5.00 |  |

## Contingency  ($240.00)

| Ref | Item | Spec | Qty | Unit $ | Line $ | Notes |
|---|---|---|--:|--:|--:|---|
| G1 | Shipping / duties / tax allowance | ~8% of parts | 1 | 130.00 | 130.00 | orders from 6-9 vendors |
| G2 | Reprint + breakage reserve | filament + screws + 1 spare cable set | 1 | 60.00 | 60.00 |  |
| H1 | Price-drift reserve | servo prices seen $26-49 (3215) and $63-97 (3250) | 1 | 50.00 | 50.00 |  |

## TOTAL: $1,943.50  (budget $2,000, margin $56.50)
