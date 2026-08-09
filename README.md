# Vene2.0

Next evolution of the **VENE** project, with longer range **LoRa** communication and improved electronics design. Everything designed to be easily 3D printed.

### Performance
- Radio Range: ??km
- Length: 0.85m
- Weight w/o payload: ~3kg
- Payload capacity: ?? kg (130 x 250 x 60 mm, higher with custom lid)
- Max range: ??km (??min) (3s 6Ah LiPo)

### VGUI2
Custom control software for Vene2.0 with multiple map and telemetry options, all easily modifiable.

- Plug & Play operation
- Flexible controller input options
- Route & Scan pattern editing
- Live telemetry & control

### Required Hardware

#### Boat
- ESP32
- RFM95W
- uBlox NEO GPS module (NEO-6M used currently, but many others are plug-and-play compatible)
- BNO085
- LiPo (3s used here)
- Generic BLDC Motor and ESC
- LiPo → 5V Buck converter
- Propeller shaft (RC stuffing box)
- APA106 LEDs (2x) in 8mm package

#### Receiver
- ESP32
- RFM95W