# RC-FlightController

Fixed-wing flight controller firmware for the Adafruit Feather ESP32 V2, built with Arduino-on-ESP32 and scheduled with FreeRTOS.

This project combines:

- sensor tasks for IMU, barometer, and GPS
- PWM output for ESC and control surfaces
- multiple autopilot / assist modes
- waypoint navigation with mission progress tracking
- LoRa telemetry for a ground station

The current codebase is functional as a firmware skeleton with working sensor ingestion, navigation math, telemetry transmission, and mode scheduling. A few subsystems are still placeholders, especially the Spektrum RX backend, airspeed sensing, and failsafe logic.

## At A Glance

| Area | Status | Notes |
| --- | --- | --- |
| Board support | Implemented | `adafruit_feather_esp32_v2` via PlatformIO |
| IMU | Implemented | MPU6050 over I2C, complementary filter for roll/pitch |
| Barometer | Implemented | BMP3XX over I2C |
| GPS | Implemented | UART2 parser for GGA and RMC sentences |
| Waypoint navigation | Implemented | Haversine distance, bearing, leg and mission progress |
| Flight scheduler | Implemented | FreeRTOS tasks plus mode dispatcher |
| LoRa telemetry TX | Implemented | Raw binary telemetry packet sent periodically |
| LoRa telemetry RX | Low-level driver only | No command protocol yet |
| Manual / Stabilize / AltHold / Glide / Waypoint modes | Implemented | Runtime-selectable by RC mode PWM mapping |
| Spektrum receiver backend | Placeholder | Mode selection logic exists, but `rx_read()` is still stubbed |
| Airspeed sensor | Placeholder | Source file exists but not implemented |
| Failsafe | Placeholder | Scaffolding exists but logic is incomplete |

## Hardware Target

- Board: Adafruit Feather ESP32 V2
- Framework: Arduino
- Build system: PlatformIO
- Serial monitor speed: `115200`

PlatformIO environment:

```ini
[env:adafruit_feather_esp32_v2]
platform = espressif32
board = adafruit_feather_esp32_v2
framework = arduino
monitor_speed = 115200
```

## Wiring and Pin Map

The source of truth for project pin assignments is [`include/config.h`](include/config.h).

### Sensor and Comms Pins

| Subsystem | Interface | Pins | Notes |
| --- | --- | --- | --- |
| IMU (MPU6050) | I2C | `SDA=22`, `SCL=20` | Uses board-default `Wire.begin()` pins for Adafruit Feather ESP32 V2 |
| Barometer (BMP3XX) | I2C | `SDA=22`, `SCL=20` | Shares the same I2C bus as the IMU |
| GPS | UART2 | `TX=8`, `RX=7` | Configured for `115200` baud |
| LoRa radio | SPI | `SCK=5`, `MOSI=19`, `MISO=21` | External SX127x-style radio expected |
| LoRa control | GPIO | `CS=27`, `RST=32`, `IRQ=14` | IRQ pin is wired, RX callback mode is not yet used in firmware |

### PWM Output Pins

| Output | GPIO | PWM channel | Purpose |
| --- | --- | --- | --- |
| ESC | `0` | `0` | Throttle output |
| Aileron servo | `1` | `1` | Roll control |
| Elevator servo | `2` | `2` | Pitch control |
| Rudder servo | `3` | `3` | Yaw / rudder control |

### PWM Operating Mode

All actuator outputs are generated with ESP32 LEDC PWM:

- Frequency: `50 Hz`
- Resolution: `16-bit`
- Output range: `1000 us` to `2000 us`

The motor mixer uses these conventions:

- throttle command is treated internally as `0..1000`, then shifted to `1000..2000 us`
- aileron / elevator / rudder are centered at `1500 us`
- control outputs are added as offsets around center

## Firmware Architecture

The main runtime lives in [`src/main.cpp`](src/main.cpp). The default Arduino `loop()` is only used for optional debug printing. Actual flight work happens in FreeRTOS tasks.

### FreeRTOS Tasks

| Task | Period | Purpose |
| --- | --- | --- |
| `TaskIMURead` | `10 ms` | Reads MPU6050 and updates filtered roll/pitch |
| `TaskBarometerRead` | `50 ms` | Reads barometer pressure and altitude |
| `TaskGPSRead` | `50 ms` | Parses incoming GPS NMEA stream and updates navigation |
| `TaskFlightControl` | `100 ms` | Reads RC input, selects flight mode, runs active mode |
| `TaskTelemetryTx` | `500 ms` | Sends telemetry snapshot over LoRa |

### Main Data Flow

1. Sensors update shared state structures.
2. GPS fixes update the waypoint navigator.
3. The flight-control task reads the RC mode channel and selects a flight mode.
4. The active flight mode computes control outputs.
5. The mixer converts those commands into PWM outputs.
6. A telemetry task packages the current state and sends it over LoRa.

## Sensor Behavior

### IMU

Current IMU driver:

- Sensor: MPU6050
- Bus: I2C
- Accelerometer range: `8G`
- Gyro range: `500 deg/s`
- Filter bandwidth: `21 Hz`

Roll and pitch are computed with a complementary filter:

- short-term motion from gyro
- long-term gravity reference from accelerometer

Current yaw is not coming from a magnetometer or AHRS fusion stack. The filtered `imu_data.yaw` is currently updated from GPS course when a valid GPS lock and enough ground speed are available.

### Barometer

Current barometer driver:

- Sensor family: BMP3XX
- Bus: I2C
- Output altitude from `bmp.readAltitude(...)`

The sea-level reference pressure is currently hard-coded in [`src/hal/sensors/baro.cpp`](src/hal/sensors/baro.cpp), so altitude accuracy depends on adjusting that value to local conditions.

### GPS

Current GPS driver:

- Interface: UART2
- Baud: `115200`
- Supported NMEA sentences: `GGA`, `RMC`

What the firmware extracts:

- decimal latitude / longitude
- altitude
- fix quality
- satellite count
- local time with configurable UTC offset
- ground speed
- track heading

The driver prints readable debug output when `GPS_DEBUG_OUTPUT_ENABLED` is set in `config.h`.

## Flight Modes

The mode selector is defined in [`src/main.cpp`](src/main.cpp) and mapped from the RC flight-mode PWM channel.

### Flight-Mode PWM Mapping

| Flight-mode PWM | Selected mode |
| --- | --- |
| `<= 1200` | `Manual` |
| `1200..1400` | `Stabilize` |
| `1400..1600` | `AltHold` |
| `1600..1800` | `Glide` |
| `> 1800` | `Waypoint` |

If the PWM value is invalid, the firmware falls back to:

- `DEFAULT_FLIGHT_MODE = FlightMode::Waypoint`

### Mode Summary

| Mode | What it does |
| --- | --- |
| `Manual` | Maps RC roll/pitch stick commands directly to servo output scaling through the mixer |
| `Stabilize` | Uses roll and pitch PIDs to track RC-commanded attitude |
| `AltHold` | Holds current barometric altitude by generating a desired pitch command from the altitude PID |
| `Glide` | Cuts throttle and holds pitch near zero while preserving roll command authority |
| `Waypoint` | Uses GPS navigation to steer toward the current waypoint and uses altitude hold toward waypoint altitude |

## Waypoint Navigation

Waypoint configuration lives in [`include/config.h`](include/config.h):

- `missionwaypoints[]`
- `num_waypoints`
- `WAYPOINT_ACCEPTANCE_RADIUS_METERS`
- `WAYPOINT_HEADING_TO_ROLL_KP`
- `WAYPOINT_MIN_GROUND_SPEED_MPS`

Each waypoint is:

```cpp
struct waypoint {
    double lat;
    double lon;
    float alt;
};
```

### Navigation Logic

The navigation backend in [`src/nav/waypoint.cpp`](src/nav/waypoint.cpp):

- computes distance to the active waypoint with the haversine formula
- computes initial bearing to the active waypoint
- tracks per-leg progress
- tracks overall mission progress
- advances to the next waypoint when the aircraft enters the acceptance radius

### Waypoint Autopilot Behavior

In `Waypoint` mode:

- heading error becomes desired roll
- altitude error becomes desired pitch
- pilot throttle is passed through
- rudder is currently held neutral

This matches normal fixed-wing turning behavior: the aircraft turns primarily by banking, not by commanding pure yaw.

## LoRa Telemetry

The LoRa driver in [`src/hal/comms/lora.cpp`](src/hal/comms/lora.cpp) initializes the radio and exposes thread-safe send / receive calls guarded by a mutex.

Current radio configuration:

- Frequency: `915000000 Hz`
- Sync word: `0xF3`
- Spreading factor: `7`

### Telemetry Transport

Telemetry is sent every `500 ms` from the telemetry task.

The current telemetry implementation:

- sends a raw in-memory `telemetrydata` struct
- uses a single LoRa packet per snapshot
- does not add framing, versioning, or serialization beyond the packed C++ layout

That means the ground station must decode the exact same struct layout to interpret packets correctly.

### Telemetry Contents

The telemetry packet currently includes:

- attitude: roll, pitch, yaw
- GPS position, altitude, speed, heading, satellites, fix quality, lock state
- barometric altitude
- current flight mode
- waypoint target and mission status:
  - active waypoint index
  - total waypoint count
  - distance to waypoint
  - desired heading
  - target lat / lon / altitude
  - leg progress percent
  - mission progress percent
  - mission complete flag

## Receiver and RC Input

The firmware already has the following logic:

- RC roll / pitch / throttle conversion helpers
- flight-mode PWM threshold selection

However, the actual receiver backend is still placeholder code in [`src/hal/comms/rx_spektrum.cpp`](src/hal/comms/rx_spektrum.cpp):

- `rx_init()` is not implemented
- `rx_read()` is not implemented
- raw PWM values are currently hard-coded test values

So the control architecture is wired, but real RC input depends on finishing the Spektrum interface.

## Project Layout

```text
include/
  config.h              Project-wide pins, gains, timing, waypoints
  datatypes.h           Shared state and telemetry structs
  flight/               Flight-mode and mixer interfaces
  hal/                  Sensors, comms, and actuators
  math/                 PID and math helpers
  nav/                  Waypoint navigation interfaces

src/
  main.cpp              Task creation, scheduler, flight-mode dispatch
  flight/               Manual, stabilize, alt-hold, glide, waypoint
  hal/sensors/          IMU, barometer, GPS
  hal/comms/            LoRa and receiver interface
  hal/actuators/        PWM outputs
  nav/waypoint.cpp      Mission navigation math
```

## Build, Upload, and Monitor

Build:

```powershell
pio run
```

Upload:

```powershell
pio run -t upload
```

Open serial monitor:

```powershell
pio device monitor
```

## Important Configuration Knobs

Most tuning happens in [`include/config.h`](include/config.h).

Key groups:

- pin assignments
- task periods and stack sizes
- PID gains and limits
- default flight mode
- RC PWM thresholds for flight-mode switching
- waypoint mission list
- GPS timezone offset
- LoRa settings

## Current Limitations

This README tries to reflect the code as it exists now, not an idealized finished system.

Known limitations:

- Spektrum receiver backend is placeholder-only
- airspeed driver is not implemented yet
- failsafe logic is incomplete
- telemetry RX command protocol is not implemented yet
- telemetry uses raw binary struct layout instead of a stable serialized protocol
- yaw control is not actively used in waypoint steering
- no persistent configuration or parameter storage yet


## Quick Start Checklist

1. Wire the Feather ESP32 V2, MPU6050, BMP3XX, GPS, LoRa radio, ESC, and servos according to the tables above.
2. Update `missionwaypoints[]` in `include/config.h`.
3. Adjust sea-level pressure in `src/hal/sensors/baro.cpp` for your location.
4. Build with `pio run`.
5. Upload and monitor at `115200`.
6. Verify GPS lock, IMU startup, and LoRa telemetry before attempting closed-loop flight.

## [next steps/TODO]
- finish the Spektrum RX backend and validate live RC mode switching
- add yaw coordination or rudder mixing for cleaner turns
-  seperate imu_yaw and gps_heading
- change roll control
- sd card

