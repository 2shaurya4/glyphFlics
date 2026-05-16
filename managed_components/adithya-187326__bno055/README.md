| Supported Targets | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-C5 | ESP32-C6 | ESP32-C61 | ESP32-H2 | ESP32-S3 |
| ----------------- | ----- | -------- | -------- | -------- | -------- | --------- | -------- | -------- |

# _BNO055 ESP-IDF Component_

This component provides a complete ESP-IDF driver for the Bosch BNO055 absolute orientation sensor, which combines accelerometer, gyroscope, and magnetometer data through sensor fusion in an onboard microcontroller.

The driver offers a clean, modular interface for initialization, configuration, data acquisition, calibration management, offset handling, and unit conversions — making it easy to integrate the BNO055 into any ESP-IDF project.

## Features

- Full I²C communication support (no batch writes required)
- Initialization and configuration with error handling
- Operation mode switching (CONFIG, NDOF, IMU, etc.)
- Unit system control (m/s², deg/s, radians, etc.)
- Offset management:
  - Read calibration offsets (`bno055_get_offsets()`)
  - Write saved offsets (`bno055_set_offsets()`)
- Sensor data acquisition:
  - Accelerometer
  - Magnetometer
  - Gyroscope
  - Euler Angles
  - Quaternion
  - Linear Acceleration
  - Gravity
  - Temperature
- Calibration monitoring with detailed status output
- Axis remapping and sign inversion
- Configurable via Kconfig
- Works across all ESP32 targets
- Fully compatible with ESP-IDF ≥ 5.3.2 (includes I²C clock-stretching fix)

## Component Structure

```bash
bno055/
├── include/
│   ├── bno055.h              # Public API
├── src/
│   ├── bno055.c              # Public interface implementation
│   ├── bno055_priv.c         # Internal logic and helpers
│   └── bno055_priv.h
├── Kconfig                   # Configuration options
├── CMakeLists.txt            # Component build definition
├── idf_component.yml         # Component manifest for ESP Registry
└── examples/
    ├── initialization/       # Simple initialization example
    ├── offsets/              # Working with sensor offsets
    ├── axis_remaps/          # Axis remapping example
    └── sensor_readings/      # Data readout example
```

## Configuration (Kconfig)

This example uses the following Kconfig options from the component:
| Setting | Description | Default |
| ----------------------------- | ------------------------- | ------- |
| `CONFIG_BNO055_I2C_FREQUENCY` | I²C frequency (kHz) | `400` |
| `CONFIG_BNO055_SDA_PIN` | SDA GPIO | `2` |
| `CONFIG_BNO055_SCL_PIN` | SCL GPIO | `3` |
| `CONFIG_BNO055_I2C_ADDR` | I²C address (0x28 / 0x29) | `0x28` |
| `CONFIG_BNO055_RESET_PIN` | Reset GPIO (-1 disables) | `-1` |

Modify these using `idf.py menuconfig → BNO055 Configuration`.

## Usage

Refer to the `/examples` directory for:

- initialization: Minimal working setup
- offsets: Working with offsets post calibration
- axis_remaps: Axis reconfiguration after calibration
- sensor_readings: Periodic calibrated sensor data acquisition

## Notes

- Requires ESP-IDF ≥ 5.3.2 for I²C clock stretching compatibility.
- Tested on ESP32, ESP32-C3, and ESP32-S3 targets.
- Logging can be controlled via esp_log_level_set() for BNO055_TAG.
- Offsets and calibration data can be saved and reloaded between sessions.
- Axis remapping requires complete calibration before use.

## License

Licensed under the **MIT License**. \
See the [LICENSE](LICENSE) file for full text.

## Maintainer

**Author:** Adithya Venkata Narayanan ([Adithya-187326](https://github.com/adithya-187326))
**Version:** 1.0.0
**Registry:** ESP Component Registry
