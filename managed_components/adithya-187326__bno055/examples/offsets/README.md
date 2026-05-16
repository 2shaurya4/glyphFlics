| Supported Targets | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-C5 | ESP32-C6 | ESP32-C61 | ESP32-H2 | ESP32-S3 |
| ----------------- | ----- | -------- | -------- | -------- | -------- | --------- | -------- | -------- |

# _BNO055 Sensor Readings with Offsets Example_

This example demonstrates how to read sensor data from the BNO055 sensor using the ESP-IDF BNO055 driver component.\
It also shows how to retrieve sensor offsets, which can be used to correct drift or calibration bias.\
It shows how to:

- Initialize the I²C master bus
- Initialize and configure the BNO055 in **NDOF mode**
- Calibrate the sensor
- Retrieve and log calibration offsets
- Continuously read data from the accelerometer (default)

## Available Sensor Types

You can replace `LINEAR_ACCELERATION` in the loop with any of the following constants defined in `bno055.h`:
| Sensor Constant | Description |
| --------------------- | ------------------------------------- |
| `ACCELEROMETER` | Raw accelerometer data |
| `MAGNETOMETER` | Raw magnetometer data |
| `GYROSCOPE` | Raw gyroscope data |
| `EULER_ANGLE` | Euler angles (Yaw, Pitch, Roll) |
| `QUATERNION` | Quaternion orientation data |
| `LINEAR_ACCELERATION` | Linear acceleration (gravity removed) |
| `GRAVITY` | Gravity vector |
| `TEMPERATURE` | On-chip temperature |

## Configuration

This example uses the following Kconfig options from the component:
| Setting | Description | Default |
| ----------------------------- | ------------------------- | ------- |
| `CONFIG_BNO055_I2C_FREQUENCY` | I²C frequency (kHz) | `400` |
| `CONFIG_BNO055_SDA_PIN` | SDA GPIO | `2` |
| `CONFIG_BNO055_SCL_PIN` | SCL GPIO | `3` |
| `CONFIG_BNO055_I2C_ADDR` | I²C address (0x28 / 0x29) | `0x28` |
| `CONFIG_BNO055_RESET_PIN` | Reset GPIO (-1 disables) | `-1` |

Modify these using `idf.py menuconfig → BNO055 Configuration`.

## Expected Behavior

- The example initializes the BNO055 with default configuration, retrieves calibration offsets, and then reconfigures it for NDOF mode with SI units.
- Sensor readings are fetched and logged every 100 ms.
- The BNO055_TAG log level is set to verbose, so all readings are automatically logged.
- You can disable verbose logs and access the latest data directly from the bno055 structure (e.g., bno055.raw_acceleration.x, etc.).

Example Output:

```bash
V (xxx) BNO055: Accel offset - X: 0.000, Y: 0.010, Z: 0.010
V (xxx) BNO055: Mag offset - X: 0.000, Y: 0.000, Z: 0.000
V (xxx) BNO055: Gyro offset - X: -0.125, Y: 0.188, Z: 0.000
```

**NOTE** \
This example only reads offsets using `bno055_get_offsets()`. \
Offsets can also be set manually using `bno055_set_offsets()` if you wish to use previously saved/set calibration data. \

## Wiring

| ESP Pin                 | BNO055 Pin | Notes                          |
| ----------------------- | ---------- | ------------------------------ |
| `CONFIG_BNO055_SDA_PIN` | SDA        | Internal Pullup, in this setup |
| `CONFIG_BNO055_SCL_PIN` | SCL        | Internal Pullup, in this setup |
| 3.3V                    | VIN        | Power                          |
| GND                     | GND        | Common ground                  |

## Build and Run

```bash
idf.py set-target <chip>
idf.py build flash monitor
```

## Notes

- Requires ESP-IDF ≥ 5.3.2 for proper I²C clock-stretching support.
- Retrieved offsets can be stored and written back to the sensor later using `bno055_set_offsets()`.
- Ensure the sensor is fully calibrated before fetching data; partial calibration produces incorrect data.
