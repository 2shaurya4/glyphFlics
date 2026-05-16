| Supported Targets | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-C5 | ESP32-C6 | ESP32-C61 | ESP32-H2 | ESP32-S3 |
| ----------------- | ----- | -------- | -------- | -------- | -------- | --------- | -------- | -------- |

# _BNO055 Sensor Readings Example_

This example demonstrates how to read sensor data from the BNO055 sensor using the ESP-IDF BNO055 driver component. It shows how to:

- Initialize the I²C master bus
- Configure and initialize the BNO055 in **NDOF mode**
- Calibrate the sensor
- Read sensor data using bno055_get_readings()
  The default sensor read in this example is GRAVITY, but you can modify it to read from any available sensor.

## Available Sensor Types

You can replace GRAVITY in the loop with any of the following constants defined in `bno055.h`:
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

- The example initializes the BNO055 sensor, configures it for NDOF mode, and continuously reads from the specified sensor (default: GRAVITY).
- The log level for BNO055_TAG is set to verbose, so all readings are logged automatically.
- You can disable verbose logs and access the latest sensor data directly from the bno055_t struct fields (e.g., bno055.gravity.x, bno055.gyroscope.z, etc.).
  Example Output:

```bash
V (xxxx) BNO055: Gravity - X: 0.012, Y: 0.004, Z: 0.998
```

The BNO055_TAG log level is set to verbose for sensor output — you can reduce it to INFO or WARN if needed.\
If you see I²C errors, check wiring (SDA/SCL pins, pull-ups, and address).

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
- The data stored inside the bno055 struct can be accessed programmatically for custom logging or filtering.
- Ensure the sensor is fully calibrated before fetching data; partial calibration produces incorrect data.
