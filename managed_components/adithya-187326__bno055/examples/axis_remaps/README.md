| Supported Targets | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-C5 | ESP32-C6 | ESP32-C61 | ESP32-H2 | ESP32-S3 |
| ----------------- | ----- | -------- | -------- | -------- | -------- | --------- | -------- | -------- |

# _BNO055 Axis Remap Example_

This example demonstrates how to remap the sensor axes on a BNO055 using the ESP-IDF BNO055 driver component.\
It shows how to:

- Initialize and configure the BNO055 in **NDOF mode**
- Calibrate the sensor before performing axis remapping
- Apply a new axis configuration
- Continuously read sensor data after remapping

## Important Notes

- Calibration is mandatory before remapping.\
  If the sensor is not fully calibrated, the remapping process will not work correctly, and the output values will be inaccurate or unstable.
- Post-remap transient behavior: \
  After remapping axes, sensor readings may fluctuate significantly for about 1 second before stabilizing upon moving significantly (orientation changes, etc.). \
  This is normal behavior; avoid using readings immediately after significant motion.

## Available Sensor Types

You can replace `GRAVITY` in the loop with any of the following constants defined in `bno055.h`:
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

- The example initializes and configures the BNO055 in NDOF mode.
- The user is prompted to move the sensor until calibration completes.
- Once calibrated, offsets are read from the sensor.
- The axes are remapped based on the following configuration, in this example:

```bash
bno055_axes_t remapped_axes = {
    .x = POSITIVE_Z,
    .y = NEGATIVE_X,
    .z = NEGATIVE_Y,
};
```

- The system continuously reads `GRAVITY` data every 100 ms. \
  Example Output:

```bash
I (xxxx) BNO055: Calibrating the sensor, please move the sensor
I (xxxx) BNO055: Calibration done
V (xxxx) BNO055: Gravity - X: -0.011, Y: 0.004, Z: 0.998
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
- The BNO055_TAG log level is set to verbose to log calibration progress and sensor readings.
- Ensure the sensor is fully calibrated before performing remaps; partial calibration produces incorrect data.
- Post-remap, allow a short settling period (~1 second), when moving, before using data to avoid transient spikes.
- You can adjust the remap configuration in `bno055_axes_t` for your requirements.
