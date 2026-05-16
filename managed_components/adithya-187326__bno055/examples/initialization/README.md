| Supported Targets | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-C5 | ESP32-C6 | ESP32-C61 | ESP32-H2 | ESP32-S3 |
| ----------------- | ----- | -------- | -------- | -------- | -------- | --------- | -------- | -------- |

# _BNO055 Basic Initialization Example_

This example demonstrates the minimal setup required to initialize a BNO055 sensor using the ESP-IDF BNO055 driver component.

It shows how to:

- Configure and initialize the I²C master bus
- Attach the BNO055 as an I²C device
- Initialize the sensor into configuration mode

## Configuration

This example uses the following Kconfig options from the component:
| Setting | Description | Default |
| ----------------------------- | ------------------------- | ------- |
| `CONFIG_BNO055_I2C_FREQUENCY` | I²C frequency (kHz) | `400` |
| `CONFIG_BNO055_SDA_PIN` | SDA GPIO | `2` |
| `CONFIG_BNO055_SCL_PIN` | SCL GPIO | `3` |
| `CONFIG_BNO055_I2C_ADDR` | I²C address (0x28 / 0x29) | `0x28` |
| `CONFIG_BNO055_RESET_PIN` | Reset GPIO (-1 disables) | `-1` |

Modify these using idf.py menuconfig → BNO055 Configuration.

## Expected Behavior

On boot, the ESP logs I²C setup and BNO055 initialization progress.
If the sensor is correctly connected, you should see:

```bash
I (xxx) MAIN: BNO055 initialized
```

For additional logging, change the log level of the `BNO055_TAG`, at the start of app_main.\
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

- Requires ESP-IDF ≥ 5.3.2 for proper I²C clock stretching support.
- This example only performs basic sensor initialization.
- For full usage (readings, calibration, unit selection, offsets, etc.), see other examples.
