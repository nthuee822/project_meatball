# Raspberry Pi ICM-20948 raw six-axis reader

This repo was originally written for Arduino. The Raspberry Pi version in
`raspberry_pi/` uses Linux I2C directly and reads:

- raw accelerometer X/Y/Z counts
- raw gyroscope X/Y/Z counts
- converted acceleration in g
- converted angular rate in degrees per second
- roll and pitch tilt angles from gravity
- optional complementary-filter roll and pitch angles

The script does not load the Arduino DMP firmware. For raw six-axis data and
tilt, direct register reads are simpler and more reliable on Raspberry Pi.
Yaw requires magnetometer fusion or another heading reference, so this script
reports roll and pitch only.

## Hardware

Connect the ICM-20948 board to Raspberry Pi 5 I2C:

| ICM-20948 | Raspberry Pi 5 |
| --- | --- |
| VCC | 3.3 V |
| GND | GND |
| SDA | GPIO 2 / pin 3 |
| SCL | GPIO 3 / pin 5 |

Most boards use I2C address `0x68` or `0x69`; the script auto-detects both.

Enable I2C if needed:

```bash
sudo raspi-config
```

Then choose `Interface Options` -> `I2C`.

## Install

```bash
cd /home/meatball/IMU/Arduino_ICM20948_DMP_Full-Function
python3 -m pip install -r raspberry_pi/requirements.txt
```

Optional check:

```bash
i2cdetect -y 1
```

## Run

Readable text output:

```bash
python3 raspberry_pi/read_imu.py
```

Read 50 samples at 20 Hz:

```bash
python3 raspberry_pi/read_imu.py --rate 20 --samples 50
```

Read at 200 Hz but print only 20 lines per second:

```bash
python3 raspberry_pi/read_imu.py --rate 200 --output-rate 20
```

The default read mode uses byte-by-byte register reads because it is the most
compatible Raspberry Pi I2C path. If your wiring is stable and you want to test
faster modes, use `--read-mode block` or `--combined-read`:

```bash
python3 raspberry_pi/read_imu.py --rate 200 --output-rate 20 --read-mode block
python3 raspberry_pi/read_imu.py --rate 200 --output-rate 20 --combined-read
```

Read at 200 Hz and print one row every 10 samples:

```bash
python3 raspberry_pi/read_imu.py --rate 200 --output-every 10
```

Benchmark the read loop without terminal output:

```bash
python3 raspberry_pi/read_imu.py --rate 200 --format none
```

CSV output:

```bash
python3 raspberry_pi/read_imu.py --format csv --rate 100 --output-rate 25
```

JSON lines output:

```bash
python3 raspberry_pi/read_imu.py --format json
```

If your board is fixed at a known address:

```bash
python3 raspberry_pi/read_imu.py --address 0x68
```

The default ranges are `+/-4g` and `+/-2000 dps`. You can change them:

```bash
python3 raspberry_pi/read_imu.py --accel-range 8 --gyro-range 500
```

Keep the sensor still during the first second. By default the script uses 100
samples to estimate gyroscope bias for the complementary filter. Raw gyro values
are still printed unchanged.

Terminal printing is much slower than I2C reads. For higher IMU rates, use
`--output-rate`, `--output-every`, or `--format none`.

If you see `Errno 121 Remote I/O error`, start with the conservative mode:

```bash
python3 raspberry_pi/read_imu.py --rate 50 --output-rate 10 --read-mode byte --retries 10 --max-consecutive-errors 25
```

The reader skips occasional bad I2C samples and prints warnings to stderr. It
stops only after `--max-consecutive-errors` failures in a row. Set
`--max-consecutive-errors 0` to keep trying forever.

If that still fails, reduce the Raspberry Pi I2C baudrate in `/boot/firmware/config.txt`,
for example `dtparam=i2c_arm=on,i2c_arm_baudrate=50000`, then reboot.
