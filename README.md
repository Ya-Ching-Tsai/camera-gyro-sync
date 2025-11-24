### Overview
This repository provides a synchronized Camera + Gyroscope data acquisition pipeline on Raspberry Pi.
It captures video using Picamera2 and reads gyroscope data from an LSM6DS3 (I²C) sensor.
Both streams share a unified monotonic timestamp to ensure precise alignment.
The output includes an MP4 video and a CSV file containing gyroscope readings for downstream applications such as video stabilization or gyro–vision fusion.

### Features
* Picamera2 video recording
* LSM6DS3 gyroscope-only acquisition via I²C
* Shared monotonic-time timestamp alignment
* CSV logging of gyro data (°/s or rad/s)
* Lightweight Python implementation suitable for real-time capture
### Setup
#### Install dependencies
> sudo apt update
> 
> sudo apt install python3-picamera2 python3-smbus i2c-tools
#### Enable I²C:
> sudo raspi-config
### Run
Start synchronized camera + gyroscope capture:
> python3 sync_capture.py
Output files:
> output/video.mp4
> 
> output/gyro_data.csv


