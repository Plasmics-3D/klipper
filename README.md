# Klipper for Ino Trident

![](https://avatars.githubusercontent.com/u/139325225?s=200&v=4)

# Installation and Configuration Guide

This will guide you through the setup of the Ino Trident for Klipper. It will replace your current Klipper Version with ours. 


## Step 1: Install the Custom Klipper Version

1. Reboot your Raspberry Pi.
2. Execute the following commands to replace your existing Klipper with our version:

```bash
rm -rf ~/klipper
cd ~/ && git clone https://github.com/Plasmics-3D/klipper.git
```

3. Run the following command to identify the USB serial port:

```bash
ls /dev/serial/by-id/*
```

Make a note of the USB serial port that looks like this for later use:

> /dev/serial/by-id/usb-STMicroelectronics_INO_Virtual_ComPort_XXXXXXXXXXX-if00

## Step 2: Update Printer Configuration

Open your `printer.cfg` and make the following changes:

```python
[extruder] 
sensor_type: PLA_INO_SENSOR
heater_type: PLA_INO
control: pid
# Important first PID Values
pid_Kp: 13.41
pid_Ki: 30.91
pid_Kd: 1.46
min_temp: 10
max_temp: 450
serial: /dev/serial/by-id/usb-STMicroelectronics_INO_Virtual_ComPort_XXXXXXXXXXX-if00  # Use the serial name you copied earlier
PLA_INO_report_time: 0.1
```

## Step 3: PID Tune

1. Run the following gcode:

```bash
INO_PID_TUNE PID=250
```

2. Once completed, execute:

```bash
INO_READ_PID_VALUES
```

Replace the old PID values in the `printer.cfg` [extruder] section with the new ones.
