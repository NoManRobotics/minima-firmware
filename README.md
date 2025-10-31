<div align="right">

[English](README.md) | [中文](README.zh-CN.md)

</div>

## MINIMA Firmware

This is the repository for releasing firmware code for robot arm MINIMA

## Includes

1. ESP32 firmware for ROS1
2. ESP32 firmware for ROS2
3. Arduino firmware for ROS1 (not recommended due to limited RAM size.)
4. Ramps fimware for Robot arms controlled by stepper motor (testing)

## Install (must read!)

**<u>First-time upload</u>** must use one of the following two ways:
1. Arduino IDE with the .ino file
2. esptool with bootloader, partition and the app

If you use esptool(either python or online web), you must upload to correct address:
- .ino.bootloader ----> address 0x1000
- .ino.partitions ----> address 0x8000
- .ino            ----> address 0x10000

and you need to press reset button after upload.

**<u>Updating firmware</u>** can be done through:

- noman-app remote firmare update
- Arduino IDE
- esptool

If you have already uploaded firmware once, you can use our app or esptool with only re-uploading the .ino file.

## Note

The ramps_6stepper.ino is experimental.

