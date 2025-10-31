<div align="right">

[English](README.md) | [中文](README.zh-CN.md)

</div>

## MINIMA 固件

这是用于发布 MINIMA 机械臂固件代码的仓库。

## 包含内容

1. ESP32 ROS1 固件
2. ESP32 ROS2 固件
3. Arduino ROS1 固件（由于RAM大小有限，不推荐使用）
4. Ramps 步进电机控制机械臂固件（测试中）

## 安装说明（必读！）

**<u>首次上传</u>**必须使用以下两种方式之一：
1. 使用 Arduino IDE 打开 .ino 文件
2. 使用 esptool 上传 bootloader、partition 和 app

如果您使用 esptool（Python 版本或在线网页版），必须上传到正确的地址：
- .ino.bootloader ----> 地址 0x1000
- .ino.partitions ----> 地址 0x8000
- .ino            ----> 地址 0x10000

上传后需要按下复位按钮。

**<u>固件更新</u>**可以通过以下方式完成：

- noman-app 远程固件更新
- Arduino IDE
- esptool

如果您已经上传过一次固件，可以使用我们的 app 或 esptool 仅重新上传 .ino 文件。

## 注意事项

ramps_6stepper.ino 是实验性功能。



