# Open-ended Remote Controlled Airsoft System (ORCAS)
# 开放式遥控气枪系统

![Title](./Title.jpg)  

This repository provides the firmware and software source code and hardware schematic for the ORCAS project. 

本仓库提供了 ORCAS 项目的固件、软件源代码和硬件原理图。

---

# Features demostration
# 功能演示

[![Demo](./Demo.jpg)](https://youtu.be/vNm99Zm-Rzw)

- The ORCAS project implements a wirelessly controlled airsoft turret that can be controlled via the web browser on a mobile phone.
- The turret equips a airsoft AEG modified from Jinming Gen8 gel blaster gearbox and a laser head for zeroing.
- The turret fires 7-8mm gel balls, and its barrel assembly contains an LED board as a tracer, but it fails to light up the glowing gel balls as expected.
- The turret equips a LED board to provide illumination of the forward aiming view.
- The turret equips a infrared laser ranging sensor to measure the distance to the aimed target.
- The turret equips an ultrasonic rangefinder that acts as a radar to measure the distance between it and surrounding objects.
- Two NEMA 17 stepper motors driven by A4988 modules are used to implement the pan/tilt rotation of the turret.  
- Two 15BY micro stepper motors driven by A4988 modules are used to implement gel balls feeding and the radar rotation.
- The above electronic components are integrated and controlled by the STM32F103 MCU, which implement driver commands through the UART interface for integration with the upper-level MCUs or computer.
- The turret equips a Raspberry Pi camera module to provide a first-person view to navigate the turning of the turret.
- The turret equips a Raspberry Pi Zero 2 W board to run a static web server that provides a user interface for connected clients to stream images captured by the camera to the client and transmit control requests from the client to the MCU.
- The turret is powered by a 12V power supply and an extra airsoft battery.

**中文说明：**
- ORCAS 项目实现了一个无线控制的气枪炮塔，可通过手机浏览器进行控制。
- 炮塔配备了基于锦明Gen8凝胶发射器改装的气枪AEG（自动电动枪）和用于归零的激光头。
- 炮塔发射 7-8mm 凝胶弹，其枪管组件包含一个作为曳光器的 LED 板，但无法按预期点亮发光凝胶弹。
- 炮塔配备了 LED 板，为前方瞄准视野提供照明。
- 炮塔配备了红外激光测距传感器，用于测量到目标的距离。
- 炮塔配备了超声波测距仪，作为雷达测量周围物体的距离。
- 使用两个由 A4988 模块驱动的 NEMA 17 步进电机来实现炮塔的水平/俯仰旋转。
- 使用两个由 A4988 模块驱动的 15BY 微型步进电机来实现凝胶弹供弹和雷达旋转。
- 上述电子组件由 STM32F103 MCU 集成控制，通过 UART 接口实现驱动命令，以便与上层 MCU 或计算机集成。
- 炮塔配备树莓派摄像头模块，提供第一人称视角以导航炮塔的转动。
- 炮塔配备树莓派 Zero 2 W 开发板，运行静态 Web 服务器，为连接的客户端提供用户界面，将摄像头捕获的图像流式传输到客户端，并将来自客户端的控制请求传输到 MCU。
- 炮塔由 12V 电源和额外的气枪电池供电。

---

# Project Structure
# 项目结构

```
ORCAS/
├── Firmware/                    # STM32 firmware source code
│   ├── bin/                    # Compiled firmware binaries
│   │   ├── ORCAS.bin          # Main firmware
│   │   ├── ORCAS_Fire_Ctrl_Test.bin        # Fire control test
│   │   ├── ORCAS_Orientation_Test.bin      # Orientation test
│   │   ├── ORCAS_Serial_Test.bin           # Serial communication test
│   │   └── ORCAS_TOF_Sensors_Test.bin      # TOF sensor test
│   ├── Core/                   # Core application code
│   │   ├── Inc/               # Header files
│   │   │   ├── fire_ctrl_utils.h           # Fire control utilities
│   │   │   ├── orientation_ctrl_utils.h    # Orientation control utilities
│   │   │   ├── tof_sensors_ctrl_utils.h    # TOF sensor control
│   │   │   ├── gy_tof10m.h                 # GY-TOF10M IR laser sensor driver
│   │   │   ├── gy_us42v2.h                 # GY-US42V2 ultrasonic sensor driver
│   │   │   ├── mma845x.h                   # MMA845x accelerometer driver
│   │   │   ├── uart_cmds_handle.h          # UART command handler
│   │   │   ├── error_code.h                # Error code definitions
│   │   │   └── main.h                      # Main program header
│   │   ├── Src/               # Source files (corresponding implementations)
│   │   └── Startup/           # STM32 startup files
│   ├── Drivers/               # Hardware abstraction layer drivers
│   │   ├── CMSIS/             # ARM Cortex Microcontroller Software Interface Standard
│   │   └── STM32F1xx_HAL_Driver/  # STM32 F1 HAL library
│   ├── ORCAS.ioc              # STM32CubeMX configuration file
│   ├── STM32F103R8TX_FLASH.ld # Linker script
│   └── .project               # STM32CubeIDE project file
│
├── Software/                   # Raspberry Pi software (Python)
│   ├── orcas_server.py        # Web server main program
│   ├── orcas_serial.py        # Serial communication with STM32
│   ├── orcas_camera.py        # Camera control module
│   ├── orcas_radar.py         # Radar functionality
│   ├── orcas_attributes.py    # System attributes management
│   ├── orcas_scenarios__calibrating.py   # Calibration scenario
│   ├── orcas_scenarios__interacting.py   # Interaction scenario
│   ├── orcas_scenarios__tracking.py      # Tracking scenario
│   ├── static/                # Static resources (UI icons and images)
│   └── templates/             # HTML templates
│       └── index.html         # Web control interface
│
├── Hardware/                   # Hardware schematics
│   └── Schematic_20250903.pdf # Circuit schematic
│
├── README.md                   # Project documentation
├── LICENSE                     # Open source license
├── Title.jpg                   # Project title image
├── Demo.jpg                    # Demo image
└── Tutorial.jpg                # Tutorial cover image
```

**中文说明：**

```
ORCAS/
├── Firmware/                    # STM32 固件源代码
│   ├── bin/                    # 编译好的固件二进制文件
│   │   ├── ORCAS.bin          # 主程序固件
│   │   ├── ORCAS_Fire_Ctrl_Test.bin        # 射击控制测试固件
│   │   ├── ORCAS_Orientation_Test.bin      # 方向控制测试固件
│   │   ├── ORCAS_Serial_Test.bin           # 串口通信测试固件
│   │   └── ORCAS_TOF_Sensors_Test.bin      # TOF传感器测试固件
│   ├── Core/                   # 核心应用代码
│   │   ├── Inc/               # 头文件
│   │   │   ├── fire_ctrl_utils.h           # 射击控制工具
│   │   │   ├── orientation_ctrl_utils.h    # 方向控制工具
│   │   │   ├── tof_sensors_ctrl_utils.h    # TOF传感器控制
│   │   │   ├── gy_tof10m.h                 # GY-TOF10M红外激光传感器驱动
│   │   │   ├── gy_us42v2.h                 # GY-US42V2超声波传感器驱动
│   │   │   ├── mma845x.h                   # MMA845x加速度计驱动
│   │   │   ├── uart_cmds_handle.h          # UART命令处理
│   │   │   ├── error_code.h                # 错误代码定义
│   │   │   └── main.h                      # 主程序头文件
│   │   ├── Src/               # 源文件（对应的实现代码）
│   │   └── Startup/           # STM32启动文件
│   ├── Drivers/               # 硬件抽象层驱动
│   │   ├── CMSIS/             # ARM Cortex微控制器软件接口标准
│   │   └── STM32F1xx_HAL_Driver/  # STM32 F1系列HAL库
│   ├── ORCAS.ioc              # STM32CubeMX配置文件
│   ├── STM32F103R8TX_FLASH.ld # 链接脚本
│   └── .project               # STM32CubeIDE项目文件
│
├── Software/                   # 树莓派软件（Python）
│   ├── orcas_server.py        # Web服务器主程序
│   ├── orcas_serial.py        # 与STM32的串口通信
│   ├── orcas_camera.py        # 摄像头控制模块
│   ├── orcas_radar.py         # 雷达功能
│   ├── orcas_attributes.py    # 系统属性管理
│   ├── orcas_scenarios__calibrating.py   # 校准场景
│   ├── orcas_scenarios__interacting.py   # 交互场景
│   ├── orcas_scenarios__tracking.py      # 追踪场景
│   ├── static/                # 静态资源（UI图标和图片）
│   └── templates/             # HTML模板
│       └── index.html         # Web控制界面
│
├── Hardware/                   # 硬件原理图
│   └── Schematic_20250903.pdf # 电路原理图
│
├── README.md                   # 项目文档
├── LICENSE                     # 开源许可证
├── Title.jpg                   # 项目标题图片
├── Demo.jpg                    # 演示图片
└── Tutorial.jpg                # 教程封面图片
```

**System Architecture / 系统架构：**

1. **STM32F103 MCU** controls all hardware components (stepper motors, sensors, laser, LEDs, etc.)
2. **Raspberry Pi Zero 2 W** runs a web server providing the user interface
3. Users access the web interface via mobile phone browser
4. The interface displays live camera feed and sends control commands
5. Raspberry Pi communicates with STM32 via UART serial port
6. STM32 executes specific hardware control actions

**中文说明：**

1. **STM32F103 微控制器**控制所有硬件组件（步进电机、传感器、激光、LED等）
2. **树莓派 Zero 2 W**运行Web服务器，提供用户界面
3. 用户通过手机浏览器访问Web界面
4. 界面显示实时摄像头画面并发送控制命令
5. 树莓派通过UART串口与STM32通信
6. STM32执行具体的硬件控制动作

---

# Hardware components
# 硬件组件

- The schematic of the turret is located in the Hardware/ folder of this repository.  
- All electronic units placed in the schematic are off-the-shelf modules that can be found and purchased on the web. You can find the part numbers of these modules in the [Chapter 01: Components and tools list](https://youtu.be/O3RVzEjgn4Q) of the tutorial.

**中文说明：**
- 炮塔的原理图位于本仓库的 Hardware/ 文件夹中。
- 原理图中的所有电子单元都是现成的模块，可以在网上找到并购买。你可以在教程的[第01章：组件和工具列表](https://youtu.be/O3RVzEjgn4Q)中找到这些模块的零件编号。

---

# Firmware binaries and the development environment
# 固件二进制文件和开发环境

- The compiled firmware binaries are located in the Firmware/bins/ folder of this repository. STM32CubeProgrammer needs to be installed to flash the firmware binary file into the MCU, and the flashing process will be described in the [Chapter 02: Firmware environment setup](https://youtu.be/3vJbjzyAfHk) of the tutorial.
- The firmware source code package is located in the Firmware/ folder of this repository. These codes are developed in STM32CubeIDE environment, which needs to be installed if you want to modify the codes to compile your custom firmware.

**中文说明：**
- 编译好的固件二进制文件位于本仓库的 Firmware/bins/ 文件夹中。需要安装 STM32CubeProgrammer 才能将固件二进制文件烧录到 MCU 中，烧录过程将在教程的[第02章：固件环境设置](https://youtu.be/3vJbjzyAfHk)中介绍。
- 固件源代码包位于本仓库的 Firmware/ 文件夹中。这些代码是在 STM32CubeIDE 环境中开发的，如果你想修改代码以编译自定义固件，需要安装该开发环境。

---

# Software and Raspberry Pi OS
# 软件和树莓派操作系统

- The software source code package runs on the Raspberry Pi is located in the Software/ folder of this repository. How to use this software will be explained in the [Chapter 11: Use cases summary](https://youtu.be/CBzjGMsVI3Y) of the tutorial.
- The tutorial will only give an overview of the installation and configuration of the Raspberry Pi OS to run the software. You will need to refer to the official Raspberry Pi documentation for detailed instructions.

**中文说明：**
- 运行在树莓派上的软件源代码包位于本仓库的 Software/ 文件夹中。如何使用此软件将在教程的[第11章：用例总结](https://youtu.be/CBzjGMsVI3Y)中说明。
- 教程将仅提供安装和配置树莓派操作系统以运行软件的概述。你需要参考树莓派官方文档以获取详细说明。

---

# Mechanical components
# 机械组件

- All the STL files of the 3D printed parts are NOT shared or open-source, and these files will be sold on Cults3D for private use only.
- Besides the 3D printed parts, there are also some off-the-shelf mechanical components used in this project and you can refer to the part names listed in the [Chapter 01: Components and tools list](https://youtu.be/O3RVzEjgn4Q) of the tutorial to search and purchase these parts on the web.

**中文说明：**
- 所有3D打印部件的STL文件不共享或开源，这些文件将在 Cults3D 上出售，仅供私人使用。
- 除了3D打印部件外，本项目还使用了一些现成的机械组件，你可以参考教程[第01章：组件和工具列表](https://youtu.be/O3RVzEjgn4Q)中列出的零件名称在网上搜索并购买这些部件。

---

# Tutorial
# 教程

- Please refer to the link of the YouTube playlist of the complete assembly guide for this project.

**中文说明：**
- 请参考本项目完整组装指南的 YouTube 播放列表链接。

[![Tutorial](./Tutorial.jpg)](https://youtube.com/playlist?list=PLm8tr3bbToy1aQuJKxj25tuSXXepTH-fG&si=vmJaBD7KAxfM6Iig)
