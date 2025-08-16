# LoRa Device Firmware (AT_Slave)

This repository contains the firmware for a LoRaWAN device, specifically designed as an AT command slave for the STM32L072Z microcontroller. It enables communication with an external host via UART using AT commands to control the embedded LoRa modem.

## Project Overview

This project implements a simple demo application that transforms the B-L072Z-LRWAN1 Discovery board (embedding the CMWX1ZZABZ-091 LoRa module with an STM32L0 MCU) into an AT command-controlled LoRa modem. It allows an external system (like a PC with a terminal) to send AT commands over UART to configure and operate the LoRa functionality, such as sending and receiving LoRaWAN messages.

The firmware utilizes the Cube Low Layer (LL) drivers for the STM32L0 microcontroller to optimize code size and performance.

## Key Features

*   **AT Command Interface:** Control the LoRa modem using a standard AT command set.
*   **UART Communication:** Interface with an external host via Universal Asynchronous Receiver-Transmitter (UART).
*   **LoRaWAN Stack Integration:** Manages LoRaWAN state machine and communication protocols.
*   **STM32L072Z Support:** Optimized for the STM32L0 microcontroller series, specifically the one integrated into the CMWX1ZZABZ-091 module.
*   **Temperature Sensing (Implied):** While the `AT_Slave` application focuses on modem control, the top-level `README.md` indicates this device is intended for "temperature sensing". This suggests that the AT commands might include functionality to read sensor data, or that this firmware serves as a base for such an application.

## Target Hardware

This application is specifically developed and tested for the following hardware:

*   **STMicroelectronics B-L072Z-LRWAN1 Discovery board (RevC)**
    *   Features the **CMWX1ZZABZ-091 LoRa module** (which integrates an STM32L0 microcontroller and a LoRa transceiver).

## Directory Structure

The project follows a typical embedded firmware structure, organized for different development environments and hardware abstraction layers:

```
lora_device/
├───.git/
├───Drivers/
│   ├───BSP/                  # Board Support Package for various ST boards and components
│   ├───CMSIS/                # ARM Cortex Microcontroller Software Interface Standard
│   ├───STM32L0xx_HAL_Driver/ # STMicroelectronics Hardware Abstraction Layer for STM32L0 series
│   ├───STM32L1xx_HAL_Driver/ # (Potentially for other projects/modules)
│   └───STM32L4xx_HAL_Driver/ # (Potentially for other projects/modules)
├───Middlewares/
│   └───Third_Party/
│       └───LoRaWAN/          # LoRaWAN stack implementation
└───Projects/
    └───B-L072Z-LRWAN1/
        └───Applications/
            └───LoRa/
                └───AT_Slave/   # Main application source code for the AT command slave
                    ├───Core/       # Core MCU initialization, interrupt handlers, and HAL MSP
                    ├───EWARM/      # IAR Embedded Workbench project files
                    ├───LoRaWAN/    # LoRaWAN application-specific files (AT commands, LoRa API)
                    ├───MDK-ARM/    # Keil MDK-ARM project files
                    ├───SW4STM32/   # System Workbench for STM32 project files
                    └───readme.txt  # Detailed description and usage instructions for AT_Slave
```

### Key Application Files (`Projects/B-L072Z-LRWAN1/Applications/LoRa/AT_Slave/`)

*   **`Core/inc/mlm32l0xx_it.h`**: Header for interrupt handlers.
*   **`Core/src/mlm32l0xx_hal_msp.c`**: STM32L0xx specific hardware HAL code.
*   **`Core/src/mlm32l0xx_hw.c`**: STM32L0xx specific hardware driver code.
*   **`Core/src/mlm32l0xx_it.c`**: STM32L0xx Interrupt handlers implementation.
*   **`LoRaWAN/App/inc/at.h`**: Header for AT commands API.
*   **`LoRaWAN/App/inc/command.h`**: Header for AT command definitions.
*   **`LoRaWAN/App/inc/lora.h`**: Header for LoRa API to drive the LoRa state machine.
*   **`LoRaWAN/App/inc/vcom.h`**: Header for virtual COM port interface.
*   **`LoRaWAN/App/src/at.c`**: AT commands API implementation.
*   **`LoRaWAN/App/src/command.c`**: Definitions of AT commands.
*   **`LoRaWAN/App/src/lora.c`**: LoRa API implementation.
*   **`LoRaWAN/App/src/main.c`**: Main program entry point.
*   **`LoRaWAN/App/src/vcom.c`**: Virtual COM port interface implementation.

## How to Use / Getting Started

To build, flash, and run this firmware on your B-L072Z-LRWAN1 Discovery board:

1.  **Toolchain Setup:**
    *   Open your preferred STM32 development toolchain (e.g., IAR Embedded Workbench, Keil MDK-ARM, System Workbench for STM32). The project files for these IDEs are located in the `AT_Slave` directory.
2.  **Build and Flash:**
    *   Rebuild all project files within your chosen IDE.
    *   Load the generated firmware image onto the target memory of your B-L072Z-LRWAN1 Discovery board using the ST-LINK debugger.
3.  **Hardware Connection:**
    *   Connect the B-L072Z-LRWAN1 Discovery board to your PC using a USB cable (Type A to micro-B) to the ST-LINK connector (CN7).
    *   Ensure that the ST-LINK connector CN8 jumpers are properly fitted.
4.  **Terminal Configuration:**
    *   Open a serial terminal application on your PC (e.g., PuTTY, Tera Term, RealTerm).
    *   Configure the serial port settings to match the firmware's UART configuration:
        *   **Baud Rate:** 9600
        *   **Data Bits:** 8
        *   **Stop Bits:** 1
        *   **Parity:** None
        *   **Flow Control:** None
        *   *(These settings are defined in `LoRaWAN/App/src/vcom.c`)*
    *   Set the terminal's new-line transmission to `CR+LF` (Carriage Return + Line Feed).
    *   Enable `Local echo` in your terminal settings to see the commands you type.
5.  **Send AT Commands:**
    *   Once connected and configured, you can send AT commands by typing them into the terminal. The device will respond accordingly.

## License

This software is provided under the terms of the STMicroelectronics copyright and redistribution license. Please refer to the `readme.txt` file within the `AT_Slave` directory for full license details.