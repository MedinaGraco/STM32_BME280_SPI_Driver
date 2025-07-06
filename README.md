# STM32 BME280 SPI Driver Project

This project contains an STM32CubeIDE-based implementation of a driver for the BME280 environmental sensor using the SPI communication protocol.

## Features
- Sensor initialization
- Reading calibration data from the sensor
- Compensation and calculation of:
  - Temperature (in °C)
  - Pressure (in hPa)
  - Humidity (in %RH)
- Compatible with STM32 microcontrollers

## Technologies
- STM32CubeIDE
- HAL (Hardware Abstraction Layer) drivers
- SPI communication protocol

## Project Structure
```
Core/
|
├── Inc/
│   ├— main.h           # Main project header
│   └— BME280_SPI.h     # Header file for BME280 driver
|
├── Src/
│   ├— main.c           # Main project source file
│   └— BME280_SPI.c     # Source file for BME280 driver
|
├── Startup/             # Startup files (auto-generated)
|
├── .gitignore           # Git ignore rules
├── README.md            # Project description (this file)
```

## How to Use
1. Configure the SPI peripheral in STM32CubeMX.
2. Connect the BME280 sensor to your STM32 board via SPI.
3. Set the correct SPI instance and GPIO pins in the driver initialization.
4. Compile and flash the project to the STM32 board.
5. Use a debugger or UART to read measured temperature, pressure, and humidity.

