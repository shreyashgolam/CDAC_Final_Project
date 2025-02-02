### Data Accession using RTOS and CAN using STM32

## Project Overview
This project demonstrates real-time data acquisition and communication using FreeRTOS and CAN bus on the STM32F407G-DISC1 development board. The system collects sensor data, transmits it via CAN bus, and forwards it to a laptop for storage and visualization. The implementation is suitable for various industrial and environmental monitoring applications.

## System Architecture
The project is based on two STM32F407G boards communicating over CAN bus:
- Node 1 (CAN Transmitter): Collects sensor data and transmits it via CAN.
- Node 2 (CAN Receiver): Receives data and sends it via **UART** to a laptop for further processing.

## Key Components
- Microcontroller: STM32F407G-DISC1 (2 units)
- Sensors:
  - BME680 – Measures temperature, humidity, pressure, and air quality.
  - MQ135  – Detects air quality and hazardous gases.
- Actuator:
  - DC Motor – Used for automation based on sensor readings.
- Communication Interfaces:
  - CAN Bus – Data transfer between STM32 boards.
  - UART    – Serial communication with the laptop.
- Software Tools:
  - STM32CubeIDE – Development using HAL and FreeRTOS.
  - Minicom      – Serial terminal for data reception.
  - CSV & MySQL Database – Data logging and storage.
  - Grafana – Data visualization.
  - Python  – Data processing and graph generation.

## Workflow
1. The first STM32 board reads sensor values and transmits them via **CAN bus**.
2. The second STM32 board receives the data and forwards it to the laptop via **UART**.
3. The laptop logs the data in a **CSV file** and **MySQL database**.
4. The stored data is analyzed and visualized using **bar graphs** and **Grafana dashboards**.
5. The **DC motor** is controlled based on sensor values for automation purposes.

## Applications
- Industrial Data Monitoring – Real-time environment tracking.
- Automated Control Systems – Actuator-based automation.
- IoT-based Data Acquisition – Can be expanded with cloud-based connectivity.

## Future Enhancements
- Implement MQTT/LoRaWAN for remote data access.
- Integrate AI-based predictive analytics for trend detection.
- Develop a mobile/web dashboard for remote monitoring.
