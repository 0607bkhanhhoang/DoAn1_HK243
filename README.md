# STM32 + LoRa + ADC Sensor Transmission

# Bùi Khánh Hoàng - HCMUT
This project reads **4 analog sensors** on `ADC1` (pins **PA1 → PA4**) of an STM32 MCU and transmits the values via **LoRa**.  
An LED on `PC13` turns ON when transmission is successful.

---

## Hardware Connections

- **STM32 MCU** (tested with STM32F103, but works with other STM32 with ADC1).
- **Sensors** connected to analog pins:
  - PA1 → ADC1_IN1
  - PA2 → ADC1_IN2
  - PA3 → ADC1_IN3
  - PA4 → ADC1_IN4
- **LoRa module** (SX1278 or similar) via SPI.
- **LED** on PC13 (active HIGH in this project).

---

## Features

- Reads 4 ADC channels sequentially (polling mode).
- Formats readings into a string:  
# DoAn1_HK243