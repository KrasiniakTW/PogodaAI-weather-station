# PogodaAI – DIY Solar-Powered Weather & Air Quality Station + AI Forecasting

**Author:** Kamil Gruszczyński  
**Team:** Krasiniak TechWorks  
**Competition:** Technikon ESA 2025  

---

## Project Overview
**PogodaAI** is a fully autonomous, low-cost weather and air quality monitoring station designed for educational, research, and environmental applications.  
It combines **modular hardware** based on the **ESP32 microcontroller** with **solar power management**, a suite of **environmental sensors**, and a **custom AI model** predicting air quality trends.

The entire system is **open-source**, **DIY-friendly**, and designed for easy replication and modification.

!!!Disclaimer!!!
The AI model and training scripts are available in a separate repository:
[PogodaAI-backend](https://github.com/krzkr80/pogodaAI-backend) made by Krzysztof Kurbanow

This repository focuses on the hardware development — including 3D design, electronics, and assembly instructions for the autonomous weather station.

---

## Main Features
- **Energy autonomy:** operates 100% off-grid using a solar panel + Li-Po battery.  
- **Modular architecture:** each sensor and component can be replaced or upgraded independently.  
- **Multi-sensor integration:** PM1.0, PM2.5, PM10 (PMSA003), temperature, humidity, pressure, VOC (BME680), light, rainfall, wind speed, and wind direction.   
- **3D-printed housing:** durable PET-G design, fully documented for easy printing and assembly.  
- **Wireless connectivity:** ESP32 with WiFi transmission and SD card logging.  
- **Open source:** all hardware, firmware, and CAD files are publicly available.

---

## Hardware Overview

| Component | Description |
|------------|--------------|
| **MCU** | ESP32 (WiFi + low power modes) |
| **Air quality** | PMSA003 (PM1.0, PM2.5, PM10) |
| **Environment** | BME680 (Temp, Humidity, Pressure, VOC) |
| **Light** | Analog photodiode |
| **Rain** | YL-83 sensor |
| **Wind** | Custom 3D-printed anemometer + wind vane |
| **Power** | DFRobot Solar Power Manager 5V + Li-Po 3.7 V |
| **Display** | LCD 4×20 + keypad |
| **Storage** | SD card module |
| **Housing** | PET-G 3D printed, modular & weatherproof |

Full electronics schematic available in `hardware/circuit_image.png`  
Assembly and printing instructions included in `/hardware/`.

---

## Custom Mechanical Components

The **anemometer** and **wind vane** were designed specifically for this project:  
- Carbon tubes (10 mm Ø, ~250 mm long)  
- 6200 bearings (2–4 pcs)  
- PET-G 3D printed parts (~100 g)  
- Gap sensor (for rotation counting)  
- Magnet + magnetometer (for direction detection)  

🧭 Detailed explanation: [`PogodaAI_anemometer_and_wind_vane_explained.pdf`](hardware/PogodaAI_anemometer_and_wind_vane_explained.pdf)

---

## Assembly Instructions
Step-by-step build guides are included:
- [`PogodaAI_weather_station_assembly_guide.pdf`](hardware/PogodaAI_weather_station_assembly_guide.pdf)  
- [`PogodaAI_weather_station_electronics_guide.pdf`](hardware/PogodaAI_weather_station_electronics_guide.pdf)  
- [`PogodaAI_printing_guide.pdf`](hardware/PogodaAI_printing_guide.pdf)

---

## Environmental Impact
- Promotes local awareness of air quality
- Enables early warnings for smog and pollution events
- Supports schools and communities with open monitoring tools
- 100% renewable power supply, zero operating cost
- Designed from recyclable PET-G materials

---

## Contributing
We welcome community contributions!
You can:
- Submit improvements to the AI model
- Modify hardware design for local conditions
- Translate documentation into other languages
- Please open a Pull Request or Issue on GitHub.

---

## License
This project is licensed under the MIT License – you are free to use, modify, and distribute it with attribution.

---

## Gallery
<img width="3024" height="4032" alt="PogodaAI Autonomiczna Stacja pogodowa" src="https://github.com/user-attachments/assets/66a52919-7820-44f5-a9b1-b2d281e7e359" />
<img width="3000" height="2002" alt="circuit_image" src="https://github.com/user-attachments/assets/e79e5be7-90fd-40ba-95d7-14b57e01fbed" />

---

## Contact
Krasiniak TechWorks
- techworks@krasiniak.pl
- GitHub: github.com/KrasiniakTW i github.com/krzkr80
