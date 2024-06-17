# ESP32 Web Server with RCWL-9610 Sensor

This project demonstrates how to set up an ESP32 web server that communicates with an RCWL-9610 distance sensor. The server collects distance data and displays it on a web page.

## Features
- Sets up an ESP32 as a Wi-Fi access point
- Serves a web page displaying distance data from the RCWL-9610 sensor
- Uses FreeRTOS tasks to manage TCP server and HTTP server functionalities
- Integrates I2C communication with the RCWL-9610 sensor

## Hardware Requirements
- ESP32 development board
- RCWL-9610 distance sensor
- Connecting wires

## Software Requirements
- ESP-IDF (Espressif IoT Development Framework)

## Usage
1. **Connect to the ESP32's Wi-Fi network:**
   - SSID: `esp_wifi`
   - Password: `password`

2. **Access the web page:**
   Open a web browser and navigate to `http://192.168.4.1`.

3. **View distance data:**
   The web page will display the distance measured by the RCWL-9610 sensor.
