# ESP32 Web Server with RCWL-9610 Sensor

This project demonstrates how to set up an ESP32 web server that communicates with an RCWL-9610 distance sensor. The server collects distance data and displays it on a web page.

## Table of Contents
- [Features](#features)
- [Hardware Requirements](#hardware-requirements)
- [Software Requirements](#software-requirements)
- [Installation](#installation)
- [Usage](#usage)
- [Code Overview](#code-overview)
- [License](#license)

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

## Installation
1. **Clone the repository:**
    ```sh
    git clone <repository-url>
    cd <repository-directory>
    ```

2. **Set up ESP-IDF:**
    Follow the [official ESP-IDF installation guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp-idf_get-started/index.html) to set up your development environment.

3. **Configure the project:**
    ```sh
    idf.py menuconfig
    ```
    Set your Wi-Fi SSID and password under `Example Configuration`.

4. **Build the project:**
    ```sh
    idf.py build
    ```

5. **Flash the firmware:**
    ```sh
    idf.py -p /dev/ttyUSB0 flash
    ```
    Replace `/dev/ttyUSB0` with your ESP32's serial port.

6. **Monitor the output:**
    ```sh
    idf.py -p /dev/ttyUSB0 monitor
    ```

## Usage
1. **Connect to the ESP32's Wi-Fi network:**
   - SSID: `esp_wifi`
   - Password: `password`

2. **Access the web page:**
   Open a web browser and navigate to `http://192.168.4.1`.

3. **View distance data:**
   The web page will display the distance measured by the RCWL-9610 sensor.

## Code Overview

### Main Components
1. **Wi-Fi Initialization:**
    - Sets up the ESP32 as a Wi-Fi access point.
    - Manages client connections and disconnections.

2. **TCP Server Task:**
    - Listens for incoming TCP connections.
    - Receives data and retransmits it.
    - Updates the distance data string.

3. **HTTP Server:**
    - Serves a simple HTML page displaying the distance data.
    - Updates the distance data via an HTTP GET request.

4. **RCWL-9610 Sensor Handling:**
    - Initializes I2C communication.
    - Reads distance data from the sensor.
