Smart Room Monitoring System
Overview
This project is a smart room monitoring system built for the ESP32 microcontroller. It integrates sensors and actuators to monitor environmental conditions (temperature, humidity, light intensity) and control devices (LEDs, fan) via Firebase. The system uses an LCD to display sensor data and connects to the internet to send data to Firebase Realtime Database and Firestore.
Features

Sensor Monitoring: Uses DHT20 for temperature and humidity, and a light sensor for ambient light intensity.
Device Control: Controls a fan and NeoPixel LEDs based on Firebase data.
Display: Shows temperature and humidity on a 16x2 I2C LCD.
Firebase Integration: Sends sensor data to Firebase Realtime Database and Firestore, and reads control commands from Firebase.
WiFi Connectivity: Connects to the internet to sync time via NTP and communicate with Firebase.

Hardware Requirements

ESP32 Development Board
DHT20 Temperature and Humidity Sensor
NeoPixel LED Strip (4 LEDs)
16x2 I2C LCD Display
Light Sensor (connected to pin 33)
Fan (connected to pin 27)
Push Button (connected to pin 35)

Software Requirements

Arduino IDE with ESP32 board support
Libraries:
Adafruit_NeoPixel
DHT20 (by DFRobot)
LiquidCrystal_I2C
Firebase_ESP_Client
LittleFS (if using .env file)



Installation

Clone the Repository:git clone https://github.com/your-username/smart-room-monitoring.git


Install Libraries:
Open Arduino IDE.
Go to Sketch > Include Library > Manage Libraries.
Search for and install the required libraries listed above.


Configure Credentials:
Create a config.h file (or upload a .env file to the ESP32 filesystem if using LittleFS).
Add your WiFi and Firebase credentials as shown in the provided .env template or config.h file.


Upload the Code:
Open main.cpp in the Arduino IDE.
Upload the code to your ESP32 board.


Upload the .env File (if using LittleFS):
Use the ESP32 Sketch Data Upload tool to upload the .env file to the filesystem.



Usage

Power on the ESP32.
The system will connect to WiFi and Firebase, sync the time via NTP, and initialize sensors.
Sensor data (temperature, humidity, light intensity) will be sent to Firebase every 5 seconds.
The LCD will display the current temperature and humidity.
The fan and LEDs can be controlled via Firebase by updating the /control/hLti6ttX5cQ04pFMnFAG path:
button_for_fan: 0 (off), 1 (low), 2 (medium), 3 (high)
button_for_led: 0 (off), 1 (hot/orange), 2 (warm/yellow), 3 (cool/white), 4 (cold/dark blue)
candel_power_for_led: 0–100 (brightness percentage)



Project Structure

main.cpp: Main code for the ESP32.
config.h: Configuration file for WiFi and Firebase credentials (alternative to .env).
.env: Environment file for storing credentials (if using LittleFS).
DHT20.cpp/DHT20.h: DHT20 sensor library files.
README.md: This file.

License
This project is licensed under the MIT License. See the LICENSE file for details.
Contributing
Feel free to submit issues or pull requests on GitHub.
