# Digital-Twin
#Tech mahindra Digital twin first phase 
Robot Firmware – ESP32 Odometry + WebSocket Control
This document provides an overview of the software requirements and steps to run an ESP32-based robot firmware that uses WebSocket communication and odometry calculation.
1. Software Requirements
Arduino Libraries
Ensure the following libraries are installed in your Arduino IDE:
- WiFi – included with the ESP32 board package
- WiFiClientSecure – included with the ESP32 board package
- WebSocketsClient – install via Library Manager or from GitHub: https://github.com/Links2004/arduinoWebSockets
- ArduinoJson – install via Library Manager, version 6+ recommended: https://arduinojson.org/
- Adafruit Sensor – install via Library Manager
- Adafruit HMC5883 Unified – install via Library Manager or GitHub
Board Setup
Install the ESP32 board package in the Arduino IDE:
1. Open File > Preferences
2. Add this to Additional Board Manager URLs:
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
3. Go to Tools > Board > Board Manager
4. Search for and install ESP32 by Espressif Systems

Then select your board type (e.g., ESP32 Dev Module) and correct COM port.
2. Wi-Fi Configuration
Update your Wi-Fi credentials in the code:
const char* ssid     = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";
Replace these with your actual network name and password.
3. WebSocket Server
This firmware connects to the WebSocket server hosted at:

wss://robot-to-3d-server.onrender.com/ws

It listens for motion commands from the server and sends real-time position and heading data in response.
4. Running the Firmware
Follow these steps to compile and run the firmware:
1. Open the project in the Arduino IDE
2. Connect your ESP32 board to the PC via USB
3. Select the correct ESP32 board and COM port
4. Ensure all required libraries are installed
5. Update Wi-Fi credentials and any relevant configuration
6. Click Upload to flash the firmware
7. Open the Serial Monitor (baud rate: 115200)
8. Monitor logs for Wi-Fi and WebSocket connection, and view real-time odometry data
5. WebSocket Message Formats
Incoming Command Format (from server):
{
  "event": "command",
  "payload": {
    "forward": true
  }
}
Supported payload keys:
- forward
- reverse
- left
- right
- stop
Outgoing Position Update Format (from ESP32):
{
  "event": "update",
  "payload": {
    "heading": 45.5,
    "X": 12.34,
    "Y": -7.89
  }
}
