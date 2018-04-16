For information on installing libraries, see: http://www.arduino.cc/en/Guide/Libraries


These are the libraries you need for compiling the Gateway:

- ArduinoJson (version 1.51)				-> ArduinoJson.h
- ESP8266 Oled library SH1106, (V 4.0.0 by Weinberg)	-> sh1106.h
- ESP8266 Oled library SSD1306 (V 4.0.0 by Weinberg)	-> ssd1306.h
- WifiManager (Version 0.12.0 by Tzapu)			-> WiFiManager.h


Some Libraries are not installed in the Arduino directory, but in the sketch directory:
- aes
- ESP8266-Oled_Driver_for_SSD1306_display
- gBase64
- Streaming
- Time
- WiFiEsp

Libraries are specified in the ESP-sc-gway.ino file as include files.