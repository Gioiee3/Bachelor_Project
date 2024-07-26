# Bachelor_Project
Anemometer uses STM32 with LoRa SX1278

Link code & hardware: https://github.com/Gioiee3/Bachelor_Project.git
Describe:
Node uses STM32 with LoRa Ra-02, AHT20, LM35, Anemometer sensor
Gateway uses ESP32 with LoRa Ra-02 and TFT LCD 2.8". 
Gateway receives data from node, display LCD and publish data to Thingsboard with MQTT

Config:
Node:   default cycle: 5 minutes, can be modified, can use adapter 12V for power supply, battery as backup source
Gateway: config ssid and password for wifi


