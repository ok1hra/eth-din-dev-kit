# eth-din-dev-kit

## Implemented functions
- Connectors (T1 and T2) for connecting two DS18B20 digital thermistors with automatic detection. Reading temperature every 20 seconds and sending to MQTT with topic /T1-Celsius
- Ethernet connection using RJ45 connector with IP address retrieval from DHCP server. Static IP address setting is not implemented
- http web server publishing two pages
    - [mqtt-wall](https://github.com/bastlirna/mqtt-wall) , which is a web MQTT client, for displaying messages that the device has sent to the MQTT broker
    - SETUP page for configuring the device contains settings
        - MQTT topic
        - Device ID
        - RS485 (USB-C) BAUDRATE
        - MQTT broker IP
        - MQTT broker PORT
        - websocet MQTT PORT
        - MQTT Login
        - MQTT Password
- Saving device settings to internal eeprom
- OTA update - allows you to update the device firmware via Ethernet via the web interface. Available at [IP]:82/update, or from the link on the main page of mqtt-wall
- MQTT client for sending and receiving messages from the broker
- Measuring POE voltage and sending the value to MQTT when it changes
- Watchdog timer, triggers a device restart if the firmware stops for longer than 60s