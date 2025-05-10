# eth-din-dev-kit

## Implemented functions
- Connectors (T1 and T2) for connecting two [DS18B20 digital thermistors](https://www.laskakit.cz/en/dallas-ds18b20-orig--digitalni-vodotesne-cidlo-teploty-1m/) with automatic detection. Reading temperature every 20 seconds and sending to MQTT with topic /T1-Celsius
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
- detect USB-C plug and publish to MQTT with topic /USBdetect
- received RS485 data forward to MQTT with topic /RS485_RX

## Mechanical and electrical properties
- Mounting on DIN rail TS35 (35mm)
- 3D printed housing
- Module width 19.5mm
- Four LEDs
    - Fail internal fuse - red (top)
    - 100M ethernet - green
    - ACT ethernet - green
    - PWR - green (bottom)
- Single board PCB contains interfaces
    - Ethernet connector RJ45, with passive POE accept 9-13.8V DC, any polarity on pin 4+5 (one polarity) and 7+8 (second polarity)
    - DC power jack 5.5/2.5mm, accept 9-13.8V DC, center positive
    - USB-C connector contains
        - 5V power input for electronic
        - serial comunications with ESP32 cpu
        - 3.3V I2C bus - Gpio33 (SDA) on SBU1, Gpio32(SCL) on SBU2
    - RS485 bus on 39512-1002 connector - B signal on pin 1
    - Two 3.3V one wire bus on S3B-XH-A(LF)(SN) connectors - pin1 +3.3V, pin2 oneWire, pin3 GND
    - GPIO piheader socket 2x10, 2.54mm pitch
        1,2 - +12V (max 400mA)
        3,4,19,20 - GND
        6 - 3.3V (max 500mA)
        5 - Gpio5
        7 - Gpio33 (SDA/SBU1)
        8 - Gpio15
        9 - Gpio12
        10 - Gpio14
        11 - Gpio2
        12 - Gpio13
        13 - Gpio4
        14 - Gpio0
        15 - Gpi39
        16 - Gpio16
