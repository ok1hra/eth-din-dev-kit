/*

Ethernet DIN rail development kit
----------------------------
Compile for HARDWARE ESP32-POE

 ___               _        ___ _____ _  _
| _ \___ _ __  ___| |_ ___ / _ \_   _| || |  __ ___ _ __
|   / -_) '  \/ _ \  _/ -_) (_) || | | __ |_/ _/ _ \ '  \
|_|_\___|_|_|_\___/\__\___|\__\_\|_| |_||_(_)__\___/_|_|_|


This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

Used MQTT-WALL, credit Adam Hořčica, is under MIT license,
see https://github.com/bastlirna/mqtt-wall/blob/master/license.txt

MQTT monitor
mosquitto_sub -v -h 192.168.1.200 -t 'OK1HRA/#'
mosquitto_sub -v -h 54.38.157.134 -t 'OK1HRA/1/#'

MQTT topic
mosquitto_pub -h 192.168.1.200 -t OK1HRA/0/Target -m '10'
mosquitto_pub -h 54.38.157.134 -t BD:2F/0/Target -m '10'
mosquitto_pub -h 54.38.157.134 -t 3D:D3/0/RxAzimuth -m '10'

TODO
- rs485/mqtt proxy
- gpio to mqtt report
- mqtt to gpio control
- ds18b20 resolution to setup
- temperature Farnhait switch

Changelog:
- detect and read T1 and T2 termistor and public to MQTT every 20s
- websocket IP port for mqtt-wall add to setup page
- Analog read Gpi39 voltage to mqtt
- detect USB-C plug and publish to MQTT with topic /USBdetect
- received RS485 data forward to MQTT with topic /RS485_RX
- Configuration web setup support
- Configuration 02 - capacity measure
- prn() use only if usb connected
- watchog reset test mqtt temp delivery

IDE 1.8.19
Using library OneWire at version 2.3.8 in folder: /home/dan/Arduino/libraries/OneWire 
Using library DallasTemperature at version 3.9.0 in folder: /home/dan/Arduino/libraries/DallasTemperature 
Using library WiFi at version 2.0.0 in folder: /home/dan/Arduino/hardware/espressif/esp32/libraries/WiFi 
Using library EEPROM at version 2.0.0 in folder: /home/dan/Arduino/hardware/espressif/esp32/libraries/EEPROM 
Using library WebServer at version 2.0.0 in folder: /home/dan/Arduino/hardware/espressif/esp32/libraries/WebServer 
Using library Ethernet at version 2.0.0 in folder: /home/dan/Arduino/hardware/espressif/esp32/libraries/Ethernet 
Using library ESPmDNS at version 2.0.0 in folder: /home/dan/Arduino/hardware/espressif/esp32/libraries/ESPmDNS 
Using library ArduinoOTA at version 2.0.0 in folder: /home/dan/Arduino/hardware/espressif/esp32/libraries/ArduinoOTA 
Using library Update at version 2.0.0 in folder: /home/dan/Arduino/hardware/espressif/esp32/libraries/Update 
Using library AsyncTCP at version 1.1.4 in folder: /home/dan/Arduino/libraries/AsyncTCP 
Using library ESPAsyncWebServer at version 1.2.3 in folder: /home/dan/Arduino/libraries/ESPAsyncWebServer 
Using library FS at version 2.0.0 in folder: /home/dan/Arduino/hardware/espressif/esp32/libraries/FS 
Using library AsyncElegantOTA at version 2.2.8 in folder: /home/dan/Arduino/libraries/AsyncElegantOTA 
Using library PubSubClient at version 2.8 in folder: /home/dan/Arduino/libraries/PubSubClient 
Using library Wire at version 2.0.0 in folder: /home/dan/Arduino/hardware/espressif/esp32/libraries/Wire 
*/
//-------------------------------------------------------------------------------------------------------
const char* REV = "20260623";

// USED
const int HWidPin          = 34;  // analog
const int VoltagePoePin    = 35;  // analog
const int TermistorT1Pin   =  5;  // one wire
const int TermistorT2Pin   = 15;  // one wire
const int RS485ReDePin     = 16;  // out
const int USBdetectPin     = 36;  // digital in

// FREE GPIO
// const int Gpi39Pin         = 39;  // analog in
const int Gpio33Pin        = 33;  // in/out (SDA/SBU1)
const int Gpio32Pin        = 32;  // in/out (SCL/SBU2)
const int Gpio14Pin        = 14;  // in/out
const int Gpio13Pin        = 13;  // in/out
const int Gpio12Pin        = 12;  // in/out - upper calibration level sensor
const int Gpio4Pin         =  4;  // in/out - lower calibration level sensor
// const int Gpio2Pin         =  2;  // in/out (RTS)
const int Gpio0Pin         =  0;  // in/out (RTS)

// Capacity measurement
/*
        +3V3
          |
         R_10M
          |
  C?------------R_1k------->Gpi39Pin
          |
          |
          \
           |----R_100-------<Gpio2Pin
          /
          |
        GND
*/

// #include "driver/adc.h"
// #include "driver/i2s.h"

// #define I2S_NUM           I2S_NUM_0
// #define SAMPLE_RATE       200000         // 200 ksps – max pro I2S built-in ADC continuous mode
// #define BUF_SAMPLES       1024
// #define BUF_COUNT         2

const int Gpi39Pin = 39;        // GPIO ADC1_CHANNEL_3
const int Gpio2Pin =  2;        // discharge
const unsigned long measureInterval = 5000;      // interval spouštění v ms
const int ignoreThreshold = 100;                 // práh pro ignorování startu
const int measureThreshold = 3000;               // (max 4095) práh nabití kondenzátoru | max presnost pri 63%=2580
const unsigned long measureTimeoutUs = 1000000;  // timeout měření v µs (1 s)

// Variables
bool USBdetect = false;
short HardwareRev = 99;
String YOUR_CALL = "";
String NET_ID = "";
int HWidValue              = 0;
float T1Celsius = 0;
float T2Celsius = 0;
String MACString;
char MACchar[18];
float VoltagePOE      = 0.0;
float VoltageGpi39      = 0.0;
long WdtTimer=0;
int BaudRate = 115200; // serial debug baudrate
int Configuration = 0; // select Configuration type
const int Configuration1pins[] = {13, 14, 12, 4, 2, 0};
int EnableSerialDebug     = 0;
#define HTTP_SERVER_PORT  80     // Web server port
unsigned int OutputWatchdog;
unsigned long WatchdogTimer=0;
bool DHCP_ENABLE = 1;
char linebuf[80];
int charcount=0;
//Are we currently connected?
boolean connected = false;
String HTTP_req;
long lastMqttReconnectAttempt = 0;
boolean MQTT_ENABLE     = 1;          // enable public to MQTT broker
int MQTT_PORT;       // MQTT broker PORT
int WS_MQTT_PORT;       // websocket MQTT broker PORT - only for connection mqtt-wall web client
boolean MQTT_LOGIN      = 0;          // enable MQTT broker login
String MQTT_USER= "";    // MQTT broker user login
String MQTT_PASS= "";   // MQTT broker password
const int MqttBuferSize = 1000; // 1000
char mqttTX[MqttBuferSize];
char mqttPath[MqttBuferSize];
long MqttStatusTimer[2]{1500,1000};
long HeartBeatTimer[2]={0,1000};
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 0;
const int   daylightOffset_sec = 0;
static bool eth_connected = false;
#define DS18B20                     // external 1wire Temperature sensor
#define OTAWEB                      // enable upload firmware via web
#define ETHERNET                    // Enable ESP32 ethernet (DHCP IPv4)
#define ETH_ADDR 0
#define ETH_TYPE ETH_PHY_LAN8720
#define ETH_POWER 0 //12                // #define ETH_PHY_POWER 0 ./Arduino/hardware/espressif/esp32/variants/esp32-poe/pins_arduino.h
#define ETH_MDC 23                  // MDC pin17
#define ETH_MDIO 18                 // MDIO pin16
#define ETH_CLK ETH_CLOCK_GPIO17_OUT    // CLKIN pin5 | settings for ESP32 GATEWAY rev f-g
#define MQTT               // Enable MQTT debug
// #define ETH_CLK ETH_CLOCK_GPIO0_OUT    // settings for ESP32 GATEWAY rev c and older
// ETH.begin(ETH_ADDR, ETH_POWER, ETH_MDC, ETH_MDIO, ETH_TYPE, ETH_CLK);
#define MAX_SRV_CLIENTS 1
#define WDT_TIMEOUT 73
#define EEPROM_SIZE 270   /*
  0|Byte    1|128
  1|Char    1|A
  2|UChar   1|255
  3|Short   2|-32768
  5|UShort  2|65535
  7|Int     4|-2147483648
  11|Uint    4|4294967295
  15|Long    4|-2147483648
  19|Ulong   4|4294967295
  23|Long64  8|0x00FFFF8000FF4180
  31|Ulong64 8|0x00FFFF8000FF4180
  39|Float   4|1234.1234
  43|Double  8|123456789.12345679
  51|Bool    1|1

  0-1   - NET_ID
  141-160 - YOUR_CALL
  161-164 - MQTT broker IP
  165-166 - MQTT_PORT
  168 - MQTT_LOGIN
  169-170 - WS_MQTT_PORT
  226-227 BaudRate
  236-245 - MQTT_USER
  246-265 - MQTT_PASS
  266 - Configuration
  267-268 - TRX_PORT (TrxNet UDP port, uint16)

  !! Increment EEPROM_SIZE #define !! */

#include "esp_adc_cal.h"
#if defined(DS18B20)
  bool ExtTemp = true;
  #include <OneWire.h>
  #include <DallasTemperature.h>
  #define TEMPERATURE_PRECISION 10 // 9: ±0,5°C | 10: ±0,25°C | 11: ±0,125°C
  OneWire oneWire1(TermistorT1Pin);
  OneWire oneWire2(TermistorT2Pin);
  DallasTemperature sensors1(&oneWire1);
  DallasTemperature sensors2(&oneWire2);
  DeviceAddress T1, T2;
  void printAddress(DeviceAddress deviceAddress);   // forward declaration (Arduino auto-prototype fails here)
#endif
#include "esp_attr.h"
#include <esp_task_wdt.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "EEPROM.h"
#include <WebServer.h>
WebServer ajaxserver(HTTP_SERVER_PORT+8);
WiFiServer server(HTTP_SERVER_PORT);
volatile bool shouldRestart = false;
unsigned long restartTime = 0;
#include <ETH.h>
#if defined(OTAWEB)
  #include <AsyncTCP.h>
  #include <ESPAsyncWebServer.h>
  #include <AsyncElegantOTA.h>
  AsyncWebServer OTAserver(82);
#endif
#if defined(MQTT)
  #include <PubSubClient.h>
  WiFiClient espClient;
  PubSubClient mqttClient(espClient);
#endif
IPAddress mqtt_server_ip(0, 0, 0, 0);
#include <Wire.h>
#include "time.h"

// ---- TrxNet (Configuration==4 / 04-TrxNetSwitch) ----------------------------
// P2P UDP control of the 8 FREE GPIO. Subscribes /s-gpio (1 byte = 8 outputs),
// publishes /gpio (current output state). See /home/dan/Arduino/libraries/TrxNet.
#include <TrxNet.h>
WiFiUDP trxUdp;                       // WiFiUDP works over ESP32 Ethernet (shared lwIP)
TrxNet  net(trxUdp);
const int trxPins[8] = {0, 2, 4, 12, 13, 14, 32, 33};  // bit0..bit7 -> GPIO
uint16_t TRX_PORT      = 5683;       // TrxNet UDP port (EEPROM 267), same on all peers
char     trxName[TRXNET_MAX_DEVICE_NAME];
bool     trxBegun      = false;
uint8_t  trxState      = 0;          // last applied 8-bit output state
volatile bool trxDirty = false;      // set in /s-gpio callback, drained in loop()
// greet queue: peers that just joined, owed a current-state snapshot
char     trxGreet[TRXNET_MAX_PEERS][TRXNET_MAX_DEVICE_NAME];
uint8_t  trxGreetCount = 0;
void onSGpio(const char* from, const uint8_t* data, size_t len);
void onTrxPeer(const TrxPeer* peer);
WiFiServer SerialServer;
WiFiClient SerialServerClients[MAX_SRV_CLIENTS];
//-------------------------------------------------------------------------------------------------------
// forward declarations (Arduino auto-prototype generation fails in this IDE)
uint32_t readADC_Cal(int ADC_Raw);
void Watchdog();
void CLI2();
void Prn(int LN, String STR);
void http();
void EthEvent(WiFiEvent_t event);
void Mqtt();
bool mqttReconnect();
void reSubscribe();
void MqttRx(char *topic, byte *payload, unsigned int length);
void AfterMQTTconnect();
void MqttPubString(String TOPIC, String DATA, bool RETAIN);
void TrxNetLoop();
String UtcTime(int format);
String Timestamp();
void handleSet();
uint32_t measureChargeTime(int ignoreThreshold, int threshold);
float casNaKapacituProcentaExpon(unsigned long namerenyCas, unsigned long casProStoProcent, float odporOhm, int urovenPrahu, int rozsahADC);

void setup() {
  Serial.begin(115200); //BaudRate
  while(!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("ETH DIN rail development kit");
  Serial.println("----------------------------");
  Serial.println("REV  "+String(REV));
  pinMode(HWidPin, INPUT);
    HWidValue = readADC_Cal(analogRead(HWidPin));
    if(Configuration==3){
      if(HWidValue<=375){
        HardwareRev=0;  // 142
      }else if(HWidValue>375 && HWidValue<=800){
        HardwareRev=1;  // ???
      }
    }else{
      if(HWidValue<=375){
        HardwareRev=1;  // 162
      }else if(HWidValue>375 && HWidValue<=710){
        HardwareRev=2;  // 588
      }else if(HWidValue>710 && HWidValue<=1000){
        HardwareRev=3;  // 860
      }
    }
  Serial.println("HW   "+String(HardwareRev));
  Serial.println("FW   "+String(REV));
  
  pinMode(VoltagePoePin, INPUT);
  // pinMode(Gpi39Pin, INPUT);
  pinMode(USBdetectPin, INPUT);
  pinMode(RS485ReDePin, OUTPUT);

  pinMode(Gpio33Pin, OUTPUT);
  pinMode(Gpio32Pin, OUTPUT);
  pinMode(Gpio14Pin, OUTPUT);
  pinMode(Gpio13Pin, OUTPUT);
  pinMode(Gpio12Pin, OUTPUT);
  pinMode(Gpio4Pin, OUTPUT);
  // pinMode(Gpio2Pin, OUTPUT);
  pinMode(Gpio0Pin, OUTPUT);

  // Capacity measurement
  pinMode(Gpio2Pin, OUTPUT);
  pinMode(Gpi39Pin, INPUT);

  // // konfigurace I2S pro ADC1 DMA continuous mode
  // i2s_config_t i2s_config = {
  //   .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
  //   .sample_rate = SAMPLE_RATE,
  //   .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
  //   .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
  //   .communication_format = I2S_COMM_FORMAT_I2S_MSB,
  //   .intr_alloc_flags = 0,
  //   .dma_buf_count = BUF_COUNT,
  //   .dma_buf_len = BUF_SAMPLES,
  //   .use_apll = false,
  //   .tx_desc_auto_clear = false,
  //   .fixed_mclk = 0
  // };
  // esp_err_t err;
  // err = i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
  // Serial.printf("i2s_driver_install: %d\n", err);
  // err = i2s_set_adc_mode(ADC_UNIT_1, ADC1_CHANNEL_3);
  // Serial.printf("i2s_set_adc_mode: %d\n", err);
  // err = i2s_adc_enable(I2S_NUM);
  // Serial.printf("i2s_adc_enable: %d\n", err);

  // i2s_zero_dma_buffer(I2S_NUM);
  // i2s_start(I2S_NUM);

  // // ADC konfigurace pro GPIO39 (ADC1_CHANNEL_3)
  // adc1_config_width(ADC_WIDTH_BIT_12);
  // adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_11);


  #if defined(DS18B20)
    sensors1.begin();
    sensors2.begin();

    // locate devices on the bus
    Serial.print("T1   found ");
    Serial.print(sensors1.getDeviceCount(), DEC);
    if (sensors1.getAddress(T1, 0)) {
      Serial.print(" address: ");
      printAddress(T1);
      sensors1.setResolution(T1, TEMPERATURE_PRECISION);
      Serial.print(" resolution: ");
      Serial.print(sensors1.getResolution(T1), DEC);
      Serial.println();
    } else {
      ExtTemp = false;
      Serial.println(" address");
    }

    Serial.print("T2   found ");
    Serial.print(sensors2.getDeviceCount(), DEC);
    if (sensors2.getAddress(T2, 0)) {
      Serial.print(" address: ");
      printAddress(T2);
      sensors1.setResolution(T2, TEMPERATURE_PRECISION);
      Serial.print(" resolution: ");
      Serial.print(sensors2.getResolution(T2), DEC);
      Serial.println();
    } else {
      ExtTemp = false;
      Serial.println(" address");
    }

  #endif  

  // Listen source
  if (!EEPROM.begin(EEPROM_SIZE)){
    if(EnableSerialDebug>0){
      Serial.println("failed to initialise EEPROM"); delay(1);
    }
  }

  // 0-1 net ID
    if(EEPROM.read(0)==0xff){
      NET_ID="0";
    }else{
      for (int i=0; i<2; i++){
        if(EEPROM.read(i)!=0xff){
          NET_ID=NET_ID+char(EEPROM.read(i));
        }
      }
    }

  // 168  - MQTT_LOGIN
  if(EEPROM.read(168)==0xff){
    MQTT_LOGIN=false;
  }else{
    if(EEPROM.readBool(168)==1){
      MQTT_LOGIN=true;
    }else{
      MQTT_LOGIN=false;
    }
  }

  // 226-227 BaudRate
  if(EEPROM.read(226)==0xff || EEPROM.readUShort(226) > 9600){
    BaudRate=115200;
  }else{
    BaudRate = EEPROM.readUShort(226);
  }

  // 236-245 - MQTT_USER
  if(EEPROM.read(236)==0xff){
    MQTT_USER="Login";
  }else{
    for (int i=236; i<246; i++){
      if(EEPROM.read(i)!=0xff){
        MQTT_USER=MQTT_USER+char(EEPROM.read(i));
      }
    }
  }

  // 246-265 - MQTT_PASS
  if(EEPROM.read(246)==0xff){
    MQTT_PASS="Password";
  }else{
    for (int i=246; i<266; i++){
      if(EEPROM.read(i)!=0xff){
        MQTT_PASS=MQTT_PASS+char(EEPROM.read(i));
      }
    }
  }

  // 266 Configuration
    if(EEPROM.read(266)==0xff){
      Configuration=0;
    }else{
      Configuration=int(EEPROM.read(266));
    }

  // 267-268 TRX_PORT (TrxNet UDP port)
    if(EEPROM.read(267)==0xff){
      TRX_PORT=5683;
    }else{
      TRX_PORT=EEPROM.readUShort(267);
      if(TRX_PORT==0) TRX_PORT=5683;
    }


  OutputWatchdog=EEPROM.readUInt(30);
  if(OutputWatchdog>10080){
    OutputWatchdog=0;
  }

  // YOUR_CALL
  // move after ETH init

  // MQTT broker IP
  if(EEPROM.read(161)==0xff){
    mqtt_server_ip[0]=54;
  }else{
    mqtt_server_ip[0]=EEPROM.readByte(161);
    if(mqtt_server_ip[0]==0){
      MQTT_ENABLE = false;
    }
  }

  if(EEPROM.read(162)==0xff){
    mqtt_server_ip[1]=38;
  }else{
    mqtt_server_ip[1]=EEPROM.readByte(162);
  }

  if(EEPROM.read(163)==0xff){
    mqtt_server_ip[2]=157;
  }else{
    mqtt_server_ip[2]=EEPROM.readByte(163);
  }

  if(EEPROM.read(164)==0xff){
    mqtt_server_ip[3]=134;
  }else{
    mqtt_server_ip[3]=EEPROM.readByte(164);
  }

  if(EEPROM.read(165)==0xff){
    MQTT_PORT=1883;
  }else{
    MQTT_PORT = EEPROM.readUShort(165);
  }

  if(EEPROM.read(169)==0xff){
    WS_MQTT_PORT=1884;
  }else{
    WS_MQTT_PORT = EEPROM.readUShort(169);
  }

  #if defined(ETHERNET)
    // mqtt_server_ip=BrokerIpArray[0];
    // MQTT_PORT = MQTT_PORT_Array[0];

    WiFi.onEvent(EthEvent);
    // ETH.begin();
    ETH.begin(ETH_ADDR, ETH_POWER, ETH_MDC, ETH_MDIO, ETH_TYPE, ETH_CLK);
    if(DHCP_ENABLE==false){
      ETH.config(IPAddress(192, 168, 0, 11), IPAddress(192, 168, 0, 1),IPAddress(255, 255, 255, 0),IPAddress(8, 8, 8, 8));
      //config(IPAddress local_ip, IPAddress gateway, IPAddress subnet, IPAddress dns1 = (uint32_t)0x00000000, IPAddress dns2 = (uint32_t)0x00000000);
    }
  #endif
    server.begin();
    // chipid=ESP.getEfuseMac();//The chip ID is essentially its MAC address(length: 6 bytes).
    //   unsigned long long1 = (unsigned long)((chipid & 0xFFFF0000) >> 16 );
    //   unsigned long long2 = (unsigned long)((chipid & 0x0000FFFF));
    //   ChipidHex = String(long1, HEX) + String(long2, HEX); // six octets
    //   YOUR_CALL=ChipidHex;

  #if defined(OTAWEB)
    OTAserver.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(200, "text/plain", "PSE QSY to /update");
    });
    AsyncElegantOTA.begin(&OTAserver);    // Start ElegantOTA
    OTAserver.begin();
  #endif

  // WDT
  esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL); //add current thread to WDT watch
  WdtTimer=millis();

  //init and get the time
   configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

   // ajax
   ajaxserver.on("/set", handleSet);
   ajaxserver.begin();                  //Start server
   Serial.println("HTTP ajax server started");

}

//-------------------------------------------------------------------------------------------------------

void loop() {
  http();
  Mqtt();
  CLI2();
  ajaxserver.handleClient();
  Watchdog();  
  #if defined(OTAWEB)
   AsyncElegantOTA.loop();
  #endif

  // SPACE FOR YOUR CODE...
  TrxNetLoop();


}

//-------------------------------------------------------------------------------------------------------
// TrxNet — 04-TrxNetSwitch: drive 8 FREE GPIO from /s-gpio, publish state on /gpio
void TrxNetLoop(){
  if(Configuration!=4) return;

  // one-shot begin once Ethernet is up and NET_ID is set (empty NET_ID = disabled)
  if(!trxBegun){
    if(eth_connected && NET_ID.length()>0){
      for(int i=0;i<8;i++) digitalWrite(trxPins[i], LOW);   // boot state: all LOW
      trxState=0;
      snprintf(trxName, sizeof(trxName), "DIN.%s", NET_ID.c_str());
      net.setPort(TRX_PORT);
      net.onPeerAdded(onTrxPeer);
      net.subscribe("/s-gpio", onSGpio);
      net.begin(trxName);
      trxBegun=true;
      Prn(1, "TrxNet begin "+String(trxName)+" port "+String(TRX_PORT));
    }
    return;
  }

  net.loop();

  // publish current state on change (telemetry -> latest wins)
  if(trxDirty){
    trxDirty=false;
    net.publish("/gpio", &trxState, 1, TRX_NON);
  }

  // greet one freshly-joined peer per iteration with current state (reliable)
  if(trxGreetCount>0){
    trxGreetCount--;
    net.publishTo(trxGreet[trxGreetCount], "/gpio", &trxState, 1, TRX_CON);
  }
}

// Incoming /s-gpio: 1 byte, bit i -> trxPins[i]. Runs inside net.loop(); keep short.
void onSGpio(const char* from, const uint8_t* data, size_t len){
  if(len < 1) return;
  uint8_t b = data[0];
  for(int i=0;i<8;i++){
    digitalWrite(trxPins[i], (b >> i) & 1);
  }
  trxState = b;
  trxDirty = true;     // defer /gpio publish to TrxNetLoop()
}

// New peer joined: queue for a current-state snapshot. Defer the send to loop().
void onTrxPeer(const TrxPeer* peer){
  if(trxGreetCount >= TRXNET_MAX_PEERS) return;
  strncpy(trxGreet[trxGreetCount], peer->name, TRXNET_MAX_DEVICE_NAME-1);
  trxGreet[trxGreetCount][TRXNET_MAX_DEVICE_NAME-1] = '\0';
  trxGreetCount++;
}

// SUBROUTINES -------------------------------------------------------------------------------------------------------

#if defined(DS18B20)
  // function to print a device address
  void printAddress(DeviceAddress deviceAddress)
  {
    for (uint8_t i = 0; i < 8; i++)
    {
      // zero pad the address if necessary
      if (deviceAddress[i] < 16) Serial.print("0");
      Serial.print(deviceAddress[i], HEX);
    }
  }
#endif

//-------------------------------------------------------------------------------------------------------
uint32_t readADC_Cal(int ADC_Raw)
{
  esp_adc_cal_characteristics_t adc_chars;

  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
  return(esp_adc_cal_raw_to_voltage(ADC_Raw, &adc_chars));
}
//-------------------------------------------------------------------------------------------------------
void Watchdog(){

  // restart from another point of code
  if (shouldRestart && millis() > restartTime) {
    ESP.restart();
  }

  // Capacity measurement
  if(Configuration==2){
    static unsigned long lastMeasure = 0;
    unsigned long now = millis();
    static float timeBuffer = 0;

    if (now - lastMeasure >= measureInterval) {
      lastMeasure = now;
      uint32_t duration = 0; // measureChargeTime(ignoreThreshold, measureThreshold);
      for (int i=0; i<100; i++){
        timeBuffer = timeBuffer + measureChargeTime(ignoreThreshold, measureThreshold);
      }
      duration = timeBuffer/100;
      timeBuffer = 0;
      if (duration > 0) {
        // Serial.printf("Čas do dosažení prahu %d: %u µs\n", measureThreshold, duration);
        Serial.printf("Čas do dosažení prahu %d: %u µs | ", measureThreshold, duration);
        MqttPubString("C-measure-µs", String(duration), false);
        // Příklad: duration µs, referenční čas 94 000 µs pro 100%, odpor 10MΩ, práh ADC 3000 z 4095
        float procenta = casNaKapacituProcentaExpon(duration, 94000, 1e7, 3000, 4095);
        Serial.print("Kapacita: ");
        Serial.print(procenta);
        Serial.println(" %");
        MqttPubString("C-measure-%", String(int(procenta)), false);
      } else {
        // Serial.println("."); // Měření neproběhlo nebo timeout
        MqttPubString("C-measure-µs", "failed or timeout", false);
      }
    }
  }

  // POE
  static float VoltageBuffer = 0;
  static float VoltageBufferGpi39 = 0;
  static long ADCTimer = 0;
  static long ADCCounter = 0;
  static long OneWireTimer = 0;
  static long RS485beaconTimer = 0;
  if(millis()-ADCTimer > 5){
    if(Configuration==3){
    // R divider | 11,22/1,22=9.19672131
      VoltageBuffer = VoltageBuffer + (readADC_Cal(analogRead(VoltagePoePin))/1000.0*9.1967)+0.3;
    }else{
    // R divider | 12,95/2,95=4,38983050847458
      VoltageBuffer = VoltageBuffer + (readADC_Cal(analogRead(VoltagePoePin))/1000.0*4.39)+0.3;
    }
    VoltageBufferGpi39 = VoltageBufferGpi39 + readADC_Cal(analogRead(Gpi39Pin))/1000.0; //*4.39;
    ADCCounter ++;
    if(ADCCounter > 34){
      VoltagePOE = VoltageBuffer/35;
      VoltageGpi39 = VoltageBufferGpi39/35;
      ADCCounter = 0;
      VoltageBuffer = 0;
      VoltageBufferGpi39 = 0;
    }
    ADCTimer=millis();
  }

  // USB detect
  if(digitalRead(USBdetectPin) != USBdetect){
    USBdetect = !USBdetect;
    MqttPubString("USBdetect", String(USBdetect), false);
  }

  // RS485 beacon
  // if(millis() - RS485beaconTimer > 1000){
  //   if(USBdetect == false){
  //     digitalWrite(RS485ReDePin, HIGH);
  //     // delay(1);
  //     Serial.println(String(millis()));
  //     Serial.flush();
  //     digitalWrite(RS485ReDePin, LOW);
  //     // delay(1);
  //   }
  //   RS485beaconTimer=millis();
  // }


  // GPIO snake
  if(Configuration==1){
    const int pins[] = {13, 14, 12, 4, 2, 0};  // pole výstupních pinů
    const int numPins = sizeof(pins) / sizeof(pins[0]);
    static int currentIndex = 0;                        // aktuální pozice v poli
    if(millis() - RS485beaconTimer > 1000){
      // digitalWrite(Gpio33Pin, OutPut);
      // digitalWrite(Gpio32Pin, OutPut);
      // digitalWrite(Gpio13Pin, OutPut);
      // digitalWrite(Gpio14Pin, OutPut);
      // digitalWrite(Gpio12Pin, OutPut);
      // digitalWrite(Gpio4Pin, OutPut);
      // digitalWrite(Gpio2Pin, OutPut);
      // digitalWrite(Gpio0Pin, OutPut);
      
      int prevIndex = (currentIndex + numPins - 1) % numPins;
      digitalWrite(pins[prevIndex], LOW);
      digitalWrite(pins[currentIndex], HIGH);
      currentIndex = (currentIndex + 1) % numPins;  RS485beaconTimer=millis();
    }
  }
  if(Configuration==3){
    const int pins[] = {13, 14, 12};  // pole výstupních pinů
    const int numPins = sizeof(pins) / sizeof(pins[0]);
    static int currentIndex = 0;                        // aktuální pozice v poli
    if(millis() - RS485beaconTimer > 1000){
      int prevIndex = (currentIndex + numPins - 1) % numPins;
      digitalWrite(pins[prevIndex], LOW);
      digitalWrite(pins[currentIndex], HIGH);
      currentIndex = (currentIndex + 1) % numPins;  RS485beaconTimer=millis();
    }
  }

  // RS485 to MQTT
  String RXstring;
  if(USBdetect == false){
    if (Serial.available()) {
      RXstring = Serial.readStringUntil('\n');
      MqttPubString("RS485_RX", RXstring, false);
    }
  }

  static float VoltagePOETmp = 0;
  static float VoltagePOETimer = 0;
  // info if change POE voltage
  if(abs(VoltagePOETmp-VoltagePOE)>0.5 || millis() - VoltagePOETimer > 20000){
    if(Configuration==3){
      MqttPubString("InputVoltage", String(VoltagePOE), false);
    }else{
      MqttPubString("VoltagePOE", String(VoltagePOE), false);
    }
    VoltagePOETmp=VoltagePOE;
    VoltagePOETimer=millis();
  }

  static float VoltageGpi39Tmp = 0;
  static float VoltageGpi39Timer = 0;
  if(Configuration<=2){
    // info if change GPII39 voltage
    if(abs(VoltageGpi39Tmp-VoltageGpi39)>0.5 || millis() - VoltageGpi39Timer > 20000){
      MqttPubString("VoltageGpi39", String(VoltageGpi39), false);
      VoltageGpi39Tmp=VoltageGpi39;
      VoltageGpi39Timer=millis();
    }
  }

  // WDT
  if(millis()-WdtTimer > 60000){
    esp_task_wdt_reset();
    WdtTimer=millis();
    if(EnableSerialDebug>0){
      Prn(0,"WDT reset ");
      Prn(1, UtcTime(1));
    }
  }

  #if defined(DS18B20)
  if (millis() - OneWireTimer > 20000) {
    OneWireTimer = millis();
    sensors1.requestTemperatures();
    sensors2.requestTemperatures();
  
    T1Celsius = sensors1.getTempC(T1);
    if(T1Celsius != -127){
      MqttPubString("celsius/T1", String(T1Celsius), false);
      shouldRestart = true;
      restartTime = millis() + 5000;
    }
  
    T2Celsius = sensors2.getTempC(T2);
    if(T2Celsius != -127){
      if(Configuration==3){
        MqttPubString("celsius", String(T2Celsius), false);
      }else{
        MqttPubString("celsius/T2", String(T2Celsius), false);
        shouldRestart = true;
        restartTime = millis() + 5000;
      }
    }
  }
  #endif
}

//-------------------------------------------------------------------------------------------------------
void CLI2(){
  static int incomingByte = 0;

  if (Serial.available() > 0) {
    incomingByte = Serial.read();
  }
  esp_task_wdt_reset();
  WdtTimer=millis();

  // ? H h
  if(incomingByte==63 || incomingByte==72 || incomingByte==104){
    Prn(1, "http://"+String(ETH.localIP()[0])+"."+String(ETH.localIP()[1])+"."+String(ETH.localIP()[2])+"."+String(ETH.localIP()[3]) );

  // @
  }else if(incomingByte==64){
      Prn(1,"** DIN module will be restarted **");
      delay(1000);
      ESP.restart();

  // CR/LF
  }else if(incomingByte==13||incomingByte==10){
    // Prn(1,"");

  // anykey
  }else{
    // Prn(0," [");
    // Prn(0, String(incomingByte) ); //, DEC);
    // Prn(1,"] unknown command");
  }
  incomingByte=0;
}

//-------------------------------------------------------------------------------------------------------
void Prn(int LN, String STR){
  if(USBdetect == true){
    Serial.print(STR);
    if(LN==1){
      Serial.println();
    }
  }
}

//-------------------------------------------------------------------------------------------------------

void http(){
  // listen for incoming clients
  WiFiClient webClient = server.available();
  if (webClient) {
    if(EnableSerialDebug>0){
      Serial.println("WIFI New webClient");
    }
    memset(linebuf,0,sizeof(linebuf));
    charcount=0;
    // an http request ends with a blank line
    boolean currentLineIsBlank = true;
    while (webClient.connected()) {
      if (webClient.available()) {
        char c = webClient.read();
        HTTP_req += c;
        // if(EnableSerialDebug>0){
        //   Serial.write(c);
        // }
        //read char by char HTTP request
        linebuf[charcount]=c;
        if (charcount<sizeof(linebuf)-1) charcount++;
        // if you've gotten to the end of the line (received a newline
        // character) and the line is blank, the http request has ended,
        // so you can send a reply
        if (c == '\n' && currentLineIsBlank) {
          // send a standard http response header

          // send a standard http response header
          webClient.println(F("HTTP/1.1 200 OK"));
          webClient.println(F("Content-Type: text/html"));
          webClient.println(F("Connection: close"));  // the connection will be closed after completion of the response
          webClient.println();
          webClient.println(F("  <!DOCTYPE html>"));
          webClient.println(F("  <html>"));
          webClient.println(F("      <head>"));
          webClient.println(F("          <meta http-equiv=\"Content-Type\" content=\"text/html;charset=utf-8\"/>"));
          webClient.println(F("          <meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">"));
          // webClient.println(F("          <meta http-equiv=\"refresh\" content=\"10\">"));
          webClient.println(F("          <link rel=\"stylesheet\" type=\"text/css\" href=\"https://remoteqth.com/mqtt-wall/style.css\">"));
          // TITLE
          webClient.print(F("           <title>ETH-DIN-DEV-KIT "));
          webClient.print(YOUR_CALL);
          webClient.println(F("</title>"));
          // END TITLE
          webClient.println(F("          <link rel=\"apple-touch-icon\" sizes=\"180x180\" href=\"style/apple-touch-icon.png\">"));
          webClient.println(F("          <link rel=\"mask-icon\" href=\"style/safari-pinned-tab.svg\" color=\"#5bbad5\">"));
          webClient.println(F("          <link rel=\"icon\" type=\"image/png\" href=\"style/favicon-32x32.png\" sizes=\"32x32\">"));
          webClient.println(F("          <link rel=\"icon\" type=\"image/png\" href=\"style/favicon-16x16.png\" sizes=\"16x16\">"));
          webClient.println(F("          <link rel=\"manifest\" href=\"style/manifest.json\">"));
          webClient.println(F("          <link rel=\"shortcut icon\" href=\"style/favicon.ico\">"));
          webClient.println(F("          <meta name='apple-mobile-web-app-capable' content='yes'>"));
          webClient.println(F("          <meta name='mobile-web-app-capable' content='yes'>"));
          webClient.println(F("          <meta name=\"msapplication-config\" content=\"style/browserconfig.xml\">"));
          webClient.println(F("          <meta name=\"theme-color\" content=\"#ffffff\">"));
          webClient.println(F("          <script type=\"text/javascript\">"));
          webClient.println(F("          var config = {"));
          webClient.println(F("              server: {"));
          webClient.print(F("                  uri: \"ws://"));
          webClient.print(mqtt_server_ip[0]);
          webClient.print(F("."));
          webClient.print(mqtt_server_ip[1]);
          webClient.print(F("."));
          webClient.print(mqtt_server_ip[2]);
          webClient.print(F("."));
          webClient.print(mqtt_server_ip[3]);
          webClient.print(":");
          webClient.print(WS_MQTT_PORT);
          webClient.println("/\",");
          if(MQTT_LOGIN==true){
            webClient.print(F("                  username: \""));
            webClient.print(String(MQTT_USER));
            webClient.println(F("\","));
            webClient.print(F("                  password: \""));
            webClient.print(String(MQTT_PASS));
            webClient.println(F("\""));
          }
          webClient.println(F("              },"));
          // TOPIC
          webClient.print(F("              defaultTopic: \""));
          webClient.print(YOUR_CALL);
          webClient.print(F("/"));
          webClient.print(NET_ID);
          webClient.println(F("/#\","));
          // END TOPIC
          webClient.println(F("              showCounter: true,"));
          webClient.println(F("              alphabeticalSort: true,"));
          webClient.println(F("              qos: 0"));
          webClient.println(F("          };"));
          webClient.println(F("          </script>"));
          // END TOPIC
          webClient.println(F("      </head>"));
          webClient.println(F("      <body>"));
          webClient.print(F("          <div id=\"frame\" "));
          webClient.println(F(">"));
          webClient.println(F("              <div id=\"footer\">"));
          webClient.println(F("                  <p class=\"status\" style=\"font-size: 150%;\">"));
          // STATUS
          webClient.print(F("Uptime: "));
          if(millis() < 60000){
            webClient.print(millis()/1000);
            webClient.print(F(" seconds"));
          }else if(millis() > 60000 && millis() < 3600000){
            webClient.print(millis()/60000);
            webClient.print(F(" minutes"));
          }else if(millis() > 3600000 && millis() < 86400000){
            webClient.print(millis()/3600000);
            webClient.print(F(" hours"));
          }else{
            webClient.print(millis()/86400000);
            webClient.print(F(" days"));
          }
          webClient.print(F(" | FW:&nbsp;"));
          webClient.println(REV);
          webClient.print(F("&nbsp;| HW:&nbsp;"));
          webClient.println(HardwareRev);
          webClient.print(F("&nbsp;| eth&nbsp;mac:&nbsp;"));
          webClient.print(MACString);
          webClient.println();

          webClient.print(F("&nbsp;| dhcp:&nbsp;"));
          if(DHCP_ENABLE==1){
            webClient.print(F("ON"));
          }else{
            webClient.print(F("OFF"));
          }
          webClient.print(F("&nbsp;| ip:&nbsp;"));
          webClient.println(ETH.localIP());
          // webClient.print(F(" | utc from ntp: "));
          // webClient.println(F("timeClient.getFormattedTime()"));
          // webClient.println(F("<br>MQTT subscribe command: $ mosquitto_sub -v -h mqttstage.prusa -t prusa-debug/prusafil/extrusionline/+/#"));
          webClient.print(F(" | MQTT&nbsp;"));
          if(MQTT_ENABLE==true){
            webClient.print(F("broker&nbsp;ip:&nbsp;"));
            webClient.print(mqtt_server_ip[0]);
            webClient.print(F("."));
            webClient.print(mqtt_server_ip[1]);
            webClient.print(F("."));
            webClient.print(mqtt_server_ip[2]);
            webClient.print(F("."));
            webClient.print(mqtt_server_ip[3]);
            webClient.print(F(":"));
            webClient.print(MQTT_PORT);
          }else{
            webClient.print(F("<span style='color: #F00; font-weight: bold;'>DISABLE</span>"));
          }
          #if defined(OTAWEB)
            webClient.print(F("&nbsp;| <a href=\"http://"));
            webClient.println(ETH.localIP());
            webClient.print(F(":82/update\" target=_blank>Upload&nbsp;FW</a>&nbsp;| <a href=\"https://github.com/ok1hra/eth-din-dev-kit/releases\" target=_blank>Releases</a><br><a href=\"http://"));
            webClient.println(ETH.localIP());
            webClient.print(F(":88/set\" onclick=\"window.open( this.href, this.href, 'width=620,height=650,left=0,top=0,menubar=no,location=no,status=no' ); return false;\"><button style='color: #fff; background-color: #060; padding: 5px 20px 5px 20px; margin:15px; border: none; -webkit-border-radius: 5px; -moz-border-radius: 5px; border-radius: 5px;} :hover {background-color: orange;} '>SETUP</button></a>"));
          #endif
          // END STATUS
          webClient.println(F("              </p>"));
          webClient.println(F("              </div>"));
          webClient.println(F("              <div id=\"header\">"));
          webClient.println(F("                  <div id=\"topic-box\">"));
          webClient.println(F("                      <input type=\"text\" id=\"topic\" value=\"\" title=\"Topic to subscribe\">"));
          webClient.println(F("                  </div>"));
          webClient.println(F("              </div>"));
          webClient.println(F("              <div id=\"toast\"></div>"));
          webClient.println(F("              <section class=\"messages\"></section>"));
          webClient.println(F("              <div id=\"footer\">"));
          webClient.println(F("                  <p class=\"status\">"));
          webClient.println(F("                      Client <code id=\"status-client\" title=\"Client ID\">?</code> is "));
          webClient.println(F("                      <code id=\"status-state\" class=\"connecting\"><em>&bull;</em> <span>connecting...</span></code> to "));
          webClient.println(F("                      <code id=\"status-host\">?</code>"));
          webClient.println(F("                      <em>via</em> MQTT Wall 0.3.0 (<a href=\"https://github.com/bastlirna/mqtt-wall\">github</a>)"));
          webClient.println(F("                      | <a href=\"https://remoteqth.com/w/\" target=\"_blank\">Wiki</a>."));
          webClient.println(F("                  </p>"));
          webClient.println(F("              </div>"));
          webClient.println(F("          </div>"));
          webClient.println(F("          <script type=\"text/javascript\" src=\"https://code.jquery.com/jquery-2.1.4.min.js\"></script>"));
          webClient.println(F("          <script type=\"text/javascript\" src=\"https://code.jquery.com/color/jquery.color-2.1.2.min.js\"></script>"));
          webClient.println(F("          <script type=\"text/javascript\" src=\"https://cdnjs.cloudflare.com/ajax/libs/paho-mqtt/1.0.1/mqttws31.min.js\"></script>"));
          webClient.println(F("          <script type=\"text/javascript\" src=\"https://remoteqth.com/mqtt-wall/wall.js\"></script>"));
          webClient.println(F("      </body>"));
          webClient.println(F("  </html>"));

          if(EnableSerialDebug>0){
            Serial.print(HTTP_req);
          }
          HTTP_req = "";

          break;
        }
        if (c == '\n') {
          // you're starting a new line
          currentLineIsBlank = true;
          // if (strstr(linebuf,"GET /h0 ") > 0){digitalWrite(GPIOS[0], HIGH);}else if (strstr(linebuf,"GET /l0 ") > 0){digitalWrite(GPIOS[0], LOW);}
          // else if (strstr(linebuf,"GET /h1 ") > 0){digitalWrite(GPIOS[1], HIGH);}else if (strstr(linebuf,"GET /l1 ") > 0){digitalWrite(GPIOS[1], LOW);}

          // you're starting a new line
          currentLineIsBlank = true;
          memset(linebuf,0,sizeof(linebuf));
          charcount=0;
        } else if (c != '\r') {
          // you've gotten a character on the current line
          currentLineIsBlank = false;
        }
      }
    }
    // give the web browser time to receive the data
    delay(1);

    // close the connection:
    webClient.stop();
   if(EnableSerialDebug>0){
     Serial.println("WIFI webClient disconnected");
   }
  }
}
//-------------------------------------------------------------------------------------------------------

void EthEvent(WiFiEvent_t event)
{
  switch (event) {
    // case SYSTEM_EVENT_ETH_START:
    case ARDUINO_EVENT_ETH_START:
      Serial.println("ETH  Started");
      //set eth hostname here
      ETH.setHostname("esp32-ethernet");
      break;
    // case SYSTEM_EVENT_ETH_CONNECTED:
    case ARDUINO_EVENT_ETH_CONNECTED:
      Serial.println("ETH  Connected");
      break;
    // case SYSTEM_EVENT_ETH_GOT_IP:
    case ARDUINO_EVENT_ETH_GOT_IP:
      MACString = ETH.macAddress();
      MACString.toCharArray( MACchar, 18 );
      Serial.print("ETH  MAC: ");
      Serial.println(MACString);
      // Serial.println("===============================");
      // Serial.print("   IPv4: ");
      // Serial.println(ETH.localIP());
      // Serial.println("===============================");
      if (ETH.fullDuplex()) {
        Serial.print("     FULL_DUPLEX, ");
      }
      Serial.print(ETH.linkSpeed());
      Serial.println("Mbps");
      eth_connected = true;

      // Load YOUR_CALL from EEPROM on cold start — independent of MQTT, so the
      // setup field persists even when MQTT is disabled (e.g. TrxNetSwitch mode).
      if (YOUR_CALL.isEmpty()){
        if(EEPROM.read(141)==0xff){
          YOUR_CALL=MACString;
          YOUR_CALL.remove(0, 12);
        }else{
          for (int i=141; i<161; i++){
            if(EEPROM.read(i)!=0xff){
              YOUR_CALL=YOUR_CALL+char(EEPROM.read(i));
            }
          }
        }
      }

      #if defined(MQTT)
        if(MQTT_ENABLE == true){
          Serial.print("     EthEvent-mqtt ");
          mqttClient.setServer(mqtt_server_ip, MQTT_PORT);
          mqttClient.setCallback(MqttRx);
          lastMqttReconnectAttempt = 0;

          char charbuf[50];
           // // memcpy( charbuf, ETH.macAddress(), 6);
           // ETH.macAddress().toCharArray(charbuf, 18);
           // // charbuf[6] = 0;
          if(MQTT_LOGIN == true){
            if (mqttClient.connect(MACchar,MQTT_USER.c_str(),MQTT_PASS.c_str())){
              Prn(1, String(MACchar));
              mqttReconnect();
              AfterMQTTconnect();
              Prn(1, "http://"+String(ETH.localIP()[0])+"."+String(ETH.localIP()[1])+"."+String(ETH.localIP()[2])+"."+String(ETH.localIP()[3]) );
              if(BaudRate!=115200){
                MqttPubString("USB-BaudRate", String(BaudRate), true);
                Serial.println("Baudrate change to "+String(BaudRate)+"...");
                Serial.flush();
                // Serial.end();
                delay(1000);
                Serial.begin(BaudRate);
                delay(500);
                Serial.println();
                Serial.println();
                Serial.println("New Baudrate "+String(BaudRate));
                Prn(1, "http://"+String(ETH.localIP()[0])+"."+String(ETH.localIP()[1])+"."+String(ETH.localIP()[2])+"."+String(ETH.localIP()[3]) );
              }
            }
          }else{
            if (mqttClient.connect(MACchar)){
              Prn(1, String(MACchar));
              mqttReconnect();
              AfterMQTTconnect();
              Prn(1, "http://"+String(ETH.localIP()[0])+"."+String(ETH.localIP()[1])+"."+String(ETH.localIP()[2])+"."+String(ETH.localIP()[3]) );
              if(BaudRate!=115200){
                MqttPubString("USB-BaudRate", String(BaudRate), true);
                Serial.println("Baudrate change to "+String(BaudRate)+"...");
                Serial.flush();
                // Serial.end();
                delay(1000);
                Serial.begin(BaudRate);
                delay(500);
                Serial.println();
                Serial.println();
                Serial.println("New Baudrate "+String(BaudRate));
                Prn(1, "http://"+String(ETH.localIP()[0])+"."+String(ETH.localIP()[1])+"."+String(ETH.localIP()[2])+"."+String(ETH.localIP()[3]) );
              }
            }
          }
        }
      #endif
      // ListCommands(0);
      break;

    // case SYSTEM_EVENT_ETH_DISCONNECTED:
    case ARDUINO_EVENT_ETH_DISCONNECTED:
      Serial.println("ETH  Disconnected");
      eth_connected = false;
      break;
    // case SYSTEM_EVENT_ETH_STOP:
    case ARDUINO_EVENT_ETH_STOP:
      Serial.println("ETH  Stopped");
      eth_connected = false;
      break;
    default:
      break;
  }
}
//-------------------------------------------------------------------------------------------------------
void Mqtt(){
  if (millis()-MqttStatusTimer[0]>MqttStatusTimer[1] && MQTT_ENABLE == true && eth_connected==1){
    if(!mqttClient.connected()){
      long now = millis();
      if (now - lastMqttReconnectAttempt > 5000) {
        lastMqttReconnectAttempt = now;
        Prn(1, "MQTT attempt to reconnect | "+String(millis()/1000) );
        if (mqttReconnect()) {
          lastMqttReconnectAttempt = 0;
        }
      }
    }else{
      // Client connected
      mqttClient.loop();
    }
    MqttStatusTimer[0]=millis();
  }
}

//-------------------------------------------------------------------------------------------------------

bool mqttReconnect() {
  // Prn(0, "MQTT");
  char charbuf[50];
  // // memcpy( charbuf, ETH.macAddress(), 6);
  // ETH.macAddress().toCharArray(charbuf, 18);
  // charbuf[6] = 0;
  if(MQTT_LOGIN == true){
    if (mqttClient.connect(MACchar,MQTT_USER.c_str(),MQTT_PASS.c_str())){
      Prn(1, "MQTT Reconnect-connected");
      reSubscribe();
    }
  }else{
    if (mqttClient.connect(MACchar)) {
      Prn(1, "MQTT Reconnect-connected");
      // IPAddress IPlocalAddr = ETH.localIP();                           // get
      // String IPlocalAddrString = String(IPlocalAddr[0]) + "." + String(IPlocalAddr[1]) + "." + String(IPlocalAddr[2]) + "." + String(IPlocalAddr[3]);   // to string
      // MqttPubStringQC(1, "IP", IPlocalAddrString, true);
      reSubscribe();
    }
  }
  return mqttClient.connected();
}

//------------------------------------------------------------------------------------
void reSubscribe(){
    String topic = String(YOUR_CALL) + "/" + String(NET_ID) + "/celsius/+";
    const char *cstr = topic.c_str();
    if(mqttClient.subscribe(cstr)==true){
      Prn(1, "MQTT subscribe "+String(cstr));
    }
}

//------------------------------------------------------------------------------------
void MqttRx(char *topic, byte *payload, unsigned int length) {
  String CheckTopicBase;
  CheckTopicBase.reserve(100);
  byte* p = (byte*)malloc(length);
  memcpy(p,payload,length);
  // static bool HeardBeatStatus;
  // Prn(1, "MQTT rx");

    // Rx send celsius for confirm
    CheckTopicBase = String(YOUR_CALL) + "/" + String(NET_ID) + "/celsius/";
    // if ( CheckTopicBase.equals( String(topic) ) ){
    if (String(topic).startsWith(CheckTopicBase)) {
      int RxCelsius = 0;
      unsigned long exp = 1;
      for (int i = length-1; i >=0 ; i--) {
        // Numbers only
        if(p[i]>=48 && p[i]<=57){
          RxCelsius = RxCelsius + (p[i]-48)*exp;
          exp = exp*10;
        }
      }
      // Prn(1, "MQTT /celsius/+ " + String(RxCelsius));
      shouldRestart = false;
    }

} // MqttRx END

//-------------------------------------------------------------------------------------------------------
void AfterMQTTconnect(){
  #if defined(MQTT)
  //    if (mqttClient.connect("esp32gwClient", MQTT_USER, MQTT_PASS)) {          // public IP addres to MQTT
        IPAddress IPlocalAddr = ETH.localIP();                           // get
        String IPlocalAddrString = String(IPlocalAddr[0]) + "." + String(IPlocalAddr[1]) + "." + String(IPlocalAddr[2]) + "." + String(IPlocalAddr[3]);   // to string
        IPlocalAddrString.toCharArray( mqttTX, 50 );                          // to array
        String path2 = String(YOUR_CALL) + "/" + String(NET_ID) + "/ip";
        path2.toCharArray( mqttPath, 100 );
        mqttClient.publish(mqttPath, mqttTX, true);
          Serial.print("MQTT TX>");
          Serial.print(mqttPath);
          Serial.print(" ");
          Serial.println(mqttTX);

        // String MAClocalAddrString = ETH.macAddress();   // to string
        // MAClocalAddrString.toCharArray( mqttTX, 50 );                          // to array
        path2 = String(YOUR_CALL) + "/" + String(NET_ID) + "/mac";
        path2.toCharArray( mqttPath, 100 );
        mqttClient.publish(mqttPath, MACchar, true);
          Serial.print("MQTT TX>");
          Serial.print(mqttPath);
          Serial.print(" ");
          Serial.println(MACchar);


  #endif
}
//-----------------------------------------------------------------------------------
void MqttPubString(String TOPIC, String DATA, bool RETAIN){
  char charbuf[50];
   // // memcpy( charbuf, mac, 6);
   // ETH.macAddress().toCharArray(charbuf, 10);
   // charbuf[6] = 0;
  // if(EnableEthernet==1 && MQTT_ENABLE==1 && EthLinkStatus==1 && mqttClient.connected()==true){
  if(mqttClient.connected()==true){
    if(MQTT_LOGIN == true){
      if (mqttClient.connect(MACchar,MQTT_USER.c_str(),MQTT_PASS.c_str())){
        String topic = String(YOUR_CALL) + "/" + String(NET_ID) + "/"+TOPIC;
        topic.toCharArray( mqttPath, 50 );
        DATA.toCharArray( mqttTX, 50 );
        mqttClient.publish(mqttPath, mqttTX, RETAIN);
      }
    }else{
      if (mqttClient.connect(MACchar)) {
        String topic = String(YOUR_CALL) + "/" + String(NET_ID) + "/"+TOPIC;
        topic.toCharArray( mqttPath, 50 );
        DATA.toCharArray( mqttTX, 50 );
        mqttClient.publish(mqttPath, mqttTX, RETAIN);
      }
    }
  }
}
//-------------------------------------------------------------------------------------------------------

String UtcTime(int format){
  tm timeinfo;
  char buf[50]; //50 chars should be enough
  if (eth_connected==false) {
    strcpy(buf, "n/a");
  }else{
    if(!getLocalTime(&timeinfo)){
      strcpy(buf, "n/a");
    }else{
      if(format==1){
        strftime(buf, sizeof(buf), "%Y-%b-%d %H:%M:%S", &timeinfo);
      }else if(format==2){
        strftime(buf, sizeof(buf), "%d", &timeinfo);
      }else if(format==3){
        strftime(buf, sizeof(buf), "%Y", &timeinfo);
      }
    }
  }
  // Serial.println(buf);
  return String(buf);
}

String Timestamp(){
  //time_t now;
  //time(&now);
  struct timeval tv;
  gettimeofday(&tv, NULL);
  char timestamp[40];
  // sprintf(timestamp,"%u.%06u",tv.tv_sec, tv.tv_usec);
  sprintf(timestamp,"%ld.%06ld",tv.tv_sec, tv.tv_usec);
  return String(timestamp);
}

//-------------------------------------------------------------------------------------------------------
// ajax rx
void handleSet() {

  String yourcallERR= "";
  String rotidERR= "";
  String mqttERR= "";
  String mqttportERR= "";
  String wsmqttportERR= "";
  String baudSELECT0= "";
  String baudSELECT1= "";
  String baudSELECT2= "";
  String baudSELECT3= "";
  String baudSELECT4= "";
  String mqtt_loginSTYLE= "";
  String mqtt_loginCHECKED= "";
  String mqtt_userSTYLE= "";
  String mqtt_userERR= "";
  String mqtt_passSTYLE= "";
  String mqtt_passERR= "";
  String mqtt_loginDisable= "";
  String confSELECT0= "";
  String confSELECT1= "";
  String confSELECT2= "";
  String confSELECT3= "";
  String confSELECT4= "";

  if ( ajaxserver.hasArg("yourcall") == false \
    && ajaxserver.hasArg("rotid") == false \
  ) {
    // MqttPubString("Debug", "Form not valid", false);
  }else{
    // MqttPubString("Debug", "Form valid", false);

    // YOUR_CALL
    if ( ajaxserver.arg("yourcall").length()<1 || ajaxserver.arg("yourcall").length()>20){
      yourcallERR= " Out of range 1-20 characters";
    }else{
      String str = String(ajaxserver.arg("yourcall"));
      if(YOUR_CALL == str){
        yourcallERR="";
      }else{
        yourcallERR=" Warning: MQTT topic has changed.";
        YOUR_CALL = String(ajaxserver.arg("yourcall"));

        int str_len = str.length();
        char char_array[str_len+1];
        str.toCharArray(char_array, str_len+1);
        for (int i=0; i<20; i++){
          if(i < str_len){
            EEPROM.write(141+i, char_array[i]);
          }else{
            EEPROM.write(141+i, 0xff);
          }
        }
        // EEPROM.commit();
      }
    }

    // NET_ID
    if ( ajaxserver.arg("rotid").length()<1 || ajaxserver.arg("rotid").length()>2){
      rotidERR= " Out of range 1-2 characters";
    }else{
      String str = String(ajaxserver.arg("rotid"));
      if(NET_ID == str){
        rotidERR="";
      }else{
        rotidERR=" Warning: MQTT topic has changed.";
        NET_ID = String(ajaxserver.arg("rotid"));

        int str_len = str.length();
        char char_array[str_len+1];
        str.toCharArray(char_array, str_len+1);
        for (int i=0; i<2; i++){
          if(i < str_len){
            EEPROM.write(i, char_array[i]);
          }else{
            EEPROM.write(i, 0xff);
          }
        }
        // EEPROM.commit();
      }
    }

    // 236-245 - MQTT_USER
    if ( ajaxserver.arg("mqttuser").length()<1 || ajaxserver.arg("mqttuser").length()>10){
      if(MQTT_LOGIN==true){
        mqtt_userERR= " Out of range 1-10 characters";
      }else{
        mqtt_userERR= "";
      }
    }else{
      String str = String(ajaxserver.arg("mqttuser"));
      if(MQTT_USER == str){
        mqtt_userERR="";
      }else{
        mqtt_userERR=" Must restart after change!";
        MQTT_USER = String(ajaxserver.arg("mqttuser"));

        int str_len = str.length();
        char char_array[str_len+1];
        str.toCharArray(char_array, str_len+1);
        for (int i=0; i<9; i++){
          if(i < str_len){
            EEPROM.write(236+i, char_array[i]);
          }else{
            EEPROM.write(236+i, 0xff);
          }
        }
        MqttPubString("MQTTuser", String(MQTT_USER), true);
      }
    }

    // 246-265 - MQTT_PASS
    if ( ajaxserver.arg("mqttpass").length()<1 || ajaxserver.arg("mqttpass").length()>20){
      if(MQTT_LOGIN==true){
        mqtt_passERR= " Out of range 1-20 characters";
      }else{
        mqtt_passERR= "";
      }
    }else{
      String str = String(ajaxserver.arg("mqttpass"));
      if(MQTT_PASS == str){
        mqtt_passERR="";
      }else{
        mqtt_passERR=" Must restart after change!";
        MQTT_PASS = String(ajaxserver.arg("mqttpass"));

        int str_len = str.length();
        char char_array[str_len+1];
        str.toCharArray(char_array, str_len+1);
        for (int i=0; i<19; i++){
          if(i < str_len){
            EEPROM.write(246+i, char_array[i]);
          }else{
            EEPROM.write(246+i, 0xff);
          }
        }
      }
    }

    // 226-227 BaudRate
    static int BaudRateTmp=115200;
    switch (ajaxserver.arg("baud").toInt()) {
      case 0: {BaudRateTmp= 1200; break; }
      case 1: {BaudRateTmp= 2400; break; }
      case 2: {BaudRateTmp= 4800; break; }
      case 3: {BaudRateTmp= 9600; break; }
      case 4: {BaudRateTmp= 115200; break; }
    }
    if(BaudRateTmp!=BaudRate){
      BaudRate=BaudRateTmp;
      EEPROM.writeUShort(226, BaudRate);
      MqttPubString("USB-BaudRate", String(BaudRate), true);
      Serial.println("Baudrate change to "+String(BaudRate)+"...");
      Serial.flush();
      // Serial.end();
      delay(1000);
      Serial.begin(BaudRate);
      delay(500);
      Serial.println();
      Serial.println();
      Serial.println("New Baudrate "+String(BaudRate));
    }

    // 266 Configuration
    if(ajaxserver.arg("ext").toInt() != Configuration){
      switch (ajaxserver.arg("ext").toInt()) {
        case 0: {Configuration= 0; break; }
        case 1: {Configuration= 1; break; }
        case 2: {Configuration= 2; break; }
        case 3: {Configuration= 3; break; }
        case 4: {Configuration= 4; break; }
      }
      EEPROM.write(266, Configuration);
      MqttPubString("Configuration", String(Configuration), true);
      Serial.println("Configuration change to "+String(Configuration)+"...");
    }

    // 267-268 TRX_PORT (TrxNet UDP port)
    if(ajaxserver.arg("trxport").length()>0 && ajaxserver.arg("trxport").toInt() != TRX_PORT){
      TRX_PORT = ajaxserver.arg("trxport").toInt();
      EEPROM.writeUShort(267, TRX_PORT);
      Serial.println("TrxNet port change to "+String(TRX_PORT)+"...");
    }

    // 161-164 - MQTT broker IP
    if ( ajaxserver.arg("mqttip0").length()<1 || ajaxserver.arg("mqttip0").toInt()>255){
      mqttERR= " Out of range number 0-255";
    }else{
      if(mqtt_server_ip[0] == byte(ajaxserver.arg("mqttip0").toInt()) ){
        mqttERR="";
      }else{
        mqttERR=" Warning: MQTT broker IP has changed.";
        mqtt_server_ip[0] = byte(ajaxserver.arg("mqttip0").toInt()) ;
        EEPROM.writeByte(161, mqtt_server_ip[0]);
      }
    }

    if ( ajaxserver.arg("mqttip1").length()<1 || ajaxserver.arg("mqttip1").toInt()>255){
      mqttERR= " Out of range number 0-255";
    }else{
      if(mqtt_server_ip[1] == byte(ajaxserver.arg("mqttip1").toInt()) ){
        mqttERR="";
      }else{
        mqttERR=" Warning: MQTT broker IP has changed.";
        mqtt_server_ip[1] = byte(ajaxserver.arg("mqttip1").toInt()) ;
        EEPROM.writeByte(162, mqtt_server_ip[1]);
      }
    }

    if ( ajaxserver.arg("mqttip2").length()<1 || ajaxserver.arg("mqttip2").toInt()>255){
      mqttERR= " Out of range number 0-255";
    }else{
      if(mqtt_server_ip[2] == byte(ajaxserver.arg("mqttip2").toInt()) ){
        mqttERR="";
      }else{
        mqttERR=" Warning: MQTT broker IP has changed.";
        mqtt_server_ip[2] = byte(ajaxserver.arg("mqttip2").toInt()) ;
        EEPROM.writeByte(163, mqtt_server_ip[2]);
      }
    }

    if ( ajaxserver.arg("mqttip3").length()<1 || ajaxserver.arg("mqttip3").toInt()>255){
      mqttERR= " Out of range number 0-255";
    }else{
      if(mqtt_server_ip[3] == byte(ajaxserver.arg("mqttip3").toInt()) ){
        mqttERR="";
      }else{
        mqttERR=" Warning: MQTT broker IP has changed.";
        mqtt_server_ip[3] = byte(ajaxserver.arg("mqttip3").toInt()) ;
        EEPROM.writeByte(164, mqtt_server_ip[3]);
      }
    }

    // 165-166 - MQTT_PORT
    if ( ajaxserver.arg("mqttport").length()<1 || ajaxserver.arg("mqttport").toInt()<1 || ajaxserver.arg("mqttport").toInt()>65535){
      mqttportERR= " Out of range number 1-65535";
    }else{
      if(MQTT_PORT == ajaxserver.arg("mqttport").toInt()){
        mqttportERR="";
      }else{
        mqttportERR=" Warning: MQTT broker PORT has changed.";
        MQTT_PORT = ajaxserver.arg("mqttport").toInt();
        EEPROM.writeUShort(165, MQTT_PORT);
      }
    }

    // 169-170 - WS_MQTT_PORT
    if ( ajaxserver.arg("wsmqttport").length()<1 || ajaxserver.arg("wsmqttport").toInt()<1 || ajaxserver.arg("wsmqttport").toInt()>65535){
      wsmqttportERR= " Out of range number 1-65535";
    }else{
      if(WS_MQTT_PORT == ajaxserver.arg("wsmqttport").toInt()){
        wsmqttportERR="";
      }else{
        wsmqttportERR=" Warning: websocket MQTT broker PORT has changed.";
        WS_MQTT_PORT = ajaxserver.arg("wsmqttport").toInt();
        EEPROM.writeUShort(169, WS_MQTT_PORT);
      }
    }

    // 168 - MQTT_LOGIN
    if(ajaxserver.arg("mqtt_login").toInt()==1 && MQTT_LOGIN==false){
      MQTT_LOGIN = true;
      EEPROM.writeBool(168, MQTT_LOGIN);
      MqttPubString("MQTToginEnable", String(MQTT_LOGIN), true);
    }else if(ajaxserver.arg("mqtt_login").toInt()!=1 && MQTT_LOGIN==true){
      MQTT_LOGIN = false;
      EEPROM.writeBool(168, MQTT_LOGIN);
      MqttPubString("MQTToginEnable", String(MQTT_LOGIN), true);
    }
    EEPROM.commit();
    // Serial.println("Interface will be restarted...");
    // delay(3000);
    // ESP.restart();
  } // else form valid

if(MQTT_LOGIN==true){
  mqtt_loginCHECKED= "checked";
  mqtt_loginSTYLE="";
}else{
  mqtt_loginCHECKED= "";
  mqtt_loginSTYLE=" style='text-decoration: line-through; color: #555;'";
}

if(MQTT_LOGIN==true){
  mqtt_loginCHECKED= "checked";
  mqtt_loginDisable="";
  mqtt_userSTYLE="";
  mqtt_passSTYLE="";
}else{
  mqtt_loginCHECKED= "";
  mqtt_loginDisable=" disabled";
  mqtt_userSTYLE=" style='text-decoration: line-through; color: #555;'";
  mqtt_passSTYLE=" style='text-decoration: line-through; color: #555;'";
}

baudSELECT0= "";
baudSELECT1= "";
baudSELECT2= "";
baudSELECT3= "";
baudSELECT4= "";
switch (BaudRate) {
  case 1200: {baudSELECT0= " selected"; break; }
  case 2400: {baudSELECT1= " selected"; break; }
  case 4800: {baudSELECT2= " selected"; break; }
  case 9600: {baudSELECT3= " selected"; break; }
  case 115200: {baudSELECT4= " selected"; break; }
}

confSELECT0= "";
confSELECT1= "";
confSELECT2= "";
switch (Configuration) {
  case 0: {confSELECT0= " selected"; break; }
  case 1: {confSELECT1= " selected"; break; }
  case 2: {confSELECT2= " selected"; break; }
  case 3: {confSELECT3= " selected"; break; }
  case 4: {confSELECT4= " selected"; break; }
}

  String HtmlSrc = "<!DOCTYPE html><html><head><title>SETUP</title>\n";
  HtmlSrc +="<meta http-equiv='Content-Type' content='text/html; charset=UTF-8'>\n";
  // <meta http-equiv = 'refresh' content = '600; url = /'>\n";
  HtmlSrc +="<style type='text/css'> button#go {background-color: #ccc; padding: 5px 20px 5px 20px; border: none; -webkit-border-radius: 5px; -moz-border-radius: 5px; border-radius: 5px;} button#go:hover {background-color: orange;} table, th, td {color: #fff; border-collapse: collapse; border:0px } .tdr {color: #0c0; height: 40px; text-align: right; vertical-align: middle; padding-right: 15px} html,body {background-color: #333; text-color: #ccc; font-family: 'Roboto Condensed',sans-serif,Arial,Tahoma,Verdana;} a:hover {color: #fff;} a { color: #ccc; text-decoration: underline;} ";
  HtmlSrc +=".b {border-top: 1px dotted #666;} .tooltip-text {visibility: hidden; position: absolute; z-index: 1; width: 300px; color: white; font-size: 12px; background-color: #DE3163; border-radius: 10px; padding: 10px 15px 10px 15px; } .hover-text:hover .tooltip-text { visibility: visible; } #right { top: -30px; left: 200%; } #top { top: -60px; left: -150%; } #left { top: -8px; right: 120%;}";
  HtmlSrc +=".hover-text {position: relative; background: #888; padding: 5px 12px; margin: 5px; font-size: 15px; border-radius: 100%; color: #FFF; display: inline-block; text-align: center; }</style>\n";
  HtmlSrc +="<link href='http://fonts.googleapis.com/css?family=Roboto+Condensed:300italic,400italic,700italic,400,700,300&subset=latin-ext' rel='stylesheet' type='text/css'></head><body>\n";
  HtmlSrc +="<H1 style='color: #666; text-align: center;'>Setup<br><span style='font-size: 50%;'>(MAC ";
  HtmlSrc +=MACString;
  HtmlSrc +="|FW ";
  HtmlSrc +=REV;
  HtmlSrc +="|HW ";
  HtmlSrc +=String(HardwareRev);
  HtmlSrc +=")</span><span style='color: #333;'>";
  HtmlSrc +=String(HWidValue);
  HtmlSrc +="</span></H1><div style='display: flex; justify-content: center;'><table><form action='/set' method='post' style='color: #ccc; margin: 50 0 0 0; text-align: center;'>\n";
  HtmlSrc +="<tr class='b'><td class='tdr'><label for='yourcall'>MQTT topic:</label></td><td><input type='text' id='yourcall' name='yourcall' size='10' value='";
  HtmlSrc += YOUR_CALL;
  HtmlSrc +="'><span style='color:red;'>";
  HtmlSrc += yourcallERR;
  HtmlSrc +="</span><span class='hover-text'>?<span class='tooltip-text' id='top' style='width: 200px;'>Used as part of an MQTT topic</span></td></tr>\n<tr><td class='tdr'><label for='rotid'>Device ID:</label></td><td><input type='text' id='rotid' name='rotid' size='2' value='";
  HtmlSrc += NET_ID;
  HtmlSrc +="'><span style='color:red;'>";
  HtmlSrc += rotidERR;
  HtmlSrc +="</span><span class='hover-text'>?<span class='tooltip-text' id='top' style='width: 300px;'>[1-2 chars]<br>Multiple device with the same TOPIC must have different IDs<br>Second part of MQTT topic</span></span></td></tr>\n";

  HtmlSrc +="<tr class='b'><td class='tdr'><label for='baud'>RS485 (USB-C) BAUDRATE:</label></td><td><select name='baud' id='baud'><option value='0'";
  HtmlSrc += baudSELECT0;
  HtmlSrc +=">1200</option><option value='1'";
  HtmlSrc += baudSELECT1;
  HtmlSrc +=">2400</option><option value='2'";
  HtmlSrc += baudSELECT2;
  HtmlSrc +=">4800</option><option value='3'";
  HtmlSrc += baudSELECT3;
  HtmlSrc +=">9600</option><option value='4'";
  HtmlSrc += baudSELECT4;
  HtmlSrc +=">115200</option></select><span class='hover-text'>?<span class='tooltip-text' id='top' style='width: 150px;'>Use for RS485 and serial terminal because share same output<br>Must restart after change</span></span></td></tr>\n";

  HtmlSrc +="<tr class='b'><td class='tdr'><label for='mqttip0'>MQTT broker IP:</label></td><td>";
  HtmlSrc +="<input type='text' id='mqttip0' name='mqttip0' size='1' value='" + String(mqtt_server_ip[0]) + "'>&nbsp;.&nbsp;<input type='text' id='mqttip1' name='mqttip1' size='1' value='" + String(mqtt_server_ip[1]) + "'>&nbsp;.&nbsp;<input type='text' id='mqttip2' name='mqttip2' size='1' value='" + String(mqtt_server_ip[2]) + "'>&nbsp;.&nbsp;<input type='text' id='mqttip3' name='mqttip3' size='1' value='" + String(mqtt_server_ip[3]) + "'>";
  HtmlSrc +="<span style='color:red;'>";
  HtmlSrc += mqttERR;
  HtmlSrc +="</span><span class='hover-text'>?<span class='tooltip-text' id='top' style='width: 250px;'>Default public broker 54.38.157.134<br>If the first digit is zero, MQTT is disabled</span></span></td></tr>\n";

  HtmlSrc +="<tr><td class='tdr'><label for='mqttport'>MQTT broker PORT:</label></td><td>";
  HtmlSrc +="<input type='text' id='mqttport' name='mqttport' size='2' value='" + String(MQTT_PORT) + "'>\n";
  HtmlSrc +="<span style='color:red;'>";
  HtmlSrc += mqttportERR;
  HtmlSrc +="</span><span class='hover-text'>?<span class='tooltip-text' id='top' style='width: 150px;'>Default public broker port 1883</span></span></td></tr>\n";

  HtmlSrc +="<tr><td class='tdr'><label for='mqtt_login'>Enable MQTT PASSWORD:</label></td><td><input type='checkbox' id='mqtt_login' name='mqtt_login' value='1' ${postData.mqtt_login?'checked':''} ";
  HtmlSrc += mqtt_loginCHECKED;
  HtmlSrc +="><span class='hover-text'>?<span class='tooltip-text' id='top' style='width: 150px;'>Enable login for<br>connect to MQTT broker<br>WARNING, does not support encryption!</span></span></td></tr>\n";
    HtmlSrc +="<tr><td class='tdr'><label for='mqttuser'><span";
    HtmlSrc += mqtt_userSTYLE;
    HtmlSrc += ">MQTT Login:</span></label></td><td><input type='text' id='mqttuser' name='mqttuser' size='10' value='";
    HtmlSrc += MQTT_USER;
    HtmlSrc +="' ";
    HtmlSrc += mqtt_loginDisable;
    HtmlSrc +="><span style='color:red;'>";
    HtmlSrc += mqtt_userERR;
    HtmlSrc +="</span><span class='hover-text'>?<span class='tooltip-text' id='top' style='width: 150px;'>Login Name max 10 character, for connect to MQTT broker</span></span></td></tr>\n";

    HtmlSrc +="<tr><td class='tdr'><label for='mqttpass'><span";
    HtmlSrc += mqtt_passSTYLE;
    HtmlSrc += ">MQTT Password:</span></label></td><td><input type='password' id='mqttpass' name='mqttpass' size='20' value='";
    HtmlSrc += MQTT_PASS;
    HtmlSrc +="' ";
    HtmlSrc += mqtt_loginDisable;
    HtmlSrc +="><span style='color:red;'>";
    HtmlSrc += mqtt_passERR;
    HtmlSrc +="</span><span class='hover-text'>?<span class='tooltip-text' id='top' style='width: 150px;'>Login Password max 20 character, for connect to MQTT broker</span></span></td></tr>\n";

  HtmlSrc +="<tr><td class='tdr'><label for='mqttport'>websocet MQTT PORT:</label></td><td>";
  HtmlSrc +="<input type='text' id='wsmqttport' name='wsmqttport' size='2' value='" + String(WS_MQTT_PORT) + "'>\n";
  HtmlSrc +="<span style='color:red;'>";
  HtmlSrc += wsmqttportERR;
  HtmlSrc +="</span><span class='hover-text'>?<span class='tooltip-text' id='top' style='width: 150px;'>websocket MQTT broker PORT - only for connection mqtt-wall web client. Default 1884</span></span></td></tr>\n";

  HtmlSrc +="<tr class='b'><td class='tdr'><label for='ext'>Configuration:</label></td><td><select name='ext' id='ext'><option value='0'";
  HtmlSrc += confSELECT0;
  HtmlSrc +=">DEFAULT</option><option value='1'";
  HtmlSrc += confSELECT1;
  HtmlSrc +=">01-GpioExtension</option><option value='2'";
  HtmlSrc += confSELECT2;
  HtmlSrc +=">02-C_measure</option><option value='3'";
  HtmlSrc += confSELECT3;
  HtmlSrc +=">03-SB_version</option><option value='4'";
  HtmlSrc += confSELECT4;
  HtmlSrc +=">04-TrxNetSwitch</option></select><span class='hover-text'>?<span class='tooltip-text' id='top' style='width: 150px;'>Version external module<br>Must restart after change</span></span></td></tr>\n";

  if(Configuration==4){
    HtmlSrc +="<tr><td class='tdr'><label for='trxport'>TrxNet UDP port:</label></td><td><input type='text' id='trxport' name='trxport' size='2' value='" + String(TRX_PORT) + "'><span class='hover-text'>?<span class='tooltip-text' id='top' style='width: 200px;'>Same port on all TrxNet peers. Use different ports to run separate networks. Default 5683</span></span></td></tr>\n";
  }

  if(Configuration==1){
    for (int i=0; i<6; i++){
      HtmlSrc +="<tr><td class='tdr'><label for='mqtt_login'>Gpio-";
      HtmlSrc += Configuration1pins[i];
      HtmlSrc +=" set as OUTPUT</label></td><td><input type='checkbox' id='mqtt_login' name='mqtt_login' value='1' ${postData.mqtt_login?'checked':''} ";
      HtmlSrc += mqtt_loginCHECKED;
      HtmlSrc +="><span class='hover-text'>?<span class='tooltip-text' id='top' style='width: 150px;'>Othervise set as INPUT<br>You MUST also switch the hardware jumper on the PCB!</span></span></td></tr>\n";
    }
  }

  HtmlSrc +="<tr class='b'><td class='tdr'></td><td><button id='go'>&#10004; Change & Restart</button></form>&nbsp; ";
  HtmlSrc +="</td></tr>\n";

  // HtmlSrc +="<tr><td class='tdr'></td><td style='height: 42px;'></td></tr>\n";
  // HtmlSrc +="<tr><td class='tdr'></td><td style='height: 42px;'></td></tr>";
  // HtmlSrc +="<tr><td class='tdr'><a href='/'><button id='go'>&#8617; Back to Control</button></a></td><td class='tdl'><a href='/cal' onclick=\"window.open( this.href, this.href, 'width=700,height=715,left=0,top=0,menubar=no,location=no,status=no' ); return false;\"><button id='go'>Calibrate &#8618;</button></a></td></tr>";
  HtmlSrc +="<tr><td class='tdr'></td><td class='tdl'><span style='color: #666;'>After change, refresh all other page for apply changes.</span><br><a href='https://remoteqth.com/w/' target='_blank'>More on Wiki &#10138;</a></td></tr>\n";
  HtmlSrc +="</body></html>\n";

  ajaxserver.send(200, "text/html", HtmlSrc); //Send web page
  // set restart after 3 second
  shouldRestart = true;
  restartTime = millis() + 3000;
  Serial.println("Interface will be restarted...");
}

// Capacity measurement
uint32_t measureChargeTime(int ignoreThreshold, int threshold) {
  // static int16_t buffer[BUF_SAMPLES];
  size_t bytes_read;

  // zkontrolujeme, zda vstupní napětí nepřesahuje ignoreThreshold
  int initial = analogRead(Gpi39Pin);
  if (initial > ignoreThreshold) {
    Serial.println("C-meause skip (raw input="+String(initial)+")");
    digitalWrite(Gpio2Pin, HIGH);
    return 0; // přeskočit měření
  }


  // Před měřením spustíme výstup LOW
  digitalWrite(Gpio2Pin, LOW);
  uint32_t start = micros();

  // Čteme DMA bloky dokud nenajdeme vzorek >= threshold nebo timeout
  while ((micros() - start) < measureTimeoutUs) {
    if (analogRead(Gpi39Pin) >= threshold) {
      uint32_t end = micros();
      digitalWrite(Gpio2Pin, HIGH);
      return end - start;
    }
  }

  // // Čteme DMA bloky dokud nenajdeme vzorek >= threshold nebo timeout
  // while ((micros() - start) < measureTimeoutUs) {
  //   esp_err_t res = i2s_read(I2S_NUM, buffer, BUF_SAMPLES * sizeof(int16_t), &bytes_read, portMAX_DELAY);
  //   if (res != ESP_OK) {
  //     Serial.printf("Chyba čtení DMA: %d\n", res);
  //     MqttPubString("C-measure-µs", "DMA read error: "+String(res), false);
  //     break;
  //   }
  //   int count = bytes_read / sizeof(int16_t);
  //   for (int i = 0; i < count; ++i) {
  //     int raw = buffer[i] & 0x0FFF;
  //     // if (i % 100 == 0) { // vypiš každý stý vzorek
  //     //   Serial.print("vzorek[");
  //     //   Serial.print(i);
  //     //   Serial.print("] = ");
  //     //   Serial.println(raw);
  //     // }
  //     if (raw >= threshold) {
  //       uint32_t end = micros();
  //       digitalWrite(Gpio2Pin, HIGH);
  //       i2s_stop(I2S_NUM);
  //       i2s_adc_disable(I2S_NUM);
  //       return end - start;
  //     }
  //   }
  // }

  // Timeout nebo chyba - zastavíme ADC-DMA a přepneme výstup HIGH
  // i2s_stop(I2S_NUM);
  // i2s_adc_disable(I2S_NUM);
  digitalWrite(Gpio2Pin, HIGH);
  return 0;
}


float casNaKapacituProcentaExpon(
  unsigned long namerenyCas,          // změřený čas v mikrosekundách
  unsigned long casProStoProcent,     // referenční čas v mikrosekundách (100 %)
  float odporOhm,                     // odpor v ohmech
  int urovenPrahu,                    // práh ADC (např. 3000)
  int rozsahADC                       // maximum ADC (např. 4095)
) {
  // Převod času na sekundy
  float t = namerenyCas / 1e6;
  float tref = casProStoProcent / 1e6;
  
  // Poměr prahu k plnému rozsahu (např. 3000/4095)
  float x = (float)urovenPrahu / (float)rozsahADC;
  if (x <= 0.0 || x >= 1.0) return 0.0; // ochrana před log(0) nebo log(záporného čísla)

  float lnOneMinusX = log(1.0 - x);

  // Výpočet kapacity podle exponenciály
  float C = -t / (odporOhm * lnOneMinusX);
  float Cref = -tref / (odporOhm * lnOneMinusX);

  if (Cref <= 0.0) return 0.0; // ochrana před dělením nulou

  float procento = (C / Cref) * 100.0;
  // Omez na rozumné hodnoty
  if (procento > 100.0) procento = 100.0;
  if (procento < 0.0) procento = 0.0;
  return procento;
}
