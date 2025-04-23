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
mosquitto_sub -v -h 192.168.1.200 -t 'OK1HRA/ROT/#'
mosquitto_sub -v -h 54.38.157.134 -t 'OK1HRA/1/ROT/#'

MQTT topic
mosquitto_pub -h 192.168.1.200 -t OK1HRA/0/ROT/Target -m '10'
mosquitto_pub -h 54.38.157.134 -t BD:2F/0/ROT/Target -m '10'
mosquitto_pub -h 54.38.157.134 -t 3D:D3/0/ROT/RxAzimuth -m '10'

TODO
- infrared photo
- rs485/mqtt proxy
- gpio to mqtt report
- mqtt to gpio control


Changelog:

IDE 1.8.19
Použití knihovny WiFi ve verzi 2.0.0 v adresáři: /home/dan/Arduino/hardware/espressif/esp32/libraries/WiFi
Použití knihovny EEPROM ve verzi 2.0.0 v adresáři: /home/dan/Arduino/hardware/espressif/esp32/libraries/EEPROM
Použití knihovny WebServer ve verzi 2.0.0 v adresáři: /home/dan/Arduino/hardware/espressif/esp32/libraries/WebServer
Použití knihovny Ethernet ve verzi 2.0.0 v adresáři: /home/dan/Arduino/hardware/espressif/esp32/libraries/Ethernet
Použití knihovny ESPmDNS ve verzi 2.0.0 v adresáři: /home/dan/Arduino/hardware/espressif/esp32/libraries/ESPmDNS
Použití knihovny ArduinoOTA ve verzi 2.0.0 v adresáři: /home/dan/Arduino/hardware/espressif/esp32/libraries/ArduinoOTA
Použití knihovny Update ve verzi 2.0.0 v adresáři: /home/dan/Arduino/hardware/espressif/esp32/libraries/Update
Použití knihovny AsyncTCP ve verzi 1.1.1 v adresáři: /home/dan/Arduino/libraries/AsyncTCP
Použití knihovny ESPAsyncWebServer ve verzi 1.2.3 v adresáři: /home/dan/Arduino/libraries/ESPAsyncWebServer
Použití knihovny FS ve verzi 2.0.0 v adresáři: /home/dan/Arduino/hardware/espressif/esp32/libraries/FS
Použití knihovny AsyncElegantOTA ve verzi 2.2.7 v adresáři: /home/dan/Arduino/libraries/AsyncElegantOTA
Použití knihovny PubSubClient ve verzi 2.8 v adresáři: /home/dan/Arduino/libraries/PubSubClient
Použití knihovny Wire ve verzi 2.0.0 v adresáři: /home/dan/Arduino/hardware/espressif/esp32/libraries/Wire

*/
//-------------------------------------------------------------------------------------------------------
const char* REV = "20250423";

// USED
const int HWidPin          = 34;  // analog
const int VoltagePoePin    = 35;  // analog
const int TermistorT1Pin   =  5;  // one wire

const int TermistorT2Pin   = 15;  // one wire
const int RS485ReDePin     = 16;  // out
const int USBdetectPin     = 36;  // in

// FREE GPIO
const int Gpi39Pin         = 39;  // analog in
const int Gpio33Pin        = 33;  // in/out (SDA/SBU1)
const int Gpio32Pin        = 33;  // in/out (SCL/SBU2)
const int Gpio14Pin        = 14;  // in/out
const int Gpio13Pin        = 13;  // in/out
const int Gpio12Pin        = 12;  // in/out
const int Gpio4Pin         =  4;  // in/out
const int Gpio0Pin         =  0;  // in/out (RTS)

// Variables
short HardwareRev = 99;
String YOUR_CALL = "";
String NET_ID = "";

// compile
#define DS18B20                     // external 1wire Temperature sensor
#define OTAWEB                      // enable upload firmware via web
#define ETHERNET                    // Enable ESP32 ethernet (DHCP IPv4)
#define ETH_ADDR 0
#define ETH_TYPE ETH_PHY_LAN8720
#define ETH_POWER 0 //12                // #define ETH_PHY_POWER 0 ./Arduino/hardware/espressif/esp32/variants/esp32-poe/pins_arduino.h
#define ETH_MDC 23                  // MDC pin17
#define ETH_MDIO 18                 // MDIO pin16
#define ETH_CLK ETH_CLOCK_GPIO17_OUT    // CLKIN pin5 | settings for ESP32 GATEWAY rev f-g
String MACString;
char MACchar[18];
// #define ETH_CLK ETH_CLOCK_GPIO0_OUT    // settings for ESP32 GATEWAY rev c and older
// ETH.begin(ETH_ADDR, ETH_POWER, ETH_MDC, ETH_MDIO, ETH_TYPE, ETH_CLK);
const char* ssid     = "";
const char* password = "";
//-------------------------------------------------------------------------------------------------------

#include "esp_adc_cal.h"

int HWidValue              = 0;
float TemperatureCelsiusDS18B20 = 0;
float TemperatureCelsius = 0;

#if defined(DS18B20)
  bool ExtTemp = true;
  #include <OneWire.h>
  #include <DallasTemperature.h>
  // Data wire is plugged into port 2 on the Arduino
  #define ONE_WIRE_BUS 5
  #define TEMPERATURE_PRECISION 10 // 9: ±0,5°C | 10: ±0,25°C | 11: ±0,125°C
  // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
  OneWire oneWire(TermistorT1Pin);
  // Pass our oneWire reference to Dallas Temperature.
  DallasTemperature sensors(&oneWire);
  // arrays to hold device addresses
  DeviceAddress insideThermometer, outsideThermometer;
  // Assign address manually. The addresses below will need to be changed
  // to valid device addresses on your bus. Device address can be retrieved
  // by using either oneWire.search(deviceAddress) or individually via
  // sensors.getAddress(deviceAddress, index)
  // DeviceAddress insideThermometer = { 0x28, 0x1D, 0x39, 0x31, 0x2, 0x0, 0x0, 0xF0 };
  // DeviceAddress outsideThermometer   = { 0x28, 0x3F, 0x1C, 0x31, 0x2, 0x0, 0x0, 0x2 };
#endif

const int AzimuthPin    = 39;  // analog
float AzimuthValue      = 0.0;
int Azimuth             = 0;
int AzimuthTarget       = 0;
int RxAzimuth           = 0;
int Status              = 0; // -3 PwmDwnCCW|-2 CCW|-1 PwmUpCCW|0 off|1 PwmUpCW|2 CW|3 PwmDwnCW
float VoltagePOE      = 0.0;
const float VoltageLimit = 11.5; // ! also change if( Number(this.responseText)<11.5){ in index.h file | (11.0) Voltage limit below which the control electronics is unstable
const int ReversePin    = 16;  //
const int PwmPin        = 4;   //

// HardwareRev 1
const int ACcwPin       = 2;
const int BrakePin      = 33;
// const int CwCcwButtPin  = 36;  // analog
const int CwInputPin    = 36;
const int CcwInputPin   = 5;
int CwCcwInputValue      = 0;
const int AZmasterPin   = 32;  // analog
int AZmaster            = 142;
int AZmasterValue       = 0;
const int LedRPin       = 15;
const int LedGPin       = 14;
const int LedBPin       = 0;
const int AZtwoWirePin  = 13;
const int AZpreampPin   = 12;

// setting PWM properties
const int PwmFreq = 1000;
const int PwmResolution = 8;
const int mosfetPWMChannel = 0;
const int greenPWMChannel = 2;

// unsigned long TimerTemp;

// interrupts
#include "esp_attr.h"

// values
char key[100];
long MeasureTimer[2]={2800000,300000};   //  millis,timer (5 min)


int RainCount;
String RainCountDayOfMonth;
bool RainStatus;
/*
1mm rain = 15,7cm^2/10 = 1,57ml     <- by rain funnel radius
10ml = 11,5 pulses = 0,87ml/pulse   <- constanta tilting measuring cup
*/
float mmInPulse = 0.2 ; // callibration rain 17,7-20,2 mm with 95 pulse

int WindDir = 0;
int WindDirShift = 0;

long RpmTimer[2]={0,3000};
long RpmPulse = 987654321;
long PeriodMinRpmPulse = 987654321;
String PeriodMinRpmPulseTimestamp;
long MinRpmPulse;
String MinRpmPulseTimestamp;
unsigned int RpmSpeed = 0;
unsigned long RpmAverage[2]={1,0};  // counter,sum time
bool RpmInterrupt = false;

int SpeedAlertLimit_ms = 0;
int SpeedAlert_ms = 3000;
// bool NeedSpeedAlert_ms = false;
long AlertTimer[2]={0,60000};
//  |alert...........|alert........... everry max 1 minutes and with publish max value in period

// 73 seconds WDT (WatchDogTimer)
#include <esp_task_wdt.h>
#define WDT_TIMEOUT 73
long WdtTimer=0;

byte InputByte[21];
// #define Ser2net                  // Serial to ip proxy - DISABLE if board revision 0.3 or lower
#define EnableOTA                // Enable flashing ESP32 Over The Air
int NumberOfEncoderOutputs = 8;  // 2-16
int EnableSerialDebug     = 0;
long FreneticModeTimer ;
#define HTTP_SERVER_PORT  80     // Web server port
int IncomingSwitchUdpPort;
#define ShiftOut                 // Enable ShiftOut register
#define UdpAnswer                // Send UDP answer confirm packet
int BroadcastPort;               // destination broadcast packet port
bool EnableGroupPrefix = 0;      // enable multi controller control
bool EnableGroupButton = 0;      // group to one from
unsigned int GroupButton[8]={1,2,3,4,5,6,7,8};
byte DetectedRemoteSw[16][4];
unsigned int DetectedRemoteSwPort[16];

int BaudRate = 115200; // serial debug baudrate
int SERIAL1_BAUDRATE; // serial1 to IP baudrate
int incomingByte = 0;   // for incoming serial data

int i = 0;
#include <WiFi.h>
#include <WiFiUdp.h>
#include "EEPROM.h"
#define EEPROM_SIZE 267   /*

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
226-227 BaudRate
236-245 - MQTT_USER
246-265 - MQTT_PASS

!! Increment EEPROM_SIZE #define !!

*/
unsigned int OutputWatchdog;
unsigned long WatchdogTimer=0;

//ajax
#include <WebServer.h>
#include "index.h"  //Web page header file
#include "index-cal.h"  //Web page header file
WebServer ajaxserver(HTTP_SERVER_PORT+8);

WiFiServer server(HTTP_SERVER_PORT);
#if defined(CN3A)
  bool DHCP_ENABLE = 0;
#else
  bool DHCP_ENABLE = 1;
#endif
// Client variables
char linebuf[80];
int charcount=0;
//Are we currently connected?
boolean connected = false;
//The udp library class
WiFiUDP UdpCommand;
uint8_t buffer[50] = "";
unsigned char packetBuffer[10];
int UDPpacketSize;
byte TxUdpBuffer[8];
#include <ETH.h>
static bool eth_connected = false;
IPAddress RemoteSwIP(0, 0, 0, 0);         // remote UDP IP switch - set from UDP DetectRemote array
int RemoteSwPort         = 0;             // remote UDP IP switch port
String HTTP_req;
#if defined(EnableOTA)
  #include <ESPmDNS.h>
  #include <ArduinoOTA.h>
#endif
#if defined(OTAWEB)
  #include <AsyncTCP.h>
  #include <ESPAsyncWebServer.h>
  #include <AsyncElegantOTA.h>
  AsyncWebServer OTAserver(82);
#endif

#define MQTT               // Enable MQTT debug
#if defined(MQTT)
  #include <PubSubClient.h>
  // #include "PubSubClient.h" // lokalni verze s upravou #define MQTT_MAX_PACKET_SIZE 128
  // WiFiClient esp32Client;
  // PubSubClient mqttClient(esp32Client);
  WiFiClient espClient;
  PubSubClient mqttClient(espClient);
  // PubSubClient mqttClient(ethClient);
   // PubSubClient mqttClient(server, 1883, callback, ethClient);
   long lastMqttReconnectAttempt = 0;
#endif
boolean MQTT_ENABLE     = 1;          // enable public to MQTT broker
IPAddress mqtt_server_ip(0, 0, 0, 0);
// byte BrokerIpArray[2][4]{
//   // {192,168,1,200},   // MQTT broker remoteqth.com
//   {54,38,157,134},   // MQTT broker remoteqth.com
// };
// IPAddress server(10, 24, 213, 92);    // MQTT broker
int MQTT_PORT;       // MQTT broker PORT
// int MQTT_PORT_Array[2] = {
//   1883,
//   1883
// };       // MQTT broker PORT
boolean ELEVATION      = 0;          // enable Elevation function
boolean MQTT_LOGIN      = 0;          // enable MQTT broker login
String MQTT_USER= "";    // MQTT broker user login
String MQTT_PASS= "";   // MQTT broker password

const int MqttBuferSize = 1000; // 1000
char mqttTX[MqttBuferSize];
char mqttPath[MqttBuferSize];
// char mqttTX[100];
// char mqttPath[100];
long MqttStatusTimer[2]{1500,1000};
long HeartBeatTimer[2]={0,1000};

// Shift register
// CC1 12 CLOCK
// CC2 13 DATA
// SBU1 14 LATCH
// SBU2 15
// const int ShiftOutClockPin = 12;
// const int ShiftOutDataPin = 13;
// const int ShiftOutLatchPin = 14;
// byte ShiftOutByte=0x00;

bool rxShiftInRead;
// https://randomnerdtutorials.com/esp32-i2c-communication-arduino-ide/
#include <Wire.h>
#define I2C_SDA 33
#define I2C_SCL 32

#if defined(BMP280)||defined(HTU21D)||defined(SHT21)
  // #include <SPI.h>
  #include <Adafruit_Sensor.h>
  TwoWire I2Cone = TwoWire(0);
#endif

#if defined(BMP280)
  #include <Adafruit_BMP280.h>
  Adafruit_BMP280 bmp(&I2Cone); // use I2C interface
  Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
  Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();
  bool BMP280enable;
#endif

#if defined(HTU21D)
  #include "Adafruit_HTU21DF.h"
  Adafruit_HTU21DF htu = Adafruit_HTU21DF();
  bool HTU21Denable;
#endif

#if defined(SHT21)
  #include "SHT2x.h"
  SHT2x internal;
  // SHT2x external;
#endif

#if defined(SHT)
  #include "SHTSensor.h"
  // Sensor with normal i2c address
  // Sensor 1 with address pin pulled to GND
  SHTSensor sht1(SHTSensor::SHT3X);
  // Sensor with alternative i2c address
  // Sensor 2 with address pin pulled to Vdd
  // SHTSensor sht2(SHTSensor::SHT3X_ALT);
#endif

// https://github.com/PaulStoffregen/RadioHead
#if defined(RF69_EXTERNAL_SENSOR)
  bool RF69enable;
  #include <SPI.h>
  #include <RH_RF69.h>
  // Change to 434.0 or other frequency, must match RX's freq!
  #define RF69_FREQ 434.0
  #define RFM69_RST     -1   // same as LED
  #define RFM69_CS      0   // "B"
  #define RFM69_INT     16   // "A"

  // Singleton instance of the radio driver
  RH_RF69 rf69(RFM69_CS, RFM69_INT);

  int16_t packetnum = 0;  // packet counter, we increment per xmission
  String received_data;
  float humidity, temp_f, temp_f2, temp_PT100, temp_dallas;
  String temp_radio;
  String humidity_radio;
  String vbat_radio;

  uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
#endif

const int RpmPin = 39;
#if HWREV==8
  const int RainPin = 36;
#endif
#if HWREV==7
  const int Rain1Pin = 36;
  const int Rain2Pin = 35;
#endif
// const int EnablePin = 13;
// const int ButtonPin = 34;

#if defined(Ser2net)
  #define RX1 3
  #define TX1 1
  HardwareSerial Serial_one(1);
#endif

const int MappingRow = 5;
const long mapping[MappingRow][2] = { // ms > m/s
  {987654321,0},
  {120,1},
  {50,2},
  {2,4},
  {1,200},
};
// WX end

// SD
// #define ETH_CLK_MODE ETH_CLOCK_GPIO17_OUT
// #define ETH_PHY_POWER 12
// #include "FS.h"
// #include "SD_MMC.h"

// ntp
#include "time.h"
const char* ntpServer = "pool.ntp.org";
// const char* ntpServer = "tik.cesnet.cz";
// const char* ntpServer = "time.google.com";
const long  gmtOffset_sec = 0;
const int   daylightOffset_sec = 0;

#define MAX_SRV_CLIENTS 1
int SerialServerIPport;
// WiFiServer SerialServer(SerialServerIPport);
WiFiServer SerialServer;
WiFiClient SerialServerClients[MAX_SRV_CLIENTS];

int TelnetServerIPport = 23;
WiFiServer TelnetServer;
WiFiClient TelnetServerClients[MAX_SRV_CLIENTS];
IPAddress TelnetServerClientAuth;
bool TelnetAuthorized = false;
int TelnetAuthStep=0;
int TelnetAuthStepFails=0;
int TelnetLoginFails=0;
long TelnetLoginFailsBanTimer[2]={0,600000};
int RandomNumber;
bool FirstListCommands=true;

int CompareInt;

// APRS
WiFiClient AprsClient;
boolean AprsON = false;
uint16_t AprsPort;
IPAddress aprs_server_ip(0, 0, 0, 0);
String AprsPassword;
String AprsCoordinates;


//-------------------------------------------------------------------------------------------------------

void setup() {

  pinMode(HWidPin, INPUT);
    HWidValue = readADC_Cal(analogRead(HWidPin));
    if(HWidValue<=200){
      HardwareRev=0;  // 162
    }else if(HWidValue>200 && HWidValue<=450){
      HardwareRev=1;  // ??
    }
  pinMode(VoltagePoePin, INPUT);

  #if defined(DS18B20)
    sensors.begin();

    // locate devices on the bus
    Serial.print("DS18B20 found ");
    Serial.print(sensors.getDeviceCount(), DEC);
    Serial.println(" devices.");

    // report parasite power requirements
    //  Serial.print("Parasite power is: ");
    //  if (sensors.isParasitePowerMode()) Serial.println("ON");
    //  else Serial.println("OFF");

    // Search for devices on the bus and assign based on an index. Ideally,
    // you would do this to initially discover addresses on the bus and then
    // use those addresses and manually assign them (see above) once you know
    // the devices on your bus (and assuming they don't change).
    //
    // method 1: by index
    if (!sensors.getAddress(insideThermometer, 0)){
      ExtTemp = false;
      Serial.println("DS18B20 unable to find address for Device 0");
    }
    //  if (!sensors.getAddress(outsideThermometer, 1)) Serial.println("Unable to find address for Device 1");

    // method 2: search()
    // search() looks for the next device. Returns 1 if a new address has been
    // returned. A zero might mean that the bus is shorted, there are no devices,
    // or you have already retrieved all of them. It might be a good idea to
    // check the CRC to make sure you didn't get garbage. The order is
    // deterministic. You will always get the same devices in the same order
    //
    // Must be called before search()
    //oneWire.reset_search();
    // assigns the first address found to insideThermometer
    //if (!oneWire.search(insideThermometer)) Serial.println("Unable to find address for insideThermometer");
    // assigns the seconds address found to outsideThermometer
    //if (!oneWire.search(outsideThermometer)) Serial.println("Unable to find address for outsideThermometer");

    // show the addresses we found on the bus
    Serial.print("DS18B20 device 0 Address: ");
    printAddress(insideThermometer);
    Serial.println();

    //  Serial.print("Device 1 Address: ");
    //  printAddress(outsideThermometer);
    //  Serial.println();

    // set the resolution to 9 bit per device
    sensors.setResolution(insideThermometer, TEMPERATURE_PRECISION);
    //  sensors.setResolution(outsideThermometer, TEMPERATURE_PRECISION);

    Serial.print("DS18B20 device 0 Resolution: ");
    Serial.print(sensors.getResolution(insideThermometer), DEC);
    Serial.println();

    //  Serial.print("Device 1 Resolution: ");
    //  Serial.print(sensors.getResolution(outsideThermometer), DEC);
    //  Serial.println();
  #endif  

///////////

  pinMode(AzimuthPin, INPUT);
  // pinMode(CwCcwButtPin, INPUT);
  pinMode(CwInputPin, INPUT);
  pinMode(CcwInputPin, INPUT);

  pinMode(ReversePin, OUTPUT);
    digitalWrite(ReversePin, LOW);


  ledcSetup(mosfetPWMChannel, PwmFreq, PwmResolution);
  ledcSetup(greenPWMChannel, PwmFreq, PwmResolution);
  ledcAttachPin(PwmPin, mosfetPWMChannel);
  ledcAttachPin(LedGPin, greenPWMChannel);
  ledcWrite(mosfetPWMChannel, 0);
  ledcWrite(greenPWMChannel, 0);

  // HardwareRev 1
  pinMode(AZmasterPin, INPUT);
  pinMode(ACcwPin, OUTPUT);
    digitalWrite(ACcwPin, LOW);
  pinMode(BrakePin, OUTPUT);
    digitalWrite(BrakePin, LOW);
  pinMode(LedRPin, OUTPUT);
    digitalWrite(LedRPin, LOW);
  // pinMode(LedGPin, OUTPUT);
  //   digitalWrite(LedGPin, HIGH);
  pinMode(LedBPin, OUTPUT);
    digitalWrite(LedBPin, LOW);

  pinMode(AZtwoWirePin, OUTPUT);
    digitalWrite(AZtwoWirePin, LOW);
  pinMode(AZpreampPin, OUTPUT);
    digitalWrite(AZpreampPin, LOW);

  // pinMode(ShiftOutClockPin, OUTPUT);
  // pinMode(ShiftOutDataPin, OUTPUT);
  // pinMode(ShiftOutLatchPin, OUTPUT);
  // digitalWrite(ShiftOutLatchPin, HIGH);

  // for (int i = 0; i < 8; i++) {
  //   pinMode(TestPin[i], INPUT);
  // }
  #if HWREV==8
    pinMode(RainPin, INPUT);
  #endif
  #if HWREV==7
    pinMode(Rain1Pin, INPUT);
    pinMode(Rain2Pin, INPUT);
  #endif

  pinMode(RpmPin, INPUT);
  // pinMode(EnablePin, OUTPUT);
  // digitalWrite(EnablePin,1);

  // pinMode(ButtonPin, INPUT);
  // SHIFT IN
  // pinMode(ShiftInLatchPin, OUTPUT);
  // pinMode(ShiftInClockPin, OUTPUT);
  // pinMode(ShiftInDataPin, INPUT);

  Serial.begin(115200); //BaudRate
  while(!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("ETH DIN rail development kit");
  Serial.println("----------------------------");
  Serial.println("HW  "+String(HardwareRev));
  Serial.println("FW  "+String(REV));

  #if defined(DS18B20)
    sensors.begin();

    // locate devices on the bus
    Serial.print("DS18B20 found ");
    Serial.print(sensors.getDeviceCount(), DEC);
    Serial.println(" devices.");

    // report parasite power requirements
    //  Serial.print("Parasite power is: ");
    //  if (sensors.isParasitePowerMode()) Serial.println("ON");
    //  else Serial.println("OFF");

    // Search for devices on the bus and assign based on an index. Ideally,
    // you would do this to initially discover addresses on the bus and then
    // use those addresses and manually assign them (see above) once you know
    // the devices on your bus (and assuming they don't change).
    //
    // method 1: by index
    if (!sensors.getAddress(insideThermometer, 0)){
      ExtTemp = false;
      Serial.println("DS18B20 unable to find address for Device 0");
    }
    //  if (!sensors.getAddress(outsideThermometer, 1)) Serial.println("Unable to find address for Device 1");

    // method 2: search()
    // search() looks for the next device. Returns 1 if a new address has been
    // returned. A zero might mean that the bus is shorted, there are no devices,
    // or you have already retrieved all of them. It might be a good idea to
    // check the CRC to make sure you didn't get garbage. The order is
    // deterministic. You will always get the same devices in the same order
    //
    // Must be called before search()
    //oneWire.reset_search();
    // assigns the first address found to insideThermometer
    //if (!oneWire.search(insideThermometer)) Serial.println("Unable to find address for insideThermometer");
    // assigns the seconds address found to outsideThermometer
    //if (!oneWire.search(outsideThermometer)) Serial.println("Unable to find address for outsideThermometer");

    // show the addresses we found on the bus
    Serial.print("DS18B20 device 0 Address: ");
    printAddress(insideThermometer);
    Serial.println();

    //  Serial.print("Device 1 Address: ");
    //  printAddress(outsideThermometer);
    //  Serial.println();

    // set the resolution to 9 bit per device
    sensors.setResolution(insideThermometer, TEMPERATURE_PRECISION);
    //  sensors.setResolution(outsideThermometer, TEMPERATURE_PRECISION);

    Serial.print("DS18B20 device 0 Resolution: ");
    Serial.print(sensors.getResolution(insideThermometer), DEC);
    Serial.println();

    //  Serial.print("Device 1 Resolution: ");
    //  Serial.print(sensors.getResolution(outsideThermometer), DEC);
    //  Serial.println();
  #endif

  #if defined(BMP280)
    // I2Cone.begin(0x76, I2C_SDA, I2C_SCL, 100000); // SDA pin, SCL pin, 100kHz frequency
    // I2Cone.begin(I2C_SDA, I2C_SCL, (uint32_t)100000); // SDA pin, SCL pin, 100kHz frequency
    Serial.print("BMP280 sensor init ");
    if(!bmp.begin(0x76)){
      Serial.println("failed!");
      // while (1) delay(10);
      BMP280enable=false;
    }else{
      Serial.println("OK");
      BMP280enable=true;
      /* Default settings from datasheet. */
      bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
        Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
        Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
        Adafruit_BMP280::FILTER_X16,      /* Filtering. */
        Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
        bmp_temp->printSensorDetails();
    }
  #endif

  #if defined(HTU21D) || defined(SHT)
    // Wire.begin(I2C_SDA, I2C_SCL);
  #endif
  #if defined(HTU21D)
    Serial.print("HTU21D sensor init ");
    if(!htu.begin()){
      Serial.println("failed!");
      // while (1);
      HTU21Denable=false;
    }else{
      Serial.println("OK");
      HTU21Denable=true;
    }
  #endif

  #if defined(SHT21)
    Serial.println(__FILE__);
    Serial.print("SHT2x_LIB_VERSION: \t");
    Serial.println(SHT2x_LIB_VERSION);

    internal.begin(&I2Cone);
    // external.begin(&I2Ctwo);

    uint8_t stat = internal.getStatus();
    Serial.print(stat, HEX);
    Serial.println();
    // stat = external.getStatus();
    // Serial.print(stat, HEX);
    // Serial.println();
    Serial.println();
  #endif

  #if !defined(BMP280) && !defined(HTU21D)
    // Wire.begin(I2C_SDA, I2C_SCL);
  #endif

  // // SD
  // if(!SD_MMC.begin()){
  //   Serial.println("SD card Mount Failed");
  //   // return;
  // }

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


  SERIAL1_BAUDRATE=EEPROM.readInt(14);
  SerialServerIPport=EEPROM.readInt(18);
  IncomingSwitchUdpPort=EEPROM.readInt(22);
  BroadcastPort=IncomingSwitchUdpPort;
  OutputWatchdog=EEPROM.readUInt(30);
  if(OutputWatchdog>10080){
    OutputWatchdog=0;
  }
  TelnetServerClientAuth[0]=EEPROM.readByte(37);
  TelnetServerClientAuth[1]=EEPROM.readByte(38);
  TelnetServerClientAuth[2]=EEPROM.readByte(39);
  TelnetServerClientAuth[3]=EEPROM.readByte(40);

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

  if(EEPROM.read(164)==0xff){
    MQTT_PORT=1883;
  }else{
    MQTT_PORT = EEPROM.readUShort(165);
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
    UdpCommand.begin(IncomingSwitchUdpPort);    // incoming udp port
    // chipid=ESP.getEfuseMac();//The chip ID is essentially its MAC address(length: 6 bytes).
    //   unsigned long long1 = (unsigned long)((chipid & 0xFFFF0000) >> 16 );
    //   unsigned long long2 = (unsigned long)((chipid & 0x0000FFFF));
    //   ChipidHex = String(long1, HEX) + String(long2, HEX); // six octets
    //   YOUR_CALL=ChipidHex;

  #if defined(EnableOTA)
    // Port defaults to 3232
    // ArduinoOTA.setPort(3232);
    // Hostname defaults to esp3232-[MAC]

    // String StringHostname = "WX-station-"+String(NET_ID, HEX);
    String StringHostname = "ROT-"+String(YOUR_CALL);
    char copy[13];
    StringHostname.toCharArray(copy, 13);

    ArduinoOTA.setHostname(copy);
    ArduinoOTA.setPassword("remoteqth");
    // $ echo password | md5sum
    // ArduinoOTA.setPasswordHash("5587ba7a03b12a409ee5830cea97e079");
    ArduinoOTA
      .onStart([]() {
        esp_task_wdt_reset();
        WdtTimer=millis();

        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
          type = "sketch";
        else // U_SPIFFS
          type = "filesystem";

        // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
        Serial.println("Start updating " + type);
      })
      .onEnd([]() {
        Serial.println("\nEnd");
      })
      .onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
      })
      .onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");
      });

    ArduinoOTA.begin();
  #endif
  #if defined(OTAWEB)
    OTAserver.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(200, "text/plain", "PSE QSY to /update");
    });
    AsyncElegantOTA.begin(&OTAserver);    // Start ElegantOTA
    OTAserver.begin();
  #endif

  #if defined(Ser2net)
    Serial_one.begin(SERIAL1_BAUDRATE, SERIAL_8N1, RX1, TX1);
  // Serial2.begin(9600);
  SerialServer.begin(SerialServerIPport);
  SerialServer.setNoDelay(true);
  #endif

  TelnetServer.begin(TelnetServerIPport);
  // TelnetlServer.setNoDelay(true);


  #if HWREV==8
    int intBuf = analogRead(RainPin);
    if(intBuf<1000){
      RainStatus=false;
    }else if(intBuf>=1000 && intBuf<=2000){
      RainStatus=true;
    }
  #endif
  #if HWREV==7
    if(digitalRead(Rain1Pin)==0 && digitalRead(Rain2Pin)==1){
      RainStatus=false;
    }else if(digitalRead(Rain1Pin)==1 && digitalRead(Rain2Pin)==0){
      RainStatus=true;
    }
  #endif

  #if defined(RF69_EXTERNAL_SENSOR)
   // clock, miso,mosi, ss
    SPI.begin(14, 15, 2, 0);
    //while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer

    Serial.println();
    Serial.print("RFM69 radio init ");
    if (!rf69.init()) {
      Serial.println("failed!");
      RF69enable = false;
      // while (1);
    }else{
      Serial.println("OK");
      RF69enable = true;

      // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
      // No encryption
      Serial.print("RFM69 setFrequency ");
      if (!rf69.setFrequency(RF69_FREQ)) {
        Serial.println("failed!");
      }else{
        Serial.println("OK");
        // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
        // ishighpowermodule flag set like this:
        rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

        // The encryption key has to be the same as the one in the server
        uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
          0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08
        };
        rf69.setEncryptionKey(key);
        Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");
      }
    }

  #endif
  //------------------------------------------------

  // digitalWrite(EnablePin,0);

  // WDT
  esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL); //add current thread to WDT watch
  WdtTimer=millis();

  //init and get the time
   configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
   RainCountDayOfMonth=UtcTime(2);

   // ajax
   ajaxserver.on("/set", handleSet);
   ajaxserver.begin();                  //Start server
   Serial.println("HTTP ajax server started");

}

//-------------------------------------------------------------------------------------------------------

void loop() {
  http();
  Mqtt();
  // CLI();
  CLI2();
  ajaxserver.handleClient();
  Watchdog();

  
  #if defined(EnableOTA)
   ArduinoOTA.handle();
  #endif

  #if defined(OTAWEB)
   // OTAserver.on("/printIp", HTTP_GET, [](AsyncWebServerRequest *request){
   //     request->send(200, "text/plain", "ok");
   //     Serial.println(request->client()->remoteIP());
   // });
   AsyncElegantOTA.loop();
  #endif
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

  // function to print the temperature for a device
  void printTemperature(DeviceAddress deviceAddress)
  {
    float tempC = sensors.getTempC(deviceAddress);
    if(tempC == DEVICE_DISCONNECTED_C)
    {
      Serial.println("DS18B20 Error: Could not read temperature data");
      return;
    }
    // Serial.print("Temp C: ");
    Serial.print(tempC);
  //  Serial.print(" Temp F: ");
  //  Serial.print(DallasTemperature::toFahrenheit(tempC));
  }

  // function to print a device's resolution
  void printResolution(DeviceAddress deviceAddress)
  {
    Serial.print("Resolution: ");
    Serial.print(sensors.getResolution(deviceAddress));
    Serial.println();
  }

  // main function to print information about a device
  void printData(DeviceAddress deviceAddress)
  {
    Serial.print("DS18B20 address: ");
    printAddress(deviceAddress);
    Serial.print(" ");
    printTemperature(deviceAddress);
    Serial.println();
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
  static float VoltageBuffer = 0;
  static long ADCTimer = 0;
  static long ADCCounter = 0;
  static long OneWireTimer = 0;
  if(millis()-ADCTimer > 5){
    // R divider | 12,95/2,95=4,38983050847458
    VoltageBuffer = VoltageBuffer + readADC_Cal(analogRead(VoltagePoePin))/1000.0*4.39;
    ADCCounter ++;
    if(ADCCounter > 34){
      VoltagePOE = VoltageBuffer/35;
      ADCCounter = 0;
      VoltageBuffer = 0;
    }
    ADCTimer=millis();
  }

  static float VoltagePOETmp = 0;
  // info if change voltage
  if(abs(VoltagePOETmp-VoltagePOE)>0.5){
    MqttPubString("VoltagePOE", String(VoltagePOE), false);
    VoltagePOETmp=VoltagePOE;
  }

  // WDT
  if(millis()-WdtTimer > 60000){
    esp_task_wdt_reset();
    WdtTimer=millis();
    if(EnableSerialDebug>0){
      Prn(3, 0,"WDT reset ");
      Prn(3, 1, UtcTime(1));
    }
  }

  #if defined(DS18B20)
    if(millis()-OneWireTimer > 20000){
      OneWireTimer = millis();
      sensors.requestTemperatures();
        // float temperatureC = sensors.getTempCByIndex(0);
        TemperatureCelsiusDS18B20 = sensors.getTempC(insideThermometer);
        TemperatureCelsius = TemperatureCelsiusDS18B20;
        MqttPubString("Temperature-Celsius-T1", String(TemperatureCelsiusDS18B20), false);
    }
  #endif

  if(!TelnetServerClients[0].connected() && FirstListCommands==false){
    FirstListCommands=true;
  }
}

//-------------------------------------------------------------------------------------------------------
void CLI2(){
  int OUT=2;
  if (Serial.available() > 0) {
    incomingByte = Serial.read();
    OUT = 0;
  }
  esp_task_wdt_reset();
  WdtTimer=millis();

  // ? H h
  if(incomingByte==63 || incomingByte==72 || incomingByte==104){
    Prn(OUT, 1, "http://"+String(ETH.localIP()[0])+"."+String(ETH.localIP()[1])+"."+String(ETH.localIP()[2])+"."+String(ETH.localIP()[3]) );

  // CR/LF
  }else if(incomingByte==13||incomingByte==10){
    // Prn(OUT, 1,"");

  // anykey
  }else{
    // Prn(OUT, 0," [");
    // Prn(OUT, 0, String(incomingByte) ); //, DEC);
    // Prn(OUT, 1,"] unknown command");
  }
  incomingByte=0;
}


//-------------------------------------------------------------------------------------------------------
void Prn(int OUT, int LN, String STR){
  if(OUT==3){
    if(TelnetAuthorized==true){
      OUT=1;
    }else{
      OUT=0;
    }
  }

  if(OUT==0){
    Serial.print(STR);
    if(LN==1){
      Serial.println();
    }
  }else if(OUT==1){
    size_t len = STR.length()+1;
    // uint8_t sbuf[len];
    char sbuf[len];
    STR.toCharArray(sbuf, len);
    //push data to all connected telnet clients
    for(i = 0; i < MAX_SRV_CLIENTS; i++){
      if (TelnetServerClients[i] && TelnetServerClients[i].connected()){
        TelnetServerClients[i].write(sbuf, len);
        // delay(1);
        if(LN==1){
          TelnetServerClients[i].write(13); // CR
          TelnetServerClients[i].write(10); // LF
        }
      }
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
          webClient.println(":1884/\",");
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
          webClient.println(F("/ROT/#\","));
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
            webClient.print(F(":88/set\" onclick=\"window.open( this.href, this.href, 'width=620,height=570,left=0,top=0,menubar=no,location=no,status=no' ); return false;\"><button style='color: #fff; background-color: #060; padding: 5px 20px 5px 20px; margin:15px; border: none; -webkit-border-radius: 5px; -moz-border-radius: 5px; border-radius: 5px;} :hover {background-color: orange;} '>SETUP</button></a>"));
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
          webClient.println(F("                      | <a href=\"https://remoteqth.com/w/\" target=\"_blank\">ROT Wiki</a>."));
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
     MeasureTimer[0]=millis()+5000-MeasureTimer[1];
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
        Serial.print("FULL_DUPLEX, ");
      }
      Serial.print(ETH.linkSpeed());
      Serial.println("Mbps");
      eth_connected = true;

      #if defined(MQTT)
        if(MQTT_ENABLE == true){
          Serial.print("EthEvent-mqtt ");
          mqttClient.setServer(mqtt_server_ip, MQTT_PORT);
          mqttClient.setCallback(MqttRx);
          lastMqttReconnectAttempt = 0;
          // Check if this is a cold start
          // If so then read in YOUR_CALL from EEPROM
          if (YOUR_CALL.isEmpty()){
            // EEPROM YOUR_CALL
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

          char charbuf[50];
           // // memcpy( charbuf, ETH.macAddress(), 6);
           // ETH.macAddress().toCharArray(charbuf, 18);
           // // charbuf[6] = 0;
          if(MQTT_LOGIN == true){
            if (mqttClient.connect(MACchar,MQTT_USER.c_str(),MQTT_PASS.c_str())){
              Prn(0, 1, String(MACchar));
              mqttReconnect();
              AfterMQTTconnect();
              Prn(0, 1, "http://"+String(ETH.localIP()[0])+"."+String(ETH.localIP()[1])+"."+String(ETH.localIP()[2])+"."+String(ETH.localIP()[3]) );
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
                Prn(0, 1, "http://"+String(ETH.localIP()[0])+"."+String(ETH.localIP()[1])+"."+String(ETH.localIP()[2])+"."+String(ETH.localIP()[3]) );
              }
            }
          }else{
            if (mqttClient.connect(MACchar)){
              Prn(0, 1, String(MACchar));
              mqttReconnect();
              AfterMQTTconnect();
              Prn(0, 1, "http://"+String(ETH.localIP()[0])+"."+String(ETH.localIP()[1])+"."+String(ETH.localIP()[2])+"."+String(ETH.localIP()[3]) );
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
                Prn(0, 1, "http://"+String(ETH.localIP()[0])+"."+String(ETH.localIP()[1])+"."+String(ETH.localIP()[2])+"."+String(ETH.localIP()[3]) );
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
        if(EnableSerialDebug>0){
          Prn(3, 1, "Attempt to MQTT reconnect | "+String(millis()/1000) );
        }
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
  // Prn(3, 0, "MQTT");
  char charbuf[50];
  // // memcpy( charbuf, ETH.macAddress(), 6);
  // ETH.macAddress().toCharArray(charbuf, 18);
  // charbuf[6] = 0;
  if(MQTT_LOGIN == true){
    if (mqttClient.connect(MACchar,MQTT_USER.c_str(),MQTT_PASS.c_str())){
      if(EnableSerialDebug>0){
        Prn(3, 0, "mqttReconnect-connected");
      }
      reSubscribe();
    }
  }else{
    if (mqttClient.connect(MACchar)) {
      if(EnableSerialDebug>0){
        Prn(3, 0, "mqttReconnect-connected");
      }
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
    String topic = String(YOUR_CALL) + "/" + String(NET_ID) + "/ROT/Target";
    const char *cstr = topic.c_str();
    if(mqttClient.subscribe(cstr)==true){
      if(EnableSerialDebug>0){
        Prn(3, 1, " > subscribe "+String(cstr));
      }
    }
    topic = String(YOUR_CALL) + "/" + String(NET_ID) + "/ROT/get";
    const char *cstr1 = topic.c_str();
    if(mqttClient.subscribe(cstr1)==true){
      if(EnableSerialDebug>0){
        Prn(3, 1, " > subscribe "+String(cstr1));
      }
    }
    topic = String(YOUR_CALL) + "/" + String(NET_ID) + "/ROT/stop";
    const char *cstr2 = topic.c_str();
    if(mqttClient.subscribe(cstr2)==true){
      if(EnableSerialDebug>0){
        Prn(3, 1, " > subscribe "+String(cstr2));
      }
    }
    topic = String(YOUR_CALL) + "/" + String(NET_ID) + "/ROT/RxAzimuth";
    const char *cstr3 = topic.c_str();
    if(mqttClient.subscribe(cstr3)==true){
      if(EnableSerialDebug>0){
        Prn(3, 1, " > subscribe "+String(cstr3));
      }
    }
}

//------------------------------------------------------------------------------------
void MqttRx(char *topic, byte *payload, unsigned int length) {
  String CheckTopicBase;
  CheckTopicBase.reserve(100);
  byte* p = (byte*)malloc(length);
  memcpy(p,payload,length);
  // static bool HeardBeatStatus;
  if(EnableSerialDebug>0){
    Prn(3, 0, "RX MQTT ");
  }

    // // RxAzimuth
    // CheckTopicBase = String(YOUR_CALL) + "/" + String(NET_ID) + "/ROT/RxAzimuth";
    // if ( CheckTopicBase.equals( String(topic) ) ){
    //   RxAzimuth = 0;
    //   unsigned long exp = 1;
    //   for (int i = length-1; i >=0 ; i--) {
    //     // Numbers only
    //     if(p[i]>=48 && p[i]<=58){
    //       RxAzimuth = RxAzimuth + (p[i]-48)*exp;
    //       exp = exp*10;
    //     }
    //   }
    //   if(AZsource == 2){ //MQTT
    //     Azimuth=RxAzimuth;
    //   }
    //     Prn(3, 1, "/RxAzimuth " + String(RxAzimuth));
    // }

} // MqttRx END

//-------------------------------------------------------------------------------------------------------
void AfterMQTTconnect(){
  #if defined(MQTT)
  //    if (mqttClient.connect("esp32gwClient", MQTT_USER, MQTT_PASS)) {          // public IP addres to MQTT
        IPAddress IPlocalAddr = ETH.localIP();                           // get
        String IPlocalAddrString = String(IPlocalAddr[0]) + "." + String(IPlocalAddr[1]) + "." + String(IPlocalAddr[2]) + "." + String(IPlocalAddr[3]);   // to string
        IPlocalAddrString.toCharArray( mqttTX, 50 );                          // to array
        String path2 = String(YOUR_CALL) + "/" + String(NET_ID) + "/ROT/ip";
        path2.toCharArray( mqttPath, 100 );
        mqttClient.publish(mqttPath, mqttTX, true);
          Serial.print("MQTT-TX ");
          Serial.print(mqttPath);
          Serial.print(" ");
          Serial.println(mqttTX);

        // String MAClocalAddrString = ETH.macAddress();   // to string
        // MAClocalAddrString.toCharArray( mqttTX, 50 );                          // to array
        path2 = String(YOUR_CALL) + "/" + String(NET_ID) + "/ROT/mac";
        path2.toCharArray( mqttPath, 100 );
        mqttClient.publish(mqttPath, MACchar, true);
          Serial.print("MQTT-TX ");
          Serial.print(mqttPath);
          Serial.print(" ");
          Serial.println(MACchar);

    // MeasureTimer[0]=2800000;
    MeasureTimer[0]=millis()-MeasureTimer[1];

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
        String topic = String(YOUR_CALL) + "/" + String(NET_ID) + "/ROT/"+TOPIC;
        topic.toCharArray( mqttPath, 50 );
        DATA.toCharArray( mqttTX, 50 );
        mqttClient.publish(mqttPath, mqttTX, RETAIN);
      }
    }else{
      if (mqttClient.connect(MACchar)) {
        String topic = String(YOUR_CALL) + "/" + String(NET_ID) + "/ROT/"+TOPIC;
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
        char char_array[str_len];
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
        char char_array[str_len];
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
        char char_array[str_len];
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
        char char_array[str_len];
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

  HtmlSrc +="<tr class='b'><td class='tdr'></td><td><button id='go'>&#10004; Change</button></form>&nbsp; ";
  HtmlSrc +="</td></tr>\n";

  // HtmlSrc +="<tr><td class='tdr'></td><td style='height: 42px;'></td></tr>\n";
  // HtmlSrc +="<tr><td class='tdr'></td><td style='height: 42px;'></td></tr>";
  // HtmlSrc +="<tr><td class='tdr'><a href='/'><button id='go'>&#8617; Back to Control</button></a></td><td class='tdl'><a href='/cal' onclick=\"window.open( this.href, this.href, 'width=700,height=715,left=0,top=0,menubar=no,location=no,status=no' ); return false;\"><button id='go'>Calibrate &#8618;</button></a></td></tr>";
  HtmlSrc +="<tr><td class='tdr'></td><td class='tdl'><span style='color: #666;'>After change, refresh all other page for apply changes.</span><br><a href='https://remoteqth.com/w/' target='_blank'>More on Wiki &#10138;</a></td></tr>\n";
  HtmlSrc +="</body></html>\n";

  ajaxserver.send(200, "text/html", HtmlSrc); //Send web page
}