/*  Simple sketch by Petr KLOSKO (https://www.klosko.net)

  This sketch is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

*/
//#define DEBUG

// Define settings

/*


  !!! Compile w/ 4M ( 1M SPIFFS ) !!!  &
                Debug Level NONE      &
                Crystal Freq 26MHz    &
                DOUT(Compatible)      &
                BuildIn LED 1


  v.20180725-01:
    - based on v. 20180725-01 of esp8266weather_v1:
    - modify to BME280 temp, hum & press sensor
    - add max SSID+PASS length to /setting [webserver]

  v.20180729-01:
    - add MASL config to RTC + web
    - improve CRC32 of RTC data

  v.20180729-05:
    - remove Absolute Humidity from BLOB - backward compatibility to v1 RRD graphs
    - add SET_SEALEVEL_CMD command

  v.20180804-01:
    - cleaning code form comments and unused functions
    - production version, uploaded to WS

  v.20180904-01:
     - Compiled w/ ESP8266 2.4.2 API

  v.20180914-01:
     - Compiled w/ 1.8.7 IDE

  v.20181112-01-DEV: [temporary ver info - no production version !!!]
    - add LOW POWER flag / POWER SAVE funct
    - refactorize WEB pages
    - add SEND Config Data
    - add PUT Config Data = not web support yet !!!
    - fix SegFault Bug in str_pos / httpResponse = delay after HTTP Request + (httpResponse.length() > 0) Test
    - add GET-CONFIG functions
    - add PUT-CONFIG functions = still in TEST mode !!! Server /push/ script modification needed !!!
    - add ResetReason do HTTP Header [due to DEBUG]

  v.20181213-01
    - add readVcc compensation factor/ratio
      - change cfgData struct
      - web settings
      - show in 3 digits after decimal point
    - modify Ucc compensation to flags
    - add ForceConfig option/functio ( depend on function )
    - implement CLEAR-STATUS command

  v.20181221-01
    - modify GET HTTP Header - remove some values included in the cfgData
    - rename SET_CONFIG_CMD to NEW_CONFIG_CMD

  v.20190405-01
     - Compiled w/ ESP8266 2.5.0 API
     - Compiled w/ 1.8.9 IDE
     - BugFix in LOWBATT statuses [bitWrite(statusByte,5,((uc <= UTHR_WARN) || (bitGet(statusByte, 5) && (uc < UTHR_WAKE) )));]

  v.20201007-01
     - Compiled w/ ESP8266 2.7.4 API
     - Compiled w/ 1.8.13 IDE
     - use Adafruit BME Library

  v.20220702-01
     - Compiled w/ ESP8266 3.0.3 API
     - Compiled w/ 1.8.19 IDE
     - HTTP response timeout BUG fixes
     - compile w/ Dallas Lib v.1.3.7.
     - use Adafruit BME Library
     - add VEML6075 UVA & UVB sensor
     - nonos-sdk-2.2.1-111
     - add enableWiFiAtBootTime() to setup due to default WiFi disabled on  in and neweer 3.0.0

  v.20220711-11
     - Compile w/ new Adafruit BusIO [1.12.0] a BME280 [2.1.2] Lib.
     - UV-A negative values fix - preread values
     - read UV A, B, I in one reading readUVABI() + pre reading
     - VEML switch off
     - VEML VEML6075_50MS integration time
     - UV values debug in POST

  v.20220712-01
     - no UV debug values

*/

const char FWver[]     = "20220712-01";

// include libraries
#include <ESP8266WiFi.h>       // WiFi library
#include <OneWire.h>           // OneWire communication library for DS18B20
#include <DS2438.h> // DS18B20 library
#include <EEPROM.h>            // EEPROM library
#include <ESP8266WebServer.h>  // Web server library
#include <ESP8266httpUpdate.h> //Update libratry
#include <Wire.h>
//#include <BME280I2C.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_VEML6075.h>

#include <BH1750.h>
#include <math.h>
#include <EnvironmentCalculations.h>
#include <base64.h>            // Base64 library
extern "C" {
#include "libb64/cdecode.h"
}

// http timeout in milis
#define HTTP_TIMEOUT 5000


// Voltage / battery Thresholds

#define UTHR_WARN  3.10
#define UTHR_SLEEP 3.00
#define UTHR_WAKE  3.25


byte addr[8];
const char APssid[]    = "esp8266weather";     // WiFi SSID
const char APpass[]    = "826682668266";       // WiFi PASS
const char devID[]     = "esp8266-07";         // Device ID Prefix - for WEB
const char devType[]   = "esp8266-ESP-12F";    // Device Type
const char HWver[]     = "ESP-12Fv1";          // HW [DPS] version
const char host[]      = "iot.klosko.net";     // IoT Server

const byte  blikOK     = 50;                   // LED Blink interval OK[ms]
const int   blikERR    = 300;                  // LED Blink interval ERROR[ms]

//  ESP pins
const byte pinLED      = 2;                    // OUT, LED
const byte pinSETUP    = 14;                   // IN,  Setup switch
const byte pinCHRG     = 12;                   // IN,  Battery Charger Charging
const byte pinCHE      = 13;                   // OUT, Battery Charger ON/OFF
const byte pinSCL      = 0;                    // OUT, I2C SCL
const byte pinSDA      = 4;                    //      I2C SDA
const byte pin1Wire    = 5;                    //      1-Wire
const String SET_THRESHOLD_CMD = "SET-THRESHOLD<";
const String SET_INTERVAL_CMD  = "SET-INTERVAL<";
const String SET_SEALEVEL_CMD  = "SET-SEALEVEL<";
const String SET_NEWFW_CMD     = "SET-NEWFW<";
const String SET_SETUP_CMD     = "SET-SETUP<1>";
const String CLEAR_STATUS_CMD  = "CLEAR-STATUS<1>";
const String SET_FLIPFLOP_CMD  = "SET-FLIPFLOP<1>";
const String GET_CONFIG_CMD    = "GET-CONFIG<1>";
const String NEW_CONFIG_CMD    = "NEW-CONFIG<";

// Defult values
byte  sleepInterval    = 1;                    // How often send data to the server. In minutes
byte  statusByte       = 0;                    // Status byte
/*
    Status Register
               bit0 = (1)  = runMode  - 0 = Client, 1 = Setup AP
                  1 = (2)  = Last CMD - 0 = ERR,    1 = OK
                  2 = (4)  = Charger  - 0 = OFF,    1 = ON
                  3 = (8)  = OTAupdteStatus LSb
                  4 = (16) = OTAupdteStatus MSb =
                  5 = (32) = Low Batt Flag
                  6 = (64) = Power Save Flag
                  7 = (128) =
*/
float chargeThreshold  = 3.45;                 // Charge Battery Low Threshold
float solarThreshold   = 3.99;                 // Solar Threshold
float MASL             = 300.00;               // meters above seal level - for pressure calculation
byte  runMode          = 0;
byte mac[6];                                   // the MAC address of your Wifi shield


/* Define EEPROM addresses
*/

struct {
  uint32_t crc32;          // 4
  uint8_t  Status;         // 1
  uint8_t  Interval;       // 1
  uint16_t Threshold;      // 2
  uint16_t MASL;           // 2
  uint16_t VccRatio;       // 2
  uint8_t  SSID_ln;        // 1
  uint8_t  TOTAL_ln;       // 1 = 12
  byte     SSID_PASS[82];  // 82 = 96
} cfgData;

struct {
  uint32_t     crc32;      // 4
  uint32_t     SSIDcrc32;  // 4
  IPAddress    IP;         // 4
  IPAddress    GW;         // 4
  IPAddress    DNS;        // 4
  IPAddress    NET;        // 4
} cfgNetwork;


byte AddrDeviceStatus      = 0;
byte AddrDeviceConfig      = 4;
byte AddrDeviceWiFiCfg     = 16;
byte AddrDeviceCfgAddr     = 80;
byte eeSize                = 254;


// -------------------------------- VARIABLES ----------------------------------
String macAddr;
String dsAddr;
String SensType;                   // Sensor type
String ssid;
String pass;
String CfgFlag;
bool   NetCfgFlag;

float VccRatio         = 1.000;

// Weather values vars.
float bm_p = -127.00, bm_t = -127.00, bm_h = -127.00, uc = -127.00, ds_t = -127.00, ds_u = -127.00, ds_s = -127.00, lux = -127.00,
      dewPoint = -127.00, seaLevel = -127.00, absHum = -127.00, heatIndex = -127.00, uv_a = -127.00, uv_b = -127.00, uv_i = -127.00;

DeviceAddress deviceAddress;

// -------------------------------- OBJECTS AND HW SETTINGS ----------------------------------
OneWire oneWire(pin1Wire);         // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DS2438 ds2438(&oneWire);             //, DS2438_address);
ESP8266WebServer server(80);
//BME280I2C bme;
Adafruit_BME280 bme;
Adafruit_VEML6075 uv = Adafruit_VEML6075();
BH1750 lightMeter(0x23);

// setup ADC - read Vcc voltage
ADC_MODE(ADC_VCC);


// -------------------------------- MAIN SETUP ----------------------------------
void setup() {
#ifdef DEBUG
  //    delay(5000);
  initSerial();
#endif

  LoadConfig();

  runMode          = (statusByte & 1);

#ifdef DEBUG
  Serial.println();
  Serial.print(F("EEssid:")); Serial.println(ssid);
  Serial.print(F("EEpass:")); Serial.println(pass);
  Serial.print(F("EEinterval:")); Serial.println(sleepInterval);
  Serial.print(F("EEthreshold:")); Serial.println(chargeThreshold);
  Serial.print(F("EEmasl:")); Serial.println(MASL);
  Serial.print(F("EEstatus:")); Serial.println(statusByte, BIN);
  Serial.print(F("runMode:")); Serial.println(runMode, BIN);

  Serial.println();
  Serial.print(F("RTCssid:")); Serial.println(ssid);
  Serial.print(F("RTCpass:")); Serial.println(pass);
  Serial.print(F("RTCdevStatus:")); Serial.println(String(statusByte, BIN));
  Serial.print(F("RTCVccRatio:")); Serial.println(VccRatio, 3);
  Serial.print(F("CFGVccRatio:")); Serial.println(cfgData.VccRatio);

  Serial.print(F("DEVICEmac:")); Serial.println(WiFi.macAddress());
  Serial.print(F("getCoreVersion():")); Serial.println(ESP.getCoreVersion());
  Serial.print(F("getFullVersion():")); Serial.println(ESP.getFullVersion());
  Serial.print(F("getSdkVersion():")); Serial.println(ESP.getSdkVersion());
  Serial.print(F("CfgDataSize:")); Serial.println(sizeof(cfgData));

  String cfgDataStr = base64::encode((uint8_t*) &cfgData, sizeof(cfgData), false);

  Serial.print(F("cfgDataStr:")); Serial.println(cfgDataStr);

  int NewConfigLen = b64decode(cfgDataStr, (uint8_t*) &cfgData);

  Serial.print(F("NewConfigLen: ")); Serial.println(NewConfigLen);

  uint32_t crcOfData = calculateCRC32(((uint8_t*)&cfgData) + 4, sizeof(cfgData) - 4);
  if (crcOfData == cfgData.crc32) {
    Serial.print(F("CRC OK : ")); Serial.print(crcOfData); Serial.print(F(" = ")); Serial.println(cfgData.crc32);
  } else {
    Serial.print(F("CRC FAILED : ")); Serial.print(crcOfData); Serial.print(F(" <> ")); Serial.println(cfgData.crc32);
  }
  cfgDataStr = base64::encode((uint8_t*) &cfgData, sizeof(cfgData), false);
  Serial.print(F("cfgDataStr:")); Serial.println(cfgDataStr);


#endif

#ifdef WIFI_IS_OFF_AT_BOOT
  enableWiFiAtBootTime(); // can be called from anywhere with the same effect
#endif


#ifdef DEBUG
  Serial.println();
  Serial.println(F("POWERSAVE TEST"));
  //    ModifyConfig("SET-CONFIG<pNwAQAAUbQEkdxo5VVBDMTk2OTIxMjtkZWZhdWx0O0lvVG5ldDtGQ1hNRERIQTsxNjE2MTYxNjE2O3BQbWw0R1ZwWms7////////////////////////////////////>");
#endif
  if (VccRatio == 0) {
    VccRatio = 1;
  }
  uc = (ESP.getVcc() / 1024.00) * VccRatio;


#ifdef DEBUG
  Serial.print(F("Ucc:")); Serial.println(uc);
  Serial.print(F("Status:")); Serial.print(statusByte); Serial.print("  =  "); Serial.print(String(statusByte, BIN));
#endif

  if ( bitGetByte(statusByte, 96)) {
    if (uc < UTHR_WAKE) {
#ifdef DEBUG
      Serial.println(F("GO TO POWERSAVE ... WAKE_RF_DEFAULT"));
#endif
      if (!readPin(pinCHRG)) { // charger OFF
#ifdef DEBUG
        Serial.println(F("Battery Low, charger OFF - Switch charger ON"));
#endif
        FlipFlopPulse(pinCHE);
      }
      deepSleep(sleepInterval, false, WAKE_RF_DEFAULT);
    } else {
      bitClear(statusByte, 5);
      bitClear(statusByte, 6);
      storeRTCstatus(statusByte);
      SaveConfig(true);
#ifdef DEBUG
      Serial.println(F("REBOOT after POWERSAVE "));
#endif
      deepSleep(sleepInterval, true, WAKE_RF_DEFAULT);
    }
  }
  /*
      // set bit 5 - warning
      if (uc <= UTHR_WARN){
        bitWrite(statusByte,5,1);
        storeRTCstatus(statusByte);
        SaveConfig(true);
        #ifdef DEBUG
          Serial.print(F("UTHR_WARN"));
          Serial.print(F("RTCdevStatus:"));Serial.println(String(statusByte,BIN));
        #endif
      }
      // set bit 6 - sleep next run
      if (uc <= UTHR_SLEEP){
        bitWrite(statusByte,6,1);
        storeRTCstatus(statusByte);
        SaveConfig(true);
        #ifdef DEBUG
          Serial.print(F("UTHR_SLEEP"));
          Serial.print(F("RTCdevStatus:"));Serial.println(String(statusByte,BIN));
        #endif
      }
  */
  // set bit 5 - warning
  // set bit 6 - sleep next run
  // v.20190305
  // after battery charging during WARN state, a WARN bit not cleared =>
  //  BugFix from
  //    bitWrite(statusByte,5,((uc <= UTHR_WARN) || bitGet(statusByte, 5)));
  //  to
  bitWrite(statusByte, 5, ((uc <= UTHR_WARN) || (bitGet(statusByte, 5) && (uc < UTHR_WAKE) )));
  bitWrite(statusByte, 6, ((uc <= UTHR_SLEEP) || bitGet(statusByte, 6)));
  storeRTCstatus(statusByte);
  SaveConfig(true);

  // if status = 11010 = OTA Update OK and Last command OK
  // then clear ststus byte and sleep
  // This caused just after OTA Update Reset
  if ( bitGetByte(statusByte, 26)) {
#ifdef DEBUG
    Serial.println();
    Serial.print(F("Status byte = 26 :")); Serial.println(statusByte);
#endif
    bitClear(statusByte, 1);
    storeRTCstatus(statusByte);
    SaveConfig(false);
    deepSleep(sleepInterval, false, WAKE_RF_DEFAULT);
  }

  initPins();

#ifdef DEBUG
  Serial.println(F("initPins OK (setup)"));
#endif

  if (ssid == "" || sleepInterval == 255 || sleepInterval == 0   // after first power off
      || readPin(pinSETUP) == LOW || runMode == 1     // go to conf via PIN or HTTP-COMMAND ,,
      || ForceSetup() ) {                             // go to conf after Force [e.g. incompatibile FW update etc.]
    storeRTCstatus(bitClear(statusByte, 0)); // set runMode = 0
    SaveConfig(false);
#ifdef DEBUG
    Serial.println(F("Go to setupWiFiAP MODE"));
#endif
    setupWiFiAP();
  } else {
#ifdef DEBUG
    Serial.println(F("Go to weatherClient MODE"));
#endif
    weatherClient(uc);
  }
}

// -----------------------------  COMM FUNCT  --------------------------------------
bool ForceSetup() {
  // force strup AP mode
  return (VccRatio > 2);
}

void SplitText(String &ssid, int i, String &rest) {
  if (i > -1) {
    rest = ssid.substring(0, i);
    ssid = ssid.substring(i + 1, ssid.length());
  } else {
    ssid = rest;
    rest = "";
  }
}

// -------------------------------- MAIN WEATHER ----------------------------------
void weatherClient(float uc) {
  bool Conn = false;
  String sr = ssid, sp = pass;

  init1Wire();
  int i2cfl = initI2C(pinSDA, pinSCL);

  lightMeter.begin(BH1750_CONTINUOUS_HIGH_RES_MODE);

  int uvfl = uv.begin();
  if (uvfl) {
    int uvfl = 1;
    uv.setIntegrationTime(VEML6075_50MS);
    uv.setHighDynamic(true);
    uv.setForcedMode(true);
    uv.setCoefficients(2.22, 1.33,  // UVA_A and UVA_B coefficients
                       2.95, 1.74,  // UVB_C and UVB_D coefficients
                       0.001461, 0.002591); // UVA and UVB responses
//    float uv_x = uv.readUVI();
//    uv.readUVABI(&uv_a, &uv_b, &uv_i);
//    uv_a = uv.readUVA();
//    uv_b = uv.readUVB();
//    uv_i = uv.readUVI();                       
  } else {
#ifdef DEBUG
    Serial.println("Failed to communicate with VEML6075 sensor, check wiring?");
#endif
    uv.shutdown(true);
  }
  


  while ((ssid.length() > 0 ) && (Conn == false)) {
    SplitText(ssid, ssid.lastIndexOf(";"), sr);
    SplitText(pass, pass.lastIndexOf(";"), sp);
#ifdef DEBUG
    Serial.println();
    Serial.print(F("wifiMulti.conn(")); Serial.print(ssid); Serial.print(F(",")); Serial.print(pass); Serial.println(F(")"));
#endif
    Conn = initWiFiClient(ssid, pass);
    ssid = sr;  pass = sp;
  }
  if (Conn == false) {
#ifdef DEBUG
    Serial.println();
    Serial.print(F("WiFi Conn ERR - Go To Deep Sleep"));
#endif
    //    WiFi.setAutoConnect(false);
    deepSleep(sleepInterval, false, WAKE_RF_DEFAULT);
  } else {
    WiFi.setAutoConnect(true);
    if (!NetCfgFlag) {
      cfgNetwork.IP   = WiFi.localIP();
      cfgNetwork.NET  = WiFi.subnetMask();
      cfgNetwork.GW   = WiFi.gatewayIP();
      cfgNetwork.DNS  = WiFi.dnsIP();
#ifdef DEBUG
      Serial.println(F("Save IP CFG "));
#endif
      SaveNetConfig(WiFi.SSID(), sizeof(cfgData) + 4); /// YES, cfgData. its only an ADDRESS
    }

  }

#ifdef DEBUG
  Serial.println();
  Serial.println(F("WiFi connected"));
  Serial.print(F("IP address  : ")); Serial.println(WiFi.localIP());
  Serial.print(F("MAC address : ")); Serial.println(macAddr);
  Serial.print(F("RSSI        : ")); Serial.println(WiFi.RSSI());
  Serial.print(F("DS ID       : ")); Serial.println(dsAddr);
#endif

  if (SensType == "Unknown") {
    ds_t  = -127.00;
#ifdef DEBUG
    Serial.println(F("readTemerature: ")); Serial.println(ds_t);
#endif
    ds_s  = -127.00;
    ds_u  = -127.00;
  } else {
    ds_t  = readTemerature();
#ifdef DEBUG
    Serial.println(F("readTemerature: ")); Serial.println(ds_t);
#endif
    ds_s  = readVoltage(DS2438_CHA);
    ds_u  = readVoltage(DS2438_CHB);
  }
  // When 0V on ADC pin of DS2438, chip read 10.22 V
  if (ds_s > 10.00) {
    ds_s = 0;
  }

  //  BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
  //  BME280::PresUnit presUnit(BME280::PresUnit_hPa);

  if (i2cfl) {
    bme.takeForcedMeasurement();
    //bme.read(bm_p, bm_t, bm_h, tempUnit, presUnit);
    bm_p = bme.readPressure() / 100.0F;
    bm_t = bme.readTemperature();
    bm_h = bme.readHumidity();
  }
  if (uvfl) {
    float uv_x = uv.readUVI();
    uv.readUVABI(&uv_a, &uv_b, &uv_i);
//    uv_a = uv.readUVA();
//    uv_b = uv.readUVB();
//    uv_i = uv.readUVI();
    uv.shutdown(true);
  }

  EnvironmentCalculations::AltitudeUnit envAltUnit  =  EnvironmentCalculations::AltitudeUnit_Meters;
  EnvironmentCalculations::TempUnit     envTempUnit =  EnvironmentCalculations::TempUnit_Celsius;

  dewPoint  = EnvironmentCalculations::DewPoint(bm_t, bm_h, envTempUnit);
  seaLevel  = EnvironmentCalculations::EquivalentSeaLevelPressure(MASL, bm_t, bm_p, envAltUnit, envTempUnit);
  absHum    = EnvironmentCalculations::AbsoluteHumidity(bm_t, bm_h, envTempUnit);
  heatIndex = EnvironmentCalculations::HeatIndex(bm_t, bm_h, envTempUnit);

  lux = lightMeter.readLightLevel();

#ifdef DEBUG
  Serial.println(F("------------------------------------------------------------------------------"));
  Serial.println(millis());
  Serial.print(F("Temp: "));              Serial.print(bm_t); Serial.println("°"); //+ String(tempUnit == BME280::TempUnit_Celsius ? 'C' :'F'));
  Serial.print(F("Humidity: "));          Serial.print(bm_h); Serial.println("% RH");
  Serial.print(F("Pressure: "));          Serial.print(bm_p); Serial.println(); //String(presUnit == BME280::PresUnit_hPa ? "hPa" : "Pa"));
  Serial.print(F("Dew point: "));         Serial.print(dewPoint); Serial.println("°" + String(envTempUnit == EnvironmentCalculations::TempUnit_Celsius ? "C" : "F"));
  Serial.print(F("Sea Level Pressure: ")); Serial.print(seaLevel); Serial.println(); //String( presUnit == BME280::PresUnit_hPa ? "hPa" :"Pa"));
  Serial.print(F("Heat Index: "));        Serial.print(heatIndex); Serial.println("°" + String(envTempUnit == EnvironmentCalculations::TempUnit_Celsius ? "C" : "F"));
  Serial.print(F("Absolute Humidity: ")); Serial.println(absHum);
  Serial.print(F("UV A: "));              Serial.println(uv_a);
  Serial.print(F("UV B: "));              Serial.println(uv_a);
  Serial.print(F("UV Idx: "));            Serial.println(uv_a);
  Serial.print(F("Iluminance: "));        Serial.print(lux); Serial.println("lux");
  Serial.println(F(""));
#endif
  ChragrgeBatt(ds_s, ds_u);


  String httpResponse = DoHTTPrequest(ds_t, ds_u, ds_s,
                                      bm_t, bm_h, bm_p,
                                      dewPoint, heatIndex, seaLevel, absHum,
                                      uc, lux, uv_a, uv_b, uv_i, false);
#ifdef DEBUG
  Serial.println(F("httpResponse(client): ")); Serial.println(httpResponse);
#endif
  if (httpResponse.length() > 0) {
    ProcessHttpResponse(httpResponse);
  }
  deepSleep(sleepInterval, false, WAKE_RF_DEFAULT);
}

// -------------------------------- INIT functions BEGIN ----------------------------------
void initSerial() {
  // Start serial
  Serial.begin(115200);
  delay(10);
#ifdef DEBUG
  Serial.println(F("initSerial"));
#endif
}

int initI2C(byte SDA, byte SCL) {
  Wire.begin(SDA, SCL);
  if (!bme.begin(0x76, &Wire)) {
#ifdef DEBUG
    Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
    Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(), 16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    Serial.print(F("initI2C: SDA=.....")); Serial.println(SDA);
    Serial.print(F("initI2C: SCL=.....")); Serial.println(SCL);
#endif
    while (1);
    bme.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X1, // temperature
                    Adafruit_BME280::SAMPLING_X1, // pressure
                    Adafruit_BME280::SAMPLING_X1, // humidity
                    Adafruit_BME280::FILTER_OFF   );
    return 0;
  }

#ifdef DEBUG
  /*    switch(bme.chipModel())
      {
        case BME280::ChipModel_BME280:
          Serial.println(F("Found BME280 sensor! Success."));
          break;
        case BME280::ChipModel_BMP280:
          Serial.println(F("Found BMP280 sensor! No Humidity available."));
          break;
        default:
          Serial.println(F("Found UNKNOWN sensor! Error!"));
      }
  */
  Serial.print(F("initI2C: SDA=")); Serial.println(SDA);
  Serial.print(F("initI2C: SCL=")); Serial.println(SCL);
#endif
  return 1;
}

void init1Wire() {
  ds2438.begin(); // Initialize the DallasTemperature DS18B20 class (not strictly necessary with the client class, but good practice).
  ds2438.getAddress(deviceAddress, 0);
  dsAddr = printAddress(deviceAddress);
  SensType = getSensType(deviceAddress);
#ifdef DEBUG
  Serial.print(F("init1Wire: dsAddr=")); Serial.println(dsAddr);
  Serial.print(F("init1Wire: SensType=")); Serial.println(SensType);
#endif
}

bool initWiFiClient(String ssid, String pass) {
  // Connect to the WiFi
  //  WiFi.forceSleepWake();
  WiFi.mode(WIFI_STA);
  NetCfgFlag = LoadNetConfig(WiFi.SSID(), sizeof(cfgData) + 4);
  if (NetCfgFlag) {
#ifdef DEBUG
    Serial.println(F("Load IP CFG OK ... config"));
#endif
    WiFi.config(cfgNetwork.IP, cfgNetwork.GW, cfgNetwork.NET, cfgNetwork.DNS);
  }
  WiFi.macAddress(mac);
  macAddr = "";
  for (int i = 0; i < 6; i++) {
    if (mac[i] < 16) macAddr += "0";
    macAddr += String(mac[i], HEX);
  }
#ifdef DEBUG
  Serial.print(F("Connecting to ")); Serial.print(ssid); Serial.print(","); Serial.println(pass);
  Serial.print(F("getAutoConnect is se to  ")); Serial.println(WiFi.getAutoConnect());
  Serial.print(F("Connecting to  ")); Serial.println(WiFi.SSID());
#endif
  WiFi.begin(ssid.c_str(), pass.c_str());
  byte c = 0;
  byte a = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    c++;
    a++;
#ifdef DEBUG
    Serial.print(F("."));
#endif
    if (c == 25) {
#ifdef DEBUG
      Serial.printf("Connection status: %d\n", WiFi.status());
      Serial.println(macAddr);
#endif
      c = 0;
      //      WiFi.disconnect();
      //      delay(300);
      //      WiFi.begin(ssid.c_str(), pass.c_str());
    }
    if (a > 75) {
      return false;
    }
  }
  return (WiFi.status() == WL_CONNECTED);
}

void initPins() {
#ifdef DEBUG
  Serial.println(F("initPins"));
#endif

  pinMode(pinLED,   OUTPUT);
  digitalWrite(pinLED,   HIGH);

  pinMode(pinCHE,   OUTPUT);
  pinMode(pinSETUP, INPUT_PULLUP);
  pinMode(pinCHRG,  INPUT);
  //  pinMode(pinSTBY,  INPUT_PULLUP);

  // Set pind to HIGH + PULLUP INPIUT PINS
  //  digitalWrite(pinCHE,   HIGH);
  digitalWrite(pinSETUP, HIGH);
  digitalWrite(pinCHRG,  HIGH);
  //  digitalWrite(pinSTBY,  HIGH);

  delay(10);
}
// -------------------------------- INIT functions END ----------------------------------

// -------------------------------- Functions BEGIN ---------------------------------------

int pow2(int p) {
  return 1 << p;
}

bool bitGet(byte statusByte, byte position) {
  return ((statusByte & pow2(position)) && (statusByte != 255));
}

bool bitGetByte(byte statusByte, byte val) {
  return (((statusByte & val) == val) && (statusByte != 255));
}
// -------------------------------- Functions END ---------------------------------------

// -------------------------------- CHARGING functions BEGIN ----------------------------------
void ChragrgeBatt(float ds_s, float ds_u) {
  bool chStat = readPin(pinCHRG);

#ifdef DEBUG
  Serial.print(F("ChargeStatus: ")); Serial.println(chStat, BIN);
  Serial.println(F("Voltages: "));
  Serial.print(F("ds_s - SOLAR : ")); Serial.print(ds_s);
  Serial.print(F("              solarThreshold : ")); Serial.println(solarThreshold);
  Serial.print(F("ds_u - BATT: ")); Serial.print(ds_u);
  Serial.print(F("              chargeThreshold : ")); Serial.println(chargeThreshold);
#endif

  /*  if ( (ds_s < solarThreshold || (ds_u >= chargeThreshold) ) && chStat){  // Low Solar voltage OR Battery voltage OK - charging not allowed
      #ifdef DEBUG
        Serial.println(F("Low Solar voltage OR Battery voltage OK, charger ON - charging not allowed - Switch charger OFF"));
      #endif
      FlipFlopPulse(pinCHE);
    }
  */
  if (ds_s > solarThreshold && ds_u < chargeThreshold && !chStat) { // Solar voltage OK, Battery Low, charger OFF
#ifdef DEBUG
    Serial.println(F("Solar voltage OK, Battery Low, charger OFF - Switch charger ON"));
#endif
    FlipFlopPulse(pinCHE);
  }
  bitWrite(statusByte, 2, readPin(pinCHRG));
  storeRTCstatus(statusByte);
  SaveConfig(false);
}

void FlipFlopPulse(byte PIN) {
#ifdef DEBUG
  Serial.print(F("FlipFlopPulse : ")); Serial.println(PIN);
#endif
  digitalWrite(PIN,   LOW);
  delay(100);
  digitalWrite(PIN,   HIGH);
  delay(100);
  digitalWrite(PIN,   LOW);
}
// -------------------------------- CHARGING functions END ----------------------------------

// -------------------------------- SENSORS functions BEGIN ----------------------------------
String printAddress(uint8_t* deviceAddress) {
  String DsAddr = "";
  for (int i = 0; i < 8; i++) {
    if (deviceAddress[i] < 16) DsAddr += "0";
    DsAddr += String(deviceAddress[i], HEX);
  }
  return DsAddr;
}

String getSensType(uint8_t* deviceAddress) {
  if ( deviceAddress[0] == 0x10) {
    return "DS18S20";
  }
  else if ( deviceAddress[0] == 0x28) {
    return "DS18B20";
  }
  else if ( deviceAddress[0] == 0x26) {
    return "DS2438";
  }
  else {
    return "Unknown";
  }
}

float readTemerature() {
  float t;
  ds2438.update();
  if (ds2438.isError()) {
    t = -127.00;
  } else {
    t = ds2438.getTemperature();
  }
  return t;
}

float readVoltagePct(byte channel, byte base) {
  float u;
  ds2438.update();
  if (ds2438.isError()) {
    u = -127.00;
  } else {
    u = ds2438.getVoltagePct(channel, base);
  }
  return u;
}

float readVoltage(byte channel) {
  float u;
  ds2438.update();
  if (ds2438.isError()) {
    u = -127.00;
  } else {
    u = ds2438.getVoltage(channel);
  }
  return u;
}

// -------------------------------- SENSORS functions END ----------------------------------

// -------------------------------- HTTP CLIENT functions BEGIN ----------------------------------

String createBLOB(float t, float th_h, float bm_sl, float th_dp, float hi, float ds_u, float ds_s, float ds_t, float bm_p, float uc, float lux, float absHum, float uv_a, float uv_b, float uv_i, byte statusByte) {
  String blob = floatToHex2(t)      +
                floatToHex2(th_h)   +
                floatToHex3(bm_sl)  +
                floatToHex2(th_dp)  +
                floatToHex2(hi)     +
                floatToHex2(ds_u)   +
                floatToHex2(ds_s)   +
                floatToHex2(ds_t)   +
                floatToHex3(bm_p)   +
                floatToHex2(uc)     +
                floatToHex3(lux)    +
                floatToHex2(absHum) +
                floatToHex3(uv_a)   +
                floatToHex3(uv_b)   +
                floatToHex3(uv_i);
  return blob;
}

String byteToHex(byte value) {
  return (value < 16 ? "0" + String(value, HEX) : String(value, HEX));
}

String floatToHex2(float value) {
  String ret = "";
  byte flag = (value < 0 ? 1 : 0);
  int  val  = abs(int(value * 100));
  byte hi   = highByte(val);
  byte lo   = lowByte(val);
  bitWrite(hi, 7, flag);
  ret += (hi < 16 ? "0" + String(hi, HEX) : String(hi, HEX));
  ret += (lo < 16 ? "0" + String(lo, HEX) : String(lo, HEX));
  return ret;
}

String floatToHex3(float value) {
  String ret = "";
  byte flag = (value < 0 ? 1 : 0);
  int  val  = int(value * 100);
  byte mid  = highByte(val);
  byte lo   = lowByte(val);
  byte hi   = (lowByte(val >> 16));
  bitWrite(hi, 7, flag);
  ret += (hi < 16  ? "0" + String(hi, HEX)  : String(hi, HEX));
  ret += (mid < 16 ? "0" + String(mid, HEX) : String(mid, HEX));
  ret += (lo < 16  ? "0" + String(lo, HEX)  : String(lo, HEX));
  return ret;
}

String DoHTTPrequest(float ds_t, float ds_u, float ds_s,
                     float bm_t, float bm_h, float bm_p,
                     float dewPoint, float heatIndex, float seaLevel, float absHum,
                     float uc, float lux, float uv_a, float uv_b, float uv_i,
                     bool pushConfig) {
  String httpResponse = "";
  String cfgDataStr = "";

#ifdef DEBUG
  Serial.print(F("ds_t ")); Serial.println(ds_t);
#endif
  String blob = createBLOB(bm_t, bm_h, seaLevel, dewPoint, heatIndex, ds_u, ds_s, ds_t, bm_p, uc, lux, absHum, uv_a, uv_b, uv_i, statusByte);
#ifdef DEBUG
  Serial.print(F("BLOB ")); Serial.println(blob);
#endif

  WiFiClient client; // Use WiFiClient class to create TCP connections
#ifdef DEBUG
  Serial.print(F("Connecting to ")); Serial.println(host);
#endif

  // Connect to the HOST and send data via GET method
  if (!client.connect(host, 80)) {
#ifdef DEBUG
    Serial.println(F("Client connection failed"));
#endif
    httpResponse = "Client connection failed";
  } else {
#ifdef DEBUG
    Serial.println(F("Client connected"));
#endif

    String url = "/push/";

    if (pushConfig) {
      cfgDataStr = "CfgData: " + base64::encode((uint8_t*) &cfgData, sizeof(cfgData), false) + "\r\n";
      bitSet(statusByte, 1);
    }

#ifdef DEBUG
    Serial.print(F("Requesting URL: ")); Serial.println(url);
    Serial.print(F("Uptime: ")); Serial.println(millis());
    Serial.print(F("CfgDataSize:")); Serial.println(sizeof(cfgData));
    Serial.print(F("CfgData:")); Serial.println(base64::encode((uint8_t*) &cfgData, sizeof(cfgData), false));
    Serial.print(F("CfgData:")); Serial.println(cfgDataStr);
#endif
    // Make a HTTP GETrequest.
    client.print(String("GET ") + url + " HTTP/1.1\r\n" +
                 "Host: " + host + "\r\n" +
                 "User-Agent: IoT." + devID + "-" + macAddr + "\r\n" +
                 "Device-Info: Device:" + devType +
                 ";MAC:" + macAddr +
                 ";FW:" + FWver +
                 ";HW:" + HWver +
                 ";WData:" + blob +
//                                                 ";Data:uva=" + String(uv_a) +
//                                                 "uvb=" + String(uv_b) +
//                                                 "uvi=" + String(uv_i) +
                 ";Vthr:3." +String((int)((UTHR_WAKE * 100) - 300)) + "/" + String((int)((UTHR_WARN * 100) - 300)) + "/" + String((int)((UTHR_SLEEP * 100) - 300)) +
                 //                                ";ChargeTV:" + chargeThreshold +
                 ";Status:" + statusByte +
                 //                                ";ResetReason:" + ESP.getResetReason() +
                 ";Interval:" + sleepInterval +
                 //                                ";VccRatio:" + VccRatio +
                 //                                ";MASL:" + MASL +
                 ";Cfg:" + CfgFlag + "<" + sizeof(cfgData) + ">"
                 ";Sensor:" + SensType + "[" + dsAddr + "]+BME280+BH1750+VEML6075"
                 ";Uptime:" + millis() +
                 //                                ";AddVal:AH=" + absHum +
                 ";SSID:" + WiFi.SSID() +
                 ";RSSI:" + WiFi.RSSI() +
                 ";IP" + ( NetCfgFlag ? "s" : "") + ":" + WiFi.localIP().toString() + "\r\n" +
                 cfgDataStr +
                 "Connection: close\r\n\r\n");

    client.flush();
    // Blik 1 time when send OK
    BlikLED(pinLED, 1, blikOK);

    // Workaroud for read response and timeout
    int NoHTTPtimeout = 1;
    unsigned long timeout = millis();

    while (NoHTTPtimeout || client.available()) { //== 0
      if (client.available()) {
        NoHTTPtimeout = 0;
        char c = client.read();
        httpResponse += c;
#ifdef DEBUG
        Serial.print(c);
#endif
      }
      if (millis() - timeout > 5000) {
        client.stop();
        // Blink 4 times when client timeout
        BlikLED(pinLED, 4, blikERR);
        NoHTTPtimeout = 0;
        httpResponse = ">>> Client Timeout !";
      }
    }
  }

#ifdef DEBUG
  Serial.println(F("DoHTTPrequest END"));
#endif
  return httpResponse;
}

int find_text(String needle, String haystack) {
  int foundpos = -1;
  for (int i = 0; i <= haystack.length() - needle.length(); i++) {
    if (haystack.substring(i, needle.length() + i) == needle) {
      foundpos = i;
    }
  }
  return foundpos;
}

void ProcessHttpResponse(String httpResponse) {
  bitClear(statusByte, 3);
  bitClear(statusByte, 4);
  if (find_text(SET_INTERVAL_CMD, httpResponse) != -1 ) {
#ifdef DEBUG
    Serial.println(F("SET-INTERVAL command received"));
#endif
    ModifyInterval(httpResponse);
  } else if (find_text(SET_THRESHOLD_CMD, httpResponse) != -1 ) {
#ifdef DEBUG
    Serial.println(F("SET-THRESHOLD command received"));
#endif
    ModifyChargeThreshold(httpResponse);
  } else if (find_text(SET_SEALEVEL_CMD, httpResponse) != -1 ) {
#ifdef DEBUG
    Serial.println(F("SET-SEALEVEL command received"));
#endif
    ModifySeaLevel(httpResponse);
  } else if (find_text(SET_NEWFW_CMD, httpResponse) != -1 ) {
#ifdef DEBUG
    Serial.println(F("SET-NEWFW command received"));
#endif
    if (!bitGet(statusByte, 5)) {
#ifdef DEBUG
      Serial.println(F("Circuit Voltage OK / Go To OTAupdate"));
#endif
      OTAupdate(httpResponse);
    }
  } else if (find_text(SET_SETUP_CMD, httpResponse) != -1 ) {
#ifdef DEBUG
    Serial.println(F("SET-SETUP command received"));
#endif
    RebootToSetup();
  } else if (find_text(CLEAR_STATUS_CMD, httpResponse) != -1 ) {
#ifdef DEBUG
    Serial.println(F("CLEAR-STATUS command received"));
#endif
    statusByte = 0;
  } else if (find_text(SET_FLIPFLOP_CMD, httpResponse) != -1 ) {
#ifdef DEBUG
    Serial.println(F("SET-FLIPFLOP command received"));
#endif
    bitSet(statusByte, 1);
    FlipFlopPulse(pinCHE);
  } else if (find_text(GET_CONFIG_CMD, httpResponse) != -1 ) {
#ifdef DEBUG
    Serial.println(F("GET-CONFIG command received"));
#endif
    DoHTTPrequest(ds_t, ds_u, ds_s,
                  bm_t, bm_h, bm_p,
                  dewPoint, heatIndex, seaLevel, absHum,
                  uc, lux, uv_a, uv_b, uv_i, true);
  } else if (find_text(NEW_CONFIG_CMD, httpResponse) != -1 ) {
#ifdef DEBUG
    Serial.println(F("NEW-CONFIG command received"));
#endif
    ModifyConfig(httpResponse);
  } else {
    bitClear(statusByte, 1);
  }
  storeRTCstatus(statusByte);
  SaveConfig(false);
}

void RebootToSetup() {
  bitSet(statusByte, 0); // set runMode = 1
  bitSet(statusByte, 1);
  storeRTCstatus(statusByte);
  SaveConfig(true);
  deepSleep(100, true, WAKE_RF_DEFAULT);
}

void ModifyChargeThreshold(String httpResponse) {
  int ro = find_text(SET_THRESHOLD_CMD, httpResponse) + SET_THRESHOLD_CMD.length();
  int rc = httpResponse.indexOf('>', ro);
  float newThreshold = httpResponse.substring(ro, rc).toFloat();
#ifdef DEBUG
  Serial.print(F("New Threshold: ")); Serial.println(newThreshold);
#endif
  if (newThreshold > 0 && chargeThreshold != newThreshold) {
    storeRTCthreshold(newThreshold);
    bitSet(statusByte, 1);
    storeRTCstatus(statusByte);
    chargeThreshold = newThreshold;
    SaveConfig(true);
  }
}

void ModifySeaLevel(String httpResponse) {
  int ro = find_text(SET_SEALEVEL_CMD, httpResponse) + SET_SEALEVEL_CMD.length();
  int rc = httpResponse.indexOf('>', ro);
  float newSeaLevel = httpResponse.substring(ro, rc).toFloat();
#ifdef DEBUG
  Serial.print(F("New SeaLevel: ")); Serial.println(newSeaLevel);
#endif
  if (newSeaLevel > 0 && MASL != newSeaLevel) {
    storeRTCmasl(newSeaLevel);
    bitSet(statusByte, 1);
    storeRTCstatus(statusByte);
    MASL = newSeaLevel;
    SaveConfig(true);
  }
}

void ModifyConfig(String httpResponse) {
  int ro = find_text(NEW_CONFIG_CMD, httpResponse) + NEW_CONFIG_CMD.length();
  int rc = httpResponse.indexOf('>', ro);
  String NewConfigStr = httpResponse.substring(ro, rc);
#ifdef DEBUG
  Serial.print(F("NewConfig: ")); Serial.println(NewConfigStr);
#endif
  int NewConfigLen = b64decode(NewConfigStr, (uint8_t*) &cfgData);
#ifdef DEBUG
  Serial.print(F("NewConfigLen: ")); Serial.println(NewConfigLen);
#endif
  uint32_t crcOfData = calculateCRC32(((uint8_t*)&cfgData) + 4, sizeof(cfgData) - 4);
  if (crcOfData == cfgData.crc32) {
#ifdef DEBUG
    Serial.print(F("CRC OK : ")); Serial.print(crcOfData); Serial.print(F(" = ")); Serial.println(cfgData.crc32);
#endif
    bitSet(statusByte, 1);
    storeRTCstatus(statusByte);
    SaveConfig(true);
  } else {
    LoadConfig();
  }
}

void ModifyInterval(String httpResponse) {
  int ro = find_text(SET_INTERVAL_CMD, httpResponse) + SET_INTERVAL_CMD.length();
  int rc = httpResponse.indexOf('>', ro);
  int newInterval = httpResponse.substring(ro, rc).toInt();
#ifdef DEBUG
  Serial.print(F("New Interval: ")); Serial.println(newInterval);
#endif
  if (newInterval > 0 && sleepInterval != newInterval) {
    BlikLED(pinLED, newInterval, blikOK);
    storeRTCinterval(newInterval);
    bitSet(statusByte, 1);
    storeRTCstatus(statusByte);
    if (newInterval < sleepInterval) {
      sleepInterval = newInterval;
    }
    SaveConfig(true);
  }
}


void OTAupdate(String httpResponse) {
  int ro = find_text(SET_NEWFW_CMD, httpResponse) + SET_NEWFW_CMD.length();
  int rc = httpResponse.indexOf('>', ro);
  String newFW = httpResponse.substring(ro, rc);
#ifdef DEBUG
  Serial.print(F("OTA Update: ")); Serial.println(newFW);
#endif
  String url  = "http://";
  url += host;
  url += "/OTAupdate/?FW=";
  url += newFW;
  //         url += ".ino.bin";
#ifdef DEBUG
  Serial.print(F("OTA Update URL: ")); Serial.println(url);
#endif
  digitalWrite(pinLED, LOW);
  bitSet(statusByte, 1);
  bitSet(statusByte, 3);
  bitSet(statusByte, 4);
  storeRTCstatus(statusByte);
  SaveConfig(true);

  WiFiClient OTAclient;

  t_httpUpdate_return ret = ESPhttpUpdate.update(OTAclient, url);
  switch (ret) {
    case HTTP_UPDATE_FAILED:
#ifdef DEBUG
      Serial.printf("[%d] HTTP_UPDATE_FAILED Error (%d): %s\n", ret, ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
      Serial.println(F("HTTP_UPDATE_FAILED"));
#endif
      bitSet(statusByte, 3);
      bitClear(statusByte, 4);
      storeRTCstatus(statusByte);
      break;
    case HTTP_UPDATE_NO_UPDATES:
#ifdef DEBUG
      Serial.println(F("HTTP_UPDATE_NO_UPDATES"));
#endif
      bitClear(statusByte, 3);
      bitSet(statusByte, 4);
      storeRTCstatus(statusByte);
      break;
    case HTTP_UPDATE_OK:
#ifdef DEBUG
      Serial.println(F("HTTP_UPDATE_OK"));
#endif
      break;
  }
  SaveConfig(true);
  digitalWrite(pinLED, HIGH);
#ifdef DEBUG
  Serial.println(F("HTTP_UPDATE_OK"));
#endif
}
// -------------------------------- HTTP CLIENT functions END ----------------------------------

// -------------------------------- ESP functions BEGIN ----------------------------------
void deepSleep(byte interval, bool justReset, WakeMode mode) {
  //  BlikLED(pinLED,2,blikOK);
#ifdef DEBUG
  Serial.print(interval);
  Serial.println(F(" min sleep mode"));
#endif
  if (justReset) {
    ESP.deepSleep(interval, mode);//WAKE_RF_DISABLED  WAKE_RF_DEFAULT
  } else {
    ESP.deepSleep(interval * 60 * 1000000, mode);//WAKE_RF_DISABLED  WAKE_RF_DEFAULT // sleep Interval minutes
  }
  delay(100);
}

void BlikLED(byte LEDport, byte count, int wait) {
  byte c = 0;
  while (c < count) {
#ifdef DEBUG
    Serial.print(F("BLIKled")); Serial.print(LEDport);
#endif
    digitalWrite(LEDport, LOW);
    delay(wait);
    digitalWrite(LEDport, HIGH);
    if (count > 1) {
      delay(wait);
    }
    c++;
  }
}

bool readPin(byte PIN) {
  return digitalRead(PIN);
}

int b64decode(String b64Text, uint8_t* output) {
  base64_decodestate s;
  base64_init_decodestate(&s);
  int cnt = base64_decode_block(b64Text.c_str(), b64Text.length(), (char*)output, &s);
  return cnt;
}

// -------------------------------- ESP functions END ----------------------------------

// -------------------------------- SETUP AP ----------------------------------
void setupWiFiAP() {
#ifdef DEBUG
  Serial.println(F("AP MODE - WiFi config for CLIENT MODE"));
  Serial.print(F("SSID")); Serial.println(APssid);
#endif

  //  initPins();
  WiFi.mode(WIFI_AP);
  if (!WiFi.softAP(APssid, APpass)) {
    BlikLED(pinLED, 5, blikERR);
  }
  WiFi.macAddress(mac);
  macAddr = "";
  for (int i = 0; i < 6; i++) {
    if (mac[i] < 16) macAddr += "0";
    macAddr += String(mac[i], HEX) + ":";
  }

  createWebServer();

  server.begin();
}

// -------------------------------- HTTP SERVER functions BEGIN ----------------------------------

String AddVccToContent() {
  String ret = F("<p>&#128267; Ucc: ");
  ret += (ESP.getVcc() / 1024.00);
#ifdef READVCC
  ret += F(" V</p><p>&#128267;&#128201; Ucc ratio:");
  ret += String(VccRatio, 3);
  ret += F(" => Real Ucc: ");
  ret += (ESP.getVcc() / 1024.00) * VccRatio;
#endif
  ret += F(" V</p>");
  return ret;
}

String HTMLHead(String Icon, String Title) {
  String ret = F("<!DOCTYPE HTML>\r\n<html><meta name='viewport' content='width=device-width, initial-scale=1.0'><h1> ");
  ret += Icon + " " + devType + " " + Title;
  ret += "</h1><p>&#128279; IP: " + WiFi.softAPIP().toString() + "</p><p>&#128223; MAC-AP: " + WiFi.softAPmacAddress() + "</p><p>&#128223; MAC-STA: " + WiFi.macAddress() + "</p>";
  ret += AddVccToContent();
  return ret;
}

String HTMLPatt() {
  String ret = F("<br /><br /><br /><a href='/'>Back to setup</a><br /><br /><br /><a href='/restart'>Restart</a><hr />&#169;2018 Petr KLOSKO, <a href='http://www.klosko.net' target=new>www.klosko.net</a> esp8266weather_v2 [v.");
  ret += FWver;
  ret += F("]&nbsp;");
  ret += ESP.getFullVersion();
  ret += F("<html>");
  return ret;
}

void handleMain() {
  String content = HTMLHead("&#128295; ", " WiFi &amp; params setup");
  content += "<form method='get' action='setting'>";
  content += "<label>&#128246; SSID: </label><input name='ssid' value='" + ssid + "' />(string)<br />";
  content += "<label>&#128272; PASS: </label><input name='pass' value='" + pass + "' />(string)<br />";
  content += "<label>&#128268; Battery Charge Threshold Voltage: </label><input name='volt' value='";
  content += getRTCthreshold();
  content += "' />[V](float)<br /><label>&#128336; Interval: </label><input name='int' value='";
  content += getRTCinterval();
  content += "' />[min](int)<br /><label>&#9968; Sea level: </label><input name='msl' value='";
  content += getRTCmasl();
  content += "' />[m](float)<br /><label>&#128201; Ucc Ratio [compensation]: </label><input name='voltR' value='";
  content += String(VccRatio, 3);
  content += "' />[V](float)<br /><input type='submit'></form>" + HTMLPatt();
  server.send(200, "text/html", content);
}

void handleRestart() {
  String content = HTMLHead("&#128295; ", " restarting ...") + HTMLPatt();
  server.send(200, "text/html", content);
  BlikLED(pinLED, 20, blikOK);
  BlikLED(pinLED, 5, blikERR);
  deepSleep(100, true, WAKE_RF_DEFAULT);
}

void handleSettings() {
  String qsid  = server.arg("ssid");
  String qpass = server.arg("pass");
  float  qcht  = server.arg("volt").toFloat();
  float  qmsl  = server.arg("msl").toFloat();
  byte   qint  = server.arg("int").toInt();
  byte   ssidpass_ln = (qsid.length() + qpass.length());
  float  qvra  = server.arg("voltR").toFloat();
  if (qvra == NULL) {
    qvra = 1;
  }

  int statusCode;
  String content = HTMLHead("&#128295; ", " WiFi parameters");
  if ( ssidpass_ln > sizeof(cfgData.SSID_PASS) ) {
    content += F("<h2>SSID and PASS length out of range</h2><h3>");
    content += ssidpass_ln;
    content += " &gt; ";
    content += sizeof(cfgData.SSID_PASS);
    content += " !!! </h3>" + HTMLPatt();
    statusCode = 507;
  } else if ((qsid.length() > 0) && (qpass.length() > 0) ) {
    storeRTCWiFiSSID_PASS(qsid, qpass);
    BlikLED(pinLED, 1 , blikERR);
    storeRTCthreshold(qcht);
    BlikLED(pinLED, 1 , blikERR);
    storeRTCinterval(qint);
    BlikLED(pinLED, 1 , blikERR);
    storeRTCmasl(qmsl);
    BlikLED(pinLED, 1 , blikERR);
    storeRTCvccRatio(qvra);
    BlikLED(pinLED, 1 , blikERR);
    storeRTCstatus(getRTCstatus() & 0);  // set runMode = 0
    BlikLED(pinLED, 1 , blikERR);

    SaveConfig(1);
    LoadConfig();

    content += F("<h2>Saved to EEPROM ... <a href='/restart'>restart to boot into new wifi</a></h2>");
    content += "<p>&#128246; ssid: " + getRTCWiFiSSID() + "</p>";
    content += "<p>&#128272; pass: " + getRTCWiFiPass() + "</p>";
    content += "<p>&#128268; Battery Charge Threshold Voltage:" + String(getRTCthreshold()) + "</p>";
    content += "<p>&#128336; Interval: " + String(getRTCinterval()) + "</p>";
    content += "<p>&#9968; Sea level: " + String(getRTCmasl()) + "</p>";
    content += "<p>&#128201; VccRatio:" + String(getRTCvccRatio(), 3) + "</p>";
    content += "<p>&#127939; Run Mode: " + String(getRTCstatus() & 1) + "</p>";
    content += F("<h3>DeviceID: ");
    content += devID;
    content += "-" + macAddr + "</h3>" + HTMLPatt();
    statusCode = 200;
  } else {
    content += "<h2>400 Bad Request (Illegal parameters)</h2>" + HTMLPatt();
    statusCode = 400;
  }
  server.send(statusCode, "text/html", content);
}

void createWebServer() {
  server.on("/", handleMain);
  server.on("/setting", handleSettings);
  server.on("/restart", handleRestart);
}
// -------------------------------- HTTP SERVER functions END ----------------------------------

void loop() {
  BlikLED(pinLED, 1 , blikERR * 3);
  BlikLED(pinLED, 2 , blikERR);
  server.handleClient();
}
