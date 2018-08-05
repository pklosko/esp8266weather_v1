/*  Simple sketch by Petr KLOSKO (https://www.klosko.net)
  
 This sketch is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.
  
*/  
//#define DEBUG

// Define settings
const char FWver[]     = "20180804-01";

/*
 * 
 * 
 !!! Compile w/ 4M ( 1M SPIFFS ) !!!

 
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
    - production versiion, uploaded to WS
     
*/

// include libraries
#include <ESP8266WiFi.h>       // WiFi library
#include <OneWire.h>           // OneWire communication library for DS18B20
#include <DS2438.h> // DS18B20 library
#include <EEPROM.h>            // EEPROM library
#include <ESP8266WebServer.h>  // Web server library
#include <ESP8266httpUpdate.h> //Update libratry
#include <Wire.h>
#include <BME280I2C.h>
#include <BH1750.h>
#include <math.h>
#include <EnvironmentCalculations.h>

// http timeout in milis
#define HTTP_TIMEOUT 5000

byte addr[8];
const char APssid[]    = "esp8266weather";     // WiFi SSID
const char APpass[]    = "826682668266";       // WiFi PASS
const char devID[]     = "esp8266-07";         // Device ID Prefix - for WEB
const char devType[]   = "esp8266-ESP-12F";    // Device Type
const char HWver[]     = "ESP-12Fv1";          // HW [DPS] version
const char host[]      = "iot.klosko.net";     // IoT Server

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
const String SET_FLIPFLOP_CMD  = "SET-FLIPFLOP<1>";

// Defult values
byte  sleepInterval    = 1;                    // How often send data to the server. In minutes
byte  statusByte       = 0;                    // Status byte
/*
 *  Status Register 
               bit0 = (1)  = runMode  - 0 = Client, 1 = Setup AP
                  1 = (2)  = Last CMD - 0 = ERR,    1 = OK
                  2 = (4)  = Charger  - 0 = OFF,    1 = ON
                  3 = (8)  = OTAupdteStatus LSb
                  4 = (16) = OTAupdteStatus MSb = 
                  5 = (32) =
                  6 = (64) =
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
  uint8_t  SSID_ln;        // 1
  uint8_t  TOTAL_ln;       // 1 = 12
  byte     SSID_PASS[84];  // 84 = 96
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

// Weather values vars.
float bm_p = -127.00, bm_t = -127.00, bm_h = -127.00, uc = -127.00, ds_t = -127.00, ds_u = -127.00, ds_s = -127.00, lux = -127;

DeviceAddress deviceAddress;

// -------------------------------- OBJECTS AND HW SETTINGS ----------------------------------
OneWire oneWire(pin1Wire);         // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DS2438 ds2438(&oneWire);             //, DS2438_address);
ESP8266WebServer server(80);
BME280I2C bme;
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
    Serial.print("EEssid:");Serial.println(ssid);
    Serial.print("EEpass:");Serial.println(pass);
    Serial.print("EEinterval:");Serial.println(sleepInterval);    
    Serial.print("EEthreshold:");Serial.println(chargeThreshold);    
    Serial.print("EEmasl:");Serial.println(MASL);  
    Serial.print("EEstatus:");Serial.println(statusByte, BIN);  
    Serial.print("runMode:");Serial.println(runMode, BIN);  
  #endif

// if status = 11010 = OTA Update OK and Last command OK
// then clear ststus byte and sleep 
// This caused just after OTA Update Reset
  if (((statusByte & 26) == 26) && (statusByte != 255)){ 
    #ifdef DEBUG
      Serial.println();
      Serial.print("Status byte = 26 :");Serial.println(statusByte);
    #endif
    bitClear(statusByte,1);  
    storeRTCstatus(statusByte);
    SaveConfig(false);
    deepSleep(sleepInterval, false);
  }
 
  initPins();    

  #ifdef DEBUG
    Serial.println("initPins OK (setup)"); 
  #endif  
  
  if (ssid == "" || sleepInterval == 255 || sleepInterval == 0   // after first power off
                 || readPin(pinSETUP) == LOW || runMode == 1){   // go to conf via PIN or HTTP-COMMAND ,, 
    storeRTCstatus(bitClear(statusByte,0));  // set runMode = 0
    SaveConfig(false);
    #ifdef DEBUG
      Serial.println("Go to setupWiFiAP MODE"); 
    #endif    
    setupWiFiAP();
  }else{
    #ifdef DEBUG
      Serial.println("Go to weatherClient MODE"); 
    #endif      
    weatherClient();
  }
}


void SplitText(String &ssid, int i, String &rest){
  if (i > -1){
    rest = ssid.substring(0, i);
    ssid = ssid.substring(i+1,ssid.length());
  }else{
    ssid = rest;
    rest = "";
  }
}

// -------------------------------- MAIN WEATHER ---------------------------------- 
void weatherClient(){
  bool Conn = false;
  String sr = ssid, sp = pass; 
    
  init1Wire();
  int i2cfl = initI2C(pinSDA, pinSCL);
  lightMeter.begin(BH1750_CONTINUOUS_HIGH_RES_MODE);
  
  uc = ESP.getVcc() / 1024.00;
  
  while ((ssid.length() > 0 ) && (Conn == false)){
    SplitText(ssid, ssid.lastIndexOf(";"), sr);
    SplitText(pass, pass.lastIndexOf(";"), sp);
    #ifdef DEBUG     
      Serial.println();
      Serial.print("wifiMulti.conn(");Serial.print(ssid);Serial.print(",");Serial.print(pass);Serial.println(")");
    #endif  
    Conn = initWiFiClient(ssid, pass);
    ssid = sr;  pass = sp;
  }  
  if (Conn == false){
    #ifdef DEBUG     
      Serial.println();
      Serial.print("WiFi Conn ERR - Go To Deep Sleep");
    #endif      
    deepSleep(sleepInterval, false);
  }else{
    WiFi.setAutoConnect(true);
    if (!NetCfgFlag){
      cfgNetwork.IP   = WiFi.localIP();
      cfgNetwork.NET  = WiFi.subnetMask();
      cfgNetwork.GW   = WiFi.gatewayIP();        
      cfgNetwork.DNS  = WiFi.dnsIP();    
      #ifdef DEBUG
        Serial.println("Save IP CFG ");
      #endif   
      SaveNetConfig(WiFi.SSID(), sizeof(cfgData)+4); /// YES, cfgData. its only an ADDRESS
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

  if (SensType == "Unknown"){
    ds_t  = -127.00;
  #ifdef DEBUG 
    Serial.println("readTemerature: ");Serial.println(ds_t);
  #endif    
    ds_s  = -127.00;
    ds_u  = -127.00;    
  }else{
    ds_t  = readTemerature();
  #ifdef DEBUG 
    Serial.println("readTemerature: ");Serial.println(ds_t);
  #endif    
    ds_s  = readVoltage(DS2438_CHA);
    ds_u  = readVoltage(DS2438_CHB);
  }
// When 0V on ADC pin of DS2438, chip read 10.22 V 
  if (ds_s > 10.00){
    ds_s = 0;
  }

  BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
  BME280::PresUnit presUnit(BME280::PresUnit_hPa); 
    
  if (i2cfl){
    bme.read(bm_p, bm_t, bm_h, tempUnit, presUnit);
  }

  EnvironmentCalculations::AltitudeUnit envAltUnit  =  EnvironmentCalculations::AltitudeUnit_Meters;
  EnvironmentCalculations::TempUnit     envTempUnit =  EnvironmentCalculations::TempUnit_Celsius;
  
  float dewPoint  = EnvironmentCalculations::DewPoint(bm_t, bm_h, envTempUnit);
  float seaLevel  = EnvironmentCalculations::EquivalentSeaLevelPressure(MASL, bm_t, bm_p, envAltUnit, envTempUnit);
  float absHum    = EnvironmentCalculations::AbsoluteHumidity(bm_t, bm_h, envTempUnit);
  float heatIndex = EnvironmentCalculations::HeatIndex(bm_t, bm_h, envTempUnit);
  #ifdef DEBUG
     Serial.println("------------------------------------------------------------------------------");
     Serial.println(millis());
     Serial.print("Temp: ");              Serial.print(bm_t);Serial.println("°"+ String(tempUnit == BME280::TempUnit_Celsius ? 'C' :'F'));
     Serial.print("Humidity: ");          Serial.print(bm_h);Serial.println("% RH");
     Serial.print("Pressure: ");          Serial.print(bm_p);Serial.println(String(presUnit == BME280::PresUnit_hPa ? "hPa" : "Pa"));
     Serial.print("Dew point: ");         Serial.print(dewPoint);Serial.println("°"+ String(envTempUnit == EnvironmentCalculations::TempUnit_Celsius ? "C" :"F"));
     Serial.print("Sea Level Pressure: ");Serial.print(seaLevel);Serial.println(String( presUnit == BME280::PresUnit_hPa ? "hPa" :"Pa"));
     Serial.print("Heat Index: ");        Serial.print(heatIndex);Serial.println("°"+ String(envTempUnit == EnvironmentCalculations::TempUnit_Celsius ? "C" :"F"));
     Serial.print("Absolute Humidity: ") ;Serial.println(absHum);
     Serial.println("");
    #endif
  ChragrgeBatt(ds_s, ds_u);
  lux = lightMeter.readLightLevel();
  
  String httpResponse = DoHTTPrequest(ds_t, ds_u, ds_s,
                                      bm_t, bm_h, bm_p,
                                      dewPoint, heatIndex, seaLevel, absHum,
                                      uc, lux);
  #ifdef DEBUG 
    Serial.println("httpResponse(client): ");Serial.println(httpResponse);
  #endif                                
  ProcessHttpResponse(httpResponse);                             
  deepSleep(sleepInterval, false);
}

// -------------------------------- INIT functions BEGIN ----------------------------------
void initSerial(){
  // Start serial
  Serial.begin(115200);
  delay(10);
  #ifdef DEBUG 
    Serial.println("initSerial"); 
  #endif
}

int initI2C(byte SDA, byte SCL){
  Wire.begin(SDA, SCL);
  if(!bme.begin()){
  #ifdef DEBUG 
    Serial.print("initI2C: SDA=.....");Serial.println(SDA);
    Serial.print("initI2C: SCL=.....");Serial.println(SCL);
  #endif    
  while(1);
     return 0;
  }
      
  #ifdef DEBUG 
    switch(bme.chipModel())
    {
      case BME280::ChipModel_BME280:
        Serial.println("Found BME280 sensor! Success.");
        break;
      case BME280::ChipModel_BMP280:
        Serial.println("Found BMP280 sensor! No Humidity available.");
        break;
      default:
        Serial.println("Found UNKNOWN sensor! Error!");
    }
    Serial.print("initI2C: SDA=");Serial.println(SDA);
    Serial.print("initI2C: SCL=");Serial.println(SCL);
  #endif
    return 1;
}

void init1Wire(){
  ds2438.begin(); // Initialize the DallasTemperature DS18B20 class (not strictly necessary with the client class, but good practice).
  ds2438.getAddress(deviceAddress,0);
  dsAddr = printAddress(deviceAddress);
  SensType = getSensType(deviceAddress);
  #ifdef DEBUG 
    Serial.print("init1Wire: dsAddr=");Serial.println(dsAddr);
    Serial.print("init1Wire: SensType=");Serial.println(SensType);
  #endif  
}

bool initWiFiClient(String ssid, String pass){
  // Connect to the WiFi
//  WiFi.forceSleepWake();
  WiFi.mode(WIFI_STA);  
  NetCfgFlag = LoadNetConfig(WiFi.SSID(), sizeof(cfgData)+4);
  if (NetCfgFlag){
    #ifdef DEBUG
      Serial.println("Load IP CFG OK ... config");
    #endif    
    WiFi.config(cfgNetwork.IP, cfgNetwork.GW, cfgNetwork.NET, cfgNetwork.DNS);
  }
  WiFi.macAddress(mac);
  macAddr = "";
  for (int i=0;i<6;i++){
    if (mac[i] < 16) macAddr += "0";
    macAddr += String(mac[i],HEX);
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
    if (c == 25){
      #ifdef DEBUG
        Serial.printf("Connection status: %d\n", WiFi.status());
        Serial.println(macAddr);
      #endif
      c = 0;      
//      WiFi.disconnect();
//      delay(300);
//      WiFi.begin(ssid.c_str(), pass.c_str());
    }
    if (a > 75){
      return false;
    }
  }
  return (WiFi.status() == WL_CONNECTED);
}  

void initPins(){              
  #ifdef DEBUG 
    Serial.println("initPins");
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


// -------------------------------- CHARGING functions BEGIN ----------------------------------
void ChragrgeBatt(float ds_s, float ds_u){
  bool chStat = readPin(pinCHRG);
   
  #ifdef DEBUG 
    Serial.print("ChargeStatus: ");Serial.println(chStat, BIN);
    Serial.println("Voltages: ");;
    Serial.print("ds_s - SOLAR : ");Serial.print(ds_s);
    Serial.print("              solarThreshold : ");Serial.println(solarThreshold);
    Serial.print("ds_u - BATT: ");Serial.print(ds_u);
    Serial.print("              chargeThreshold : ");Serial.println(chargeThreshold);
  #endif 

/*  if ( (ds_s < solarThreshold || (ds_u >= chargeThreshold) ) && chStat){  // Low Solar voltage OR Battery voltage OK - charging not allowed
    #ifdef DEBUG 
      Serial.println("Low Solar voltage OR Battery voltage OK, charger ON - charging not allowed - Switch charger OFF");
    #endif 
    FlipFlopPulse(pinCHE);
  }
*/
   if (ds_s > solarThreshold && ds_u < chargeThreshold && !chStat){  // Solar voltage OK, Battery Low, charger OFF
    #ifdef DEBUG 
      Serial.println("Solar voltage OK, Battery Low, charger OFF - Switch charger ON");
    #endif   
    FlipFlopPulse(pinCHE);
  }
  bitWrite(statusByte,2,readPin(pinCHRG)); 
  storeRTCstatus(statusByte);  
  SaveConfig(false);
}

void FlipFlopPulse(byte PIN){
  #ifdef DEBUG 
    Serial.print("FlipFlopPulse : ");Serial.println(PIN);
  #endif 
  digitalWrite(PIN,   LOW); 
  delay(100);
  digitalWrite(PIN,   HIGH);
  delay(100);
  digitalWrite(PIN,   LOW);  
}
// -------------------------------- CHARGING functions END ----------------------------------

// -------------------------------- SENSORS functions BEGIN ----------------------------------
String printAddress(uint8_t* deviceAddress){
  String DsAddr = "";
  for(int i=0;i<8;i++) {
  if (deviceAddress[i]<16) DsAddr += "0";
    DsAddr += String(deviceAddress[i],HEX);
  }  
  return DsAddr;
}

String getSensType(uint8_t* deviceAddress){
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

float readTemerature(){
  float t;
  ds2438.update();
  if (ds2438.isError()) {
    t = -127.00;
  }else{
    t = ds2438.getTemperature();
  }  
  return t;
}

float readVoltagePct(byte channel, byte base){
  float u;
  ds2438.update();
  if (ds2438.isError()) {
    u = -127.00;
  }else{
    u = ds2438.getVoltagePct(channel, base);
  }  
  return u;
}

float readVoltage(byte channel){
  float u;
  ds2438.update();
  if (ds2438.isError()) {
    u = -127.00;
  }else{
    u = ds2438.getVoltage(channel);
  }  
  return u;
}

// -------------------------------- SENSORS functions END ----------------------------------

// -------------------------------- HTTP CLIENT functions BEGIN ----------------------------------

String createBLOB(float t, float th_h, float bm_sl, float th_dp, float hi, float ds_u, float ds_s, float ds_t, float bm_p, float uc, float lux, byte statusByte){
  String blob = floatToHex2(t)     + 
                floatToHex2(th_h)  + 
                floatToHex3(bm_sl) + 
                floatToHex2(th_dp) +
                floatToHex2(hi)    + 
                floatToHex2(ds_u)  + 
                floatToHex2(ds_s)  + 
                floatToHex2(ds_t)  + 
                floatToHex3(bm_p)  + 
                floatToHex2(uc)    +
                floatToHex3(lux);
  return blob;
}

String byteToHex(byte value){
  return (value<16 ? "0"+ String(value,HEX) : String(value,HEX));
}

String floatToHex2(float value){
  String ret = "";
  byte flag = (value < 0 ? 1 : 0);
  int  val  = abs(int(value*100));
  byte hi   = highByte(val);
  byte lo   = lowByte(val);  
  bitWrite(hi, 7, flag);
  ret += (hi<16 ? "0"+ String(hi,HEX) : String(hi,HEX));
  ret += (lo<16 ? "0"+ String(lo,HEX) : String(lo,HEX));
  return ret;
}

String floatToHex3(float value){
  String ret = "";
  byte flag = (value < 0 ? 1 : 0);
  int  val  = int(value*100);
  byte mid  = highByte(val);
  byte lo   = lowByte(val); 
  byte hi   = (lowByte(val >> 16));
  bitWrite(hi, 7, flag);
  ret += (hi<16  ? "0"+ String(hi,HEX)  : String(hi,HEX));
  ret += (mid<16 ? "0"+ String(mid,HEX) : String(mid,HEX));
  ret += (lo<16  ? "0"+ String(lo,HEX)  : String(lo,HEX));
  return ret;
}

String DoHTTPrequest(float ds_t, float ds_u, float ds_s,
                     float bm_t, float bm_h, float bm_p,
                     float dewPoint, float heatIndex, float seaLevel, float absHum,
                     float uc, float lux){
  String httpResponse = "";
  #ifdef DEBUG
    Serial.print(F("ds_t ")); Serial.println(ds_t);
  #endif  
  String blob = createBLOB(bm_t, bm_h, seaLevel, dewPoint, heatIndex, ds_u, ds_s, ds_t, bm_p, uc, lux, statusByte);
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
  }else{
    #ifdef DEBUG 
      Serial.println(F("Client connected")); 
    #endif 

    String url = "/push/";
    #ifdef DEBUG 
      Serial.print(F("Requesting URL: ")); Serial.println(url); 
      Serial.print(F("Uptime: ")); Serial.println(millis()); 
    #endif
    
  // Make a HTTP GETrequest.
    client.print(String("GET ") + url + " HTTP/1.1\r\n" +
                 "Host: " + host + "\r\n" + 
                 "User-Agent: IoT." + devID + "-" + macAddr + "\r\n" + 
                 "Device-Info: Device:" + devType +
                                ";MAC:" + macAddr + 
                                ";FW:"+ FWver + 
                                ";HW:"+ HWver +
                                ";WData:" + blob +
                                ";ChargeTV:" + chargeThreshold + 
                                ";Status:" + statusByte +
                                ";Interval:" + sleepInterval + 
                                ";MASL:" + MASL + 
                                ";Cfg:"+ CfgFlag +
                                ";Sensor:" + SensType + 
                                "+BME280+BH1750;SensID:" + dsAddr + 
                                ";Uptime:" + millis() + 
                                ";AddVal:AH=" + absHum + 
                                ";Conn:WiFi;SSID:" + WiFi.SSID() +
                                ";RSSI:" + WiFi.RSSI() + 
                                ";IP" + ( NetCfgFlag ? "s" :"") + ":" + WiFi.localIP().toString() + "\r\n" + 
                 "Connection: close\r\n\r\n");
  
    BlikLED(pinLED,1,100);
  }
  
  // Workaroud for timeout
  unsigned long timeout = millis();
  while(client.connected() && (millis() - timeout < (HTTP_TIMEOUT+10))){
    while (client.available()) { //== 0
      char c = client.read();
      httpResponse += c;
      if (millis() - timeout > HTTP_TIMEOUT) {
        client.stop();
        delay(1000);
        httpResponse = "Client Timeout";
        #ifdef DEBUG 
          Serial.println(F("httpResponse: >>> Client Timeout"));
        #endif 
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
    if (haystack.substring(i,needle.length()+i) == needle) {
      foundpos = i;
    }
  }
  return foundpos;
}

void ProcessHttpResponse(String httpResponse){
  bitClear(statusByte,3);  
  bitClear(statusByte,4);    
  if (find_text(SET_INTERVAL_CMD, httpResponse) != -1 ){
    #ifdef DEBUG
      Serial.println(F("SET-INTERVAL command received"));
    #endif    
    ModifyInterval(httpResponse);
  }else if (find_text(SET_THRESHOLD_CMD, httpResponse) != -1 ){
    #ifdef DEBUG
      Serial.println(F("SET-THRESHOLD command received"));
    #endif  
    ModifyChargeThreshold(httpResponse);
  }else if (find_text(SET_SEALEVEL_CMD, httpResponse) != -1 ){
    #ifdef DEBUG
      Serial.println(F("SET-SEALEVEL command received"));
    #endif  
    ModifySeaLevel(httpResponse);
  }else if (find_text(SET_NEWFW_CMD, httpResponse) != -1 ){
    #ifdef DEBUG
      Serial.println(F("SET-NEWFW command received"));
    #endif  
    OTAupdate(httpResponse);
  }else if (find_text(SET_SETUP_CMD, httpResponse) != -1 ){
    #ifdef DEBUG
      Serial.println(F("SET-SETUP command received"));
    #endif  
    RebootToSetup();
  }else if (find_text(SET_FLIPFLOP_CMD, httpResponse) != -1 ){
    #ifdef DEBUG
      Serial.println(F("SET-FLIPFLOP command received"));
    #endif  
    bitSet(statusByte,1);    
    FlipFlopPulse(pinCHE);
  }else{
    bitClear(statusByte,1);    
  }
  storeRTCstatus(statusByte);     
  SaveConfig(false);
}

void RebootToSetup(){
  bitSet(statusByte,0);  // set runMode = 1
  bitSet(statusByte,1);    
  storeRTCstatus(statusByte);   
  SaveConfig(true);
  deepSleep(100,true);
}

void ModifyChargeThreshold(String httpResponse){
  int ro = find_text(SET_THRESHOLD_CMD, httpResponse) + SET_THRESHOLD_CMD.length();
  int rc = httpResponse.indexOf('>', ro);
  float newThreshold = httpResponse.substring(ro,rc).toFloat();
  #ifdef DEBUG
    Serial.print(F("New Threshold: "));Serial.println(newThreshold);
  #endif  
  if (newThreshold > 0 && chargeThreshold != newThreshold){
    storeRTCthreshold(newThreshold);
    bitSet(statusByte,1);  
    storeRTCstatus(statusByte);
    chargeThreshold = newThreshold;
    SaveConfig(true);
  }
}

void ModifySeaLevel(String httpResponse){
  int ro = find_text(SET_SEALEVEL_CMD, httpResponse) + SET_SEALEVEL_CMD.length();
  int rc = httpResponse.indexOf('>', ro);
  float newSeaLevel = httpResponse.substring(ro,rc).toFloat();
  #ifdef DEBUG
    Serial.print(F("New SeaLevel: "));Serial.println(newSeaLevel);
  #endif  
  if (newSeaLevel > 0 && MASL != newSeaLevel){
    storeRTCmasl(newSeaLevel);
    bitSet(statusByte,1);  
    storeRTCstatus(statusByte);
    MASL = newSeaLevel;
    SaveConfig(true);
  }
}


void ModifyInterval(String httpResponse){
  int ro = find_text(SET_INTERVAL_CMD, httpResponse) + SET_INTERVAL_CMD.length();
  int rc = httpResponse.indexOf('>', ro);
  int newInterval = httpResponse.substring(ro,rc).toInt();
  #ifdef DEBUG
    Serial.print(F("New Interval: "));Serial.println(newInterval);
  #endif  
  if (newInterval > 0 && sleepInterval != newInterval){
    BlikLED(pinLED,newInterval,75);
    storeRTCinterval(newInterval);
    bitSet(statusByte,1);   
    storeRTCstatus(statusByte);
    if (newInterval < sleepInterval){
      sleepInterval = newInterval;
    }
    SaveConfig(true);
  }
}


void OTAupdate(String httpResponse){ 
  int ro = find_text(SET_NEWFW_CMD, httpResponse) + SET_NEWFW_CMD.length();
  int rc = httpResponse.indexOf('>', ro);
  String newFW = httpResponse.substring(ro,rc);  
  #ifdef DEBUG
    Serial.print(F("OTA Update: "));Serial.println(newFW);
  #endif   
  String url  = "http://";
         url += host;
         url += "/OTAupdate/?FW=";
         url += newFW;
//         url += ".ino.bin";
  #ifdef DEBUG
    Serial.print(F("OTA Update URL: "));Serial.println(url);
  #endif          
  digitalWrite(pinLED,LOW);
  bitSet(statusByte,1);  
  bitSet(statusByte,3);  
  bitSet(statusByte,4);   
  storeRTCstatus(statusByte); 
  SaveConfig(true);      
  
  t_httpUpdate_return ret = ESPhttpUpdate.update(url);
   switch (ret) {
      case HTTP_UPDATE_FAILED:
        #ifdef DEBUG
          Serial.printf("[%d] HTTP_UPDATE_FAILED Error (%d): %s\n", ret, ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
          Serial.println(F("HTTP_UPDATE_FAILED"));
        #endif 
        bitSet(statusByte,3);  
        bitClear(statusByte,4);  
        storeRTCstatus(statusByte); 
        break;
      case HTTP_UPDATE_NO_UPDATES:
        #ifdef DEBUG
          Serial.println(F("HTTP_UPDATE_NO_UPDATES"));
        #endif 
        bitClear(statusByte,3);  
        bitSet(statusByte,4); 
        storeRTCstatus(statusByte); 
        break;
      case HTTP_UPDATE_OK:
        #ifdef DEBUG
          Serial.println(F("HTTP_UPDATE_OK"));
        #endif 
        break;
   }
  SaveConfig(true);   
  digitalWrite(pinLED,HIGH);         
  #ifdef DEBUG
    Serial.println(F("HTTP_UPDATE_OK"));
  #endif 
}
// -------------------------------- HTTP CLIENT functions END ----------------------------------

// -------------------------------- ESP functions BEGIN ----------------------------------
void deepSleep(byte interval, bool justReset){
  BlikLED(pinLED,2,75);
  #ifdef DEBUG 
    Serial.print(interval);
    Serial.println(" min sleep mode"); 
  #endif   
  if (justReset){
    ESP.deepSleep(interval, WAKE_RF_DEFAULT);//WAKE_RF_DISABLED  WAKE_RF_DEFAULT    
  }else{
    ESP.deepSleep(interval * 60 * 1000000, WAKE_RF_DEFAULT);//WAKE_RF_DISABLED  WAKE_RF_DEFAULT // sleep Interval minutes
  }
  delay(100);
}

void BlikLED(byte LEDport, byte count, int wait){
  byte c=0;
  while(c<count){
    #ifdef DEBUG
      Serial.print("BLIKled");Serial.print(LEDport); 
    #endif 
    digitalWrite(LEDport, LOW);
    delay(wait);
    digitalWrite(LEDport, HIGH);    
    if (count > 1){
      delay(wait);
    }
    c++;
  }
}

bool readPin(byte PIN){
  return digitalRead(PIN);
}
// -------------------------------- ESP functions END ----------------------------------

// -------------------------------- SETUP AP ----------------------------------
void setupWiFiAP(){
  #ifdef DEBUG
    Serial.println("AP MODE - WiFi config for CLIENT MODE");
    Serial.print("SSID");Serial.println(APssid); 
  #endif   
      
//  initPins();  
  WiFi.mode(WIFI_AP);
  if (!WiFi.softAP(APssid, APpass)){
    BlikLED(pinLED,5,300);
  }
  WiFi.macAddress(mac);
  macAddr = "";
  for (int i=0;i<6;i++){
    if (mac[i] < 16) macAddr += "0";
    macAddr += String(mac[i],HEX) + ":";
  }  

  createWebServer();
        
  server.begin();  
}

// -------------------------------- HTTP SERVER functions BEGIN ----------------------------------
void handleMain(){
  String content = "<!DOCTYPE HTML>\r\n<html><h1>&#128295; ";
    content += devType;
    content += " WiFi &amp; params setup</h1><p>IP: " + WiFi.softAPIP().toString() + "</p><p>MAC: " + macAddr + "</p>";
    content += "<form method='get' action='setting'>";
    content += "<label>&#128246; SSID: </label><input name='ssid' value='" + ssid + "' />(string)<br />";
    content += "<label>&#128272; PASS: </label><input name='pass' value='" + pass + "' />(string)<br />";
    content += "<label>&#128268; Battery Charge Threshold Voltage: </label><input name='volt' value='";
    content += getRTCthreshold();
    content += "' />[V](float)<br /><label>&#128336; Interval: </label><input name='int' value='";
    content += getRTCinterval();
    content += "' />[min](int)<br /><label>&#9968; Sea level: </label><input name='msl' value='";
    content += getRTCmasl();
    content += "' />[m](float)<br /><input type='submit'></form><br /><br /><br /><a href='/restart'>Restart</a><hr />&#169;2017 Petr KLOSKO, <a href=www.klosko.net target=new>www.klosko.net</a> v.";
    content += FWver;
    content += "</html>";
  server.send(200, "text/html", content); 
}

void handleRestart(){
  String content = "<!DOCTYPE HTML>\r\n<html><h1>&#128295; ";
    content += devType;
    content += " restarting ...</h1><p>IP: " + WiFi.softAPIP().toString() + "</p><p>MAC: " + macAddr + "</p>";
    content += "<hr />&#169;2017 Petr KLOSKO, <a href=www.klosko.net target=new>www.klosko.net</a> v.";
    content += FWver;
    content += "</html>";
  server.send(200, "text/html", content); 
  BlikLED(pinLED,20,75);
  BlikLED(pinLED,5,300);
  deepSleep(100,true);
}

void handleSettings(){
  String qsid  = server.arg("ssid");
  String qpass = server.arg("pass");
  float  qcht  = server.arg("volt").toFloat(); 
  float  qmsl  = server.arg("msl").toFloat(); 
  byte   qint  = server.arg("int").toInt(); 
  byte   ssidpass_ln = (qsid.length() + qpass.length());
    
  int statusCode;
  String content = "<!DOCTYPE HTML>\r\n<html><h1>&#128295; ";
    content += devType;
    content += " WiFi parameters</h1><p>IP: " + WiFi.softAPIP().toString() + "</p><p>MAC: " + macAddr + "</p>";

  if ( ssidpass_ln > sizeof(cfgData.SSID_PASS) ) {
    content += "<h2>SSID and PASS length out of range</h2>";
    content += "<h3>";
    content += ssidpass_ln;
    content += " &gt; ";
    content += sizeof(cfgData.SSID_PASS);
    content += " !!! </h3><br /><a href='/'>Back to setup</a><br /><br /><br /><hr />&#169;2017 Petr KLOSKO, <a href=www.klosko.net target=new>www.klosko.net</a> v.";
    content += FWver;
    content += "</html>";
    statusCode = 507;    
  }else if ((qsid.length() > 0) && (qpass.length() > 0) ) {
    storeRTCWiFiSSID_PASS(qsid, qpass);
    BlikLED(pinLED, 1 , 300);
    storeRTCthreshold(qcht);
    BlikLED(pinLED, 1 , 300);
    storeRTCinterval(qint);
    BlikLED(pinLED, 1 , 300);
    storeRTCmasl(qmsl);
    BlikLED(pinLED, 1 , 300);
    storeRTCstatus(getRTCstatus() & 0);  // set runMode = 0
    BlikLED(pinLED, 1 , 300);        

    SaveConfig(1);
    LoadConfig();
    
    content += "<h2>Saved to EEPROM ... <a href='/restart'>restart to boot into new wifi</a></h2>";
    content += "<p>&#128246; ssid: "+getRTCWiFiSSID()+"</p>";
    content += "<p>&#128272; pass: "+getRTCWiFiPass()+"</p>";
    content += "<p>&#128268; Battery Charge Threshold Voltage: ";
    content += getRTCthreshold();
    content += "</p>";
    content += "<p>&#128336; Interval: ";
    content += getRTCinterval();
    content += "<p>&#9968; Sea level: ";
    content += getRTCmasl();
    content += "<p>&#127939; Run Mode: ";
    content += (getRTCstatus() & 1);
    content += "</p>";
    content += "<h3>DeviceID: ";
    content += devID;
    content += "-" + macAddr + "</h3><br /><a href='/'>Back to setup</a><br /><br /><br /><a href='/restart'>Restart</a><hr />&#169;2017 Petr KLOSKO, <a href=www.klosko.net target=new>www.klosko.net</a> v.";
    content += FWver;
    content += "</html>";
    statusCode = 200;
  }else{
      content += "<h2>404 not found</h2></html>";
    statusCode = 404;
  }
  server.send(statusCode, "text/html", content); 
}

void createWebServer(){
  server.on("/", handleMain);
  server.on("/setting", handleSettings);
  server.on("/restart", handleRestart);
}
// -------------------------------- HTTP SERVER functions END ----------------------------------

void loop() {
  BlikLED(pinLED, 1 , 300*3);
  BlikLED(pinLED, 2 , 300);    
  server.handleClient();
}


