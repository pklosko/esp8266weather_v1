/*  Simple sketch for sending data to the TMEP.cz
  by Petr KLOSKO (https://www.klosko.net)
  
 This sketch is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.
  
*/  
//#define DEBUG

// Define settings
const char FWver[]     = "20170714-01";

/*
 v.20170325-01:
    - based on v. 20170301-01 of esp8255thermo:

 v.20170327-01:
    - Added Battery and charge switch support
    - Added OTA update
    
 v.20170328-01:
    - Changed statusByte solution
    - Added BLOB data http GET
    - Fix "bit" operations
    - Fix "HTTP Commands" operations       

 v.20170330-03:     
    - Fix OTA update
    - Fix Charging (test)
    
 v.20170331-01:
    - Modify Charging
    - Modify Ststus info
    - Change PUSH/GET of data
    
 v.20170331-02:
    - Change I2C pins
    
 v.20170401-01:
    - Modify Sleep interval after connect error

 v.20170403-02:
    - Change LED, I2C & 1WIRE pins = Onboard LED is connected to GPIO2 instead of GPIO1

 v.20170404-01:
    - Deleted solarCellVoltage variable [unused]

 v.20170405-01:
    - Go to deepSleep after OTA update [do not push data]

 v.20170411-03:
    - Solar Intensity [BH1750] support added

 v.20170519-01:
    - Solve BUG : When 0V on DS2428 ADC pin, chip read 10.22V 

 v.20170606-01:
    - Modify HTTP request & timeout handling - due to delay during WUderground PUSH (/push/ PHP script)

 v.20170714-01:
    - Add Heat Index calculation         
*/

// include libraries
#include <ESP8266WiFi.h>       // WiFi library
#include <OneWire.h>           // OneWire communication library for DS18B20
#include <DS2438.h> // DS18B20 library
#include <EEPROM.h>            // EEPROM library
#include <ESP8266WebServer.h>  // Web server library
#include <ESP8266httpUpdate.h> //Update libratry
#include <Wire.h>
#include <BMP280.h>
#include <AM2320.h>
#include <BH1750.h>
#include <math.h>

// constants for DewPoint calculation
#define TRH_b 17.67
#define TRH_c 243.50
#define TRH_d 234.50

// constants for Sea level pressure calculation
// MODIFY FOR YOUR SEAL LEVEL !!!
#define MASL 300.00

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
const String SET_NEWFW_CMD     = "SET-NEWFW<";
const String SET_SETUP_CMD     = "SET-SETUP<1>";
const String SET_FLIPFLOP_CMD  = "SET-FLIPFLOP<1>";
/*
 * const byte pinSCL      = 0;                    // OUT, I2C SCL
const byte pinSDA      = 4;                    //      I2C SDA
const byte pin1Wire    = 2;                    //      1-Wire
 */
// Defult values
byte  sleepInterval    = 1;                    // How often send data to the server. In minutes
byte  statusByte       = 0;                    // Status byte
float chargeThreshold  = 3.45;                 // Charge Battery Low Threshold
float solarThreshold   = 3.99;                 // Solar Threshold
//float solarCellVoltage = 6.00;                 // Solar cell voltage
byte  runMode          = 0;    
byte mac[6];                                   // the MAC address of your Wifi shield


/* Define EEPROM addresses
EEPROM map>
addr   0 Device status
       0 byte  Status Register 
               bit0 = (1)  = runMode  - 0 = Client, 1 = Setup AP
                  1 = (2)  = Last CMD - 0 = ERR,    1 = OK
                  2 = (4)  = Charger  - 0 = OFF,    1 = ON
                  3 = (8)  = OTAupdteStatus LSb
                  4 = (16) = OTAupdteStatus MSb = 
                  5 = (32) =
                  6 = (64) =
                  7 = (128) = 

                                    
addr   4 Device config
offset 0 byte  DeepSleep Interval
       1 byte  ChargeThresholdVoltage * 100  LSB
       2 byte  ChargeThresholdVoltage * 100  MSB 
              
addr   16 WiFi STA config
offset  0 byte  ssid_length
        1 byte  total_length
        2 varial  ssid pass String
*/
byte AddrDeviceStatus      = 0;        
byte AddrDeviceConfig      = 4;  
byte AddrDeviceWiFiCfg     = 16;   
byte eeSize                = 254;


// -------------------------------- VARIABLES ----------------------------------
String macAddr;
long   rssi;
String dsAddr;
String SensType;                   // Sensor type
String ssid;
String pass;
//String httpResponse;

// Weather values vars.
float th_t = -127.00, th_h = -127.00, th_dp = -127.00, bm_sl = -127.00, uc = -127.00, ds_t = -127.00, ds_u = -127.00, ds_s = -127.00, lux = -127;
double bm_p = -127.00, bm_t = -127.00;

DeviceAddress deviceAddress;

// -------------------------------- OBJECTS AND HW SETTINGS ----------------------------------
OneWire oneWire(pin1Wire);         // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DS2438 ds2438(&oneWire);             //, DS2438_address);
ESP8266WebServer server(80);
BMP280 bmp;
AM2320 th;
BH1750 lightMeter(0x23);

// setup ADC - read Vcc voltage
ADC_MODE(ADC_VCC);


// -------------------------------- MAIN SETUP ---------------------------------- 
void setup() {
  #ifdef DEBUG
    delay(5000);
    initSerial();
  #endif  
  EEPROM.begin(eeSize); 

// read setting from EEPROM
  statusByte       = getEEstatus();
    runMode        = (statusByte & 1);
  ssid             = getEEWiFiSSID();
  pass             = getEEWiFiPass();
  sleepInterval    = getEEinterval();  
  chargeThreshold  = getEEthreshold();

  #ifdef DEBUG
    Serial.println();
    Serial.print("EEssid:");Serial.println(ssid);
    Serial.print("EEpass:");Serial.println(pass);
    Serial.print("EEinterval:");Serial.println(sleepInterval);    
    Serial.print("EEthreshold:");Serial.println(chargeThreshold);    
    Serial.print("EEstatus:");Serial.println(statusByte, BIN);  
    Serial.print("runMode:");Serial.println(runMode, BIN);  
  #endif

  if ((statusByte & 26) == 26){
    bitClear(statusByte,1);  
    storeEEstatus(statusByte);
    deepSleep(sleepInterval, false);
  }

  
  initPins();    

  #ifdef DEBUG
    Serial.println("initPins OK (setup)"); 
  #endif  
    
  if (ssid == "" || sleepInterval == 255 || sleepInterval == 0   // after first power off
                 || readPin(pinSETUP) == LOW || runMode == 1){   // go to conf via PIN or HTTP-COMMAND ,, 
    storeEEstatus(bitClear(statusByte,0));  // set runMode = 0
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

//    setupWiFiAP();
  
}

// -------------------------------- MAIN WEATHER ---------------------------------- 
void weatherClient(){
//  EEPROM.end();
  init1Wire();
  int i2cfl = initI2C(pinSDA, pinSCL);
  lightMeter.begin(BH1750_CONTINUOUS_HIGH_RES_MODE);
  
  uc = ESP.getVcc() / 1024.00;
  

  initWiFiClient();

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
  if (i2cfl){
    readTHDP(th_t, th_h, th_dp);
    readTPSL(bm_p, bm_t, bm_sl);
  }
  ChragrgeBatt(ds_s, ds_u);
  lux = lightMeter.readLightLevel();
  
  String httpResponse = DoHTTPrequest(ds_t, ds_u, ds_s,
                                      th_t, th_h, th_dp,
                                      bm_p, bm_t, bm_sl,
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
  if(!bmp.begin(SDA, SCL)){
  #ifdef DEBUG 
    Serial.print("initI2C: SDA=.....");Serial.println(SDA);
    Serial.print("initI2C: SCL=.....");Serial.println(SCL);
  #endif    
  while(1);
     return 0;
  }
  else bmp.setOversampling(4);  
  #ifdef DEBUG 
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

void initWiFiClient(){
  // Connect to the WiFi
  #ifdef DEBUG 
    Serial.print(F("Connecting to ")); Serial.print(ssid); Serial.println(pass); 
  #endif  
//  WiFi.forceSleepWake();
  WiFi.mode(WIFI_STA);  
  WiFi.begin(ssid.c_str(), pass.c_str());
  WiFi.macAddress(mac);
  macAddr = "";
  for (int i=0;i<6;i++){
    if (mac[i] < 16) macAddr += "0";
    macAddr += String(mac[i],HEX);
  }
  byte c = 0;
  byte a = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    c++;
    a++;
    #ifdef DEBUG 
      Serial.print(F("."));
    #endif    
    if (c == 50){
      #ifdef DEBUG
        Serial.printf("Connection status: %d\n", WiFi.status());
        Serial.println(macAddr);
      #endif      
      c = 0;      
      WiFi.disconnect();
      delay(500);
      WiFi.begin(ssid.c_str(), pass.c_str());
    }
    if (a > 100){
     //    ESP.reset();
      deepSleep(sleepInterval, false);
    }
  }
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
  storeEEstatus(statusByte);  
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

void readTHDP(float &th_t, float &th_h, float &th_dp){
  if (th.Read() == 0) {
    th_t = th.t;
    th_h = th.h;
    float trh = log((th_h/100.00) * exp((TRH_b-(th_t/TRH_d))*(th_t/(TRH_c+th_t))) );
    th_dp = (TRH_c * trh) / (TRH_b - trh);
  }else{
    th_t = -127.00; 
    th_h = -127.00; 
    th_dp = -127.00;
  }
}

void readTPSL(double &bm_p, double &bm_t, float &bm_sl){
  char result = bmp.startMeasurment();
  if(result!=0){
    delay(result);
    result = bmp.getTemperatureAndPressure(bm_t,bm_p);
    if(result!=0){
      bm_sl = bmp.sealevel(bm_p,MASL);
    }else{
      bm_sl = -127.00;  
      bm_t  = -127.00;
      bm_p  = -127.00;          
    }
  }else{
    bm_sl = -127.00;
    bm_t  = -127.00;
    bm_p  = -127.00;
  }
}

float convertCtoF(float c) {
  return c * 1.8 + 32;
}

float convertFtoC(float f) {
  return (f - 32) * 0.55555;
}

//boolean isFahrenheit: True == Fahrenheit; False == Celcius
float computeHeatIndex(float temperature, float percentHumidity, bool isFahrenheit) {
  // Using both Rothfusz and Steadman's equations
  // http://www.wpc.ncep.noaa.gov/html/heatindex_equation.shtml
  float hi;

  if (!isFahrenheit)
    temperature = convertCtoF(temperature);

  hi = 0.5 * (temperature + 61.0 + ((temperature - 68.0) * 1.2) + (percentHumidity * 0.094));

  if (hi > 79) {
    hi = -42.379 +
             2.04901523 * temperature +
            10.14333127 * percentHumidity +
            -0.22475541 * temperature*percentHumidity +
            -0.00683783 * pow(temperature, 2) +
            -0.05481717 * pow(percentHumidity, 2) +
             0.00122874 * pow(temperature, 2) * percentHumidity +
             0.00085282 * temperature*pow(percentHumidity, 2) +
            -0.00000199 * pow(temperature, 2) * pow(percentHumidity, 2);

    if((percentHumidity < 13) && (temperature >= 80.0) && (temperature <= 112.0))
      hi -= ((13.0 - percentHumidity) * 0.25) * sqrt((17.0 - abs(temperature - 95.0)) * 0.05882);

    else if((percentHumidity > 85.0) && (temperature >= 80.0) && (temperature <= 87.0))
      hi += ((percentHumidity - 85.0) * 0.1) * ((87.0 - temperature) * 0.2);
  }

  return isFahrenheit ? hi : convertFtoC(hi);
}

// -------------------------------- SENSORS functions END ----------------------------------

// -------------------------------- HTTP CLIENT functions BEGIN ----------------------------------

String createBLOB(float t, float th_h, double bm_sl, float th_dp, float hi, float ds_u, float ds_s, float ds_t, double bm_p, float uc, float lux, byte statusByte){
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
                     float th_t, float th_h, float th_dp,
                     double bm_p, double bm_t, double bm_sl,
                     float uc, float lux){
  String httpResponse = "";
  float t, hi;
  if (th_t != -127.00 && bm_t != -127.00){
    t = (th_t > bm_t) ? bm_t : th_t;
  }else{
    t = (th_t < bm_t) ? bm_t : th_t;
  }
  hi = computeHeatIndex(t, th_h, false);
    
  #ifdef DEBUG
    Serial.print(F("ds_t ")); Serial.println(ds_t);
  #endif  
  String blob = createBLOB(t, th_h, bm_sl, th_dp, hi, ds_u, ds_s, ds_t, bm_p, uc, lux, statusByte);
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
    // If you didn't get a connection to the server
    // Blink 3 times when host connection error
//    delay(1000);
    httpResponse = "Client connection failed";
  }else{
    #ifdef DEBUG 
      Serial.println(F("Client connected")); 
    #endif 

    String url = "/push/";
    
    /*?t=";
           url += t;
           url += "&h=";
           url += th_h;
           url += "&psl=";
           url += bm_sl;
           url += "&dp=";
           url += th_dp;
                      
           url += "&ub=";
           url += ds_u;
           url += "&us=";
           url += ds_s;

//           url += "&si=";
//           url += (ds_s * 100) / solarCellVoltage;
           
           url += "&tb=";
           url += ds_t;

           url += "&p=";
           url += bm_p;

//           url += "&ut=";
//           url += millis();
           url += "&uc=";
           url += uc;
//           url += "&blob=";
//           url += blob;
//           url += "&ch=";
//           url += sb_ch;
//           url += "&sb=";
//           url += statusByte;
      */     
    #ifdef DEBUG 
      Serial.print(F("Requesting URL: ")); Serial.println(url); 
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
                                ";Sensor:" + SensType + 
                                "+BMP280+AM2320+BH1750;SensID:" + dsAddr + 
                                ";Uptime:" + millis() + 
//                                ";DBG:" + ds_s +                                 
                                ";Conn:WiFi;SSID:" + ssid +
                                ";RSSI:" + WiFi.RSSI() + 
                                ";IP:" + WiFi.localIP().toString() + "\r\n" + 
                 "Connection: close\r\n\r\n");
  
    BlikLED(pinLED,1,200);
  }
  
  // Workaroud for timeout
  unsigned long timeout = millis();
  while(client.connected() && (millis() - timeout < (HTTP_TIMEOUT+10))){
    while (client.available()) { //== 0
      char c = client.read();
      httpResponse += c;
//      #ifdef DEBUG 
//        Serial.print(c);
//      #endif 
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
  storeEEstatus(statusByte);     
}

void RebootToSetup(){
  bitSet(statusByte,0);  // set runMode = 1
  bitSet(statusByte,1);    
  storeEEstatus(statusByte);   
  deepSleep(1,true);
}

void ModifyChargeThreshold(String httpResponse){
  int ro = find_text(SET_THRESHOLD_CMD, httpResponse) + SET_THRESHOLD_CMD.length();
  int rc = httpResponse.indexOf('>', ro);
  float newThreshold = httpResponse.substring(ro,rc).toFloat();
  #ifdef DEBUG
    Serial.print(F("New Threshold: "));Serial.println(newThreshold);
  #endif  
  if (newThreshold > 0 && chargeThreshold != newThreshold){
    storeEEthreshold(newThreshold);
    bitSet(statusByte,1);  
    chargeThreshold = newThreshold;
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
    storeEEinterval(newInterval);
    bitSet(statusByte,1);   
    if (newInterval < sleepInterval){
      sleepInterval = newInterval;
    }
  }
}


void OTAupdate(String httpResponse){ 
  int ro = find_text(SET_NEWFW_CMD, httpResponse) + SET_NEWFW_CMD.length();
  int rc = httpResponse.indexOf('>', ro);
  String newFW = httpResponse.substring(ro,rc);  
  #ifdef DEBUG
    Serial.print(F("OTA Update: "));Serial.println(newFW);
  #endif   
//  t_httpUpdate_return ret = ESPhttpUpdate.update(host, 80, "/OTAupdate", devID);
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
  storeEEstatus(statusByte); 
        
  t_httpUpdate_return ret = ESPhttpUpdate.update(url);
   switch (ret) {
      case HTTP_UPDATE_FAILED:
        #ifdef DEBUG
          Serial.printf("[%d] HTTP_UPDATE_FAILED Error (%d): %s\n", ret, ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
          Serial.println(F("HTTP_UPDATE_FAILED"));
        #endif 
        bitSet(statusByte,3);  
        bitClear(statusByte,4);  
        storeEEstatus(statusByte); 
        break;
      case HTTP_UPDATE_NO_UPDATES:
        #ifdef DEBUG
          Serial.println(F("HTTP_UPDATE_NO_UPDATES"));
        #endif 
        bitClear(statusByte,3);  
        bitSet(statusByte,4); 
        storeEEstatus(statusByte); 
        break;
      case HTTP_UPDATE_OK:
        #ifdef DEBUG
          Serial.println(F("HTTP_UPDATE_OK"));
        #endif 
        break;
   }
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
    content += getEEthreshold();
    content += "' />(float)<br /><label>&#128336; Interval: </label><input name='int' value='";
    content += getEEinterval();
    content += "' />(integer)<br /><input type='submit'></form><br /><br /><br /><a href='/restart'>Restart</a><hr />&#169;2017 Petr KLOSKO, <a href=www.klosko.net target=new>www.klosko.net</a> v.";
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
  deepSleep(1,true);
}

void handleSettings(){
  String qsid  = server.arg("ssid");
  String qpass = server.arg("pass");
  float  qcht  = server.arg("volt").toFloat(); 
  byte   qint  = server.arg("int").toInt(); 
    
  int statusCode;
  String content = "<!DOCTYPE HTML>\r\n<html><h1>&#128295; ";
    content += devType;
    content += " WiFi parameters</h1><p>IP: " + WiFi.softAPIP().toString() + "</p><p>MAC: " + macAddr + "</p>";

  if ((qsid.length() > 0) && 
      (qpass.length() > 0)
     ) {
    storeEEWiFiSSID_PASS(qsid, qpass);
    BlikLED(pinLED, 1 , 300);
    storeEEthreshold(qcht);
    BlikLED(pinLED, 1 , 300);
    storeEEinterval(qint);
    BlikLED(pinLED, 1 , 300);
    storeEEstatus(getEEstatus() & 0);  // set runMode = 0
    BlikLED(pinLED, 1 , 300);        
    content += "<h2>Saved to EEPROM ... <a href='/restart'>restart to boot into new wifi</a></h2>";
    content += "<p>&#128246; ssid: "+getEEWiFiSSID()+"</p>";
    content += "<p>&#128272; pass: "+getEEWiFiPass()+"</p>";
    content += "<p>&#128268; Battery Charge Threshold Voltage: ";
    content += getEEthreshold();
    content += "</p>";
    content += "<p>&#128336; Interval: ";
    content += getEEinterval();
    content += "<p>&#128336; Run Mode: ";
    content += (getEEstatus() & 1);
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

// -------------------------------- EEPROM functions BEGIN ----------------------------------
float getEEthreshold(){
  return ((EEPROM.read(AddrDeviceConfig+1)*256) + EEPROM.read(AddrDeviceConfig+1+1)) / 100.00;
}

void storeEEthreshold(float threshold){
  unsigned int vInt = (int)(threshold * 100);
  EEPROM.write(AddrDeviceConfig+1,   (int)(vInt/256));
  EEPROM.write(AddrDeviceConfig+1+1, (int)(vInt % 256));  
  EEPROM.commit();  
}

String getEEWiFiSSID(){
  #ifdef DEBUG
    Serial.print("Read ssid from EEPROM :");    
  #endif    
  String esid = "";
  byte ssid_ln  = EEPROM.read(AddrDeviceWiFiCfg);
  #ifdef DEBUG
    Serial.print(AddrDeviceWiFiCfg);Serial.print("; ssid_ln=");Serial.println(ssid_ln);  
  #endif  
  if (ssid_ln != 255 && ssid_ln != 0){
    for(byte i=0; i<ssid_ln; i++){
      esid += char(EEPROM.read(i+AddrDeviceWiFiCfg+2));
    }  
  }else{
    esid = "";
  }
  #ifdef DEBUG
    Serial.println(esid);    
  #endif   
  return esid;
}

String getEEWiFiPass(){
  #ifdef DEBUG
    Serial.print("Read pass from EEPROM : addr=");    
  #endif 
  String epas = "";
  byte ssid_ln  = EEPROM.read(AddrDeviceWiFiCfg);
  byte total_ln = EEPROM.read(AddrDeviceWiFiCfg+1)-ssid_ln; 
  #ifdef DEBUG
    Serial.print(AddrDeviceWiFiCfg);Serial.print("; ssid_ln=");Serial.print(ssid_ln);Serial.print("; total_ln=");Serial.println(total_ln);  
  #endif   
  for(byte i=0; i<total_ln; i++){
    epas += char(EEPROM.read(i+AddrDeviceWiFiCfg+2+ssid_ln));
  }
  #ifdef DEBUG
    Serial.println(epas);    
  #endif   
  return epas;   
}

void storeEEWiFiSSID_PASS(String esid, String epas){
  byte es_ln = esid.length();
  byte ep_ln = es_ln + epas.length();
  #ifdef DEBUG
    Serial.println("Store ssid & pass to EEPROM :");
    Serial.print("SSID :");Serial.print(esid);Serial.print("  / ");Serial.print(es_ln);
    Serial.print("PASS :");Serial.print(epas);Serial.print("  / ");Serial.println(ep_ln);
  #endif  
  EEPROM.write(AddrDeviceWiFiCfg, es_ln);
  EEPROM.write(AddrDeviceWiFiCfg+1, ep_ln);  
  String data = esid + epas;
  #ifdef DEBUG
    Serial.println(data);
  #endif  
  for(byte i=0; i<ep_ln; i++){
    EEPROM.write(i+AddrDeviceWiFiCfg+2, data[i]);       
  }
  EEPROM.commit();
}

byte getEEstatus(){
  return EEPROM.read(AddrDeviceStatus);
}

void storeEEstatus(byte statusByte){
  #ifdef DEBUG
    Serial.print(F("Write status  to EEPROM "));Serial.println(statusByte);
    Serial.print(F("Write status  to EEPROM "));Serial.println(statusByte, BIN);
  #endif  
  EEPROM.write(AddrDeviceStatus, statusByte);
  EEPROM.commit();  
}

byte getEEinterval(){
  return EEPROM.read(AddrDeviceConfig);
}

void storeEEinterval(byte interval){
  #ifdef DEBUG
    Serial.print(F("Write interval  to EEPROM "));Serial.println(interval);
  #endif  
  EEPROM.write(AddrDeviceConfig, interval);
  EEPROM.commit();  
}
// -------------------------------- EEPROM functions END ----------------------------------

void loop() {
  BlikLED(pinLED, 1 , 300*3);
  BlikLED(pinLED, 2 , 300);    
  server.handleClient();
}

