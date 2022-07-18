bool LoadNetConfig(String CRCssid, int addr){
  if (ESP.rtcUserMemoryRead(addr, (uint32_t*) &cfgNetwork, sizeof(cfgNetwork))){  
    uint32_t crcOfSSID = calculateCRC32((uint8_t*) CRCssid.c_str(), sizeof(CRCssid)); 
    uint32_t crcOfData = calculateCRC32(((uint8_t*)&cfgNetwork)+4, sizeof(cfgNetwork)-4); 
    #ifdef DEBUG
      printNetMemory();
      Serial.print(F("CRCcalc  ")); Serial.println(crcOfSSID);      
      Serial.print(F("CRCram  ")); Serial.println(cfgNetwork.SSIDcrc32);  
      Serial.print(F("CRCcalcData  ")); Serial.println(crcOfData);      
      Serial.print(F("CRCramData  ")); Serial.println(cfgNetwork.crc32);
    #endif 
    return ((crcOfSSID == cfgNetwork.SSIDcrc32) && (crcOfData == cfgNetwork.crc32));
  }else{
    return false;
  }
}

bool SaveNetConfig(String CRCssid, int addr){
  cfgNetwork.SSIDcrc32 = calculateCRC32((uint8_t*) CRCssid.c_str(), sizeof(CRCssid));  
  cfgNetwork.crc32 = calculateCRC32(((uint8_t*)&cfgNetwork)+4, sizeof(cfgNetwork)-4); 
    #ifdef DEBUG
      Serial.print(F("SAVE NET CONFIG CRCram  ")); Serial.println(cfgNetwork.SSIDcrc32);  
      Serial.print(F("SAVE NET CONFIG CRCramData  ")); Serial.println(cfgNetwork.crc32);  
    #endif   
  return ESP.rtcUserMemoryWrite(addr, (uint32_t*) &cfgNetwork, sizeof(cfgNetwork));
}

void LoadConfig(){
  CfgFlag = "EE";   
  if (ESP.rtcUserMemoryRead(0, (uint32_t*) &cfgData, sizeof(cfgData))){  
    #ifdef DEBUG
      printMemory();
    #endif
    uint32_t crcOfData = calculateCRC32(((uint8_t*)&cfgData)+4, sizeof(cfgData)-4);
    if (crcOfData != cfgData.crc32){
      #ifdef DEBUG
        Serial.println();
        Serial.print("rtcUserMemoryRead  CRC FAILED");
      #endif
      EEPROM.begin(eeSize);
      EEPROM.get(AddrDeviceCfgAddr,cfgData);
      SaveConfig(false);
    }else{
      CfgFlag = "RTC";    
    }
  }else{
    #ifdef DEBUG
      Serial.println();
      Serial.print("rtcUserMemoryRead  FAILED");
    #endif
    EEPROM.begin(eeSize);
    EEPROM.get(AddrDeviceCfgAddr,cfgData);
    SaveConfig(false);
  }
  statusByte       = getRTCstatus();
  ssid             = getRTCWiFiSSID();
  pass             = getRTCWiFiPass();
  sleepInterval    = getRTCinterval();  
  chargeThreshold  = getRTCthreshold();
  MASL             = getRTCmasl();
  VccRatio         = getRTCvccRatio();  
}

void SaveConfig(bool ToEE){
  cfgData.crc32 = calculateCRC32(((uint8_t*)&cfgData)+4, sizeof(cfgData)-4);
  // Write struct to RTC memory
  if (ESP.rtcUserMemoryWrite(0, (uint32_t*) &cfgData, sizeof(cfgData))) {
    #ifdef DEBUG
      Serial.println("SaveConfig: ");
      printMemory();
      Serial.println();      
    #endif    
  }
  if (ToEE){
    EEPROM.begin(eeSize);
    EEPROM.put(AddrDeviceCfgAddr,cfgData);
    EEPROM.commit();
    EEPROM.end();
  }  
}

String getRTCWiFiSSID(){
  #ifdef DEBUG
    Serial.print("Read ssid from RTC :");    
  #endif    
  String esid = "";
//  EEPROM.begin(eeSize); 
  byte ssid_ln  = cfgData.SSID_ln;
  #ifdef DEBUG
    Serial.print("; ssid_ln=");Serial.println(ssid_ln);  
  #endif  
  if (ssid_ln != 255 && ssid_ln != 0){
    for(byte i=0; i<ssid_ln; i++){
      esid += char(cfgData.SSID_PASS[i]);
    }  
  }else{
    esid = "";
  }  
  #ifdef DEBUG
    Serial.println(esid);    
  #endif   
  return esid;
}

String getRTCWiFiPass(){
  #ifdef DEBUG
    Serial.print("Read pass from RTC : addr=");    
  #endif  
  String epas = "";
  byte ssid_ln  = cfgData.SSID_ln;
  byte total_ln = cfgData.TOTAL_ln-ssid_ln; 
  #ifdef DEBUG
    Serial.print("; ssid_ln=");Serial.print(ssid_ln);Serial.print("; total_ln=");Serial.println(total_ln);  
  #endif  
  for(byte i=0; i<total_ln; i++){
    epas += char(cfgData.SSID_PASS[i+ssid_ln]);
  }
  #ifdef DEBUG
    Serial.println(epas);    
  #endif    
  return epas;   
}

void storeRTCWiFiSSID_PASS(String esid, String epas){
  byte es_ln = esid.length();
  byte ep_ln = es_ln + epas.length();
  #ifdef DEBUG
    Serial.println("Store ssid & pass to RTC :");
    Serial.print("SSID :");Serial.print(esid);Serial.print("  / ");Serial.print(es_ln);
    Serial.print("PASS :");Serial.print(epas);Serial.print("  / ");Serial.println(ep_ln);
  #endif
  cfgData.SSID_ln  = es_ln;
  cfgData.TOTAL_ln = ep_ln;
  String data = esid + epas;
  #ifdef DEBUG
    Serial.println(data);
  #endif
  for(byte i=0; i<ep_ln; i++){
    #ifdef DEBUG
      Serial.print(data[i]);
    #endif    
    cfgData.SSID_PASS[i] = data[i];
  }
  #ifdef DEBUG
    Serial.println();
  #endif
}

byte getRTCinterval(){
  return cfgData.Interval;
}

byte getRTCstatus(){
  return cfgData.Status;
}

float getRTCthreshold(){
  return cfgData.Threshold  / 100.00;
}

float getRTCmasl(){
  return cfgData.MASL  / 100.00;
}

float getRTCvccRatio(){
  return cfgData.VccRatio  / 1000.00;  
}
  
void storeRTCvccRatio(float ratio){
  uint16_t VccR = (uint16_t)(ratio * 1000);
  #ifdef DEBUG
    Serial.print(F("Write VccR to RTC "));Serial.println(VccR);
  #endif
  cfgData.VccRatio = VccR;
}
  
void storeRTCinterval(byte interval){
  #ifdef DEBUG
    Serial.print(F("Write interval  to RTC "));Serial.println(interval);
  #endif
  cfgData.Interval = interval;
}

void storeRTCstatus(byte Status){
  #ifdef DEBUG
    Serial.print(F("Write status  to RTC "));Serial.println(Status);
  #endif
  cfgData.Status = Status;
}

void storeRTCthreshold(float threshold){
  uint16_t THR = (uint16_t)(threshold * 100);
  #ifdef DEBUG
    Serial.print(F("Write threshold to RTC "));Serial.println(THR);
  #endif
  cfgData.Threshold = THR;
}

void storeRTCmasl(float masl){
  uint16_t MAS = (uint16_t)(masl * 100);
  #ifdef DEBUG
    Serial.print(F("Write MASL to RTC "));Serial.println(MAS);
  #endif
  cfgData.MASL = MAS;
}

uint32_t calculateCRC32(const uint8_t *data, size_t length) {
  uint32_t crc = 0xffffffff;
  while (length--) {
    uint8_t c = *data++;
    for (uint32_t i = 0x80; i > 0; i >>= 1) {
      bool bit = crc & 0x80000000;
      if (c & i) {
        bit = !bit;
      }
      crc <<= 1;
      if (bit) {
        crc ^= 0x04c11db7;
      }
    }
  }
  return crc;
}

#ifdef DEBUG
//prints all rtcData, including the leading crc32
  void printMemory() {
    char buf[3];
    uint8_t *ptr = (uint8_t *)&cfgData;
    for (size_t i = 0; i < sizeof(cfgData); i++) {
      sprintf(buf, "%02X", ptr[i]);
      Serial.print(buf);
      if ((i + 1) % 32 == 0) {
        Serial.println();
      } else { 
        Serial.print(" ");
      }
    }
    Serial.println();
  }

//prints all rtcData, including the leading crc32
  void printNetMemory() {
    char buf[3];
    uint8_t *ptr = (uint8_t *)&cfgNetwork;
    for (size_t i = 0; i < sizeof(cfgNetwork); i++) {
      sprintf(buf, "%02X", ptr[i]);
      Serial.print(buf);
      if ((i + 1) % 32 == 0) {
        Serial.println();
      } else { 
        Serial.print(" ");
      }
    }
    Serial.println();
  }  
#endif
