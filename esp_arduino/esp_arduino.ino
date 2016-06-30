//                    arduinoESP8266 wifi & eeprom setting template
// ----------------------------------- libs -----------------------------------
#include <ESP8266WiFi.h>
#include <EEPROM.h>
#include <ArduinoJson.h>
#include <WiFiUdp.h>

static char s_hw_serial_number[] = "123456";
static char s_hw_version[] = "0";

#define EEPROM_START 0

/*settings managment stuff*/
uint32_t memcrc;
uint8_t *p_memcrc = (uint8_t*)&memcrc;

struct eeprom_data_t {  
  char ssid[32];
  char pwd[32];
  long long serial;
  bool configured;
} eeprom_data;

static PROGMEM prog_uint32_t crc_table[16] = {
  0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac, 0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
  0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c, 0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c
};

/**/
// ----------------------------------- crc_update -----------------------------------
unsigned long crc_update(unsigned long crc, byte data)
{
  byte tbl_idx;
  tbl_idx = crc ^ (data >> (0 * 4));
  crc = pgm_read_dword_near(crc_table + (tbl_idx & 0x0f)) ^ (crc >> 4);
  tbl_idx = crc ^ (data >> (1 * 4));
  crc = pgm_read_dword_near(crc_table + (tbl_idx & 0x0f)) ^ (crc >> 4);
  return crc;
}
// ----------------------------------- crc_byte -----------------------------------
unsigned long crc_byte(byte *b, int len)
{
  unsigned long crc = ~0L;
  uint8_t i;

  for (i = 0 ; i < len ; i++)
  {
    crc = crc_update(crc, *b++);
  }
  crc = ~crc;
  return crc;
}
// -----------------------------------  -----------------------------------

// ----------------------------------- base64 https://github.com/adamvr/arduino-base64/blob/master/Base64.cpp -----------------------------------
const char b64_alphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
                            "abcdefghijklmnopqrstuvwxyz"
                            "0123456789+/";
int base64_encode(char *output, char *input, int inputLen) {
  int i = 0, j = 0;
  int encLen = 0;
  unsigned char a3[3];
  unsigned char a4[4];

  while (inputLen--) {
    a3[i++] = *(input++);
    if (i == 3) {
      a3_to_a4(a4, a3);

      for (i = 0; i < 4; i++) {
        output[encLen++] = b64_alphabet[a4[i]];
      }

      i = 0;
    }
  }

  if (i) {
    for (j = i; j < 3; j++) {
      a3[j] = '\0';
    }

    a3_to_a4(a4, a3);

    for (j = 0; j < i + 1; j++) {
      output[encLen++] = b64_alphabet[a4[j]];
    }

    while ((i++ < 3)) {
      output[encLen++] = '=';
    }
  }
  output[encLen] = '\0';
  return encLen;
}
inline void a3_to_a4(unsigned char * a4, unsigned char * a3) {
  a4[0] = (a3[0] & 0xfc) >> 2;
  a4[1] = ((a3[0] & 0x03) << 4) + ((a3[1] & 0xf0) >> 4);
  a4[2] = ((a3[1] & 0x0f) << 2) + ((a3[2] & 0xc0) >> 6);
  a4[3] = (a3[2] & 0x3f);
}
int base64_enc_len(int plainLen) {
  int n = plainLen;
  return (n + 2 - ((n + 2) % 3)) / 3 * 4;
}
/**/

// ----------------------------------- setup -----------------------------------

WiFiServer server(12345);
StaticJsonBuffer<512> jsonBuffer;

void setupSettings(const char *ssid, const char *pwd, long long serial) {
  strcpy(eeprom_data.ssid, ssid);
  strcpy(eeprom_data.pwd, pwd);
  //eeprom_data.serial = serial;
  eeprom_data.configured = true;
  
  int i;
  byte eeprom_data_tmp[sizeof(eeprom_data)];

  EEPROM.begin(sizeof(eeprom_data) + sizeof(memcrc));

  memcpy(eeprom_data_tmp, &eeprom_data, sizeof(eeprom_data));

  for (i = EEPROM_START; i < sizeof(eeprom_data); i++)
  {
    EEPROM.write(i, eeprom_data_tmp[i]);
  }
  memcrc = crc_byte(eeprom_data_tmp, sizeof(eeprom_data_tmp));

  EEPROM.write(i++, p_memcrc[0]);
  EEPROM.write(i++, p_memcrc[1]);
  EEPROM.write(i++, p_memcrc[2]);
  EEPROM.write(i++, p_memcrc[3]);


  EEPROM.commit();
}

bool readSettings() {
  int i;
  uint32_t datacrc;
  byte eeprom_data_tmp[sizeof(eeprom_data)];
  
  EEPROM.begin(sizeof(eeprom_data) + sizeof(memcrc));
  
  for (i = EEPROM_START; i < sizeof(eeprom_data); i++)
  {
    eeprom_data_tmp[i] = EEPROM.read(i);
  }
  
  p_memcrc[0] = EEPROM.read(i++);
  p_memcrc[1] = EEPROM.read(i++);
  p_memcrc[2] = EEPROM.read(i++);
  p_memcrc[3] = EEPROM.read(i++);
  
  datacrc = crc_byte(eeprom_data_tmp, sizeof(eeprom_data_tmp));
  
  if (memcrc == datacrc)
  {
    memcpy(&eeprom_data, eeprom_data_tmp,  sizeof(eeprom_data));
    return true;
  }
  else
  {
    return false;
  }
}

bool doBinding() {
  WiFi.begin("Rescan Bind AP", "\"123456789\"");
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  // Start the server
  server.begin();
  Serial.println("Server started");
  Serial.print("IP:");
  Serial.println(WiFi.localIP());
  
  //binding algo
  WiFiClient client;
  do {
    client = server.available();
    Serial.print(".");
    delay(500);
  } while (!client);
 
  // Wait until the client sends some data
  Serial.println("new client");
  //while(!client.available()){
  //  delay(1);
  //}

  JsonObject& root = jsonBuffer.createObject();
  root["id"] = s_hw_serial_number;
  root["version"] = s_hw_version;
  root.printTo(client);
  client.flush();
  
  client.flush();
  String response = client.readStringUntil('\r');
  
  
  JsonObject& object = jsonBuffer.parseObject(response);
  const char* ssid = object["SSID"];
  const char* pwd = object["PWD"];
  Serial.println(ssid);
  Serial.println(pwd);
  if (strlen(ssid) > 0 && strlen(pwd) > 0) {
    Serial.println("Trying to store settings...!");
    setupSettings(ssid, pwd, 0);
    Serial.println("Settings wrttent successfully!");
  }
}

String IpAddress2String(const IPAddress& ipAddress)
{
  return String(ipAddress[0]) + String(".") +\
  String(ipAddress[1]) + String(".") +\
  String(ipAddress[2]) + String(".") +\
  String(ipAddress[3])  ; 
}

WiFiUDP udp;
bool startAdvertising(WiFiClient& client) {
  WiFi.begin(eeprom_data.ssid, eeprom_data.pwd);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  // Start the server
  server.begin();
  Serial.println("Start advertising from ");
  Serial.println(WiFi.localIP());
  
  udp.begin(666);
  IPAddress ip = WiFi.localIP();
  ip[3] = 255;

  JsonObject& root = jsonBuffer.createObject();
  root["id"] = s_hw_serial_number;
  root["version"] = s_hw_version;
  root["ip"] = IpAddress2String(WiFi.localIP());
  String s;
  root.printTo(s);
    
  do {
  // transmit broadcast package
    udp.beginPacket(ip, 6666);
    udp.write(s.c_str());
    udp.endPacket();

    client = server.available();
    delay(500);
  } while (!client);

  return true;
}

bool clientReady = false;

void setup() {
  EEPROM.begin(512);
  
  delay(1);
  Serial.begin(115200); 
  Serial.println("ready!!!");

  bool needBinding = !readSettings();
  if (needBinding || !eeprom_data.configured) {
    Serial.println("Binding!!!");
    doBinding();
    ESP.reset();
    return;
  }

  Serial.println("Configured!!!");
  server.begin();
  WiFiClient client;

  if (!startAdvertising(client)) {
    
  }
  
  clientReady = true;
  /*if (eeprom_data.STAenabled == true)
  {
    //WiFi.mode(WIFI_AP_STA);
    WiFi.mode(WIFI_STA); // AP on STA timeout or STA connect
    STAinit();
  }
  else
  {
    //WiFi.mode(m): set mode to WIFI_AP, WIFI_STA, or WIFI_AP_STA.
    WiFi.mode(WIFI_AP);
    softAPinit();

  }




  nextSensTick = millis();
  nextSerialTick =  millis() ;
  nextwifiCheckTick = millis() ;
  nextSenderTick = 20000 + millis() ;*/

}

// ----------------------------------- loop -----------------------------------
void loop() {
  if (clientReady) {
      delay(500);
      Serial.println("!!!!");
  }
}
