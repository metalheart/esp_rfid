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

#define MAX_SCAN_TASKS 16
#define SCAN_TASK_TAG_LEN 14

enum task_status_t {
  TS_NEW = 1,
  TS_COMPLETED = 1 << 1
};

typedef struct scan_task {
  byte tag[SCAN_TASK_TAG_LEN];
  task_status_t status;
} scan_task_t;

typedef struct scan_tasks {
  scan_task_t tasks[MAX_SCAN_TASKS];
  byte num_tasks;
} scan_tasks_t;

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


WiFiServer server(12345);

StaticJsonBuffer<512> jsonBuffer;

bool doBinding() {
  WiFi.begin("Rescan Bind AP", "\"123456789\"");
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print("]");
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
    Serial.print("]");
    delay(500);
  } while (!client);
 
  // Wait until the client sends some data
  Serial.println("new client");
  
  JsonObject& root = jsonBuffer.createObject();
  root["id"] = s_hw_serial_number;
  root["version"] = s_hw_version;
  root.printTo(client);
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
  return String(ipAddress[0]) + String(".")
    + String(ipAddress[1]) + String(".")
    + String(ipAddress[2]) + String(".")
    + String(ipAddress[3]); 
}

bool startAdvertising(WiFiClient& client) {
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(eeprom_data.ssid, eeprom_data.pwd);
  }
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print("-");
  }
  
  Serial.println("");
  Serial.println("WiFi connected");

  // Start the server
  server.begin();

  IPAddress ip = WiFi.localIP();
  ip[3] = 255;

  Serial.println("Start advertising from ");
  Serial.println(ip);

  JsonObject& connectPkt = jsonBuffer.createObject();
  connectPkt["id"] = s_hw_serial_number;
  connectPkt["version"] = s_hw_version;
  connectPkt["ip"] = IpAddress2String(WiFi.localIP());
  
  WiFiUDP udp;
  
  do {
  // transmit broadcast package
    udp.beginPacket(ip, 6666);
    connectPkt.printTo(udp);
    udp.endPacket();
    delay(500);
    client = server.available();
    
    Serial.print("...");
  } while (!client.connected());

  udp.stopAll();
  
  return true;
}

WiFiClient client;
  
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

  if (!startAdvertising(client)) {
    
  }
}

// ----------------------------------- loop -----------------------------------
scan_tasks_t _scan_tasks;

bool allTasksCompleted(const scan_tasks_t& tasks) {
  
  for (int i = 0; i < tasks.num_tasks; ++i) {
      if (!(tasks.tasks[i].status & TS_COMPLETED)) {
        return false;
      }
  }

  return true;
}

bool readyForNewTsak(const scan_tasks_t& tasks) {
  return tasks.num_tasks == 0 || allTasksCompleted(tasks);
}

//static char taskBuffer[512];
const char* readJsonSafe(WiFiClient& client) {
  static char tmpBuff[512+1] = "";
  
  int available = client.available();
  //Serial.print("readJsonSafe: available ");
  //Serial.print(available);
  //Serial.println(" bytes");
  
  if (available > 1) {
    if (available >= 512) {
      available = 512;
    }
    client.peekBytes(tmpBuff, available);
    
    int brackets = 0;
    int msgLen = 0;
    bool msgRead = false;
    
    while (msgLen < available && !msgRead) {
      if (tmpBuff[msgLen] == '{') {
        brackets += 1;
      } else if (tmpBuff[msgLen] == '}') {
        brackets -= 1;
        if (brackets == 0) {
          msgRead = true;
          ++ msgLen;
          break;
        }
      }
      ++ msgLen;
    }

    if (brackets == 0 && msgRead) {
      client.read((uint8_t*)tmpBuff, msgLen);
      tmpBuff[msgLen] = 0;
      return tmpBuff;
    }
  }

  return NULL;
}

void loop() {
  if (WiFi.status() == WL_CONNECTED && client.connected()) {

    const char* szJsonStr = readJsonSafe(client);
    
    if (szJsonStr) {
      //Serial.println(szJsonStr);
      JsonObject& object = jsonBuffer.parseObject(szJsonStr);
      if (object.success()) {
        String type = object.get<String>("type");
        if (type == "task")
        {
          if (readyForNewTsak(_scan_tasks)) {
            //try to fetch new tasks from server
            Serial.println("Waiting for messages...");
            
          
            Serial.println("Incoming msg...");
          
            JsonArray& taskArray = object["tasks"].asArray();
            int numTasks = taskArray.size() > MAX_SCAN_TASKS ? MAX_SCAN_TASKS : taskArray.size();
            for (int i = 0; i < numTasks; ++i) {
              const char *tag = taskArray.get<const char*>(i);
              if (strlen(tag) == SCAN_TASK_TAG_LEN) {
                Serial.print("Task read:");
                Serial.println(tag);
                
                memcpy(_scan_tasks.tasks[i].tag, tag, SCAN_TASK_TAG_LEN);
              } else {
                //handle error
              }
              _scan_tasks.tasks[i].status = TS_NEW;
            }
            _scan_tasks.num_tasks = numTasks;
      
          }
        }
        else
        {
          client.println("{\"type\":\"pong\"}");
        }
      }
    }
    
    //try to read scanned tag from serial
    if (Serial.available() > 0) 
    {
      char tag[14+1];
      for (int i = 0; i < 14; ++i) {
        tag[i] = Serial.read();
      }
      tag[14] = 0;
      Serial.flush();

      Serial.print("RFID read:");
      Serial.println(tag);

      for (int i = 0; i < _scan_tasks.num_tasks; ++i) {
        if (memcmp(tag, (const char*)_scan_tasks.tasks[i].tag, SCAN_TASK_TAG_LEN) ==0)
        {
          _scan_tasks.tasks[i].status = TS_COMPLETED;
          Serial.print("Task completed!");
        }
      }

      //send status over the network..
    }
  } else {
    startAdvertising(client);
  }

  delay(100);
}
