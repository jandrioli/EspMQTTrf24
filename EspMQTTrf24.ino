/*
 Basic ESP8266 MQTT example

 This sketch demonstrates the capabilities of the pubsub library in combination
 with the ESP8266 board/library.

 It connects to an MQTT server then:
  - publishes "hello world" to the topic "outTopic" every two seconds
  - subscribes to the topic "inTopic", printing out any messages
    it receives. NB - it assumes the received payloads are strings not binary
  - If the first character of the topic "inTopic" is an 1, switch ON the ESP Led,
    else switch it off

 It will reconnect to the server if the connection is lost using a blocking
 reconnect function. See the 'mqtt_reconnect_nonblocking' example for how to
 achieve the same result without blocking the main loop.

 To install the ESP8266 board, (using Arduino 1.6.4+):
  - Add the following 3rd party board manager under "File -> Preferences -> Additional Boards Manager URLs":
       http://arduino.esp8266.com/stable/package_esp8266com_index.json
  - Open the "Tools -> Board -> Board Manager" and click install for the ESP8266"
  - Select your ESP8266 in "Tools -> Board"

*/
#include <ArduinoJson.h>
#include "FS.h"
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

// Update these with values suitable for your network.

#define useCredentialsFile
#ifdef useCredentialsFile
#include "credentials.h"
#else
mySSID = "    ";
myPASSWORD = "   ";
#endif

#define MOD                "esp1/"
#define CAT                "feeds/"
#define T_SWITCH1          "switch1"
#define T_SWITCH2          "switch2"
#define T_SWITCH3          "switch3"
#define T_SWITCH4          "switch4"
#define T_SCHEDULE1        "schedule1"
#define T_SCHEDULE2        "schedule2"
#define T_DURATION1        "duration1"
#define T_DURATION2        "duration2"
#define T_TEMP             "temperatureThreshold"
#define T_COMMAND          "provideStatus"
#define T_CURSTATUS        "currentStatus"

// 
// SW Logic and firmware definitions
// 
#define THIS_NODE_ID 3                  // master is 0, unoR3 debugger is 1, promicro_arrosoir is 2, etc
#define DEFAULT_ACTIVATION 600          // 10h from now we activate (in case radio is down and can't program)
#define DEFAULT_DURATION 10             // max 10s of activation time by default

/**
 * exchange data via radio more efficiently with data structures.
 * we can exchange max 32 bytes of data per msg. 
 * schedules are reset every 24h (last for a day) so an INTEGER is
 * large enough to store the maximal value of a 24h-schedule.
 * temperature threshold is rarely used
 */
struct relayctl {
  unsigned long uptime = 0;                      // current running time of the machine (millis())  4 bytes  
  unsigned long sched1 = DEFAULT_ACTIVATION;     // schedule in minutes for the relay output nbr1   4 bytes
  unsigned long sched2 = DEFAULT_ACTIVATION;     // schedule in minutes for the relay output nbr2   4 bytes
  unsigned int  maxdur1 = DEFAULT_DURATION;      // max duration nbr1 is ON                         2 bytes
  unsigned int  maxdur2 = DEFAULT_DURATION;      // max duration nbr2 is ON                         2 bytes
  unsigned long sched3 = DEFAULT_ACTIVATION;     // schedule in minutes for the relay output nbr1   4 bytes
  unsigned long sched4 = DEFAULT_ACTIVATION;     // schedule in minutes for the relay output nbr2   4 bytes
  unsigned int  maxdur3 = DEFAULT_DURATION;      // max duration nbr1 is ON                         2 bytes
  unsigned int  maxdur4 = DEFAULT_DURATION;      // max duration nbr2 is ON                         2 bytes
  unsigned int  temp_thres = 999;                // temperature at which the syatem is operational  4 bytes
  float         temp_now   = 20;                 // current temperature read on the sensor          4 bytes
  short         battery    =  0;                 // current temperature read on the sensor          2 bytes
  bool          state1 = false;                  // state of relay output 1                         1 byte
  bool          state2 = false;                  // "" 2                                            1 byte
  bool          state3 = false;                  // state of relay output 1                         1 byte
  bool          state4 = false;                  // "" 2                                            1 byte
  bool          waterlow = false;                // indicates whether water is low                  1 byte
  byte          nodeid = 3;           // nodeid is the identifier of the slave           1 byte
} myData;

WiFiClient espClient;
PubSubClient client(espClient);
String g_nwSSID = "", g_nwPASS = "", g_nwMQTT = "192.168.8.1";
long lastMsg = 0;
char msg[50];
int value = 0;


bool loadConfig() {
  File configFile = SPIFFS.open("/config.json", "r");
  if (!configFile) {
    Serial.println("Failed to open config file");
    return false;
  }

  size_t size = configFile.size();
  if (size > 1024) {
    Serial.println("Config file size is too large");
    return false;
  }

  // Allocate a buffer to store contents of the file.
  std::unique_ptr<char[]> buf(new char[size]);

  // We don't use String here because ArduinoJson library requires the input
  // buffer to be mutable. If you don't use ArduinoJson, you may as well
  // use configFile.readString instead.
  configFile.readBytes(buf.get(), size);

  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& json = jsonBuffer.parseObject(buf.get());

  if (!json.success()) {
    Serial.println("Failed to parse config file");
    return false;
  }

  const char* nwSSID = json["ssid"];
  const char* nwPASS = json["pass"];
  const char* nwMQTT = json["mqtt"];
  /*Serial.println((nwSSID));  
  Serial.println((nwPASS));
  Serial.println((nwMQTT));*/

  // Real world application would store these values in some variables for
  // later use.

  g_nwSSID = String(nwSSID);
  g_nwPASS = String(nwPASS);
  g_nwMQTT = String(nwMQTT);

  Serial.println(("["+g_nwSSID+"]"));  
  Serial.println(("["+g_nwPASS+"]"));
  Serial.println(("["+g_nwMQTT+"]"));
  if (g_nwSSID.length() < 4 || g_nwPASS.length() < 6)
  {
    Serial.println("SSID or PSK were too short, defaulting to hard-coded nw.");
    g_nwSSID = mySSID;
    g_nwPASS = myPASSWORD;
  }
  return true;
}

bool saveConfig() {
  
  char cSSID[g_nwSSID.length()], cPASS[g_nwPASS.length()], cMQTT[g_nwMQTT.length()];
  g_nwSSID.toCharArray(cSSID, g_nwSSID.length());    
  g_nwPASS.toCharArray(cPASS, g_nwPASS.length());    
  g_nwMQTT.toCharArray(cMQTT, g_nwMQTT.length());
  Serial.println("Saving new SSID,PASS,MQTT ");
  Serial.print  (cSSID);
  Serial.print  (cPASS);
  Serial.println(cMQTT);
  Serial.print  (g_nwSSID);
  Serial.print  (g_nwPASS);
  Serial.println(g_nwMQTT);
  
  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& json = jsonBuffer.createObject();
  json["ssid"] = cSSID;
  json["pass"] = cPASS;
  json["mqtt"] = cMQTT;

  File configFile = SPIFFS.open("/config.json", "w");
  if (!configFile) {
    Serial.println("Failed to open config file for writing");
    return false;
  }

  json.printTo(configFile);
  return true;
}



void setup_wifi() {

  delay(10);

    // Connect to WiFi network
    Serial.println(F("Connecting"));
    
    
    WiFi.mode(WIFI_STA);
    WiFi.begin(mySSID, myPASSWORD);

    int timeout = 90;
    while (WiFi.status() != WL_CONNECTED) 
    {
      Serial.print(".");
      if (timeout == 60) // a basic connect timeout sorta thingy
      {
        Serial.println();
        Serial.print("Failed to connect to WiFi nw. Status is now ");
        Serial.println(WiFi.status());
        WiFi.printDiag(Serial);
        WiFi.begin(mySSID2, myPASSWORD2);
      }
      if (timeout == 30) // a basic connect timeout sorta thingy
      {
        Serial.println();
        Serial.print("Failed to connect to WiFi nw. Status is now ");
        Serial.println(WiFi.status());
        WiFi.printDiag(Serial);
        WiFi.begin(mySSID3, myPASSWORD3);
      }
      if (--timeout < 1) // a basic connect timeout sorta thingy
      {
        break;
      }
      delay(1000);
    }
  

  Serial.println(F(""));
  Serial.println(F("WiFi connected"));
  Serial.println(F("IP address: "));
  WiFi.printDiag(Serial);
  Serial.println(WiFi.localIP());  
}

void setup_spiffs()
{
  uint32_t realSize = ESP.getFlashChipRealSize();
  uint32_t ideSize = ESP.getFlashChipSize();
  FlashMode_t ideMode = ESP.getFlashChipMode();

  printf("Flash real id:   %08X\n", ESP.getFlashChipId());
  printf("Flash real size: %u\n\n", realSize);

  printf("Flash ide  size: %u\n", ideSize);
  printf("Flash ide speed: %u\n", ESP.getFlashChipSpeed());
  printf("Flash ide mode:  %s\n", (ideMode == FM_QIO ? "QIO" : ideMode == FM_QOUT ? "QOUT" : ideMode == FM_DIO ? "DIO" : ideMode == FM_DOUT ? "DOUT" : "UNKNOWN"));
  if(ideSize != realSize) 
  {
      Serial.println("Flash Chip configuration wrong!\n");
  } 
  else 
  {
      Serial.println("Flash Chip configuration ok.\n");
      Serial.println("Mounting SPIFFS...");
      if (!SPIFFS.begin()) {
        Serial.println("Failed to mount file system");
        return;
      }
      if (!loadConfig()) {
        Serial.println("Failed to load config");
      } else {
        Serial.println("Config loaded");
      }
  }
  Serial.println(F("- - - - -"));  
}

void setup() 
{
  //
  // Print preamble
  //
  Serial.begin(115200);
  delay(500);
  Serial.println(F("\n\rESP MQTT RF24 - An MQTT client that ctrols RF24 nodes"));  
  delay(500);
  Serial.println(F("Warning! Always query the controller node before attempting to program it!"));  
  delay(500);
  Serial.println(F("If you have trouble interfacing, try the Serial interface to configure me."));  
  delay(500);
  Serial.println(F("- - - - -"));  
  
  
  //prepare and configure SPIFFS
  setup_spiffs();
  
  // setting up WLAN related stuff 
  setup_wifi();

  char cMQTTserver[g_nwMQTT.length()+1];
  g_nwMQTT.toCharArray(cMQTTserver, g_nwMQTT.length()+1);
  client.setServer(cMQTTserver, 1880);
  client.setCallback(callback);
}


void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // Switch on the LED if an 1 was received as first character
  String sTopic(topic), sPayload = "";
  for (int i = 0; i < length; i++) 
  {
    if (!isDigit( payload[i] ))
      break;
    sPayload += (char)payload[i];
  }
  if (sTopic.indexOf(T_SWITCH1)>=0) 
  {
    myData.state1 = ((char)payload[0] == '1')?true:false;
//    digitalWrite(HW_RELAY1, myData.state1);   
  }
  if (sTopic.indexOf(T_SWITCH2)>=0) 
  {
    myData.state2 = ((char)payload[0] == '1')?true:false;
//    digitalWrite(HW_RELAY2, myData.state2);   
  }
  if (sTopic.indexOf(T_SWITCH3)>=0) 
  {
    myData.state3 = ((char)payload[0] == '1')?true:false;
//    digitalWrite(HW_RELAY1, myData.state1);   
  }
  if (sTopic.indexOf(T_SWITCH4)>=0) 
  {
    myData.state4 = ((char)payload[0] == '1')?true:false;
//    digitalWrite(HW_RELAY2, myData.state2);   
  }
  if (sTopic.indexOf(T_TEMP)>=0) 
  {
    myData.temp_thres = sPayload.toInt();
  }
  if (sTopic.indexOf(T_SCHEDULE1)>=0) 
  {
    myData.sched1 = sPayload.toInt();
  }
  if (sTopic.indexOf(T_SCHEDULE2)>=0) 
  {
    myData.sched2 = sPayload.toInt();
  }
  if (sTopic.indexOf(T_DURATION1)>=0) 
  {
    myData.maxdur1 = sPayload.toInt();
  }
  if (sTopic.indexOf(T_DURATION2)>=0) 
  {
    myData.maxdur2 = sPayload.toInt();
  }
  if (sTopic.indexOf(T_COMMAND)>=0) 
  {
    String sUp = String(myData.uptime);
    String sS1 = String(myData.sched1);
    String sS2 = String(myData.sched2);
    String sD1 = String(myData.maxdur1);
    String sD2 = String(myData.maxdur2);
    String sT1 = String(myData.temp_thres);
    String sT2 = String(myData.temp_now);
    String sB1 = String(myData.state1);
    String sB2 = String(myData.state2);
    String sWL = String(myData.waterlow);
    String sStatus("Uptime:"+sUp+" Schedule1:"+sS1+" Schedul2:"+sS2+" Duration1:"+sD1+" Duration2:"+sD2+" Temperature:"+sT1+"/"+sT2+" WaterLow:"+sWL);
    char qS[sStatus.length()] ;
    sStatus.toCharArray(qS, sStatus.length());
    Serial.println(sStatus);
    client.publish(MOD CAT T_CURSTATUS, (uint8_t*)qS, sStatus.length());
  }
}

void reconnect() {
  short max_retries = 0;
  // Loop until we're reconnected
  while (!client.connected()) 
  {
    if (max_retries++ > 2) 
    {
      break;
    }
    Serial.print("Connect to MQTT broker... "); 
    // Attempt to connect
    if (client.connect("ESP8266Client")) 
    {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("outTopic", "hello world");
      // ... and resubscribe
      client.subscribe(MOD CAT T_COMMAND, 1);
      client.subscribe(MOD CAT T_SWITCH1, 1);
      client.subscribe(MOD CAT T_SWITCH2, 1);
      client.subscribe(MOD CAT T_SWITCH3, 1);
      client.subscribe(MOD CAT T_SWITCH4, 1);
      client.subscribe(MOD CAT T_DURATION1, 1);
      client.subscribe(MOD CAT T_DURATION2, 1);
      client.subscribe(MOD CAT T_DURATION3, 1);
      client.subscribe(MOD CAT T_DURATION4, 1);
      client.subscribe(MOD CAT T_SCHEDULE1, 1);
      client.subscribe(MOD CAT T_SCHEDULE2, 1);
      client.subscribe(MOD CAT T_SCHEDULE3, 1);
      client.subscribe(MOD CAT T_SCHEDULE4, 1);
      client.subscribe(MOD CAT T_TEMP, 1);
    } else 
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(2000);
    }
  }
}


void loop() 
{

  if (!client.connected()) 
  {
    reconnect();
  }
  if (client.connected()) 
  {
    client.loop();
    long now = millis();
    if (now - lastMsg > 10000) {
      lastMsg = now;
      ++value;
      snprintf (msg, 75, "hello world #%ld", value);
      Serial.print("Publish message: ");
      Serial.println(msg);
      client.publish("outTopic", msg);
    }
  }

  /* 
   *  Because I dunno how I will eventually need to debug this thing,
   *  I am placing serial iface support here too. Sucks. 
   */
  while (Serial.available())
  {
    String s1 = Serial.readStringUntil('\n');
    Serial.println(F("CAREFUL, end of line is only NL and no CR!!!"));
    Serial.print("You typed:");
    Serial.println(s1);
    if (s1.indexOf("setnewssid ")>=0)
    {
      s1 = s1.substring(s1.indexOf(" ")+1);
      g_nwSSID = s1.substring(0, s1.length());
      Serial.println(("new ssid is now [" + g_nwSSID + "]"));
    }
    else if (s1.indexOf("setnewpass ")>=0)
    {
      s1 = s1.substring(s1.indexOf(" ")+1);
      g_nwPASS = s1.substring(0, s1.length());
      Serial.println(("new pass is now [" + g_nwPASS + "]"));
    }
    else if (s1.indexOf("setnewmqtt ")>=0)
    {
      s1 = s1.substring(s1.indexOf(" ")+1);
      g_nwMQTT = s1.substring(0, s1.length());
      Serial.println(("new mqtt is now [" + g_nwMQTT + "]"));
    }
    else if (s1.indexOf("setnewcfg")>=0)
    {
      saveConfig();
    }
    else if ((s1.indexOf("setnewpass")!=0) && (s1.indexOf("setnewssid")!=0) && (s1.indexOf("setnewmqtt")!=0))
    {
      Serial.println("** Serial interface expects:\n\r"\
        "** 0 - setnewssid: set a new SSID for the module\n\r"\
        "** 1 - setnewpass: set a new PSK key for the nw\n\r"\
        "** 2 - setnewmqtt: set a new MQTT server\n\r"\
        "** 3 - setnewcfg : save the configuration into a file on the flash");
    }
  }
}


