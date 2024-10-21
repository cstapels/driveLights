#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>  // only for esp_wifi_set_channel()
#include <ArduinoJson.h>
#include <Time.h>
#include <TimeLib.h>
#include "ThingSpeak.h"

// Global copy of friend
esp_now_peer_info_t myFriend;
#define prod
//#define demo
#define CHANNEL 1
#define PRINTSCANRESULTS 1
#define DELETEBEFOREPAIR 0
#define MAX_SEND_TRIES 10
#define MYRANK 1
#define MYSENDER MYRANK - 1
#define MYRECIEVER MYRANK + 1
#define LEDPin 2
#define TIMEOUT 15000
#define BATTERY_PIN 35

#define MORNING 6
#define NIGHT 18
#define BEDTIME 23
#define POWER_SWITCH 26
short messageReady = 0;
WiFiClient client;

// Network information
char *ssid = "still_waters";
const char *password = "33turkeys511";
#ifdef demo
const char *writeAPIKey = "U0Q9H69T2119MWGV";  //battery channel
const char *readAPIKey = "TS0D8HVAOZLRAJ1N";   //control channel
long readChannelId = 2060365;
long dataChannelId = 2061311;
const char *dataWriteAPIKey = "DSOLKXCIRX93TFJT";
#endif

#ifdef prod
const char *writeAPIKey = "Y0A92ZT5UMPKK3N1";  //battery channel
const char *readAPIKey = "Z74I62ST2YVADWOX";   //control channel
long readChannelId = 249466;
long dataChannelId = 364593;
const char *dataWriteAPIKey = "OV5KJROGLZED81ZH";
#endif

int wait2Sleep = 10;  // how long to stay awake before sleeping
long theSetTime = millis();
long sleepStart = 0;
long waitMessageTime = 1;
bool sleepOn = false;
int sleepUnit = 5;
String createdChange = "";  //change later but using to see tsp change
int deliverySuccess = 0;
long timeTimer = millis();
int deviceNum = 1;
bool checkBatteryFlag = false;
bool powerOn = false;
long TSPReadTime = 2000;
long lastTSPTime = millis();
long timeReadChannelId = 444067;
const char *timeReadAPIKey = "NBU9ULD2EJ1ELM9T";  //time control channel
int lastTime = 0;
bool POWERON = 0;
long measureBatteryTime = 9000;  //10000;

typedef struct struct_message {
  int bright;
  int color1;
  int color2;
  int color3;
  int pattern;
  int time;
  long sleepTime;
  int lightSpeed;
} struct_message;

struct_message myData;
struct_message returnData;

// Init ESP Now with fallback
void InitESPNow() {
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  } else {
    Serial.println("ESPNow Init Failed");
    // Retry InitESPNow, add a counte and then restart?
    // InitESPNow();
    // or Simply Restart
    ESP.restart();
  }
}

// config AP SSID
void configDeviceAP() {
  char SSID[10];
  //char SSID[10] ="Friend_";
  //SSID[7]=char(MYRANK);
  String SSIDString = "Friend_" + String(MYRANK);
  strcpy(SSID, SSIDString.c_str());
  //const char *SSID = "Friend_1";
  bool result = WiFi.softAP(SSID, "Friend_1_Password", CHANNEL, 0);
  if (!result) {
    Serial.println("AP Config failed.");
  } else {
    Serial.print("AP Config Success. Broadcasting with AP: " + String(SSID));
    Serial.print(" AP CHANNEL ");
    Serial.println(WiFi.channel());
  }
}

// Scan for devices in AP mode
void ScanForFriend() {
  int16_t scanResults = WiFi.scanNetworks(false, false, false, 300, CHANNEL);  // Scan only on one channel was 300 changed to 100
  // reset on each scan
  bool friendFound = 0;
  memset(&myFriend, 0, sizeof(myFriend));

  Serial.println("");
  if (scanResults == 0) {
    Serial.println("No WiFi devices in AP Mode found");
  } else {
    Serial.print("Found ");
    Serial.print(scanResults);
    Serial.println(" devices ");
    for (int i = 0; i < scanResults; ++i) {
      // Print SSID and RSSI for each device found
      String SSID = WiFi.SSID(i);
      int32_t RSSI = WiFi.RSSI(i);
      String BSSIDstr = WiFi.BSSIDstr(i);

      if (PRINTSCANRESULTS) {
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(SSID);
        Serial.print(" (");
        Serial.print(RSSI);
        Serial.print(")");
        Serial.println("");
      }
      delay(10);
      // Check if the current device is my target
      String target = "Friend_" + String(MYRECIEVER);
      //      if (SSID.indexOf("Friend") == 0) {
      if (SSID.indexOf(target) == 0) {
        // SSID of interest
        Serial.println("Found a Friend.");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(SSID);
        Serial.print(" [");
        Serial.print(BSSIDstr);
        Serial.print("]");
        Serial.print(" (");
        Serial.print(RSSI);
        Serial.print(")");
        Serial.println("");
        // Get BSSID => Mac Address of the Friend
        int mac[6];
        if (6 == sscanf(BSSIDstr.c_str(), "%x:%x:%x:%x:%x:%x", &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5])) {
          for (int ii = 0; ii < 6; ++ii) {
            myFriend.peer_addr[ii] = (uint8_t)mac[ii];
          }
        }

        myFriend.channel = CHANNEL;  // pick a channel
        myFriend.encrypt = 0;        // no encryption

        friendFound = 1;
        // we are planning to have only one slave in this example;
        // Hence, break after we find one, to be a bit efficient
        break;
      }
    }
  }

  if (friendFound) {
    Serial.println("Friend Found, processing..");
  } else {
    Serial.println("Friend Not Found, trying again.");
  }

  // clean up ram
  WiFi.scanDelete();
}

// Check if the friend is already paired with the master.
// If not, pair the friend with master
bool manageFriend() {
  if (myFriend.channel == CHANNEL) {
    if (DELETEBEFOREPAIR) {
      deletePeer();
    }

    Serial.print("Friend Status: ");
    // check if the peer exists
    bool exists = esp_now_is_peer_exist(myFriend.peer_addr);
    if (exists) {
      // Slave already paired.
      Serial.println("Already Paired");
      return true;
    } else {
      // Slave not paired, attempt pair
      esp_err_t addStatus = esp_now_add_peer(&myFriend);
      if (addStatus == ESP_OK) {
        Serial.println("Pair success");  // Pair success
        return true;
      } else if (addStatus == ESP_ERR_ESPNOW_NOT_INIT) {
        // How did we get so far!!
        Serial.println("ESPNOW Not Init");
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_ARG) {
        Serial.println("Invalid Argument");
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_FULL) {
        Serial.println("Peer list full");
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_NO_MEM) {
        Serial.println("Out of memory");
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_EXIST) {
        Serial.println("Peer Exists");
        return true;
      } else {
        Serial.println("Not sure what happened");
        return false;
      }
    }
  } else {
    Serial.println("No Friend found to process");
    return false;
  }
}

void deletePeer() {
  esp_err_t delStatus = esp_now_del_peer(myFriend.peer_addr);
  Serial.print("Friend Delete Status: ");
  if (delStatus == ESP_OK) {
    // Delete success
    Serial.println("Success");
  } else if (delStatus == ESP_ERR_ESPNOW_NOT_INIT) {
    // How did we get so far!!
    Serial.println("ESPNOW Not Init");
  } else if (delStatus == ESP_ERR_ESPNOW_ARG) {
    Serial.println("Invalid Argument");
  } else if (delStatus == ESP_ERR_ESPNOW_NOT_FOUND) {
    Serial.println("Peer not found.");
  } else {
    Serial.println("Not sure what happened");
  }
}

uint8_t data = 0;
// send data
void sendData() {
  int sendTries = 0;
  const uint8_t *peer_addr = myFriend.peer_addr;
  Serial.print("Sending bytes: ");
  Serial.println(String(sizeof(myData)));

  while (!deliverySuccess && sendTries < MAX_SEND_TRIES) {
    Serial.print("Deliver succ " + String(deliverySuccess));
    esp_err_t result = esp_now_send(peer_addr, (uint8_t *)&myData, sizeof(myData));
    sendTries++;
    Serial.println("Send num " + String(sendTries));
    delay(240);  //probably better to move this counter to the function that calls it

    Serial.print("Send Status: ");
    if (result == ESP_OK) {
      Serial.println("Success");
      //messageReady=0;
    } else if (result == ESP_ERR_ESPNOW_NOT_INIT) {
      // How did we get so far!!
      Serial.println("ESPNOW not Init.");
    } else if (result == ESP_ERR_ESPNOW_ARG) {
      Serial.println("Invalid Argument");
    } else if (result == ESP_ERR_ESPNOW_INTERNAL) {
      Serial.println("Internal Error");
    } else if (result == ESP_ERR_ESPNOW_NO_MEM) {
      Serial.println("ESP_ERR_ESPNOW_NO_MEM");
    } else if (result == ESP_ERR_ESPNOW_NOT_FOUND) {
      Serial.println("Peer not found.");
    } else {
      Serial.println("Not sure what happened");
    }
  }
  deliverySuccess = 0;  //reste for next time
  myData.pattern = 0;   //for some reason I am triggering on this as a mechanisn to send data.  Probably should use a flag
}

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Sent to: ");
  Serial.print(macStr);
  Serial.print(" Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  status == ESP_NOW_SEND_SUCCESS ? deliverySuccess = 1 : deliverySuccess = 0;
}

// callback when data is recv from Master
//void OnDataRecv(const uint8_t *mac_addr, const uint8_t *recData, int data_len) {
void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *recData, int data_len) {
  uint8_t *mac_addr = recv_info->src_addr;
  uint8_t *des_addr = recv_info->des_addr;
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Recv from: ");
  Serial.print(macStr);
  Serial.print(" Recv Data: ");
  Serial.println(data_len);
  //Serial.println("");
  memcpy(&returnData, recData, data_len);
  // for (int y=0; y<data_len;y++){
  //   Serial.println(String(recData[y]));
  // }
  messageReady = 1;

  Serial.println(String(returnData.pattern));
  //data=*Rdata;
}

void connectWiFi() {


  WiFi.begin(ssid, password);
  long startTime = millis();
  long waitTime = 15000;
  while ((WiFi.status() != WL_CONNECTED) && (startTime - millis() < waitTime)) {
delay(3000);
   // blinkX(2, 950);
    Serial.println("Connecting to WiFi");
  }
  Serial.println("Connected");
  blinkX(3, 675);
  ThingSpeak.begin(client);
}

String readTSPTime() {

  //assume we are already connected
  int statusCode = ThingSpeak.readMultipleFields(timeReadChannelId, timeReadAPIKey);
  if (statusCode == 200) {
    int myYear = ThingSpeak.getFieldAsInt(1);
    int myMonth = ThingSpeak.getFieldAsInt(2);
    int myDay = ThingSpeak.getFieldAsInt(3);
    int myHour = ThingSpeak.getFieldAsInt(4);
    int myMin = ThingSpeak.getFieldAsInt(5);
    int mySecond = ThingSpeak.getFieldAsInt(6);
    String currentTime = ThingSpeak.getCreatedAt();

    setTime(myHour, myMin, mySecond, myDay, myMonth, myYear);

    String timeString = String(myYear) + String("-") + String(myMonth) + "-" + String(myDay) + " " + String(myHour) + ":" + String(myMin) + ":" + String(mySecond);

    return timeString;
  }
  return ("time Failed");
}

int32_t getWiFiChannel(const char *ssid) {
  if (int32_t n = WiFi.scanNetworks()) {
    for (uint8_t i = 0; i < n; i++) {
      if (!strcmp(ssid, WiFi.SSID(i).c_str())) {
        return WiFi.channel(i);
      }
    }
  }
  return 0;
}

String getCall() {
  if (!client.connect("api.thingspeak.com", 80)) {
    Serial.println("Connection failed");
    //lastConnectionTime = millis();
    client.stop();
    return "Connection failed";
  }

  //String url = "http://api.thingspeak.com/channels/" + String(readChannelId) + "/fields/" + String(myField) + "/last.txt?api_key=" + String(readAPIKey);
  String url = "http://api.thingspeak.com/channels/" + String(readChannelId) + "/feeds/last_data_age.txt?api_key=" + String(readAPIKey);

  Serial.println(url);
  client.println(String("GET ") + url);
  unsigned long timeout = millis();
  String answer = getResponse();
  Serial.print("answer is ");
  Serial.println(answer);
  return answer;
}

void setup() {
  pinMode(LEDPin, OUTPUT);
  pinMode(POWER_SWITCH, OUTPUT);
  blinkX(MYRANK, 150);
  Serial.begin(115200);
  //Set device in STA mode to begin with
  WiFi.mode(WIFI_AP_STA);

  configDeviceAP();
  connectWiFi();
  Serial.println(readTSPTime());
  //adjustDayTime();
  pingConnection();
}

void startESPNow() {
  WiFi.disconnect();
  client.stop();
  configDeviceAP();
  esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE);
  // Serial.print("STA MAC: ");
  // Serial.println(WiFi.macAddress());
  //Serial.print("STA CHANNEL ");
  //Serial.println(WiFi.channel());
  InitESPNow();
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
}

void processNow() {

  ScanForFriend();
  //sendData();

  if (myFriend.channel == CHANNEL) {  // check if friend channel is defined
    bool isPaired = manageFriend();   // Add friend as peer if it has not been added already
    if (isPaired) {                   // pair success or already paired
                                      //   if (messageReady == 1) {      // Send data to device
      sendData();
      //  messageReady = 0;
      //processMessage();
      //    }
    } else {
      Serial.println("Friend pair failed!");
    }
  } else {
    // No friend found to process
  }
}

void processMessage() {
  //do stuff with the message
  blinkX(int(MYRANK) + 10, 100);
  Serial.println("message returned");

  ThingSpeak.setField(1, returnData.bright);
  ThingSpeak.setField(2, returnData.color1);
  ThingSpeak.setField(3, returnData.color2);
  ThingSpeak.setField(4, returnData.color3);
  ThingSpeak.setField(5, returnData.pattern);
  ThingSpeak.setField(6, returnData.time);
  ThingSpeak.setField(7, returnData.sleepTime);
  ThingSpeak.setField(8, returnData.lightSpeed);

  int statusCode = 0;
  statusCode = ThingSpeak.writeFields(dataChannelId, dataWriteAPIKey);
  if (statusCode == 200) {
    Serial.println("Wrote to TSPK");
  } else {
    Serial.println("Failed to Write" + String(statusCode));
  }
  Serial.println("Control " + String(returnData.bright));
  Serial.println("Direction " + String(returnData.color1));
  Serial.println("Start Node " + String(returnData.color2));
  Serial.println("Batt Level " + String(returnData.color3));
  Serial.println("control pattern " + String(returnData.pattern));
  Serial.println("time " + String(returnData.time));
  Serial.println("sleeptime " + String(returnData.sleepTime));
  Serial.println("light time " + String(returnData.lightSpeed));
}

void pingConnection() {
  if ((WiFi.status() != WL_CONNECTED)) {
    connectWiFi();
  }
  ThingSpeak.setField(2, POWERON);
  ThingSpeak.setField(3, 1);
  ThingSpeak.setField(4, analogRead(BATTERY_PIN));

  ThingSpeak.setField(7, getStrength(3));
  ThingSpeak.setStatus("TestPing");
  int statusCode = 0;
  statusCode = ThingSpeak.writeFields(dataChannelId, dataWriteAPIKey);
  if (statusCode == 200) {
    Serial.println("wrote ping");
  } else {
    Serial.println("Failed to Write" + String(statusCode));
  }
}

void measureDevice(int deviceRank) {
  int patternNumber = 300 + deviceRank;
  myData.bright = 0;
  myData.pattern = patternNumber;
  myData.time = 26;  //things worked at 36 but stays awake for a while 16 seemed too short for devices bigger than 5
}

// Take measurements of the Wi-Fi strength and return the average result.
int getStrength(int points) {
  long rssi = 0;
  long averageRSSI = 0;

  for (int i = 0; i < points; i++) {
    rssi += WiFi.RSSI();
    delay(20);
  }

  averageRSSI = rssi / points;
  return averageRSSI;
}

void blinkX(int numTimes, int delayTime) {
  for (int g = 0; g < numTimes; g++) {  // Turn the LED on and wait.
    digitalWrite(LEDPin, HIGH);
    delay(delayTime);
    digitalWrite(LEDPin, LOW);  // Turn the LED off and wait.
    delay(delayTime);
  }
}

// Wait for a response from the server indicating availability,
// and then collect the response and build it into a string.
String getResponse() {
  String response = "";
  long startTime = millis();

  delay(200);
  while (client.available() < 1 & ((millis() - startTime) < TIMEOUT)) {
    delay(5);
  }

  if (client.available() > 0) {  // Get response from server.
    char charIn;
    do {
      charIn = client.read();  // Read a char from the buffer.
      response += charIn;      // Append the char to the string response.
    } while (client.available() > 0);
  }
  client.stop();

  return response;
}

void readFromTSP() {
  int statusCode = 0;
  statusCode = ThingSpeak.readMultipleFields(readChannelId, readAPIKey);
  Serial.println("Code: " + String(statusCode));
  if (statusCode == 200) {
    String createdAt = ThingSpeak.getCreatedAt();  // Created-at timestamp
    Serial.println("Created at " + createdAt);
    // Serial.println("sleep on "+ String(sleepOn) + " SleepStart " + String(sleepStart) + "millis " + String(millis())  + " Set Time " + String(theSetTime) + " myData.time " + String(myData.time));


    if (createdChange != createdAt) {
      createdChange = createdAt;
      lastTime = getCall().toInt();  //risky conversion here
      Serial.println("last read age " + String(lastTime));
      blinkX(2, 345);
      if (lastTime < 10) {
        myData.bright = ThingSpeak.getFieldAsInt(1);
        myData.color1 = ThingSpeak.getFieldAsInt(2);
        myData.color2 = ThingSpeak.getFieldAsInt(3);
        myData.color3 = ThingSpeak.getFieldAsInt(4);
        myData.pattern = ThingSpeak.getFieldAsInt(5);
        myData.time = ThingSpeak.getFieldAsInt(6);
        myData.sleepTime = ThingSpeak.getFieldAsInt(7);
        myData.lightSpeed = ThingSpeak.getFieldAsInt(8);

        Serial.println("Bright " + String(myData.bright));
        Serial.println("C1 " + String(myData.color1));
        Serial.println("C2 " + String(myData.color2));
        Serial.println("C3 " + String(myData.color3));
        Serial.println("pattern " + String(myData.pattern));
        Serial.println("time " + String(myData.time));
        Serial.println("sleeptime " + String(myData.sleepTime));
        Serial.println("light speed " + String(myData.lightSpeed));

        //if (sleepOn) { delay(sleepUnit * 1000); }  //this only works if sleep unit is small
        //delay until the sleep time is over
        theSetTime = millis();  //effect start time
      }
    }
  }
}

void adjustDayTime() {

  //adjust the wiat to sleep time and tine sleep unit
  if ((hour() > MORNING) and (hour() < NIGHT)) {  //daytime
    sleepUnit = 120;
    wait2Sleep = 35;
    Serial.println("Daytime");
  }
  if ((hour() > NIGHT) and (hour() < BEDTIME)) {  //nighttime
    sleepUnit = 5;
    wait2Sleep = 35;
    Serial.println("Nighttime");
  }
  if ((hour() > BEDTIME) or (hour() < MORNING)) {  //morning
    sleepUnit = 300;
    wait2Sleep = 35;
    Serial.println("Morningtime");
  }
}

void chainSendThenWait(int wait) {
  waitMessageTime = 5;

  startESPNow();
  processNow();
  if (wait) {
    waitMessageTime = 15 * 1000;
  }
  long tempTime = millis();
  while (((millis() - tempTime) < waitMessageTime) && (~messageReady)) {
    delay(5);  //long delay to wait for message, take this out later I hope
  }
}

void flipPower() {
  if (!POWERON) {
    POWERON = 1;
    digitalWrite(POWER_SWITCH, HIGH);
    Serial.println("poweron");
    delay(1000);
  }
}

void loop() {
  int bigTime;

  if (messageReady) {
    if ((WiFi.status() != WL_CONNECTED)) { connectWiFi(); }
    processMessage();  //this verrsion sends data to ThingSpeak
    messageReady = 0;
  }

  if (millis() - lastTSPTime > TSPReadTime) {
    if ((WiFi.status() != WL_CONNECTED)) { connectWiFi(); }
    lastTSPTime = millis();
    readFromTSP();  //get the latest control info
    Serial.println("Sleep on " + String(sleepOn));
    if(POWERON){
      blinkX(1,350);
    }
  }

  if (myData.color1 == 333) {  //should write a process special commands function here for a range of entries for control
    pingConnection();          // connects to wifi if needed
    myData.color1 = 0;
    checkBatteryFlag = false;  //dont check battery while light command is being sent?
  }
  if (myData.color1 == 444) {  //should write a process special commands function here for a range of entries for control
                               //use this to reset the battery measure time.  color is 444 and pattern is the time between measurements
    measureBatteryTime = long(myData.pattern);
    myData.color1 = 0;
    myData.pattern = 0;
    checkBatteryFlag = false;  //dont check battery while light command is being sent?
  }

  if ((myData.color1 + myData.color2 + myData.color3 > 0) && (!sleepOn)) {
    Serial.println("some color");
    flipPower();
    chainSendThenWait(0);
    myData.color1 = 0;
    myData.color2 = 0;
    myData.color3 = 0;
  }

  if ((myData.pattern > 300) && (!sleepOn)) {
    Serial.println("waiting for message");
    flipPower();
    //seen an issue where is not wating until the device wakes.  putting in a small delay here, might fix later
    delay(2000);
    chainSendThenWait(1);
    myData.pattern = 0;
    wait2Sleep = 20;  //this is th line that was keeping me from going back to sleep
  }


  //check here for update device info
  //make sure its not asleep
  if ((checkBatteryFlag) && (!sleepOn)) {
    checkBatteryFlag = false;

    if (deviceNum == 1) {
      pingConnection();
    } else {

      flipPower();
      pingConnection();
      delay(1500);
      measureDevice(deviceNum);
    }

    chainSendThenWait(1);
    Serial.println("Sent to device " + String(deviceNum));
    theSetTime = millis();  //effect start time
  }

  bigTime = max(long(myData.time), max(long(myData.sleepTime), long(wait2Sleep)));  //compare the length of the current effect to the current sleep time

  //if (((millis() - theSetTime + (wait2Sleep * 1000)) > (bigTime * 1000)) && (sleepOn == 0)) {
  if ((millis() - theSetTime) > (bigTime * 1000)) {
    if (POWERON) {
      digitalWrite(POWER_SWITCH, LOW);
      Serial.println("powerOFF");
      POWERON = 0;

    }  //send sleep message
       // Serial.println("Power Sleep " + String(sleepUnit));
    sleepOn = true;
    // sleepStart = millis();

    // myData.bright = 0;
    // myData.color1 = 0;
    // myData.color2 = 0;
    // myData.color3 = 0;
    // myData.pattern = 1;
    // myData.time = 5;
    // myData.sleepTime = sleepUnit;
    // myData.lightSpeed = 1;

    // chainSendThenWait(0);
    //  adjustDayTime(); //readjust the wait time
    theSetTime = millis();  //effect start time
  }

  if (millis() - sleepStart > (sleepUnit * 1000 + 2000)) { sleepOn = false; }  // if the sleep time is expired, turn the boolean off?
                                                                               //increased 1000 to 2000 since I saw the device wakung up too late to connect

  //occasionally read a new device's battery
  //dont need to be awake, it will check later
  if ((millis() - timeTimer) > measureBatteryTime * 1000) {
    timeTimer = millis();
    //turn power on?

    checkBatteryFlag = true;
    //Serial.println("Beepo " + String(checkBatteryFlag));
    deviceNum++;
    if (deviceNum > 10) {
      deviceNum = 1;
    }
    Serial.println(readTSPTime());
    //adjustDayTime();
  }
}
