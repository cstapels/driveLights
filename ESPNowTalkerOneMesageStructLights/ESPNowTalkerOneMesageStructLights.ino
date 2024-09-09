#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>  // only for esp_wifi_set_channel()
#include <FastLED.h>
#include "effects.h"

// Global copy of friends
esp_now_peer_info_t myFriend;
esp_now_peer_info_t mySendFriend;
#define prod
//#define demo
#define CHANNEL 1
#define PRINTSCANRESULTS 0
#define DELETEBEFOREPAIR 0

#define MYRANK 7                                                     
#define MYSENDER MYRANK - 1
#define MYRECIEVER MYRANK + 1
#define LEDPin 2
#define POWER_PIN 27 //change for 3
#define BATTERY_PIN 35
#define NUM_LEDS 150
#define DATA_PIN 4
#define MAX_TIME 40000 * 1000
#define AUTO_SLEEP_TIME 15000 * 1000  //fifteen seconds
#define MAX_SEND_TRIES 3

//d2 power 27 data 4 four pin
//device 4 Power pin 27 data 4 3 pin
//d3 power 26 data 4 four pin
//d5 power 27 data 4
//d6 27 power 4 data 2 LED 4 pin
//d7 power 27 data 4
//d8 power 27 data 4 led2
//d9 power 27 data 4 led2
//d10 power 27 data 4 led2

//#define FRAMES_PER_SECOND 60
short messageReady = 0;
short upMessage = 0;
short downMessage = 0;
long startTime = millis();
bool timerOn = 0;
uint8_t gHue = 0;  // rotating "base color" used by many of the patterns
RTC_DATA_ATTR int sleepTime = 0;
int friendTries = 0;
int powerOn = 0;
int step = 0;
int myDirection = 1;
int deliverySuccess = 0;
int32_t sendStrength = 0;
int32_t recStrength = 0;
//for flare fx
int thisdelay = 20;    // A delay value for the sequence(s)                        ********* CHANGE ME **************
int thissat = 255;     // Standard fully saturated LED
int thisbright = 255;  // Standard fully bright LED
int gravity = -8;      // Gravity                                                 ********* CHANGE ME **************
int timeinc = 2;       // A time increment.
int maxcount = 100;    // Maximum number of explosion frames.
int mycount = 0;       // Repeat the explosion counter.
// Inital speed variables can be changed to make it go HIGHER!!!
int streamervelocity = 500;   // Velocity of the initial streamer that goes into the air. ********* CHANGE ME **************
int explosionvelocity = 500;  // Maximum velocity of the explosion.                       ********* CHANGE ME **************
uint8_t thisstatus = 0;       // Used to determine which state our finite state machine is in.
#define numgravs 5            // How many gravs we are using. The first one is our streamer.  ********* CHANGE ME **************
typedef struct {              // Define a structure for the gravs.
  long distold;               // I defined this as 'long' so that it'll work with longer strips.
  long distance;              // Ditto.
  int velold;
  int velocity;
  int thishue;
  int thissat;
  int thisbright;
} gravs;
gravs mygravs[numgravs];



CRGB leds[NUM_LEDS];

typedef struct struct_message {
  int bright;  //keep bright==0 to keep lights from turning on in message
  int color1;  //message direction 260 = in
  int color2;  //message start node(for other direction)
  int color3;  // battery level
  int pattern;
  int time;
  long sleepTime;
  int lightSpeed;
} struct_message;

struct_message myData;


bool friendFound = 0;
bool sendFriendFound = 0;
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

void setUpLights() {
  if (MYRANK == 4) {
    FastLED.addLeds<WS2812B, DATA_PIN, BRG>(leds, NUM_LEDS);  //3 pin
  } else {
    FastLED.addLeds<WS2812B, DATA_PIN>(leds, NUM_LEDS);  //four pin
  }
  //red is blue green is pink blue is greenish
  //FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);
  //green is blue red is red blue is green
  //

  FastLED.clear();
  for (int led = 0; led < NUM_LEDS; led++) {
    //leds[led] = CRGB::Red;
    leds[led] = CRGB(0, 2, 222);  //red second?, green thuird
  }
  FastLED.show();
}

// config AP SSID
void configDeviceAP() {
  char SSID[10];
  String SSIDString = "Friend_" + String(MYRANK);
  strcpy(SSID, SSIDString.c_str());
  //const char *SSID = "Friend_1";
  bool result = WiFi.softAP(SSID, "Friend_1_Password", CHANNEL, 0);
  if (!result) {
    Serial.println("AP Config failed.");
  } else {
    Serial.println("AP Config Success. Broadcasting with AP: " + String(SSID));
    Serial.print("AP CHANNEL ");
    Serial.println(WiFi.channel());
  }
}

// Scan for friends in AP mode
void ScanForFriend() {
  int16_t scanResults = WiFi.scanNetworks(false, false, false, 1400, CHANNEL);  // Scan only on one channel
  friendFound = 0;   // reset on each scan
  memset(&myFriend, 0, sizeof(myFriend));
  memset(&mySendFriend, 0, sizeof(mySendFriend));

  Serial.println("scan");
  if (scanResults == 0) {
    Serial.println("No WiFi devices in AP Mode found");
  } else {
    // Serial.print("Found " + String(scanResults)+ " devices");
    for (int i = 0; i < scanResults; ++i) {  // Print SSID and RSSI for each device found
      String SSID = WiFi.SSID(i);
      int32_t RSSI = WiFi.RSSI(i);
      String BSSIDstr = WiFi.BSSIDstr(i);

      if (PRINTSCANRESULTS) { Serial.println(String(i + 1) + ": " + SSID + " ("); }
      delay(10);

      String target = "Friend_" + String(MYRECIEVER);  // Check if the current device is my target
      if (SSID.indexOf(target) == 0) {                 //this is a string find operation to match the SSID
        Serial.print("Found a Friend. ");
        Serial.println(String(i + 1) + ": " + SSID + " [ (" + RSSI + ")");
        recStrength = RSSI;
        int mac[6];  // Get BSSID => Mac Address of the Friend
        if (6 == sscanf(BSSIDstr.c_str(), "%x:%x:%x:%x:%x:%x", &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5])) {
          for (int ii = 0; ii < 6; ++ii) {
            myFriend.peer_addr[ii] = (uint8_t)mac[ii];
          }
        }
        myFriend.channel = CHANNEL;  // pick a channel
        myFriend.encrypt = 0;        // no encryption
        friendFound = 1;
      }

      String fTarget = "Friend_" + String(MYSENDER); //device 10 is confused with device 1!!
      String rankTarget=SSID;
      rankTarget.remove(0,7);
      //if (SSID.indexOf(fTarget) == 0) {  //this is a string find operation to match the SSID
      if(rankTarget== String(MYSENDER)){
        Serial.print("Found a sender. ");
        Serial.println(String(i + 1) + ": " + SSID + " [ (" + RSSI + ")");
        sendStrength = RSSI;
        // Get BSSID => Mac Address of the SendFriend
        int macS[6];
        if (6 == sscanf(BSSIDstr.c_str(), "%x:%x:%x:%x:%x:%x", &macS[0], &macS[1], &macS[2], &macS[3], &macS[4], &macS[5])) {
          for (int ii = 0; ii < 6; ++ii) {

            mySendFriend.peer_addr[ii] = (uint8_t)macS[ii];
            //Serial.print(String(macS[ii]) + " : ");
          }
        }
        mySendFriend.channel = CHANNEL;  // pick a channel
        mySendFriend.encrypt = 0;        // no encryption
        sendFriendFound = 1;
      }
    }
  }

  // clean up ram
  WiFi.scanDelete();
}

// Check if the friend is already paired with the master. If not, pair the friend with master
bool manageFriend(esp_now_peer_info_t checkFriend) {
  if (checkFriend.channel == CHANNEL) {
    if (DELETEBEFOREPAIR) {
      deletePeer();
    }
    bool exists = esp_now_is_peer_exist(checkFriend.peer_addr);  // check if the peer exists
    if (exists) {                                                // Slave already paired.
                                                                 //Serial.println("Already Paired");
      return true;
    } else {  // Friend not paired, attempt pair

      esp_err_t addStatus = esp_now_add_peer(&checkFriend);
      if (addStatus == ESP_OK) {
        // Pair success
        Serial.println("Pair success");
        return true;
      } else if (addStatus == ESP_ERR_ESPNOW_NOT_INIT) {

        Serial.println("ESPNOW Not Init");  // How did we get so far!!
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
  } else {  // No friend found to process
    //Serial.println("No Friend found to process");
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
    Serial.println("ESPNOW Not Init");
  } else if (delStatus == ESP_ERR_ESPNOW_ARG) {
    Serial.println("Invalid Argument");
  } else if (delStatus == ESP_ERR_ESPNOW_NOT_FOUND) {
    Serial.println("Peer not found.");
  } else {
    Serial.println("Not sure what happened");
  }
}

//uint8_t data = 0;
// send data
void sendData(const uint8_t *peer_addr) {  //zero for out, 1 for in
                                           // uint8_t *peer_addr;

  int sendTries = 0;
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           peer_addr[0], peer_addr[1], peer_addr[2], peer_addr[3], peer_addr[4], peer_addr[5]);
 
  //}
  Serial.print("Sending: ");
   Serial.println(macStr);
  //Serial.println(data);
  //myData.pattern=1234;

  while (!deliverySuccess && sendTries < MAX_SEND_TRIES) {
    Serial.print("Deliver succ " + String(deliverySuccess));
    esp_err_t result = esp_now_send(peer_addr, (uint8_t *)&myData, sizeof(myData));
    Serial.println("Send size: " + String(sizeof(myData)));

    Serial.println("Send data: " + String(myData.pattern));
    sendTries++;
    Serial.println("Send num " + String(sendTries));
    delay(240);  //probably better to move this counter to the function that calls it


    Serial.print("Send Status: ");
    if (result == ESP_OK) {
      Serial.println("Success");
      //messageReady=0;
    } else if (result == ESP_ERR_ESPNOW_NOT_INIT) {
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
  deliverySuccess = 0;
}

// callback when data is sent from Master to friend
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Sent to: ");
  Serial.println(macStr);
  Serial.print("Last Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  //status == ESP_NOW_SEND_SUCCESS ? deliverySuccess = 1 : deliverySuccess = 0;
}

// callback when data is recv from Master
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *recData, int data_len) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Recv from: " + String(macStr));
  Serial.println("Last Packet Recv Data length: " + String(sizeof(recData)));
  memcpy(&myData, recData, sizeof(myData));

  messageReady = 1;

  friendFound = 0;  ///do I really need to reset on new data?
  sendFriendFound = 0;
  friendTries = 4;  //retry for friend
}

void startSleep() {
  //turn power off just in case
      digitalWrite(POWER_PIN, LOW);
  esp_sleep_enable_timer_wakeup(myData.sleepTime * 1000000ULL);
  Serial.println("Sleep for every " + String(myData.sleepTime) + " Seconds");
  Serial.flush();
  esp_deep_sleep_start();
}


void setup() {
  pinMode(LEDPin, OUTPUT);
  pinMode(POWER_PIN, OUTPUT);
  digitalWrite(POWER_PIN, LOW);  //make sure its off
  blinkX(MYRANK, 50);
  setUpLights();
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);  //Set device in STA mode to begin with
  configDeviceAP();
  esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE);

  Serial.print("STA MAC: " + WiFi.macAddress());  // This is the mac address of the Master in Station Mode
  Serial.print("STA CHANNEL " + WiFi.channel());
  InitESPNow();  // Init ESPNow with a fallback logic
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
  myData.sleepTime = 0; //just in case
}

void processMessage() {
  //should decide measure data, then send upstream or downstream or set lights
  startTime = millis();
  timerOn = 1;
  downMessage = 1;
  if (myData.lightSpeed > 1) { myData.lightSpeed == 1; }  //stop divide by zero

  //maybe brigt zero is off?
  Serial.print("Bright " + String(myData.bright));
  Serial.print(" Color1 " + String(myData.color1));
  Serial.print(" Color 2 " + String(myData.color2));
  Serial.println(" Color  3 " + String(myData.color3));
  Serial.println("Pattern " + String(myData.pattern));  //pattern==200 is collect data
  Serial.print(" time " + String(myData.time));
  Serial.print(" sleeptime " + String(myData.sleepTime));
  Serial.println(" fx speed " + String(myData.lightSpeed));

  Serial.println("message ready " + String(messageReady));
  //do stuff with the message
  //blinkX(int(MYRANK), 100);

  if (myData.bright > 0) {  //right now lights are off for processing message
    powerOn = 1;
    digitalWrite(POWER_PIN, HIGH);
    FastLED.setBrightness(myData.bright);
  } else {
    //turn power off
    digitalWrite(POWER_PIN, LOW);
    FastLED.setBrightness(0);
    // myData.pattern = 0;
    powerOn = 0;
  }

  //check to se if we want to measure
  Serial.println("Pattern is " + String(myData.pattern));
  if (myData.pattern == (300 + MYRANK)) {//collect data for sending
    //blinkX(5, 50);
    myData.color1 = 260;
    myData.color2 = MYRANK;
    myData.color3 = analogRead(BATTERY_PIN);  //change to battery level
    myData.pattern = 123;
    myData.time = 30;  //need to put time on to keep it awake
    myData.sleepTime = sendStrength;
    myData.lightSpeed = recStrength;
    Serial.println("Ready to send back");
    Serial.println(String(recStrength) + " " + String(sendStrength));
  }
//Serial.println("Color is " + String(myData.color1));
    if (myData.color1 == 260) {  // Send data to device
      upMessage = 1;
      downMessage = 0;
       Serial.println("should send back");
    } 
  messageReady = 0;
}

void blinkX(int numTimes, int delayTime) {
  for (int g = 0; g < numTimes; g++) {

    // Turn the LED on and wait.
    digitalWrite(LEDPin, HIGH);
    delay(delayTime);

    // Turn the LED off and wait.
    digitalWrite(LEDPin, LOW);
    delay(delayTime);
  }
}

void loop() {

  bool isPaired;
  bool isSendPaired;
  //things to do in the loop
  // 1. check for pairing, a few times?   then occassionally later?
  // 2. check to see if there is a message to send upstream
  // 3. check to see if there is a measage to send downstream
  // 4. gather local data?
  // 5. manage light display

  long nowTime = millis() - startTime;  //check to see if the lights have been on for a while
  if (timerOn) {
    if ((nowTime > MAX_TIME) || (nowTime > myData.time * 1000)) {
      //use time to keep device on long enough to send a message upstream
      digitalWrite(POWER_PIN, LOW);
      FastLED.setBrightness(0);
      FastLED.show();
      myData.pattern = 0;
      Serial.println("timer expired");
      timerOn = 0;
      powerOn = 0;

             //go to sleep if there is positive sleep time
      if (myData.sleepTime > 0) {
        startSleep();
    }
  }
    }
  // if ((!timerOn) && (nowTime > AUTO_SLEEP_TIME)) {  //Go to sleep if enoguh time has passed
  //   myData.sleepTime = 10;
  //   startSleep();
  // }

  bool friendsFound = friendFound || sendFriendFound;  // In the loop we scan for friend
  if ((!friendsFound) && (friendTries < 6)) {          //or from and
    //Serial.println("friendFound " + String(friendFound));
    ScanForFriend();
    delay(25);
    friendTries++;
    friendsFound = friendFound || sendFriendFound;
  }
  //consider timet to reset friendTries every so often

  //if ((myFriend.channel == CHANNEL) || (mySendFriend.channel == CHANNEL)) {  // check if friend channel is defined
  if (friendsFound) {
    isPaired = manageFriend(myFriend);
    isSendPaired = manageFriend(mySendFriend);
    //Serial.println("Sendpairtest");
    if (upMessage) {
      if (isSendPaired) {
        Serial.println("send upstream " + String(mySendFriend.peer_addr[5]));
        sendData(mySendFriend.peer_addr);  //send the message back up the chain dont process message
                                           //consider changing color1 back to nothing??
      }
      upMessage = 0;
    }
    if (downMessage) {
      if (isPaired) {  // pair success or already paired
        sendData(myFriend.peer_addr);
      }
      downMessage = 0;
    }

    else {
      // No slave found to process
    }
  }

  //delay(3);
  if (messageReady > 0) {
    processMessage();  //turn power on and check for device info if instructed
  }

  if (powerOn) {
    doLights();
  }

  // wait for 1seconds to run the logic again
  delay(10);
}
