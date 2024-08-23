#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>  // only for esp_wifi_set_channel()
#include <FastLED.h>
//#include "effects.h"


// Global copy of slave
esp_now_peer_info_t myFriend;
#define CHANNEL 1
#define PRINTSCANRESULTS 0
#define DELETEBEFOREPAIR 0

#define MYRANK 8
#define MYSENDER MYRANK - 1
#define MYRECIEVER MYRANK + 1
#define LEDPin 2
#define POWER_PIN 27

#define NUM_LEDS 150
#define DATA_PIN 4
#define MAX_TIME 40000 * 1000


//#define FRAMES_PER_SECOND 60

short messageReady = 0;
long startTime = millis();
bool timerOn = 1;
uint8_t gHue = 0;  // rotating "base color" used by many of the patterns
RTC_DATA_ATTR int sleepTime = 0;
int friendTries = 0;
int powerOn = 0;
int step = 0;

CRGB leds[NUM_LEDS];

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
bool friendFound = 0;
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
  FastLED.addLeds<WS2812B, DATA_PIN>(leds, NUM_LEDS); //four pin
    //red is blue green is pink blue is greenish
    //FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);
    //green is blue red is red blue is green
//FastLED.addLeds<WS2812B, DATA_PIN, BRG>(leds, NUM_LEDS);  //3 pin

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
  int16_t scanResults = WiFi.scanNetworks(false, false, false, 300, CHANNEL);  // Scan only on one channel
  // reset on each scan
  friendFound = 0;
  memset(&myFriend, 0, sizeof(myFriend));

  Serial.println("s");
  if (scanResults == 0) {
    Serial.println("No WiFi devices in AP Mode found");
  } else {
    // Serial.print("Found ");
    // Serial.print(scanResults);
    // Serial.println(" devices ");
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
    //Serial.println("Friend Found, processing..");
  } else {    // Serial.println("Friend Not Found, trying again.");
  }
  // clean up ram
  WiFi.scanDelete();
}

// Check if the slave is already paired with the master.
// If not, pair the slave with master
bool manageFriend() {
  if (myFriend.channel == CHANNEL) {
    if (DELETEBEFOREPAIR) {
      deletePeer();
    }

    // Serial.print("Friend Status: ");
    // check if the peer exists
    bool exists = esp_now_is_peer_exist(myFriend.peer_addr);
    if (exists) {
      // Slave already paired.
      // Serial.println("Already Paired");
      return true;
    } else {
      // Slave not paired, attempt pair
      esp_err_t addStatus = esp_now_add_peer(&myFriend);
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
  const uint8_t *peer_addr = myFriend.peer_addr;
  Serial.print("Sending: ");
  Serial.println(data);
  esp_err_t result = esp_now_send(peer_addr, (uint8_t *)&myData, sizeof(myData));
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

// callback when data is sent from Master to Slave
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Sent to: ");
  Serial.println(macStr);
  Serial.print("Last Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// callback when data is recv from Master
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *recData, int data_len) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Recv from: ");
  Serial.println(macStr);
  Serial.print("Last Packet Recv Data length: ");
  Serial.println(String(sizeof(recData)));
  Serial.println("");
  messageReady = 1;
  memcpy(&myData, recData, sizeof(myData));
  friendFound = 0;
  friendTries = 4;  //retry for friend
  // Serial.println("I got " + String(data_len));
  // Serial.println("data " + String(myData.color1));
  //   Serial.println("data " + String(myData.color2));
  // char str[80];
  //     for(int i = 0; i < sizeof(myData); i++)
  //   {
  //       sprintf(str,"%c",((char*)&myData)[i]);
  //       Serial.print(str);
  //   }
}

void startSleep() {
  esp_sleep_enable_timer_wakeup(myData.sleepTime * 1000000ULL);
  Serial.println("Sleep for every " + String(myData.sleepTime) + " Seconds");
  Serial.flush();
  esp_deep_sleep_start();
}

void setup() {
  pinMode(LEDPin, OUTPUT);

  pinMode(POWER_PIN, OUTPUT);
   digitalWrite(POWER_PIN, LOW); //make sure its off

  blinkX(MYRANK, 150);
  setUpLights();

  Serial.begin(115200);
  //Set device in STA mode to begin with
  WiFi.mode(WIFI_STA);

  configDeviceAP();
  esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE);
  Serial.println("ESPNow/Basic/Master Example");
  // This is the mac address of the Master in Station Mode
  Serial.print("STA MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.print("STA CHANNEL ");
  Serial.println(WiFi.channel());
  // Init ESPNow with a fallback logic
  InitESPNow();
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
}

void processMessage() {
  startTime = millis();
  timerOn = 1;
  if (myData.lightSpeed > 1) { myData.lightSpeed == 1; }  //stop divide by zero

  //maybe brigt zero is off?
  Serial.println("Bright " + String(myData.bright));
  Serial.println("Color1 " + String(myData.color1));
  Serial.println("Color 2 " + String(myData.color2));
  Serial.println("Color  3 " + String(myData.color3));
  Serial.println("Pattern " + String(myData.pattern));
  Serial.println("time " + String(myData.time));
  Serial.println("sleeptime " + String(myData.sleepTime));
  Serial.println("fx speed " + String(myData.lightSpeed));
  //do stuff with the message
  blinkX(int(MYRANK), 100);
  // int c2=myData.color1;
  // int c3=myData.color2;
  // int c1=myData.color3;
  //
  //   int c2=myData.color1;
  // int c1=myData.color2;
  // int c3=myData.color3;
  //red - green  blue--red green  blue

  //   int c3=myData.color1;
  // int c2=myData.color2;
  // int c1=myData.color3;
  //green blue    blue red   read  green

  //   int c1=myData.color1;
  // int c2=myData.color2;
  // int c3=myData.color3;
  //red green    green blue    blue  red

  // Serial.println("Color1 "  + String(c1));
  // Serial.println("Color 2 "  + String(c2));
  // Serial.println("Color  3 "  + String(c3));

  int c2 = myData.color1;
  int c3 = myData.color2;
  int c1 = myData.color3;
  //blue red   red  green  green  blue

  if (myData.bright > 0) {
    powerOn = 1;
    digitalWrite(POWER_PIN, HIGH);
    FastLED.setBrightness(myData.bright);
  } else {
    //turn power off
    digitalWrite(POWER_PIN, LOW);
    FastLED.setBrightness(0);
    myData.pattern = 0;
    powerOn = 0;
  }
  // for(int led = 0; led < NUM_LEDS; led++) {
  //           //leds[led] = CRGB(myData.color1,myData.color2,myData.color3);
  //           leds[led] = CRGB(c1,c2,c3);
  //           //leds[led] = CRGB::Red;
  //                }
  //       FastLED.show();
  //start timer
  //turn lights on
  //
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
  if (timerOn) {
    long nowTime = millis() - startTime;
    if ((nowTime > MAX_TIME) || (nowTime > myData.time * 1000)) {
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

  // In the loop we scan for friend
  if ((!friendFound) && (friendTries < 6)) {
    //  Serial.println("frinedFound " + String(friendFound));
    ScanForFriend();
    delay(50);
    friendTries++;
  }

  if (myFriend.channel == CHANNEL) {  // check if friend channel is defined
                                      // `friend` is defined
                                      // Add friend as peer if it has not been added already
    isPaired = manageFriend();
    if (isPaired) {
      // pair success or already paired
      // Send data to device
      if (messageReady == 1) {
        sendData();
            //processMessage();
      }
    } else {
      // friend pair failed
      Serial.println("Friend pair failed!");
    }
  } else {
    // No slave found to process
  }
  //delay(3);
  if (messageReady == 1) {
    processMessage();
    messageReady = 0;
  }

  if (powerOn) {
    if (myData.pattern == 0) {
            //fill_solid(leds, NUM_LEDS, CRGB(myData.color3, myData.color1, myData.color2));
      fill_solid(leds, NUM_LEDS, CRGB(myData.color1, myData.color2, myData.color3));
      FastLED.show();
    }
    //myData.color3,myData.color1,myData.color2  //maybe for three pin
    //myData.color2,myData.color3,myData.color1  red blue   green red    blue grrren
    //myData.color1,myData.color2,myData.color3
    if (myData.pattern == 1) {
      Fire2012();  // run simulation frame

      FastLED.show();  // display this frame
      FastLED.delay(1000 / myData.lightSpeed);
    }

    if (myData.pattern == 2) {
      rainbow();
      FastLED.show();  // display this frame
      FastLED.delay(1000 / myData.lightSpeed);
    }

    if (myData.pattern == 3) {
      rainbow();
      addGlitter(80);
      if (millis() % 20 == 0) {
        gHue++;
      }
      FastLED.show();  // display this frame
      FastLED.delay(1000 / myData.lightSpeed);
    }

    if (myData.pattern == 4) {
      confetti();  // run simulation frame

      FastLED.show();  // display this frame
      FastLED.delay(1000 / myData.lightSpeed);
    }

    if (myData.pattern == 5) {
      sinelon();  // run simulation frame

      FastLED.show();  // display this frame
      FastLED.delay(1000 / myData.lightSpeed);
    }

    if (myData.pattern == 6) {
      bpm();  // run simulation frame

      FastLED.show();  // display this frame
      FastLED.delay(1000 / myData.lightSpeed);
    }

    if (myData.pattern == 7) {
      juggle();  // run simulation frame

      FastLED.show();  // display this frame
      FastLED.delay(1000 / myData.lightSpeed);
    }

    if (myData.pattern == 8) {
      step++;
      RGBLoop(step);  // run simulation frame

      FastLED.show();  // display this frame
      FastLED.delay(100 / myData.lightSpeed);
      if (step > 1535) { step = 0; }
    }

    // if (myData.pattern==9){
    //   step++;
    //  // send up flare
    //   flare();
    //   // explode
    //   explodeLoop();

    //   FastLED.show(); // display this frame
    //   FastLED.delay(100 / myData.lightSpeed);
    //   if (step>1535){step=0;}

    // wait before sending up another
    // delay(random16(1000, 4000));
    //}
    if (myData.pattern == 10) {
  
      candyCane(2); // run simulation frame

      FastLED.show();  // display this frame
      FastLED.delay(3000 / myData.lightSpeed);

    }
        if (myData.pattern == 11) {
  
      candyCane(3); // run simulation frame

      FastLED.show();  // display this frame
      FastLED.delay(3000 / myData.lightSpeed);

    }

  }


  // wait for 1seconds to run the logic again
  delay(10);
}