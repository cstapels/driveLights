

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>  // only for esp_wifi_set_channel()
#include "ThingSpeak.h"

// Global copy of slave
esp_now_peer_info_t myFriend;
#define CHANNEL 1
#define PRINTSCANRESULTS 0
#define DELETEBEFOREPAIR 0

#define MYRANK 6
#define MYSENDER MYRANK-1
#define MYRECIEVER MYRANK+1
#define LEDPin 2

short messageReady=0;
WiFiClient client;

// Network information
char * ssid = "still_waters";
const char * password = "33turkeys511";
const char * writeAPIKey = "Y0A92ZT5UMPKK3N1"; //battery channel
const char * readAPIKey = "Z74I62ST2YVADWOX"; //control channel
long readChannelId=249466;

String createdChange= ""; //change later but using to see tsp change

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
  String SSIDString="Friend_" + String(MYRANK);
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

// Scan for slaves in AP mode
void ScanForFriend() {
  int16_t scanResults = WiFi.scanNetworks(false, false, false, 300, CHANNEL);  // Scan only on one channel
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
      String target= "Friend_" + String(MYRECIEVER);
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

// Check if the slave is already paired with the master.
// If not, pair the slave with master
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
        // Pair success
        Serial.println("Pair success");
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
    // No slave found to process
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
  //data++;
  //char newData[20]="123456789123456789";
  const uint8_t *peer_addr = myFriend.peer_addr;
  Serial.print("Sending bytes: ");
  Serial.println(String(sizeof(myData)));
  esp_err_t result = esp_now_send(peer_addr, (uint8_t *)&myData, sizeof(myData));


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

// callback when data is sent 
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
  Serial.print("Last Packet Recv Data: ");
  //Serial.println(*Rdata);
  Serial.println("");
  messageReady=1;
  memcpy(&myData, recData, sizeof(recData));
  //data=*Rdata;
}

void connectWiFi() {

  while ((WiFi.status() != WL_CONNECTED)) {
      WiFi.begin(ssid, password);
    delay(7500);
    Serial.println("Connecting to WiFi");
      }
  Serial.println("Connected");
 // blinkX(5, 100);
  ThingSpeak.begin(client);
}


int32_t getWiFiChannel(const char *ssid) {
  if (int32_t n = WiFi.scanNetworks()) {
    for (uint8_t i=0; i<n; i++) {
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

// Wait for a response from the server indicating availability,
// and then collect the response and build it into a string.
String getResponse() {
  String response;
  long TIMEOUT = 15000;
  long startTime = millis();

  delay(200);
  while (client.available() < 1 & ((millis() - startTime) < TIMEOUT)) {
    delay(5);
  }

  if (client.available() > 0) { // Get response from server.
    char charIn;
    do {
      charIn = client.read(); // Read a char from the buffer.
      response += charIn; // Append the char to the string response.
    } while (client.available() > 0);
  }
  client.stop();

  return response;
}

void setup() {
  pinMode(LEDPin,OUTPUT);
  blinkX(MYRANK,150);
  Serial.begin(115200);
  //Set device in STA mode to begin with
  WiFi.mode(WIFI_AP_STA);

  configDeviceAP();
/*
int32_t channel = getWiFiChannel(ssid);

WiFi.printDiag(Serial); // Uncomment to verify channel number before
esp_wifi_set_promiscuous(true);
esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
esp_wifi_set_promiscuous(false);
WiFi.printDiag(Serial); // Uncomment to verify channel change after



  esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE);
  //Serial.println("ESPNow/Basic/Master Example");
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
   connectWiFi(); 
   */
}

void startESPNow(){
  WiFi.disconnect();
  client.stop();

 configDeviceAP();
 esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE);
 Serial.print("STA MAC: ");
 Serial.println(WiFi.macAddress());
 Serial.print("STA CHANNEL ");
 Serial.println(WiFi.channel());
 InitESPNow();
 esp_now_register_send_cb(OnDataSent);
 esp_now_register_recv_cb(OnDataRecv);
}

void processNow(){
 ScanForFriend();

  if (myFriend.channel == CHANNEL) {  // check if slave channel is defined
    // `friend` is defined
    // Add friend as peer if it has not been added already
    bool isPaired = manageFriend();
    if (isPaired) {
      // pair success or already paired
      // Send data to device
     if (messageReady==1){
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
}

void processMessage(){
  //do stuff with the message
  blinkX(int(MYRANK),100);
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
 

   if ((WiFi.status() != WL_CONNECTED)) {
      connectWiFi();
    }

  int statusCode = 0;
  statusCode = ThingSpeak.readMultipleFields(readChannelId,readAPIKey);
  //Serial.println("Code: " + String(statusCode));
  if(statusCode == 200)
    {
      String createdAt = ThingSpeak.getCreatedAt(); // Created-at timestamp
    Serial.println("Created at " + createdAt);
        if (createdChange!=createdAt)
    { 
      int lastTime=getCall().toInt(); //risky conversion here
      Serial.println("last read age "+String(lastTime));
      
      if (lastTime<10){
     myData.bright=ThingSpeak.getFieldAsInt(1);
      myData.color1=ThingSpeak.getFieldAsInt(2);
      myData.color2=ThingSpeak.getFieldAsInt(3);
      myData.color3=ThingSpeak.getFieldAsInt(4);
      myData.pattern=ThingSpeak.getFieldAsInt(5);
      myData.time=ThingSpeak.getFieldAsInt(6);
      myData.sleepTime=ThingSpeak.getFieldAsInt(7);
       myData.lightSpeed=ThingSpeak.getFieldAsInt(8);
       
    Serial.println("Bright "   + String(myData.bright));
    Serial.println("Color1 "   + String(myData.color1));
    Serial.println("Color 2 "  + String(myData.color2));
    Serial.println("Color  3 " + String(myData.color3));
    Serial.println("pattern "  + String(myData.pattern));
    Serial.println("time "     + String(myData.time));
    Serial.println("sleeptime "     + String(myData.sleepTime));
    Serial.println("light time "     + String(myData.lightSpeed));


    messageReady=1;
    processMessage();
    startESPNow();
    processNow();
    messageReady=0;
    } 
        createdChange=createdAt;
    }
    }

delay(1000);

}