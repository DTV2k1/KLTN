

#include <esp_now.h>
#include <WiFi.h>
#define WIFI_SSID "Hieu Van"
#define WIFI_PASSWORD "99999999"
#include "soc/rtc_wdt.h"

#define LED_GREEN 18
#define LED_YELLOW 19 
#define LED_RED 21
#include <math.h>
#include <esp_now.h>
#include <esp_wifi.h>
//#define PHONE "0983765211"
uint8_t MAC_NODE_01[] = {0x80, 0x7D, 0x3A, 0xB9, 0x0F, 0xE0};
uint8_t MAC_NODE_02[] = {0x80, 0x7D, 0x3A, 0xB9, 0x0F, 0x00};

String success;

typedef struct struct_gwData {
    int board_ID;    
    int mq135;
     //boolean flame;
    int flame;
    int mq7;
    int mq2;
    float dht_h;
    float dht_t;
    int state;
} struct_gwData;

struct_gwData dataRX; 


typedef struct struct_nodeData {
    int board_ID;    
    int mq135;
    //boolean flame;
    int flame;
    int mq7;
    int mq2;
    float dht_h;
    float dht_t;
    int state;
} struct_nodeData;

struct_nodeData dataTX_n1; 
struct_nodeData dataTX_n2; 

struct_nodeData dataRX_n1; 
struct_nodeData dataRX_n2; 

const String PHONE = "0983765211";

//GSM Module RX pin to ESP32 Pin 
//GSM Module TX pin to ESP32 Pin 

//#define rxPin 16
//#define txPin 17
//#define BAUD_RATE 9600
//HardwareSerial sim800(1);

#define sim800 Serial2

String smsStatus,senderNumber,receivedDate,msg;
//boolean isReply = false;
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
void setup() 
{
  //Serial.begin(9600);
  Serial.begin(115200);
  pinMode(4, OUTPUT);
  pinMode(LED_RED,OUTPUT);
  pinMode(LED_GREEN,OUTPUT);
  pinMode(LED_YELLOW,OUTPUT);
  
  Serial.println("esp32 serial initialize");
  
  //sim800.begin(BAUD_RATE, SERIAL_8N1, rxPin, txPin);
  sim800.begin(9600);
  Serial.println("SIM800L serial initialize");

  sim800.print("AT+CMGF=1\r"); //SMS text mode
    // delay(1000);
  setupWifi();

  //WiFi.mode(WIFI_STA);
 //WiFi.mode(WIFI_AP_STA);
    int32_t channel = getWiFiChannel(WIFI_SSID);
      // rtc_wdt_protect_off();
      // rtc_wdt_enable();
     esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  //esp_now_register_send_cb(OnDataSent);

  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, MAC_NODE_01, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
       
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  
  memcpy(peerInfo.peer_addr, MAC_NODE_02, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
       
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  
  esp_now_register_recv_cb(OnDataRecv); 

  rtc_wdt_protect_off();
  rtc_wdt_disable();

}


void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status == 0){
    success = "Delivery Success :)";
  }
  else{
    success = "Delivery Fail :(";
  }
  Serial.println(">>>>>");
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
    //digitalWrite(4, HIGH);
    memcpy(&dataRX, incomingData, sizeof(dataRX));
    Serial.println();
    Serial.println("<<<<< Receive Data:");
    Serial.print("Bytes received: ");
    Serial.println(len);

    Serial.println("Receive Data: ");
    Serial.println("dataRX.board_ID: " + String(dataRX.board_ID) );
    Serial.println("dataRX.state: " + String(dataRX.state) );
    Serial.println("   dataRX.mq135: " + String(dataRX.mq135) + " ppm");
    Serial.println("   dataRX.flame: " + String(dataRX.flame));
    Serial.println("     dataRX.mq7: " + String(dataRX.mq7)   + " ppm");
    Serial.println("     dataRX.mq2: " + String(dataRX.mq2)   + " ppm");
    Serial.println("   dataRX.dht_h: " + String(dataRX.dht_h) + " %RH");
    Serial.println("   dataRX.dht_t: " + String(dataRX.dht_t) + " *C");
  
    
    Serial.println("<<<<<");
     // digitalWrite(19,HIGH);
GREEN_ON();
     if( dataRX.board_ID == 1)
    {
     
     memcpy(&dataRX_n1, incomingData, sizeof(dataRX_n1)); 

     if(dataRX_n1.state == 1)
     {
         //RED_ON();
      Serial.println("ESP32_BOARD_SLAVE_01: FIRE DETECTED !!!");
      send("ESP32_BOARD_SLAVE_01: FIRE DETECTED !!!");
     }
     else {
      if(dataRX_n1.flame == 0 || dataRX_n1.dht_t >= 40 || dataRX_n1.dht_h <= 40 || dataRX_n1.mq2 >= 35 || dataRX_n1.mq7 >200 ||dataRX_n1.mq135 >2300)
     {
       YELLOW_ON();
     }
     else{
       GREEN_ON();
     }
     }
     

    }
    
    if( dataRX.board_ID == 2)
    {
     memcpy(&dataRX_n2, incomingData, sizeof(dataRX_n2)); 
    } 
   // digitalWrite(4, LOW);
 
}

void loop() 
{
  while(1)
  {

  }
} 

//***************************************************
void parseData(String buff){
  Serial.println(buff);

  unsigned int len, index;
  //////////////////////////////////////////////////
  //Remove sent "AT Command" from the response string.
  index = buff.indexOf("\r");
  buff.remove(0, index+2);
  buff.trim();
  //////////////////////////////////////////////////
  
  //////////////////////////////////////////////////
  if(buff != "OK"){
    index = buff.indexOf(":");
    String cmd = buff.substring(0, index);
    cmd.trim();
    
    buff.remove(0, index+2);
    
    if(cmd == "+CMTI"){
      //get newly arrived memory location and store it in temp
      index = buff.indexOf(",");
      String temp = buff.substring(index+1, buff.length()); 
      temp = "AT+CMGR=" + temp + "\r"; 
      //get the message stored at memory location "temp"
      sim800.println(temp); 
    }
    else if(cmd == "+CMGR"){
      extractSms(buff);
      
      
      if(senderNumber == PHONE){
        //doAction();
      }
    }
  //////////////////////////////////////////////////
  }
  else{
  //The result of AT Command is "OK"
  }
}

//************************************************************
void extractSms(String buff){
   unsigned int index;
   
    index = buff.indexOf(",");
    smsStatus = buff.substring(1, index-1); 
    buff.remove(0, index+2);
    
    senderNumber = buff.substring(0, 13);
    buff.remove(0,19);
   
    receivedDate = buff.substring(0, 20);
    buff.remove(0,buff.indexOf("\r"));
    buff.trim();
    
    index =buff.indexOf("\n\r");
    buff = buff.substring(0, index);
    buff.trim();
    msg = buff;
    buff = "";
    msg.toLowerCase();
}


void send(String text)
{
    sim800.print("AT+CMGF=1\r");
    delay(1000);
    sim800.print("AT+CMGS=\""+PHONE+"\"\r");
    delay(1000);
    sim800.print(text);
    delay(1000);
    sim800.write(0x1A); //ascii code for ctrl-26 //sim800.println((char)26); //ascii code for ctrl-26
    delay(1000);
    Serial.println("SMS Sent Successfully.");
}

void setupWifi()
{ //int counttime  = 0;


  Serial.print("Connecting to ");
  // Print the WiFi name
  Serial.println(WIFI_SSID);
  WiFi.mode(WIFI_AP_STA);
  // Establish the ESP8266 at station mode and connect it to the defined WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  // Print "..." if the ESP8266 has not connected to the defined WiFi
 // while (WiFi.status() != NULL) {
    
      //delay(5000);
      Serial.print(".");
      if(WiFi.status() == WL_CONNECTED)
      {
        Serial.println("");
        Serial.println("WiFi connected");
        Serial.println("IP address: ");
        Serial.println(WiFi.localIP());

      // break;
      }
      else if(WiFi.status() == WL_DISCONNECTED)
      {
        Serial.println("");
        Serial.println("WiFi disconnected");
          WiFi.mode(WIFI_STA);
     //  break;
      }  

  //}
}
void RED_ON()
{
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN,LOW);
  digitalWrite(LED_YELLOW,LOW);
}
void YELLOW_ON()
{
  digitalWrite(LED_YELLOW,HIGH);
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN,LOW);
}
void GREEN_ON()
{
  digitalWrite(LED_GREEN,HIGH);
   digitalWrite(LED_YELLOW,LOW);
  digitalWrite(LED_RED, LOW);
}