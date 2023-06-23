#define WIFI_SSID "Hieu Van"
#define WIFI_PASSWORD "99999999"
#include "DHT.h"
#include <FirebaseESP32.h>
#define FIREBASE_HOST "https://firealarm-b9a27-default-rtdb.asia-southeast1.firebasedatabase.app"
#define FIREBASE_AUTH "MKjdQeZGiD9BCT8yo24fTtIZkMQZOdbchjOnWLtD"
#define Gmail "swolff0123"
#define Roomname "Phòng Bếp"
#define PhoneCall "0352103277"


#include <WiFi.h>
#include <math.h>


#include <esp_now.h>
#include <esp_wifi.h>

#define Pin_MQ2 36
#define Pin_MQ7 39
#define Pin_Flame 34
#define Pin_MQ135 35
#define Pin_DHT11 32 // dht_pin is output
#define DHTType DHT11
#define Pin_Buzzer 33
#define LED_GREEN 18
#define LED_YELLOW 19 
#define LED_RED 21
#define RL 1.0
#define board "ESP-32"
#define Voltage_Resolution 3.3
#define ADC_bit_Resolution 12.0
#define RatioMQ2CleanAir 9.83 //RS / R0 = 9.83 ppm 
#define RatioMQ7CleanAir 26.45
#define RatioMQ135CleanAir 3.6
#define shape_sv 24
#define dim 6
#define gamma 0.1


String Room = Roomname;
String Host = Gmail;
String PhoneCalled = PhoneCall;
FirebaseData fbdt;
DHT dht(Pin_DHT11,DHTType);
float temp;       
float humidity;
float MQ2_R0, MQ7_R0, MQ135_R0;

uint8_t MAC_GATEWAY[] = {0x80, 0x7D, 0x3A, 0xB9, 0x23, 0x48};

boolean esp_now_state = false;

String success;

typedef struct struct_nodeData {
    int board_ID;    
    int mq135;
    int flame;
    int mq7;
    int mq2;
    float dht_h;
    float dht_t;
    int state;
} struct_nodeData;

struct_nodeData dataTX;  
struct_nodeData dataRX;  



// void setupWifi()
// { //int counttime  = 0;


//   Serial.print("Connecting to ");
//   // Print the WiFi name
//   Serial.println(WIFI_SSID);
//   WiFi.mode(WIFI_AP_STA);
//   // Establish the ESP8266 at station mode and connect it to the defined WiFi
//   WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
//   // Print "..." if the ESP8266 has not connected to the defined WiFi
//   while (WiFi.status() != NULL) {
    
//       delay(5000);
//       Serial.print(".");
//       if(WiFi.status() == WL_CONNECTED)
//       {
//         Serial.println("");
//         Serial.println("WiFi connected");
//         Serial.println("IP address: ");
//         Serial.println(WiFi.localIP());
//         break;
//       }
//       else if(WiFi.status() == WL_DISCONNECTED)
//       {
//         Serial.println("");
//         Serial.println("WiFi disconnected");
//         break;
//       }
       
//     //delay(500);
//     //
//   }
// }
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
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status ==0){
    success = "Delivery Success :)";
  }
  else{
    success = "Delivery Fail :(";
  }
  Serial.println(">>>>>");
}
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&dataRX, incomingData, sizeof(dataRX)); // sao chep chuoi incomingData dài sizeof(dataRX) ký tự tra ve dataRX 
  
  Serial.println();
  Serial.println("<<<<< Receive Data:");
  Serial.print("Bytes received: ");
  Serial.println(len);
  
  Serial.println("Receive Data: ");
  Serial.println("dataRX.board_ID: " + String(dataRX.board_ID));
  
  Serial.println("<<<<<");
  //esp_now_state = true;
}

float calculateR0(uint8_t pin,float sensor_ratio_cleanair)
{
  float sensor_value = 0;
  float sensor_volt;
  float sensor_Rs_air = 0;
  float sensor_R0 = 0;
  for(int x = 0 ; x < 500 ; x++) //Start for loop 
    {
    sensor_value = sensor_value + analogRead(pin); //Add analog values of sensor 500 times 
    }
    sensor_value = sensor_value/500;
    sensor_volt = sensor_value*(Voltage_Resolution/(pow(2,ADC_bit_Resolution))); //A x [VC/(2^ADC_bit)]
   sensor_Rs_air = ((Voltage_Resolution*RL)/sensor_volt)-RL; //RS = [(VC x RL) / VRL] - RL
  sensor_R0 = sensor_Rs_air/sensor_ratio_cleanair;
  //  Serial.print("R0 = "); //Display "R0"
  //  Serial.println(sensor_R0); //Display value of R0 

  return sensor_R0;
  delay(500); //Wait 1 second 
}
float get_ppm(uint8_t pin,float sensor_R0,float sensor_a,float sensor_b)
{
  // sensor_a = [log(y) - log(y0)] / [log(x) - log(x0)] = log(y/y0) / log(x/x0) 
  //Sensor_b = b = log(y) - m*log(x)
  //Read datasheet
  float sensor_value =analogRead(pin);
  float sensor_volt = sensor_value*Voltage_Resolution/(pow(2,ADC_bit_Resolution)); 
  //float sensor_volt = sensor_value*(Voltage_Resolution/4096);
  float sensor_Rs_gas = ((Voltage_Resolution*RL)/sensor_volt)-RL;
  float ratio = sensor_Rs_gas/sensor_R0;
  double ppm_log = (log10(ratio)-sensor_b)/sensor_a; //Get ppm value in linear scale according to the the ratio value  
  double ppm = pow(10, ppm_log); //Convert ppm value to log scale 
  //  Serial.println("ppm"); 
  //  Serial.println(ppm);
  //   Serial.println("R0"); 
  //  Serial.println(sensor_R0);
 delay(500);
 return ppm;
 }

// int svm_rbf_function(double first_feature[dim])
// {
//    Serial.println(first_feature[0]);
//   double feature[dim] = {};
//   double Mean[] = {3.85299953e+01, 5.04533069e+01, 1.19295459e+03, 8.71737657e+02, 1.25777585e+03, 6.59175594e-01};
//   double Scale[] = {1.45411613e+01, 2.96216089e+01, 5.45280630e+02, 6.13351358e+02, 5.52166698e+02, 4.73986424e-01};
//   int k=0 ;
//    for(int k=0; k<dim ;k++)
//       {
//         feature[k] = (first_feature[k] - Mean[k]) / Scale[k];
//       }
//   double dual_coef[shape_sv] = {-3.52040720e+02, -4.08858684e+01, -3.56450344e+00, -4.58763988e+02,
//       -2.55944043e+01, -4.35908864e-02, -1.20001635e+02, -3.24188609e+01,
//       -3.97675597e+02, 3.12589984e+00, 6.03482972e-02, 1.14879056e+03,
//       3.87602504e+01, 3.94362111e+00, 2.21726320e+02, 1.32852985e+01,
//       1.29686536e+00};
//       double suport_vector[shape_sv][dim] = {{-0.84793746, 0.65987952, 0.95372068, 0.3998725, 0.88057493, 0.71905943},{-0.49720894, 0.69363866, 1.83216743, 1.35038805, 1.71908982, 0.71905943},{-0.87544558, 0.7273978, 5.32211352, 5.2551646, 5.1383471, 0.71905943}, {-0.84793746, 0.65987952, 0.98122945, 0.32324432, 0.85159817, 0.71905943},{-0.57285626, 0.82867521, 2.95269137, 2.2503616, 2.83107286, 0.71905943}, {-0.84793746, 0.65987952, 1.01790781, 0.43248024, 0.88963016, 0.71905943}, {-0.29777507, 0.32228813, -0.01458806, -0.17402367, -0.08471328, 0.71905943}, {-0.78604419, 0.76115693, -0.43088746, 0.3509609, -0.51030939, -1.39070564},{-0.84793746, 0.65987952, 0.98306337, 0.4373714, 0.9638831, 0.71905943}, {0.99510654, -1.33190966, -2.18778097, -1.42126963, -2.27789154, -1.39070564},
//        {1.48337565, -1.60198276, 5.32211352, -0.50988337, 5.1383471, -1.39070564}, {-0.81355231, 0.5586021, 0.83818384, 0.23194266, 0.76285685, 0.71905943}, {-0.68976577, 0.59236124, -0.63078453, 0.7357322, -0.69322517, -1.39070564}, {1.48337565, -1.26439138, -0.45656232, -0.50825298, -0.51030939, 0.71905943}, {-0.78604419, 0.59236124, 1.53140487, 1.206914, 1.44199959, 0.71905943}, {-0.49720894, 0.62612038, 4.01269601, 2.4117699, 3.88148028, 0.71905943}, {0.87819703, -1.1968731, 5.32211352, 5.2551646, 5.1383471, 0.71905943}
//       };

//       double intercept = -12.02457;
//       double decision = 0;
//       int i =0; int j =0; 
//       //double euclid_square = 0;
//       double kernel_function =0;
//       double euclid_square[shape_sv] = {};
//         for(int i =0;i<shape_sv;i++)
//       {
//         for(int j=0;j<dim;j++)
//         {
//             euclid_square[i] += pow(suport_vector[i][j]-feature[j],2); // tinh ra dc khoang cach euclid binh phuong cua feature va 1 diem support vector 
            
//         } 

//         kernel_function = kernel_function + dual_coef[i]*exp((-1)*gamma*euclid_square[i]);
//       }
       
//         kernel_function = kernel_function + intercept;
//         Serial.println(kernel_function);
//         int state;
//       if(kernel_function > 0)
//       {
//           state =1;
//           digitalWrite(Pin_Buzzer,HIGH);
//       }
//       else if(kernel_function <0)
//       {
//         state = 0;
//         digitalWrite(Pin_Buzzer,LOW);
//       }
//       return state;
// }
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
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
     
     setupWifi();
       //WiFi.mode(WIFI_AP_STA);
       WiFi.mode(WIFI_STA);
       int32_t channel = getWiFiChannel(WIFI_SSID);
       esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  
  // Establish the ESP8266 at station mode and connect it to the defined WiFi
  //WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  // Connect to Firebase 
  Firebase.begin(FIREBASE_HOST,FIREBASE_AUTH);
  Firebase.reconnectWiFi(true);
  // Set database time out to 1 minute (max 15 minutes)
  Firebase.setReadTimeout(fbdt, 1000*60); //Gioi han thoi gian cho khi cho de doc du lieu
  //tiny, small, medium, large and unlimited.
  // Size and its write timeout e.g tiny(1s), small(10s), medium(30s), and large(60s)
  Firebase.setwriteSizeLimit(fbdt,"tiny");
      dht.begin();
      pinMode(Pin_MQ2,INPUT);
      pinMode(Pin_MQ7,INPUT);
      pinMode(Pin_Flame,INPUT);
     //pinMode(Pin_Flame,INPUT_PULLUP);
      pinMode(Pin_MQ135,INPUT);
      pinMode(Pin_Buzzer,OUTPUT);
      pinMode(LED_RED,OUTPUT);
  pinMode(LED_GREEN,OUTPUT);
  pinMode(LED_YELLOW,OUTPUT);
      MQ2_R0 = calculateR0(Pin_MQ2,RatioMQ2CleanAir);
      MQ135_R0 = calculateR0(Pin_MQ135, RatioMQ135CleanAir);
      MQ7_R0 =   calculateR0(Pin_MQ7,RatioMQ7CleanAir);
      if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, MAC_GATEWAY, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
       
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  
  esp_now_register_recv_cb(OnDataRecv); 
}

void loop() {
  // put your main code here, to run repeatedly:
        
        // MQ2.update();
        // MQ135.update();
   //   digitalWrite(Pin_Buzzer,LOW);
        temp =  dht.readTemperature();
        humidity = dht.readHumidity();
       double smoke = analogRead(Pin_MQ2);
       double co2 = analogRead(Pin_MQ135);
       double co = analogRead(Pin_MQ7);
       int flame = digitalRead(Pin_Flame);
      
       //float smoke_ppm = get_ppm(Pin_MQ2,MQ2_R0,-0.326,3.514); //(800,2) and (3000,1.3)
       int smoke_ppm = get_ppm(Pin_MQ2,MQ2_R0,-0.417,1.5);
        int co2_ppm = get_ppm(Pin_MQ135, MQ135_R0,-0.48,2.256); //(100,1.1) and (200,0.8)
        
        int co_ppm = get_ppm(Pin_MQ7,MQ7_R0,-0.596,2.607); //(400,0.38) and (1000,0.22)
        // double temp =29.80;
        // double humidity = 71.00;
        // double smoke = 979.00;
        // double co2 = 636.00;
        // double co = 999.00;
        // double flame = 1;
        // double temp =51.20; 
        // double humidity = 17.00;
        // double smoke = 1700.00;
        // double co2 = 1368.00;
        // double co = 1750.00;
        // double flame = 0;
        //dht.temperature
       //smoke = map(smoke,0,4096,300,10000);
        // co = map(co,0,4096,0,1024);
        // air = map(air,0,4096,0,1024);
        // Serial.println("smoke");
        //co = map(co,0,4096,10,1000);
        //double feature[] = {29.80,71.00,979.00,636.00,999.00,1};
       // delay(200);


double intercept = 1.37106109 - 0.5;
double Mean[] = {3.23248486e+01, 7.49422149e+01, 6.00445037e+02, 3.45425947e+02, 2.06969332e+03, 7.86923945e-01};
double Scale[] = {7.35784681, 16.87728975, 155.57059104, 197.78410145, 254.11609512,0.40948095};
double dual_coef[] = {-51.7663164 , -2.66268739, -33.25152521,  -0.64398986, -14.67060775, -4.51556171 , -4.00422984 , -1.39574649, -10.98857427, -25.3501548
,-5.0401129 , -24.11086205 , -5.43169849 ,  3.57464344 , 61.11794637,  9.20441614 ,  7.33085074 , 16.10712099 , 20.07180147 , 14.65944763,12.50996721 , 1.90236569 , 31.75562385 ,  5.5978836
};
double suport_vector[shape_sv][dim] ={
{6.45775052e-02 ,6.26750558e-02, 1.58484300e+00, 1.76745275e+00,
1.52806803e+00 , 5.20356451e-01},
{1.32532168e-01, -5.58273833e-02, -5.10668738e-01, -5.78539660e-01,
  -8.52733544e-01 , 5.20356451e-01},
{-4.65468866e-01 ,-5.89088359e-01 , 7.68493336e-01 , 1.17589863e+00,
   1.30769632e+00, -1.92175959e+00},
{-5.19832596e-01 , 5.95936031e-01 , 2.41401426e-01 ,-2.15359491e-03,
   2.72736293e-01 ,-1.92175959e+00},
{2.00486831e-01 , 3.42383625e-03 , 2.40762062e+00 , 3.06179338e+00,
   3.29891218e+00 , 5.20356451e-01},
{2.00486831e-01 ,-5.58273833e-02 , 2.20835416e+00 , 1.25679492e+00,
   3.68062748e+00 ,-1.92175959e+00},
{-6.42150989e-01 ,-3.52083481e-01 ,-1.21131530e+00 ,-5.33035497e-01,
  -4.71018249e-01,-1.92175959e+00},
{-5.19832596e-01, -5.89088359e-01 , 7.68493336e-01 , 1.16578659e+00,
   1.29589069e+00 , 5.20356451e-01},
{-5.19832596e-01 , 6.55187251e-01,  6.20650483e-01 , 4.52888036e-01,
   5.87553030e-01 ,-1.92175959e+00},
{7.71306000e-01 ,-4.70585920e-01 ,-2.72834584e-01 , 2.86039438e-01,
   1.08338939e+00,  5.20356451e-01},
{2.00486831e-01 , 6.26750558e-02 , 2.70330632e+00 , 2.46012723e+00,
   1.48871594e+00 , 5.20356451e-01},
{6.45775052e-02 , 6.26750558e-02 , 1.34700885e+00 , 1.64105229e+00,
   1.90978333e+00 , 5.20356451e-01},
{-5.19832596e-01 , 5.36684812e-01 ,-3.43542035e-01 ,-5.78539660e-01,
  -2.46711324e-01, -1.92175959e+00},
{-8.32424046e-01 ,-1.15078603e-01 , 3.07612743e+00 , 2.16182216e+00,
   1.99242272e+00 ,-1.92175959e+00},
{-2.07241147e-01 , 6.26750558e-02 , 1.55913120e+00 , 2.25283048e+00,
   1.79566226e+00 , 5.20356451e-01},
{9.75169989e-01 ,-1.12234933e+00 , 1.68126225e+00 , 1.95958143e+00,
   2.13409025e+00 , 5.20356451e-01},
{1.32532168e-01 , 4.18182373e-01 , 2.41401426e-01 , 1.00905003e+00,
   2.72830684e+00, -1.92175959e+00},
{-2.88786742e-01 ,-4.70585920e-01, -5.42842796e-02 , 9.07929666e-01,
   5.04913637e-01, -1.92175959e+00},
{-4.65468866e-01 , 1.81177495e-01 , 7.68493336e-01 , 3.01207492e-01,
  -1.60136721e-01 ,-1.92175959e+00},
{1.32532168e-01 , 4.18182373e-01 , 2.22117576e-01 , 1.02421808e+00,
   2.72830684e+00,  5.20356451e-01},
{2.54850561e-01 ,-5.29837139e-01 , 1.62341070e+00, 1.87868514e+00,
   2.14196067e+00 ,-1.92175959e+00},
{-3.43150472e-01 ,-5.89088359e-01 , 3.56727151e-03,  1.07983428e+00,
   6.26905122e-01 ,-1.92175959e+00},
{8.39260663e-01 ,-4.11334700e-01 , 2.54257326e-01,  7.96697267e-01,
   6.11164286e-01 , 5.20356451e-01},
{-8.32424046e-01 ,-1.15078603e-01 , 3.06327153e+00 ,2.78371239e+00,
   1.98061709e+00 , 5.20356451e-01}
};
         double first_feature[dim] = {temp, humidity, smoke, co2, co, flame};
        // int state = svm_rbf_function(&first_feature[dim]);  

      
      double decision = 0;
      int i =0; int j =0; 
      //double euclid_square = 0;
      double kernel_function =0;
      double euclid_square[shape_sv] = {};
      double feature[dim] = {0,0,0,0,0,0};
     // double feature[dim] = {};
    
      int k=0 ;
      for(int k=0; k<dim;k++)
      {
        feature[k] = (first_feature[k] - Mean[k]) / Scale[k];
      }
  Serial.println(feature[5]);
      // double dual_coef[shape_sv] = {-3.52040720e+02, -4.08858684e+01, -3.56450344e+00, -4.58763988e+02,
      // -2.55944043e+01, -4.35908864e-02, -1.20001635e+02, -3.24188609e+01,
      // -3.97675597e+02, 3.12589984e+00, 6.03482972e-02, 1.14879056e+03,
      // 3.87602504e+01, 3.94362111e+00, 2.21726320e+02, 1.32852985e+01,
      // 1.29686536e+00};
       
 
    
        for(int i =0;i<shape_sv;i++)
      {
        for(int j=0;j<dim;j++)
        {
            euclid_square[i] += pow(suport_vector[i][j]-feature[j],2); // tinh ra dc khoang cach euclid binh phuong cua feature va 1 diem support vector 
          
        } 

        kernel_function = kernel_function + dual_coef[i]*exp((-1)*gamma*euclid_square[i]);
      }
       
        kernel_function = kernel_function + intercept;
         Serial.println(kernel_function);
        int state;
      if(kernel_function > 0)
      {
          state =1;
          //digitalWrite(Pin_Buzzer,HIGH);
      }
      else if(kernel_function <0)
      {
        state = 0;
      }
     
      //while(1) 
      //{ 
        //if(esp_now_state == true)
      //{   
        //  if (isnan(humidity) || isnan(temp)) {
        //   Serial.println(F("Failed to read from DHT sensor!"));
        //   return;
        // }
        // dataTX.board_ID = 1;
        // dataTX.mq135    = co2_ppm;
        // dataTX.flame    = flame;
        // dataTX.mq7      = co_ppm;
        // dataTX.mq2      = smoke_ppm;
        // dataTX.dht_h    = humidity;
        // dataTX.dht_t    = temp;

        // Serial.println();
        // Serial.print(">>>>> ");
        // Serial.println("Send data");
      
        // esp_err_t result = esp_now_send(MAC_GATEWAY, (uint8_t *) &dataTX, sizeof(dataTX));
         
        // if (result == ESP_OK) {
        //   Serial.println("Sent with success");
        // }
        // else {
        //   Serial.println("Error sending the data");
        // }
  
        // delay(1000 + random(500, 1000));

        //  esp_now_state = false;
      //}

      


  // while(1)
  // //{
  //     //if(state == true)
  //     {    
        
Firebase.setString(fbdt,String(Host+"/PhoneCall"),PhoneCalled);
Firebase.setInt(fbdt,String(Host+"/Phòng"+"/"+Room+"/state"),state);
Firebase.setInt(fbdt,String(Host+"/"+"/Phòng"+"/"+Room+"/Temperature"),temp);
Firebase.setInt(fbdt,String(Host+"/"+"/Phòng"+"/"+Room+"/Humidity"),humidity);
Firebase.setInt(fbdt,String(Host+"/"+"/Phòng"+"/"+Room+"/Smoke"),smoke_ppm);
Firebase.setInt(fbdt,String(Host+"/"+"/Phòng"+"/"+Room+"/CO2"),co2_ppm);
Firebase.setInt(fbdt,String(Host+"/"+"/Phòng"+"/"+Room+"/CO"),co_ppm);      
Firebase.setInt(fbdt,String(Host+"/"+"/Phòng"+"/"+Room+"/Flame"),flame);  
              
      
      Serial.println("----------------");
     
       //digitalWrite(33, !digitalRead(33));
       Serial.println("ESP32!");
       Serial.println("MQ135  (CO2): " + String(co2_ppm ) + "ppm" );
      // Serial.println("FLAME         : " + String( flame1  ) );
       Serial.println("FLAME         : " + String( flame  ) );
       Serial.println("MQ7       (CO): " + String(co_ppm   ) + "ppm" );
       Serial.println("MQ2      (SMOKE): " + String( smoke_ppm ) + "ppm" );

      if (isnan(humidity) || isnan(temp)) {
          Serial.println(F("Failed to read from DHT sensor!"));
          return;
        }
      
        Serial.print(F("Humidity: "));
        Serial.print(humidity);
        Serial.print(F("%  Temperature: "));
        Serial.print(temp);
        Serial.println(F(" * C "));
        Serial.println("----------------");

         
         dataTX.board_ID = 1;
          dataTX.state = state;
        dataTX.mq135    = co2_ppm;
        //dataTX.flame    = flame1;
         dataTX.flame    = flame;
        dataTX.mq7      = co_ppm;
        dataTX.mq2      = smoke_ppm;
        dataTX.dht_h    = humidity;
        dataTX.dht_t    = temp;
        
        Serial.println();
        Serial.print(">>>>> ");
        Serial.println("Send data");
      
        esp_err_t result = esp_now_send(MAC_GATEWAY, (uint8_t *) &dataTX, sizeof(dataTX));
         
        if (result == ESP_OK) {
          Serial.println("Sent with success");
        }
        else {
          Serial.println("Error sending the data");
        }

        esp_now_state = false;
        if(state == 1)
        {
          //digitalWrite(Pin_Buzzer,HIGH);
          //delay(3000);
          //digitalWrite(Pin_Buzzer,LOW);
        }
        delay(500);


         
      //}
  //}         
}

void RED_ON()
{
  digitalWrite(Pin_Buzzer, HIGH);
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