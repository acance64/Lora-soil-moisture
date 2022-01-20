/*
  This is a simple example show the Heltec.LoRa recived data in OLED.

  The onboard OLED display is SSD1306 driver and I2C interface. In order to make the
  OLED correctly operation, you should output a high-low-high(1-0-1) signal by soft-
  ware to OLED's reset pin, the low-level signal at least 5ms.

  OLED pins to ESP32 GPIOs via this connecthin:
  OLED_SDA -- GPIO4
  OLED_SCL -- GPIO15
  OLED_RST -- GPIO16
  
  by Aaron.Lee from HelTec AutoMation, ChengDu, China
  成都惠利特自动化科技有限公司
  www.heltec.cn
  
  this project also realess in GitHub:
  https://github.com/Heltec-Aaron-Lee/WiFi_Kit_series
*/


#include <WiFi.h>
#include <PubSubClient.h>

#include "heltec.h" 
#include "images.h"



#define BAND    868E6  //you can set band here directly,e.g. 868E6,915E6, 433E6
String rssi = "RSSI --";
String packSize = "--";
String packet ;

// Update these with values suitable for your network.

const char* ssid = "YouShallNotConnect"; // your wifi SSID name
const char* password = "macbookpro" ;// wifi pasword

//const char* ssid = "Antonio's iPhone"; // your wifi SSID name
//const char* password = "pippopippo" ;// wifi pasword

//const char* ssid = "The Promised LAN"; // your wifi SSID name
//const char* password = "macbookair" ;// wifi pasword


  long lastMsg = 0;
  //char msg[50];
  char newmsg[50];
  int value = 0;


 


/*
//Thingspeak mqtt: DOESN'T WORK, UNABLE TO PUT "field1=" in front of payload newmsg
#define CHANNEL "channels/775891/publish/FBURLFLJ1FZSMDWK"
#define iniPacket "field1="



*/




/*
//Home MQTT
String whereTo="Home mqtt";
#define CHANNEL "humidity"
#define iniPacket ""
const char* mqtt_server = "192.168.1.106";
int lenIniPacket=0;
*/

//Thingspeak, other stuff in setup, do not change whereTo

String whereTo="thingspeak";
  #define CHANNEL "channels/775891/publish/FBURLFLJ1FZSMDWK"
   #define iniPacket "field1="
  const char* mqtt_server = "mqtt.thingspeak.com";
  int lenIniPacket=7;
  

 WiFiClient espClient;
 PubSubClient client(espClient);
 


  
 
 
void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
    Heltec.display->drawString(0, 20, "WIFI connected");
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
  if ((char)payload[0] == '1') {
    digitalWrite(BUILTIN_LED, LOW);   // Turn the LED on (Note that LOW is the voltage level
    // but actually the LED is on; this is because
    // it is active low on the ESP-01)
  } else {
    digitalWrite(BUILTIN_LED, HIGH);  // Turn the LED off by making the voltage HIGH
  }

}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      // Once connected, publish an announcement...
    //  client.publish("outTopic", "hello world");
      // ... and resubscribe
      client.subscribe("inTopic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void logo(){
  Heltec.display->clear();
  Heltec.display->drawXbm(0,5,logo_width,logo_height,logo_bits);
  Heltec.display->display();
}

void LoRaData(){
  Heltec.display->clear();
  Heltec.display->setTextAlignment(TEXT_ALIGN_LEFT);
  Heltec.display->setFont(ArialMT_Plain_10);
  Heltec.display->drawString(0 , 15 , "Received "+ packSize + " bytes");
  Heltec.display->drawStringMaxWidth(0 , 26 , 128, newmsg);
   Heltec.display->drawStringMaxWidth(0 , 37 , 128, "Sending to "+ whereTo);
  Heltec.display->drawString(0, 0, rssi);  
  Heltec.display->display();
}

void cbk(int packetSize) {
  packet ="";
  packSize = String(packetSize,DEC);
  for (int i = 0; i < packetSize; i++) { packet += (char) LoRa.read(); }
  rssi = "RSSI " + String(LoRa.packetRssi(), DEC) ;
  LoRaData();
}

void setup() { 

if (whereTo="thingspeak"){
  newmsg[0]='f';
  newmsg[1]='i';
  newmsg[2]='e';
  newmsg[3]='l';
  newmsg[4]='d';
  newmsg[5]='1';
  newmsg[6]='='; }

   //WIFI Kit series V1 not support Vext control
  Heltec.begin(true /*DisplayEnable Enable*/, true /*Heltec.Heltec.Heltec.LoRa Disable*/, true /*Serial Enable*/, true /*PABOOST Enable*/, BAND /*long BAND*/);
 
  Heltec.display->init();
  Heltec.display->flipScreenVertically();  
  Heltec.display->setFont(ArialMT_Plain_10);
  logo();
  delay(1500);
  Heltec.display->clear();

  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  
  Heltec.display->drawString(0, 0, "Heltec.LoRa Initial success!");
  Heltec.display->drawString(0, 10, "Wait for incoming data...");
  Heltec.display->display();
  delay(1000);
  //LoRa.onReceive(cbk);
  LoRa.receive();


  
}



void loop() {


  
  int packetSize = LoRa.parsePacket();
  value=99;

  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  
 

    
 if (packetSize) {  packet =iniPacket;   //we have a non null packet
  packSize = String(packetSize,DEC);
  //for (int i = 0; i < packetSize; i++) { packet += (char) LoRa.read(); }
  
  for (int i=0; i < packetSize; i++) { newmsg[i+lenIniPacket] = LoRa.read(); }

  
  //for (int i = LeniniPacket+1; i < packetSize; i++) { msg[i] = LoRa.read(); }
  
  rssi = "RSSI " + String(LoRa.packetRssi(), DEC) ;
  LoRaData();
  
  


     //This is to publish numerical value value into msg 
     //snprintf (msg, 50, "%ld", value);     
    
    //char *newmsg = new char[strlen(iniPacket) + strlen(msg) + 1];
   
    //sprintf(newmsg, "%s%s", iniPacket, msg);
    

    Serial.print("Publish message: ");
  Serial.println(newmsg);
  
      //newmsg.concat(msg);
      //home MQTT
      client.publish(CHANNEL, newmsg);


    /*
    //Thingspeak
    String payload
    char charBuf[msg.length()+1];
    msg.toCharArray(charBuf,packetsize);
    client.publish(CHANNEL, charBuf);
   */
   }
  delay(10);
}
