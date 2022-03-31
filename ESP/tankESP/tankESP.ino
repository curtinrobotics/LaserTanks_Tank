//libraries
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

//constants
#define DEBUG 1
#define DELAY 500
#define SERIAL_SPEED 9600
#define AP_NAME "LaserTanks"
#define AP_PASS "LaserTanks"
#define UDP_PORT 100

const IPAddress GATEWAY(192,168,0,1);
const IPAddress SUBNET(255,255,0,0);

const IPAddress GM_IP(192,168,0,1);

#define TANK_NUM 3

const IPAddress CONTROLLER_IP(192,168,TANK_NUM,1);
const IPAddress TANK_IP(192,168,TANK_NUM,2);
#define HOSTNAME "Tank " + TANK_NUM

//variables
WiFiUDP udp;

void setup(){
  delay(DELAY);
  startSerial();
  connectToAP();
  startUDP();
  delay(DELAY);
}

void loop(){
  receiveControlData();
  //sendStatusData();
}

//functions
void startSerial(void){
  Serial.begin(SERIAL_SPEED);
  #ifdef DEBUG
    Serial.println();
    Serial.println("Serial ready");
  #endif
}

void startUDP(void){
  boolean result = (boolean)udp.begin(UDP_PORT);
  #ifdef DEBUG
    if(result == true)
    {
      Serial.println("UDP ready");
    }
    else
    {
      Serial.println("UDP failed");
    }
  #endif
}

void connectToAP(void){
  boolean result = false;

  WiFi.hostname(HOSTNAME);
  WiFi.begin(AP_NAME, AP_PASS);
  WiFi.setAutoReconnect(true);
  
  result = WiFi.config(TANK_IP, GATEWAY, SUBNET);
  #ifdef DEBUG
    if(result == true)
    {
      Serial.println("Configuration ready");
    }
    else
    {
      Serial.println("Configuration failed");
    }

    Serial.println("Connecting");
  #endif
  while(WiFi.status() != WL_CONNECTED)
  {
    delay(DELAY);
  }
  #ifdef DEBUG
    Serial.println("Connected");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
  #endif
}

void receiveControlData(void){
  if(udp.parsePacket() > 0)
  { 
    byte readByte = udp.read();
    #ifdef DEBUG
      Serial.print("Tank<-Controller - ");
      Serial.print(readByte, BIN);
      Serial.write('\t');
    #endif
    Serial.write(readByte);
    #ifdef DEBUG
      Serial.println();
    #endif
  }
}

void sendStatusData(void){
  if(Serial.available() > 0)
  {    
    byte readByte = Serial.read();
    
    udp.beginPacket(GM_IP, UDP_PORT);
    udp.write(readByte);
    udp.endPacket();
    #ifdef DEBUG 
      Serial.print("Tank->Server - ");
      Serial.write(readByte);
      Serial.println();
    #endif
  }
}
