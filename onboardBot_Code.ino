#include "ESP8266WiFi.h"
 
const char* ssid = "Tinkerers' Lab";
const char* password =  "tinker@tl";
int one_f=D0;
int one_b=D1;
int two_f=D2;
int two_b=D3;


 
WiFiServer wifiServer(80);
 
void setup() {
 
  Serial.begin(115200);
  pinMode(one_f, OUTPUT);
  pinMode(one_b, OUTPUT);
  pinMode(two_f, OUTPUT);
  pinMode(two_b, OUTPUT);
  analogWrite(one_f,0);
  analogWrite(two_f,0);
  analogWrite(one_b,0);
  analogWrite(two_b,0);
 
  delay(1000);
 
  WiFi.begin(ssid, password);
 
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting..");
  }
 
  Serial.print("Connected to WiFi. IP:");
  Serial.println(WiFi.localIP());
 
  wifiServer.begin();
  WiFiClient client = wifiServer.available();
}
 
void loop() {
 
  WiFiClient client = wifiServer.available();
  
  if (client) {
 
    while (client.connected()) {
 
      while (client.available()>0) {
        
        int a = client.read()-48;
        int b = client.read()-48;
        int c = client.read()-48;
        int d = client.read()-48;
        
        int dir = client.read()-48;
        int e= a*1000 +b*100 +c*10 +d;
        client.println("got em'");
        Serial.print(dir);
        if(dir==1){     Serial.println("ONWARDS");
                        analogWrite(one_f,e);analogWrite(one_b,0);
                        analogWrite(two_f,e);analogWrite(two_b,0);}
        
        else if(dir==2){Serial.println("BACKWARDS");
                        analogWrite(one_b,e);analogWrite(one_f,0);
                        analogWrite(two_b,e);analogWrite(two_f,0);}
        else if(dir==3){Serial.println("BACKWARDS");
                        analogWrite(one_b,0);analogWrite(one_f,e);
                        analogWrite(two_b,e);analogWrite(two_f,0);}
        else if(dir==4){Serial.println("BACKWARDS");
                        analogWrite(one_b,e);analogWrite(one_f,0);
                        analogWrite(two_b,0);analogWrite(two_f,e);}
        
        else            {analogWrite(one_f,0);analogWrite(one_b,0);
                        analogWrite(two_f,0);analogWrite(two_b,0); Serial.println("WHICHWAY!?!?!?!?!");}
        
        /*
        if(a=="1"){
          Serial.println("ONE");
          digitalWrite(one,HIGH);
          digitalWrite(two,LOW);}
        else if(a=="2"){
          Serial.println("TWO");
          digitalWrite(two,HIGH);
          digitalWrite(one,LOW);}
        else {
          Serial.println("WHAT");
            digitalWrite(one,HIGH);
            digitalWrite(two,LOW);
            delay(500);
            digitalWrite(two,HIGH);
            digitalWrite(one,LOW);
            delay(500);
            
          }*/
      }
 
      delay(10);
    }
 
    client.stop();
    Serial.println("Client disconnected");
 
  }
}
