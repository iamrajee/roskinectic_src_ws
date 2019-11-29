#include<WiFi.h>        // Include the Wi-Fi library



const char* ssid     = "user";         // The SSID (name) of the Wi-Fi network you want to connect to
const char* password = "password";     // The password of the Wi-Fi network

void setup() {
  Serial.begin(74880);         // Start the Serial communication to send messages to the computer
  delay(1000);
  Serial.println('\n');
  
  WiFi.begin(ssid, password);             // Connect to the network
  Serial.print("Connecting to ");
  Serial.print(ssid); Serial.println(" ...");
int i=0;
  while(WiFi.status() != WL_CONNECTED) { // Wait for the Wi-Fi to connect
    delay(1000);
   Serial.print(++i); Serial.print(' ');
  }
  Serial.println('\n');
  Serial.println("Connection established!");  
  Serial.print("IP address:\t");
  Serial.println(WiFi.localIP());// Send the IP address of the ESP8266 to the computer  
float temp,dist=0.0;
 int c=0;
  while(true){
   
temp=(WiFi.RSSI()+45.0)/15.0;//assigning distance b
    dist=pow(10,-1*temp);
    if(c==0){
      c=1;
      }
      else{
Serial.print("Signal Strength(dB) : ");
Serial.println(WiFi.RSSI());
Serial.print("Approximated distance : ");
Serial.println(dist);
delay(500);
c=0;
}
}
}
void loop() { }
