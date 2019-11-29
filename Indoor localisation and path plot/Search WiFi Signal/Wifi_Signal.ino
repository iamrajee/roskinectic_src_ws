#include <WiFi.h>        // Include the Wi-Fi library


void setup() {
  Serial.begin(74880);         // Start the Serial communication to send messages to the computer
while (true){
  Serial.println("Available Networks are: ");
    Serial.println("---------------------------------------");
      int i,j,t=0;
  int num=WiFi.scanNetworks();
    for(i=0;i<num;i++){
      float dist=0.0;
      float temp=0.0;
      temp=(WiFi.RSSI(i)+45)/20.0;
      dist=pow(10,-1*temp);
    Serial.println(WiFi.SSID(i));
    Serial.print("Approximated distance is: ");
    Serial.println(dist);
    }
  delay(1000);
}

   }

void loop() { 
  

  }
