<----------- Read me ---------->
Folder name : WiFi Signal 

Files attached in the folder:
1. Arduino code.ino
2. Results.doc

AIM Of the Project

To Connect to a known WiFi signal with fixed location and get
the signal strengths and distances from that node

COMPONENTS Required

1.Node MCU(ESP 8266)
2.WiFi router

WORKING

#ESP 8266 in NodeMCU module can detect the wifi signals of nearby 
devices and return the RSSI value
#The RSSI values were then converted to distances by taking constants 
from known distances and values
#The distances taken from three known Nodes(WiFi Points) can be used 
 to predict the current location using method of triangulation


RESULT

The obtained results taken from the Serial moniter of the arduino IDE
The Serial plotter also depicts the changing distaces when the ESP is moving

REFERENCE

