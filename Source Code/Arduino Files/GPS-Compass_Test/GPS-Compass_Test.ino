#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

SoftwareSerial mySerial(3, 2);                       //change pins if needed
Adafruit_GPS GPS(&mySerial);
#define GPSECHO  true     

Adafruit_BNO055 bno = Adafruit_BNO055(55);
void setup(){
  Serial.begin(115200);                            // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  delay(2000);
  Serial.println("GPS Testing");
  GPS.begin(9600);                                // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);    //set update rate: 1 Hz update rate, going over is not suggested for parsing

  if(!bno.begin(bno.OPERATION_MODE_COMPASS))
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  delay(1000);
    
  //bno.setExtCrystalUse(true);
}
uint32_t timer = millis();
int GetCoordinates() {
  char c = GPS.read();          
 if ((c) && (GPSECHO))                       //GPSECHO too continue updating coordinates
   // Serial.write(c);                      //output of all the NMEA
  if (GPS.newNMEAreceived()) {              //if nmea found,parse
    if (!GPS.parse(GPS.lastNMEA()))         // this also sets the newNMEAreceived() flag to false
      return 0;                               // we can fail to parse a sentence in which case we should just wait for another
  }
  if (timer > millis())  timer = millis();  //reset timer if needed
  if (millis() - timer > 2000) {
    timer = millis();                       // reset the timer
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
    if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print(GPS.latitudeDegrees, 6); 
      Serial.print(", ");
      Serial.print(GPS.longitudeDegrees, 6);
      Serial.print("\n");
      
      return 1;
    }
  }
  return 0;
}
void loop() {
  if(GetCoordinates()) {
    sensors_event_t event; 
    bno.getEvent(&event);
    Serial.println(event.orientation.x);
  }
  
}
