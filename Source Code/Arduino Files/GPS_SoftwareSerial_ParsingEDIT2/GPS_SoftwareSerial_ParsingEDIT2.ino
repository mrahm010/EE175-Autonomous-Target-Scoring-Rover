#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

// Connect the GPS Power pin to 5V
// Connect the GPS Ground pin to ground
// Connect the GPS TX (transmit) pin to Digital 8
// Connect the GPS RX (receive) pin to Digital 7

SoftwareSerial mySerial(3, 2);                       //change pins if needed
Adafruit_GPS GPS(&mySerial);

#define GPSECHO  true                               //false to stop echoing of data to console, true for GPS sentences

void setup(){
  Serial.begin(115200);                            // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  delay(2000);
  Serial.println("GPS Testing");

  GPS.begin(9600);                                // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800

  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time

  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);    //set update rate: 1 Hz update rate, going over is not suggested for parsing

  //GPS.sendCommand(PGCMD_ANTENNA);            //request updates on antenna if needed
  //delay(1000);
  //mySerial.println(PMTK_Q_RELEASE);         //ask for firmware
}

uint32_t timer = millis();

void GetCoordinates() {
  char c = GPS.read();          
 if ((c) && (GPSECHO))                       //GPSECHO too continue updating coordinates
   // Serial.write(c);                      //output of all the NMEA


  if (GPS.newNMEAreceived()) {              //if nmea found,parse
    if (!GPS.parse(GPS.lastNMEA()))         // this also sets the newNMEAreceived() flag to false
      return;                               // we can fail to parse a sentence in which case we should just wait for another
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

      Serial.print("Angle: "); Serial.println(GPS.angle);

    }
  }
  
}
void loop() {
  GetCoordinates();
}
