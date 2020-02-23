
#include <SoftwareSerial.h>

//--- Begin Pin Declarations ---//
const byte HC12RxdPin = 4;                // "RXD" Pin on HC12
const byte HC12TxdPin = 5;                // "TXD" Pin on HC12
const byte HC12SetPin = 6;                // "SET" Pin on HC12
const byte  GPSTxdPin = 7;                // "TXD" on GPS (if available)
const byte  GPSRxdPin = 8;                // "RXD" on GPS
//--- End Pin Declarations ---//

//--- Begin variable declarations ---//
char byteIn;                              // Temporary variable
String buffer3 = "";          // Read/Write Buffer 3 -- GPS

boolean debug = false;
boolean HC12End = false;                  // Flag for End of HC12 String
boolean GPSEnd = false;                   // Flag for End of GPS String
boolean commandMode = false;              // Send AT commands to remote receivers
//--- End variable declarations ---//

// Create Software Serial Ports for HC12 & GPS
// Software Serial ports Rx and Tx are opposite the HC12 Rxd and Txd
SoftwareSerial HC12(HC12TxdPin, HC12RxdPin);
SoftwareSerial GPS(GPSRxdPin, GPSTxdPin);

void setup() {
  buffer3.reserve(82);                    // Reserve 82 bytes for longest NMEA sentence

  pinMode(HC12SetPin, OUTPUT);            // Output High for Transparent / Low for Command
  digitalWrite(HC12SetPin, HIGH);         // Enter Transparent mode
  delay(80);                              // 80 ms delay before operation per datasheet
  HC12.begin(4800);                       // Open software serial port to HC12
  GPS.begin(9600);                        // Open software serial port to GPS
  GPS.listen();
}

void loop() {
  while (GPS.available()) {
    byteIn = GPS.read();
    buffer3 += char(byteIn);
    if (byteIn == '\n') {
      GPSEnd = true;
    }
  }

  if (GPSEnd) {
    // GPRMC, GPVTG, GPGGA, GPGSA, GPGSV, GPGLL
    if (buffer3.startsWith("$GPRMC")||buffer3.startsWith("$GPGGA")||buffer3.startsWith("$GPGLL")) {
      HC12.print(buffer3);                // Transmit RMC, GGA, and GLL sentences
      buffer3 = "";                       // Clear buffer
    } else {
      buffer3 = "";                       // Delete GSA, GSV, VTG sentences
    }
    GPSEnd = false;                       // Reset GPS flag
  }
}
