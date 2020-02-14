#include <SoftwareSerial.h>
struct GPSCoor {
  double latitude = 0.0;
  double longitude = 0.0;
};
struct GPSPack {
  GPSCoor CoordinateOrigin;
  GPSCoor Coordinate1;
  GPSCoor Coordinate2;
  GPSCoor Coordinate3;
};
int coordinatesRecieved = 0;
void fetchCoordinates(GPSPack &Route) {
  int counter = 1; //tells which coordinate we are readingg
  int longitude = 0; //tells whether it is reeading lat or long
  String inStringLatitude = "";    // string to hold input
  String inStringLongitude = "";
  char inputLatitude[11];
  char inputLongitude[11];
  //GPSCoor Coor1;
  //GPSCoor Coor2;
  //GPSCoor Coor3;
  while (Serial.available() > 0) {
    int inChar = Serial.read();
    if (inChar == ',') {    //tells us to switch from reading latitude to reading longitude
      longitude = 1;
    }
    else if (inChar == '.') {
      if (longitude == 0) {
        inStringLatitude += '.';
      }
      else {
        inStringLongitude += '.';
      }
    }
    else if (inChar == '-') {  //negative?
      if (longitude == 0) {
        inStringLatitude += '-';
      }
      else {
        inStringLongitude += '-';
      }
    }
    else if (isDigit(inChar)) {
      // convert the incoming byte to a char and add it to the string:
      if (longitude == 0) {
        inStringLatitude += (char)inChar;
      }
      else {
        inStringLongitude += (char)inChar;
      }
    }
    // if you get a newline, print the string, then the string's value:
    else if (inChar == '|') {
      longitude = 0;
      inStringLatitude.toCharArray(inputLatitude, sizeof(inputLatitude));
      inStringLongitude.toCharArray(inputLongitude, sizeof(inputLongitude));
      if(counter == 1) {
        Route.Coordinate1.latitude = atof(inputLatitude);
        Serial.print("Latitude1:");
        Serial.println(Route.Coordinate1.latitude);
        Route.Coordinate1.longitude = atof(inputLongitude);
        Serial.print("Longitude1:");
        Serial.println(Route.Coordinate1.longitude);
      }
      else if (counter == 2) {
        Route.Coordinate2.latitude = atof(inputLatitude);
        Serial.print("Latitude2:");
        Serial.println(Route.Coordinate2.latitude);
        Route.Coordinate2.longitude = atof(inputLongitude);
        Serial.print("Longitude2:");
        Serial.println(Route.Coordinate2.longitude);
      }
      else if (counter == 3) {
        Route.Coordinate3.latitude = atof(inputLatitude);
        Serial.print("Latitude3:");
        Serial.println(Route.Coordinate3.latitude);
        Route.Coordinate3.longitude = atof(inputLongitude);
        Serial.print("Longitude3:");
        Serial.println(Route.Coordinate3.longitude);
      }
      inStringLatitude = "";
      inStringLongitude = "";
      counter++;
    }
  }
  if (counter >= 3) {  //are the coordinates recieved and ready to use?
    coordinatesRecieved = 1;
  }
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

}

void loop() {
  // put your main code here, to run repeatedly:
  GPSPack Route;
  while (coordinatesRecieved == 0) {
    fetchCoordinates(Route);
  }
  Serial.print(Route.Coordinate1.latitude);
  
}
