#include "MQ135.h"
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <SD.h>
#include <avr/sleep.h>

SoftwareSerial mySerial(8, 7);
Adafruit_GPS GPS(&mySerial);

const int mq135Pin = 0;    // Pin sur lequel est branché de MQ135
const int dustSensorPin = 4;
const int LEDOK = 5;
const int LEDError = 6;

unsigned long duration;
unsigned long lowpulseoccupancy = 0;
float ratio = 0;
float concentration = 0;

MQ135 gasSensor = MQ135(mq135Pin);  // Initialise l'objet MQ135 sur le Pin spécifié
File myFile;
char file[7] = "CO2.csv";
#define chipSelect 10
#define led 7
#define GPSECHO true
// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = true;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy
const int sampletime = 10000;

//char entry[60];
String entry = "";



// different commands to set the update rate from once a second (1 Hz) to 10 times a second (10Hz)
// Note that these only control the rate at which the position is echoed, to actually speed up the
// position fix you must also send one of the position fix rate commands below too.
// #define PMTK_SET_NMEA_UPDATE_100_MILLIHERTZ  "$PMTK220,10000*2F" // Once every 10 seconds, 100 millihertz.
// #define PMTK_SET_NMEA_UPDATE_200_MILLIHERTZ  "$PMTK220,5000*1B"  // Once every 5 seconds, 200 millihertz.
// #define PMTK_SET_NMEA_UPDATE_1HZ  "$PMTK220,1000*1F"
// #define PMTK_SET_NMEA_UPDATE_5HZ  "$PMTK220,200*2C"
// #define PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,100*2F"


void setup()  
{
  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);
    
  pinMode(dustSensorPin,INPUT);
  pinMode(LEDOK, OUTPUT);
  pinMode(LEDError, OUTPUT);
  digitalWrite(LEDOK, HIGH);
  digitalWrite(LEDError, HIGH);
  delay(200);
  digitalWrite(LEDOK, LOW);
  delay(200);
  digitalWrite(LEDError, LOW);
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  float rzero = gasSensor.getRZero();
  Serial.println(rzero);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_100_MILLIHERTZ);   // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  // the nice thing about this code is you can have a timer0 interrupt go off
  // every 1 millisecond, and read data from the GPS for you. that makes the
  // loop code a heck of a lot easier!
  useInterrupt(true);

  delay(1000);
  // Ask for firmware version
  //Serial.println("Version");
  //Serial.println(mySerial.println(PMTK_Q_RELEASE));


  while (!SD.begin(chipSelect)) {
      Serial.println(F("Cannot begin SD"));
      delay(500);
    }
  SD.remove(file);
  if(!SD.exists(file)){      
    myFile = SD.open(file, FILE_WRITE);
    myFile.println(rzero);
    myFile.println(F("date,lat,lon,co2,pcs"));
    myFile.close();
  }
}


// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
//#ifdef UDR0
  //if (GPSECHO)
    //if (c) UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
//#endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}
float ppm;
uint32_t timer = millis();
void loop()                     // run over and over again
{
  //calcul the occupancy by addition
  duration = pulseIn(dustSensorPin, LOW);
  lowpulseoccupancy = lowpulseoccupancy+duration;
  // in case you are not using the interrupt above, you'll
  // need to 'hand query' the GPS, not suggested :(
  //if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    //char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    //if (GPSECHO)
      //if (c) Serial.print(c);
  //}
  
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > sampletime) {
    
  
    timer = millis(); // reset the timer
    ratio = lowpulseoccupancy/(sampletime*10.0);
    concentration = 1,1*pow(ratio,3)-3.8*pow(ratio,2)+520*ratio+0.62;
    entry = ' ';
    if(concentration > 5000){
      digitalWrite(LEDError, HIGH);
    }else{
      digitalWrite(LEDError, LOW);
    }
    
    //Serial.println(ppm);
    //Serial.println(concentration);
    lowpulseoccupancy = 0;
    entry.concat(GPS.year);
    entry +='-';
    entry.concat(GPS.month);
    entry += '-';
    entry.concat(GPS.day);
    entry += ' ';
    entry.concat(GPS.hour);
    entry += ':';
    entry.concat(GPS.minute);
    entry += ':';
    entry.concat(GPS.seconds);
    //Serial.println("Writing to SD");
    //myFile = SD.open(file, FILE_WRITE);
    //myFile.println("");
    //myFile.print(GPS.year, DEC);myFile.print('-');myFile.print(GPS.month, DEC);myFile.print('-');myFile.print(GPS.day, DEC);myFile.print(' ');myFile.print(GPS.hour, DEC);myFile.print(':');myFile.print(GPS.minute, DEC);
    //myFile.print(':');myFile.print(GPS.seconds, DEC);
    //myFile.write(GPS.fix);myFile.write(',');myFile.write((int)GPS.fixquality);
    //myFile.print(',');myFile.print(concentration);myFile.print(',');myFile.print(ppm);
    
    
    if (GPS.fix) {
      digitalWrite(LEDOK, HIGH);
      //Serial.print("\nTime: ");
      //Serial.print(GPS.hour, DEC); Serial.print(':');
      //Serial.print(GPS.minute, DEC); Serial.print(':');
      //Serial.print(GPS.seconds, DEC); Serial.print('.');
      //Serial.println(GPS.milliseconds);
      //Serial.print("Date: ");
      //Serial.print(GPS.day, DEC); Serial.print('/');
      //Serial.print(GPS.month, DEC); Serial.print("/20");
      //Serial.println(GPS.year, DEC);
      //Serial.print("Fix: "); Serial.print((int)GPS.fix);
      //Serial.print(" quality: "); Serial.println((int)GPS.fixquality); 
      //Serial.print("Location: ");
      //Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      //Serial.print(", "); 
      //Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      //Serial.print("Location (in degrees, works with Google Maps): ");
      //Serial.print(GPS.latitudeDegrees, 4);
      //Serial.print(", "); 
      //Serial.println(GPS.longitudeDegrees, 4);
      entry += ',';
      entry.concat(GPS.latitude/100);
      entry += ',';
      entry.concat(GPS.longitude/100);
      entry += ',';
      entry.concat(gasSensor.getPPM());
      entry += ',';
      entry.concat(concentration);      
      
    }else{
      digitalWrite(LEDOK, LOW);
    }
    myFile = SD.open(file, FILE_WRITE);
    myFile.println(entry);
    myFile.close();
    Serial.println("end loop");
  }
}
