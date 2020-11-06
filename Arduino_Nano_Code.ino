#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include<Adafruit_GPS.h>
#include <WireMW.h>
#include <Wire.h>   //I2C library 0x28H
#include <WireMW.h>
#include <Wire.h>   //I2C library 0x28H
byte fetch_pressure(unsigned int *p_Pressure); //convert value to byte data type

 
#define SEALEVELPRESSURE_HPA (1013.25)
#define TRUE 1
#define FALSE 0
 
Adafruit_BMP3XX bmp; // I2C
SoftwareSerial debol(2,3);
Adafruit_GPS GPS(&debol);

#define USE_AVG 
const int sharpLEDPin = 7;
const int sharpVoPin = A5;
#ifdef USE_AVG
#define N 100
static unsigned long VoRawTotal = 0;
static int VoRawCount = 0;
#endif
static float Voc = 0.6;
static float VocT = 0.6;
const float K = 0.5;
void printValue(String text, unsigned int value, bool isLast = false) {
  Serial.print(text);
  Serial.print("=");
  Serial.print(value);
  if (!isLast) {
    Serial.print(", ");
  }
}
void printFValue(String text, float value, String units, bool isLast = false) {
  Serial.print(text);
  Serial.print("=");
  Serial.print(value);
  Serial.print(units);
  if (!isLast) {
    Serial.print(", ");
  }
    else {
    Serial.println();
  }
}



 
void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("BMP388 test");
 
  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }
 
  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  //bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);

  Wire.begin();
  delay(500);
  Serial.println("circuit is working ");

  pinMode(sharpLEDPin, OUTPUT);
  
  delay(2000);
  Serial.println("");
  Serial.println("GP2Y1014AU0F Demo");
  Serial.println("=================");
  VocT = Voc;
}
 
void loop() {
  if (! bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }
  Serial.print("Temperature = ");
  Serial.print(bmp.temperature);
  Serial.println(" *C");
 
  Serial.print("Pressure = ");
  Serial.print(bmp.pressure / 100.0);
  Serial.println(" hPa");
 
  Serial.print("Approx. Altitude = ");
  Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");
 
  Serial.println();
  delay(1000);

  GPS.parse(GPS.lastNMEA());
  GPS.newNMEAreceived();
  Serial.println(GPS.day);
  Serial.println(GPS.month);
  Serial.println(GPS.year);
  Serial.println(GPS.hour);
  Serial.println(GPS.minute);
  Serial.println(GPS.seconds);
  Serial.println(GPS.milliseconds);
  Serial.println((int)GPS.fix);
  Serial.println((int)GPS.fixquality);
  if(GPS.fix)
  {
  Serial.println(GPS.latitudeDegrees);
  Serial.println(GPS.longitudeDegrees);
  Serial.println(GPS.latitude);
  Serial.println(GPS.longitude);
  Serial.println(GPS.lat);
  Serial.println(GPS.lon);
  Serial.println(GPS.speed);
  Serial.println(GPS.angle);
  Serial.println(GPS.altitude);
  Serial.println(GPS.satellites);
  }
  byte _status;
  unsigned int P_dat;
  float PR;
 
  while(1)
  {
    _status = fetch_pressure(&P_dat);
   
    switch(_status)
    {
      case 0: Serial.println("Read_MR.");
      break;
      case 1: Serial.println("Read_DF2.");
      break;
      case 2: Serial.println("Read_DF3.");
      break;
      default: Serial.println("Read_DF4.");
      break;
    }
   
   
    PR = (float)((P_dat - (0.1*16383)) / (0.8*16383)) ;
   
Serial.println(P_dat);
Serial.println(PR);
    Serial.print(" ");
   
    delay(1000);
  }
}
 
  byte fetch_pressure(unsigned int *p_P_dat)
  {
   
   
  byte address, Press_H, Press_L, _status;
  unsigned int P_dat;
  address= 0x28;
  Wire.beginTransmission(address); 
  Wire.endTransmission();
  delay(100);
 
  Wire.requestFrom((int)address, (int) 4);//Request 4 bytes need 4 bytes are read
  Press_H = Wire.read();
  Press_L = Wire.read();
 byte Temp_H = Wire.read();
 byte  Temp_L = Wire.read();
  Wire.endTransmission();
 
 
  _status = (Press_H >> 6) & 0x03;
      Press_H = Press_H & 0x3f;
      P_dat = (((unsigned int)Press_H) << 8) | Press_L;
      *p_P_dat = P_dat;
      return(_status);

      

 
}
