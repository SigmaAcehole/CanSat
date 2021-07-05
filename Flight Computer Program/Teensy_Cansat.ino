#include <EEPROM.h>   
#include<Wire.h>
#include<SoftwareSerial.h>
#include<XBee.h>
#include<printf_Teensy.h>
#include<sps30_Teensy.h>
#include<TinyGPS.h>
#include<Servo.h>
#include<SD.h>


XBee xbee = XBee();
SPS30 sps30;
TinyGPS gps;
Servo servo;

#define gpsPort Serial3
#define AirSpeedAddress 0x28         //For MS4525DO
#define BarometerAddress 0x77        //For MS5611
#define SP30_COMMS SERIALPORT2       //For SPS30
#define TX_PIN 10                        
#define RX_PIN 9                         

int LED=11;
int BUZZER=12;
float VOLSENSE=23;
int PacketCount0;
int count;
int state0;
int t1,t2;

int SAVE[6];    //array to store calibration data

struct sps_values val;    //SPS30 OUTPUT

int C1,C2,C3,C4,C5,C6,DT,DP,dT,P,pressure;   //constants related to altimeter



byte fetch_pressure(unsigned int *p_Pressure);  // For airspeed sensor
unsigned int P_dat;
unsigned int T_dat;
double PR,TR,V,VV;


static void smartdelay(unsigned long ms);    //For GPS
static void print_float(float val, float invalid, int len, int prec);
static void print_int(unsigned long val, unsigned long invalid, int len);
static void print_date(TinyGPS &gps);
static void print_str(const char *str, int len);
float flat, flon;
  
const String TEAMID = "TEAM B";
String SOFTWARESTATE="";
int HOUR,MINUTE,SECOND;
String GPSTIME;
float PRESSURE,TEMPERATURE,ALTITUDE,PARTICLECOUNT,VOLTAGE,AIRSPEED,GPSLATITUDE,GPSLONGITUDE,GPSALTITUDE,GPSSATS;
int PACKETCOUNT,MISSIONTIME; 

String TELEMETRYPACKAGE;

void setup()
{
  Wire.begin();
  Serial.begin(9600);
  Serial1.begin(9600);
  Serial2.begin(9600);
  gpsPort.begin(9600);
  servo.attach(6);

  delay(10);
  
  xbee.setSerial(Serial1);
  sps30.SetSerialPin(RX_PIN,TX_PIN);
  sps30.start();
  
  

  delay(50);

  pinMode(BUZZER,OUTPUT);
  pinMode(LED,OUTPUT);
  digitalWrite(LED,HIGH);
  delay(5000);
  digitalWrite(LED,LOW);

  
}

void loop()
{
  C1=EEPROM.read(0);
  C2=EEPROM.read(1);
  C3=EEPROM.read(2);
  C4=EEPROM.read(3);
  C5=EEPROM.read(4);
  C6=EEPROM.read(5);
  
  PRESSURE= getPressure();   // in Pa
  TEMPERATURE= getTemperature();  // in C
  ALTITUDE= getAltitude();   // in m
  PARTICLECOUNT= getParticle();  //in mg/m^3
  VOLTAGE= getVoltage();    // in V
  AIRSPEED= getAirspeed();  // in m/s
  GPSSATS= (gps.satellites(), TinyGPS::GPS_INVALID_SATELLITES, 5);
  GPSALTITUDE=(gps.f_altitude(), TinyGPS::GPS_INVALID_F_ALTITUDE, 7, 2);
  GPSLATITUDE=(flat, TinyGPS::GPS_INVALID_F_ANGLE, 10, 6);
  GPSLONGITUDE=(flon, TinyGPS::GPS_INVALID_F_ANGLE, 11, 6);
  GPSTIME=String(HOUR)+":"+String(MINUTE)+":"+String(SECOND);    //in HH:MM:SS
  

  float RefHeight = 213.36;     //in coordinates 12°58'32.2"N 79°09'38.3"E

  float RefAlt= GPSALTITUDE-RefHeight;

  //Software state
  count=EEPROM.read(8);
  if(RefAlt<3)
  {
    SOFTWARESTATE="GROUND OPERATION";
  }
  else if(RefAlt > 450 && count==0)
  {
    count++;
    EEPROM.write(8,count);
  }
  else if (RefAlt<450 && RefAlt>110 && count==1)
  {
    SOFTWARESTATE="GLIDING";
  }
  else if(RefAlt<110 && count==1)
  {
    SOFTWARESTATE="LANDING";
  }
  else
  {
    SOFTWARESTATE="ASCENDING";
  }
/////MISSIONTIME/////
  state=EEPROM.read(6);
  if(state==0)
  {
    t1=SECOND;
    state++;
    EEPROM.write(6,state);
  }
  if(state==1)
  {
    t2=SECOND;
  }
  MISSIONTIME= t2-t1;

  PacketCount=EEPROM.read(7);
  PACKETCOUNT=PacketCount;

  TELEMETRYPACKAGE= TEAMID + "," + String(MISSIONTIME) + "," + String(PACKETCOUNT) + "," + String(ALTITUDE) + "," + String(PRESSURE) + "," + String(TEMPERATURE) + "," + 
                    String(VOLTAGE) + "," + String(GPSTIME) + "," + String(GPSLATITUDE) + "," + String(GPSLONGITUDE) + "," + String(GPSALTITUDE) + "," + String(GPSSATS) 
                    + "," + String(AIRSPEED) + "," + SOFTWARESTATE + "," + String(PARTICLECOUNT);
  PacketCount++;
  EEPROM.write(7,PacketCount);

  Serial1.println(TELEMETRYPACKAGE);   //Send Telemetry package to GCS
  delay(1000);

 File dataFile = SD.open("datalog.txt", FILE_WRITE);
 //SD card datalogging
  if (dataFile) {
    dataFile.println(TELEMETRYPACKAGE);
    dataFile.close();
  }

  if(RefAlt<20 && count==1)     //Ring buzzer 
  {
    digitalWrite(BUZZER,HIGH);
  }

  if(RefAlt < 450 && RefAlt > 400 && count==1)       //Payload deployment
  {
    servo.write(60);
  }
  if(RefAlt<100 && count==1)       //Second parachute deployment
  {
    servo.write(120);
  }
  

if(Serial1.available()==true)  // Check if commmand sent from GCS
{
  char C=Serial1.read();
  if(C=='c')
  BarometerCalibration();
}
 
}

///////////FUNCTION FOR ALTIMETER//////////////

void DigitalPressure()  //get ADC converted value of pressure
{
  Wire.beginTransmission(BarometerAddress);
  Wire.write(0x00);
  Wire.write(0x48);   //commanding ADC for pressure conversion
  Wire.endTransmission();
  delay(2);
  Wire.requestFrom(BarometerAddress,2);
  int DP=Wire.read();
  
}

void DigitalTemperature()  //get ADC converted value of temperature
{
  Wire.beginTransmission(BarometerAddress);
  Wire.write(0x00);
  Wire.write(0x58);   //commanding ADC for temperature conversion 
  Wire.endTransmission();
  delay(2);
  Wire.requestFrom(BarometerAddress,2);
  DT=Wire.read();
  
}

void BarometerCalibration()    // Calibrate Altimeter and Read caliberation data
{
  Wire.beginTransmission(BarometerAddress);
  Wire.write(0x1E);    //Resets caliberation data
  Wire.endTransmission();
  
  Wire.beginTransmission(BarometerAddress);
  Wire.write(1);    
  Wire.endTransmission();
  Wire.requestFrom(BarometerAddress,1);
  C1=Wire.read();

  Wire.beginTransmission(BarometerAddress);
  Wire.write(2);    
  Wire.endTransmission();
  Wire.requestFrom(BarometerAddress,1);
  C2=Wire.read();

  Wire.beginTransmission(BarometerAddress);
  Wire.write(3);    
  Wire.endTransmission();
  Wire.requestFrom(BarometerAddress,1);
  C3=Wire.read();

  Wire.beginTransmission(BarometerAddress);
  Wire.write(4);   
  Wire.endTransmission();
  Wire.requestFrom(BarometerAddress,1);
  C4=Wire.read();

  Wire.beginTransmission(BarometerAddress);
  Wire.write(5);   
  Wire.endTransmission();
  Wire.requestFrom(BarometerAddress,1);
  C5=Wire.read();

  
  Wire.beginTransmission(BarometerAddress);
  Wire.write(6);    
  Wire.endTransmission();
  Wire.requestFrom(BarometerAddress,1);
  C6=Wire.read();

  int SAVE[6]={C1,C2,C3,C4,C5,C6};
  for(int i=0;i<6;i++)
  EEPROM.write(i,SAVE[i]);  
}

float getTemperature()  //get actual temperature
{ 
  DigitalTemperature();
  dT= DT- C5*pow(2,8);
  int TEMP= 2000+dT*C6/pow(2,23);
  float temperature= TEMP/100;
  return temperature;      //Temperature= TEMP/100 C
}

float getPressure()  //get actual temperature compensated pressure
{
  DigitalPressure();
  int oFF= C2*pow(2,16)+(C4*dT)/pow(2,7);
  int SENS=C1*pow(2,15)+(C3*dT)/pow(2,5);
  P=(DP*SENS/pow(2,21) - oFF)/pow(2,15);
  float pressure=100000*P/100;
  return pressure;     // Pressure= P/100 hPa
}

float getAltitude()    // get Altitude from ground
{
  float Altitude= (1-pow((P/101325),0.190284))*44307.694;   // in meters
  return Altitude;
}

/////////////FUNCTION FOR DUST PARTICLE SENSOR/////////////

float getParticle()
{
   sps30.GetValues(&val);
   float PM10= 1000*val.NumPM10;
   return PM10;
}

//////////////FUNCTION FOR VOLTAGE SENSOR/////////////

float getVoltage()
{
  int val= analogRead(23);
  float vol= map(val,0,1023,0,2.5);
  float voltage=37*vol/25;
  return voltage;
}

//////////////////FUNCTION FOR AIRSPEED SENSOR/////////////

float getAirspeed()
{
  byte  _status = fetch_pressure(&P_dat, &T_dat);
  PR = (double)((P_dat-819.15)/(14744.7)) ;
  PR = (PR - 0.49060678) ;
  PR = abs(PR);
  V = ((PR*13789.5144)/1.225);
  VV = (sqrt((V)));    //airspeed in m/s
  TR = (double)((T_dat*0.09770395701));
  TR = TR-50;

  return VV;
}

byte fetch_pressure(unsigned int *p_P_dat, unsigned int *p_T_dat)
{
  byte address, Press_H, Press_L, _status;
  unsigned int P_dat;
  unsigned int T_dat;

  Wire.beginTransmission(AirSpeedAddress);
  Wire.endTransmission();
  delay(100);

  Wire.requestFrom(AirSpeedAddress,4);
  Press_H = Wire.read();
  Press_L = Wire.read();
  byte Temp_H = Wire.read();
  byte  Temp_L = Wire.read();
  Wire.endTransmission();


  _status = (Press_H >> 6) & 0x03;
  Press_H = Press_H & 0x3f;
  P_dat = (((unsigned int)Press_H) << 8) | Press_L;
  *p_P_dat = P_dat;

  Temp_L = (Temp_L >> 5);
  T_dat = (((unsigned int)Temp_H) << 3) | Temp_L;
  *p_T_dat = T_dat;
  return (_status);
}

/////////////////////////////FUNCTION FOR GPS/////////////////////////////


static void smartdelay(unsigned long ms)   //To set frequency for GPS
{
  unsigned long start = millis();
  do {
    while (gpsPort.available())
      gps.encode(gpsPort.read());
  } while (millis() - start < ms);
}


static void print_date(TinyGPS &gps)
{
  int year;
  byte month, day, hour, minute, second, hundredths;
  unsigned long age;
  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age); 
  HOUR=hour;
  MINUTE=minute;
  SECOND=second;
  smartdelay(1000);
}
