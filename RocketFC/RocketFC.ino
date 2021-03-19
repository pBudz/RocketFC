#include <InternalTemperature.h>
#include <Adafruit_Sensor.h>
#include <ArduinoJson.h>
///#include <I2C.h>
#include <Adafruit_MLX90393.h>
#include <TeensyThreads.h>
#include <String.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BME280.h>
#include <SD.h>
#include <Adafruit_GPS.h>
#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10
#define GPSSerial Serial1
#define GPSECHO false
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_GPS GPS(&GPSSerial);
uint32_t timer = millis();
int redLED = 3;
int greenLED = 4;
int yellowLED = 5;
int testLED = 8;
int count = 0;
const int buzzer = 24;

float humidity;
float pressure;
float temperature;

#define ALTITUDE_inHg (30.07*33.864)//check nearest airfield or weather station for local curent air pressure.
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;
const int chipSelect = BUILTIN_SDCARD;
Adafruit_BME280 bme;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
unsigned long delayTime;
double launchzeroalt;
File myFile;

//todo
StaticJsonDocument<35000> bme280;
StaticJsonDocument<225000> totallist;
StaticJsonDocument<35000> acceler;
StaticJsonDocument<35000> orientation;
StaticJsonDocument<35000> linearacc;
StaticJsonDocument<35000> magnetometer;
StaticJsonDocument<35000> angvelocity;


int correcttime;

void setup() {
  bme280["atmospheric sensors"] = "atmo";
  acceler["acceleration"] = "accel";
  orientation["orientation"] = "orien";
  linearacc["Linear accel"] = "linear";
  magnetometer["magnetometer"] = "mag";
  angvelocity["angular velocity"] = "angvel";
  pinMode(buzzer, OUTPUT);
  
  Serial.begin(9600);
  Serial.begin(115200);
  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  Serial.println("Complete!");

  delay(1000);
  bno.setExtCrystalUse(true);

  while (!Serial) {
    digitalWrite(greenLED, LOW);
    digitalWrite(redLED, LOW);
    digitalWrite(yellowLED, LOW);
    // wait for serial port to connect.
  }
  Serial.print("Initializing SD card...");//Preps SD card for writing

  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");
  while (!Serial);
  myFile = SD.open("t.txt", FILE_WRITE);
  
  // 9600 baud is the default rate for the Ultimate GPS
  GPSSerial.begin(9600);

  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(1000);
  GPSSerial.println(PMTK_Q_RELEASE);

  unsigned status;
  status = bme.begin();
  pinMode(redLED, OUTPUT);
  pinMode(greenLED, OUTPUT);
  pinMode(yellowLED, OUTPUT);
  pinMode(testLED, OUTPUT);
  if (status) {
    digitalWrite(yellowLED, LOW);
  }

  launchzeroalt = (bme.readPressure() / 100.0F);
  correcttime = millis();
}
JsonArray total = totallist.createNestedArray("total");
JsonArray atmo = bme280.createNestedArray("atmo");
JsonArray accel2 = acceler.createNestedArray("accel2");
JsonArray orient = orientation.createNestedArray("orient");
JsonArray linear = linearacc.createNestedArray("linear");
JsonArray magnet = magnetometer.createNestedArray("magnet");
JsonArray angvel = angvelocity.createNestedArray("angvel");


void loop() {
  timer = millis() - correcttime;
  sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

  int8_t boardTemp = bno.getTemp();
//  Serial.println();
//  Serial.print(F("temperature: "));
 // Serial.println(boardTemp);

  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
//  Serial.println();
//  Serial.print("Calibration: Sys=");
//  Serial.print(system);
//  Serial.print(" Gyro=");
//  Serial.print(gyro);
//  Serial.print(" Accel=");
//  Serial.print(accel);
//  Serial.print(" Mag=");
//  Serial.println(mag);

//  Serial.println("--");
  delay(BNO055_SAMPLERATE_DELAY_MS);
  digitalWrite(buzzer, LOW);
  atmo.add((bme.readPressure() / 100.0F) * 0.03);
  atmo.add(bme.readTemperature() * 9 / 5 + 32);
  atmo.add(bme.readAltitude(launchzeroalt) * 3.28084);
  atmo.add(timer);
  orient.add(printEventOrientationX(&orientationData));
  orient.add(printEventOrientationY(&orientationData));
  orient.add(printEventOrientationZ(&orientationData));
  orient.add(timer);
  linear.add(printEventLinearAccelerometerX(&linearAccelData));
  linear.add(printEventLinearAccelerometerY(&linearAccelData));
  linear.add(printEventLinearAccelerometerZ(&linearAccelData));
  linear.add(timer);
  magnet.add(printEventMagneticX(&magnetometerData));
  magnet.add(printEventMagneticY(&magnetometerData));
  magnet.add(printEventMagneticZ(&magnetometerData));
  magnet.add(timer);
  angvel.add(printRotationVectorX(&angVelocityData));
  angvel.add(printRotationVectorY(&angVelocityData));
  angvel.add(printRotationVectorZ(&angVelocityData));
  angvel.add(timer);
  accel2.add(printEventAccelerometerX(&accelerometerData));
  accel2.add(printEventAccelerometerY(&accelerometerData));
  accel2.add(printEventAccelerometerZ(&accelerometerData));
  accel2.add(timer);

  if (myFile) {                         //code to write to microssd
    if (millis() - correcttime >= 1000) {
      
      total.add(atmo);
      total.add(accel2);
      total.add(orient);
      total.add(linear);
      total.add(magnet);
      total.add(angvel);
      Serial.println("Writing to test.txt...");
      serializeJsonPretty(totallist, myFile);
      myFile.close();
      Serial.println("done.");

      while (millis() - correcttime >= 1000){
        //tone(buzzer, 5000);
      }
    }
  }
  

}

double printEventAccelerometerX(sensors_event_t* event) {
  double x = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    x = event->acceleration.x;
   // Serial.print(x);
    return x;
}
}
double printEventAccelerometerY(sensors_event_t* event) {
  double y = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    y = event->acceleration.y;
    //Serial.print(y);
    return y;
}
}
double printEventAccelerometerZ(sensors_event_t* event) {
  double z = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    z = event->acceleration.z;
   // Serial.print(z);
    return z;
}
}
double printEventOrientationX(sensors_event_t* event) {
  double x = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ORIENTATION) {
    x = event->orientation.x;
  //  Serial.print(x);
    return x;
}
}
double printEventOrientationY(sensors_event_t* event) {
  double y = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ORIENTATION) {
    y = event->orientation.y;
  //  Serial.print(y);
    return y;
}
}
double printEventOrientationZ(sensors_event_t* event) {
  double z = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ORIENTATION) {
    z = event->orientation.z;
  //  Serial.print(z);
    return z;
}
}
double printEventLinearAccelerometerX(sensors_event_t* event) {
  double x = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    x = event->acceleration.x;
   // Serial.print(x);
    return x;
}
}
double printEventLinearAccelerometerY(sensors_event_t* event) {
  double y = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    y = event->acceleration.y;
  //  Serial.print(y);
    return y;
}
}
double printEventLinearAccelerometerZ(sensors_event_t* event) {
  double z = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    z = event->acceleration.z;
   // Serial.print(z);
    return z;
}
}
double printEventMagneticX(sensors_event_t* event){
  double x = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    x = event->magnetic.x;
   // Serial.print(x);
    return x;   
}
}
double printEventMagneticY(sensors_event_t* event){
  double y = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    y = event->magnetic.y;
   // Serial.print(y);
    return y;   
}
}
double printEventMagneticZ(sensors_event_t* event){
  double z = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    z = event->magnetic.z;
   // Serial.print(z);
    return z;   
}
}

double printRotationVectorX(sensors_event_t* event){
  double x = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
    x = event->magnetic.x;
  //  Serial.print(x);
    return x;   
}
}
double printRotationVectorY(sensors_event_t* event){
  double y = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
    y = event->magnetic.y;
   // Serial.print(y);
    return y;   
}
}
double printRotationVectorZ(sensors_event_t* event){
  double z = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
    z = event->magnetic.z;
  //  Serial.print(z);
    return z;   
}
}
