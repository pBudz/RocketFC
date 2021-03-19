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
StaticJsonDocument<256000> bme280;
int correcttime;

void setup() {
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

  /*
    //bm
    // * example json object
    StaticJsonDocument<200> doc;
    doc["sensor"] = "gps";
    doc["time"] = 1351824120;
    JsonArray data = doc.createNestedArray("data");
    data.add(48.756080);
    data.add(2.302038);
    // * example json object
    bme280["sensor"] = "atmo";
    bme280["time"] = millis();
    JsonArray atmo = doc.createNestedArray("atmo");
    atmo.add((bme.readPressure() / 100.0F) * 0.03);
    /*
    StaticJsonDocument<200> GPS;
    GPS["sensor"] = "gps";
    GPS["time"] = gps.millis()
    StaticJsonDocument<200> BNO055;
    StaticJsonDocument<200> BME280;
    JsonArray data = doc.createNestedArray("data");
    JsonArray data = doc.createNestedArray("data");
    JsonArray data = doc.createNestedArray("data");
  */
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

JsonArray atmo = bme280.createNestedArray("atmo");

void loop() {

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

  atmo.add((bme.readPressure() / 100.0F) * 0.03);
  atmo.add(bme.readTemperature() * 9 / 5 + 32);
  atmo.add(bme.readAltitude(launchzeroalt) * 3.28084);
  atmo.add(printEventAccelerometerX(&accelerometerData));
  atmo.add(printEventAccelerometerY(&accelerometerData));
  atmo.add(printEventAccelerometerZ(&accelerometerData));
  atmo.add(printEventOrientationX(&orientationData));
  atmo.add(printEventOrientationY(&orientationData));
  atmo.add(printEventOrientationZ(&orientationData));
  atmo.add(printEventLinearAccelerometerX(&linearAccelData));
  atmo.add(printEventLinearAccelerometerY(&linearAccelData));
  atmo.add(printEventLinearAccelerometerZ(&linearAccelData));
  atmo.add(printEventMagneticX(&magnetometerData));
  atmo.add(printEventMagneticY(&magnetometerData));
  atmo.add(printEventMagneticZ(&magnetometerData));
  atmo.add(printRotationVectorX(&angVelocityData));
  atmo.add(printRotationVectorY(&angVelocityData));
  atmo.add(printRotationVectorZ(&angVelocityData));

  if (myFile) {                         //code to write to microssd
    if (millis() - correcttime >= 180000) {
      Serial.println("Writing to test.txt...");
      serializeJsonPretty(bme280, myFile);
      myFile.close();
      Serial.println("done.");
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
//double printGyroscopeX(sensors_event_t* event){
//  double x = -1000000; //dumb values, easy to spot problem
//  if (event->type == SENSOR_TYPE_GYROSCOPE) {
//    x = event->magnetic.x;
//    Serial.print(x);
//    return x;  
//}
//}
//double printGyroscopeY(sensors_event_t* event){
//  double y = -1000000; //dumb values, easy to spot problem
//  if (event->type == SENSOR_TYPE_GYROSCOPE) {
//    y = event->magnetic.y;
//    Serial.print(y);
//    return y;  
//}
//}
//double printGyroscopeZ(sensors_event_t* event){
//  double z = -1000000; //dumb values, easy to spot problem
//  if (event->type == SENSOR_TYPE_GYROSCOPE) {
//    z = event->magnetic.z;
//    Serial.print(z);
//    return z;  
//}
//}
//get grav and ang velocity
//
//
//char printEvent(sensors_event_t* event) {
//  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
//  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
//    Serial.print("Accl:");
//    x = event->acceleration.x;
//    y = event->acceleration.y;
//    z = event->acceleration.z;
//  }
// 
//  else if (event->type == SENSOR_TYPE_ORIENTATION) {
//    Serial.print("Orient:");
//    x = event->orientation.x;
//    y = event->orientation.y;
//    z = event->orientation.z;
//  }
//  else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
//    Serial.print("Mag:");
//    x = event->magnetic.x;
//    y = event->magnetic.y;
//    z = event->magnetic.z;
//  }
//  else if (event->type == SENSOR_TYPE_GYROSCOPE) {
//    Serial.print("Gyro:");
//    x = event->gyro.x;
//    y = event->gyro.y;
//    z = event->gyro.z;
//  }
//  else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
//    Serial.print("Rot:");
//    x = event->gyro.x;
//    y = event->gyro.y;
//    z = event->gyro.z;
//  }
//  else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
//    Serial.print("Linear:");
//    x = event->acceleration.x;
//    y = event->acceleration.y;
//    z = event->acceleration.z;
//  }
//  else {
//    Serial.print("Unk:");
//  }
//
//  Serial.print("\tx= ");
//  Serial.print(x);
//  Serial.print(" |\ty= ");
//  Serial.print(y);
//  Serial.print(" |\tz= ");
//  Serial.println(z);
//}
