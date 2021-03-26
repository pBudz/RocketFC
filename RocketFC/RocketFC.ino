//#include <I2C.h>
#include <TeensyThreads.h>
#include <String.h>
#include <Wire.h>
#include <SPI.h>
#include <InternalTemperature.h>



//------------------------------------------
//CORE libraries for hardware interface
#include <Adafruit_Sensor.h>
#include <ArduinoJson.h>
#include <SD.h>
//------------------------------------------
//------------------------------------------
//BME Library & hardware interface definitions for BME280, atmospheric sensor
#include <Adafruit_BME280.h>
#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10
#define ALTITUDE_inHg (29.96*33.864)//check nearest airfield or weather station for local curent air pressure.
//------------------------------------------
//------------------------------------------
//BNO055 & math logic library for IMU's.
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>
//------------------------------------------
//------------------------------------------
//GPS Libraries and definitions
#define GPSSerial Serial1
#define GPSECHO false
#include <Adafruit_GPS.h>
Adafruit_GPS GPS(&GPSSerial);
//------------------------------------------
uint32_t timer = millis();

const int buzzer = 32;
const int yellowLED = 30;
const int redLED = 31;

float humidity;
float pressure;
float temperature;

uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;
const int chipSelect = BUILTIN_SDCARD;
//------------------------------------------
//declares BME object
Adafruit_BME280 bme;
//instantiates BNO055
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
//------------------------------------------
unsigned long delayTime;
double launchzeroalt;
File myFile;

//todo
//Declare Json Object, add names of different lists to be sent to SD file.
StaticJsonDocument<430000> totallist;
JsonArray atmo = totallist.createNestedArray("atmo");
JsonArray accel2 = totallist.createNestedArray("accel2");
JsonArray orient = totallist.createNestedArray("orient");
JsonArray linear = totallist.createNestedArray("linear");
JsonArray magnet = totallist.createNestedArray("magnet");
JsonArray angvel = totallist.createNestedArray("angvel");

int correcttime;
int recordtime = 110000; //time in milliseconds you want to record data.
void setup() {
  //------------------------------------------
  //instantiate

  //------------------------------------------
  pinMode(buzzer, OUTPUT);
  pinMode(redLED, OUTPUT);
  pinMode(yellowLED, OUTPUT);

  Serial.begin(9600);
  Serial.begin(115200);
  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("no BNO055 detected");
    while (1);
  }
  Serial.println("Complete!");

  delay(1000);
  bno.setExtCrystalUse(true);

 // while (!Serial) {

    // wait for serial port to connect.
  //}
  Serial.print("Initializing SD card...");//Preps SD card for writing

  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");
  //while (!Serial);
  myFile = SD.open("t.txt", FILE_WRITE);//change field 1 to whatever you want to name the file.. "example.txt"

  // 9600 baud is the default rate for the Ultimate GPS
  //--------------------------------------------------------------
  //gps code currently not in use
//  GPSSerial.begin(9600);
//  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
//  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
//  GPS.sendCommand(PGCMD_ANTENNA);
//  delay(1000);
//  GPSSerial.println(PMTK_Q_RELEASE);
 //--------------------------------------------------------------
  unsigned status;
  status = bme.begin();
  //if (status) {

 // }

  launchzeroalt = (bme.readPressure() / 100.0F);
  correcttime = millis();

  int eeAddress = 0;
  long bnoID;
  bool foundCalib = false;

  EEPROM.get(eeAddress, bnoID);

    adafruit_bno055_offsets_t calibrationData;
    sensor_t sensor;

    /*
    *  Look for the sensor's unique ID at the beginning oF EEPROM.
    *  This isn't foolproof, but it's better than nothing.
    */
    bno.getSensor(&sensor);
    if (bnoID != sensor.sensor_id)
    {
        Serial.println("\nNo Calibration Data for this sensor exists in EEPROM");
        delay(500);
    }
    else
    {
        Serial.println("\nFound Calibration for this sensor in EEPROM.");
        eeAddress += sizeof(long);
        EEPROM.get(eeAddress, calibrationData);

        displaySensorOffsets(calibrationData);

        Serial.println("\n\nRestoring Calibration data to the BNO055...");
        bno.setSensorOffsets(calibrationData);

        Serial.println("\n\nCalibration data loaded into BNO055");
        foundCalib = true;
    }
    bno.setExtCrystalUse(true);


  
}



void loop() {
  timer = millis() - correcttime;
  sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;// declare variables to assign values to from the bno055
  
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
    Serial.println();
    Serial.print("Calibration: Sys=");
    Serial.print(system);
    Serial.print(" Gyro=");
    Serial.print(gyro);
    Serial.print(" Accel=");
    Serial.print(accel);
    Serial.print(" Mag=");
    Serial.println(mag);

  //  Serial.println("--");
  if(accel == 3 ){
    tone(buzzer, 2000);
    digitalWrite(yellowLED, HIGH);
  }
  delay(BNO055_SAMPLERATE_DELAY_MS);
  digitalWrite(buzzer, LOW);
  atmo.add((bme.readPressure() / 100.0F) * 0.03);
  atmo.add(bme.readTemperature() * 9 / 5 + 32);
  atmo.add(bme.readAltitude(launchzeroalt) * 3.28084);
  Serial.println(bme.readAltitude(launchzeroalt) * 3.28084);
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
    if (millis() - correcttime >= recordtime) {

      Serial.println("Writing to test.txt...");
      serializeJsonPretty(totallist, myFile);
      myFile.close();
      Serial.println("done.");

      while (millis() - correcttime >= recordtime) {
        digitalWrite(redLED, HIGH);
      }
    }
  }


}
//--------------------------------------------------------------
// follow functions use previously declared sensor events to find what TYPE of event is detected. if the type is of the the- 
// -type specified in the function, the function reports the appropriate data.
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
double printEventMagneticX(sensors_event_t* event) {
  double x = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    x = event->magnetic.x;
    // Serial.print(x);
    return x;
  }
}
double printEventMagneticY(sensors_event_t* event) {
  double y = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    y = event->magnetic.y;
    // Serial.print(y);
    return y;
  }
}
double printEventMagneticZ(sensors_event_t* event) {
  double z = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    z = event->magnetic.z;
    // Serial.print(z);
    return z;
  }
}

double printRotationVectorX(sensors_event_t* event) {
  double x = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
    x = event->magnetic.x;
    //  Serial.print(x);
    return x;
  }
}
double printRotationVectorY(sensors_event_t* event) {
  double y = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
    y = event->magnetic.y;
    // Serial.print(y);
    return y;
  }
}
double printRotationVectorZ(sensors_event_t* event) {
  double z = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
    z = event->magnetic.z;
    //  Serial.print(z);
    return z;
  }
}

void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData)
{
    Serial.print("Accelerometer: ");
    Serial.print(calibData.accel_offset_x); Serial.print(" ");
    Serial.print(calibData.accel_offset_y); Serial.print(" ");
    Serial.print(calibData.accel_offset_z); Serial.print(" ");

    Serial.print("\nGyro: ");
    Serial.print(calibData.gyro_offset_x); Serial.print(" ");
    Serial.print(calibData.gyro_offset_y); Serial.print(" ");
    Serial.print(calibData.gyro_offset_z); Serial.print(" ");

    Serial.print("\nMag: ");
    Serial.print(calibData.mag_offset_x); Serial.print(" ");
    Serial.print(calibData.mag_offset_y); Serial.print(" ");
    Serial.print(calibData.mag_offset_z); Serial.print(" ");

    Serial.print("\nAccel Radius: ");
    Serial.print(calibData.accel_radius);

    Serial.print("\nMag Radius: ");
    Serial.print(calibData.mag_radius);
}
void displayCalStatus(void)
{
    /* Get the four calibration values (0..3) */
    /* Any sensor data reporting 0 should be ignored, */
    /* 3 means 'fully calibrated" */
    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);

    /* The data should be ignored until the system calibration is > 0 */
    Serial.print("\t");
    if (!system)
    {
        Serial.print("! ");
    }

    /* Display the individual values */
    Serial.print("Sys:");
    Serial.print(system, DEC);
    Serial.print(" G:");
    Serial.print(gyro, DEC);
    Serial.print(" A:");
    Serial.print(accel, DEC);
    Serial.print(" M:");
    Serial.print(mag, DEC);
}
