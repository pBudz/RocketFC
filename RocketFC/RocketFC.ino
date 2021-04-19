
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
int recordtime = 210000; //time in milliseconds you want to record data.


int timer = millis();

const int yellowLED = 30;
const int redLED = 31;
const int blueLED = 29;
int correcttime;
float humidity;
float pressure;
float temperature;
unsigned long delayTime;
double launchzeroalt;
int BNO055_SAMPLERATE_DELAY_MS = 100;
const int chipSelect = BUILTIN_SDCARD;
File outputFile;

//------------------------------------------
//declares BME object
Adafruit_BME280 bme;
//instantiates BNO055
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
//------------------------------------------



//Declare Json Object, add names of different lists to be sent to SD file.
StaticJsonDocument<430000> totallist;
JsonArray atmo = totallist.createNestedArray("atmo");
JsonArray accel2 = totallist.createNestedArray("accel2");
JsonArray orient = totallist.createNestedArray("orient");
JsonArray magnet = totallist.createNestedArray("magnet");



void setup() {

  pinMode(redLED, OUTPUT);
  pinMode(yellowLED, OUTPUT);
  pinMode(blueLED, OUTPUT);

  Serial.begin(9600);
  Serial.begin(115200);
  Serial.println("Orientation Sensor Test"); Serial.println("");


  if (!bno.begin())
  {

    Serial.print("no BNO055 detected");
    digitalWrite(blueLED, HIGH);
    while (1);
  }
  Serial.println("~BNO055 Found~");

  delay(1000);

  Serial.print("Initializing SD card...");//Preps SD card for writing

  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed!");
    digitalWrite(blueLED, HIGH);
    while (1);
  }
  Serial.println("initialization done.");
  outputFile = SD.open("t.txt", FILE_WRITE);// replace string value with the name of your file.

  unsigned status;
  status = bme.begin();

  launchzeroalt = (bme.readPressure() / 100.0F);
  correcttime = millis();
//------------------------------------------
//------------------------------------------

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
        EEPROM.get(eeAddress, calibrationData); //get calibration data from BNO055 ROM.

        Serial.println("\n\nRestoring Calibration data to the BNO055...");
        bno.setSensorOffsets(calibrationData);

        Serial.println("\n\nCalibration data loaded into BNO055");
        foundCalib = true;
    }
    bno.setExtCrystalUse(true);
//------------------------------------------
//------------------------------------------
}

int count = 0;

void loop() {
  timer = millis() - correcttime;
  double heading;
  Serial.println();
  displayCalStatus();
  
  sensors_event_t orientation, magnetometer, accelerometer;// declare variables to assign values to from the bno055
//  bno.getEvent(&angVelo, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&orientation, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&magnetometer, Adafruit_BNO055::VECTOR_MAGNETOMETER);
//  bno.getEvent(&linearAccel, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&accelerometer, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  
  uint8_t system, gyro, accel, mag = 0;

  
  bno.getCalibration(&system, &gyro, &accel, &mag);
    if(gyro == 3 && mag == 3 && system == 3){
    digitalWrite(yellowLED, HIGH);
        }
  //Serial.print(printEventLinearAccelerometerX(&linearAccel));
  delay(BNO055_SAMPLERATE_DELAY_MS);
  atmo.add((bme.readPressure() / 100.0F) * 0.03);
  atmo.add(bme.readTemperature() * 9 / 5 + 32);
  atmo.add(bme.readHumidity());
  atmo.add(bme.readAltitude(launchzeroalt) * 3.28084);
  //Serial.println(bme.readAltitude(launchzeroalt) * 3.28084);
  //atmo.add(timer);
  orient.add(printEventOrientationX(&orientation));
  orient.add(printEventOrientationY(&orientation));
  orient.add(printEventOrientationZ(&orientation));
  //orient.add(timer);

  heading = atan2 (printEventMagneticY(&magnetometer),printEventMagneticX(&magnetometer)) * 180 / PI;
  if(heading < 0){
    heading = heading * -2;
  }
  magnet.add(heading);
  //Serial.println(heading);
  //magnet.add(timer);
//  angvel.add(printRotationVectorX(&angVelo));
//  angvel.add(printRotationVectorY(&angVelo));
//  angvel.add(printRotationVectorZ(&angVelo));
  //angvel.add(timer);
  accel2.add(printEventAccelerometerX(&accelerometer));
  accel2.add(printEventAccelerometerY(&accelerometer));
  accel2.add(printEventAccelerometerZ(&accelerometer));
  accel2.add(timer);

  if (outputFile) {                         //code to write to microssd
    if (millis() - correcttime >= recordtime) {

      Serial.println("Writing to test.txt...");
      serializeJsonPretty(totallist, outputFile);
      outputFile.close();
      Serial.println("done.");

      while (millis() - correcttime >= recordtime) {
        digitalWrite(redLED, HIGH);
      }
    }
  }


}
//--------------------------------------------------------------
// following functions use previously declared sensor events to find what TYPE of event is detected. if the type is of the the- 
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
//double printEventLinearAccelerometerX(sensors_event_t* event) {
//  double x = -1000000; //dumb values, easy to spot problem
//  if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
//    x = event->acceleration.x;
//    // Serial.print(x);
//    return x;
//  }
//}
//double printEventLinearAccelerometerY(sensors_event_t* event) {
//  double y = -1000000; //dumb values, easy to spot problem
//  if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
//    y = event->acceleration.y;
//    //  Serial.print(y);
//    return y;
//  }
//}
//double printEventLinearAccelerometerZ(sensors_event_t* event) {
//  double z = -1000000; //dumb values, easy to spot problem
//  if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
//    z = event->acceleration.z;
//    // Serial.print(z);
//    return z;
//  }
//}
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

//double printRotationVectorX(sensors_event_t* event) {
//  double x = -1000000; //dumb values, easy to spot problem
//  if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
//    x = event->magnetic.x;
//    //  Serial.print(x);
//    return x;
//  }
//}
//double printRotationVectorY(sensors_event_t* event) {
//  double y = -1000000; //dumb values, easy to spot problem
//  if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
//    y = event->magnetic.y;
//    // Serial.print(y);
//    return y;
//  }
//}
//double printRotationVectorZ(sensors_event_t* event) {
//  double z = -1000000; //dumb values, easy to spot problem
//  if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
//    z = event->magnetic.z;
//    //  Serial.print(z);
//    return z;
//  }
//}
//------------------------------------------
//------------------------------------------
//------------------------------------------
//------------------------------------------
//------------------------------------------
//------------------------------------------
//------------------------------------------ following is debug prints for testing, not written by me.
//
//void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData)
//{
//    Serial.print("Accelerometer: ");
//    Serial.print(calibData.accel_offset_x); Serial.print(" ");
//    Serial.print(calibData.accel_offset_y); Serial.print(" ");
//    Serial.print(calibData.accel_offset_z); Serial.print(" ");
//
//    Serial.print("\nGyro: ");
//    Serial.print(calibData.gyro_offset_x); Serial.print(" ");
//    Serial.print(calibData.gyro_offset_y); Serial.print(" ");
//    Serial.print(calibData.gyro_offset_z); Serial.print(" ");
//
//    Serial.print("\nMag: ");
//    Serial.print(calibData.mag_offset_x); Serial.print(" ");
//    Serial.print(calibData.mag_offset_y); Serial.print(" ");
//    Serial.print(calibData.mag_offset_z); Serial.print(" ");
//
//    Serial.print("\nAccel Radius: ");
//    Serial.print(calibData.accel_radius);
//
//    Serial.print("\nMag Radius: ");
//    Serial.print(calibData.mag_radius);
//}
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
