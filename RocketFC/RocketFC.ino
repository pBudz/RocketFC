#include <InternalTemperature.h>
#include <ArduinoJson.h>
///#include <I2C.h>
#include <Adafruit_MLX90393.h>
#include <TeensyThreads.h>
#include <String.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <SD.h>

#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10
#define GPSSerial Serial1
int redLED = 3;
int greenLED = 4;
int yellowLED = 5;
int testLED = 8;
int count = 0;

float humidity;
float pressure;
float temperature;

#define ALTITUDE_inHg (30.07*33.864)//check nearest airfield or weather 
//station for local curent air pressure.
const int chipSelect = BUILTIN_SDCARD;
Adafruit_BME280 bme;
unsigned long delayTime;
double launchzeroalt;
File myFile;

//todo



void setup() {

char json[] = "{\"sensor\":\"gps\",\"time\":1351824120,\"data\":[48.756080,2.302038]}";     
StaticJsonDocument<256> jsonDocument;  

//JsonObject root = jsonDocument.parseObject(json);
deserializeJson(jsonDocument, json);

auto error = deserializeJson(jsonDocument, json);
if (error) {
    Serial.print(F("deserializeJson() failed with code "));
    Serial.println(error.c_str());
    return;
}





const char* sensor = jsonDocument["sensor"];
long time = jsonDocument["time"];
JsonArray data = jsonDocument["data"];
double latitude = jsonDocument["data"][0];
double longitude = jsonDocument["data"][1];

Serial.println(sensor);
Serial.println(time);
Serial.println(data);
Serial.println(latitude);
Serial.println(longitude);
//todo
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect.
  }


  Serial.print("Initializing SD card...");

  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");
  while (!Serial);
  myFile = SD.open("t.txt", FILE_WRITE);
  

  // make this baud rate fast enough so we aren't waiting on it
  Serial.begin(115200);

  // 9600 baud is the default rate for the Ultimate GPS
  GPSSerial.begin(9600);



  unsigned status;
  Serial.begin(115200);
  status = bme.begin();
  pinMode(redLED, OUTPUT);
  pinMode(greenLED, OUTPUT);
  pinMode(yellowLED, OUTPUT);
  pinMode(testLED, OUTPUT);
  Serial.begin(9600);
  if (status) {
    digitalWrite(yellowLED, LOW);
  }


  launchzeroalt = (bme.readPressure() / 100.0F);
}



void loop() {

  //GPS();
  Serial.println("-----------------------------------------------------");
  if ((bme.readTemperature() * 9 / 5 + 32) > 80) {
    digitalWrite(testLED, HIGH);
  }
  else {
    digitalWrite(testLED, LOW);
  }

  // put your main code here, to run repeatedly:
  digitalWrite(redLED, HIGH);

 // baroData();
  Serial.println("-----------------------------------------------------");
  delay(2000);

  digitalWrite(redLED, LOW);
  delay(50);
  if (myFile) {
    for (int i = 0; i < 100; i++){
    Serial.println("Writing to test.txt...");
//    myFile.print(bme.);
    // close the file:
    
    Serial.println("done.");
  }myFile.close();
  }

}

void GPS() {

  if (Serial.available()) {
    char c = Serial.read();

    GPSSerial.write(c);
  }

  char c = GPSSerial.read();

  Serial.write(c);

}
char baroData() {

  digitalWrite(greenLED, HIGH);
  Serial.print("Temperature = ");
  Serial.print(bme.readTemperature() * 9 / 5 + 32);
  
  Serial.println(" *F");
  Serial.print("Pressure = ");

  Serial.print((bme.readPressure() / 100.0F) * 0.03);
  Serial.println(" inHg");
  Serial.print((bme.readPressure()) / 100.0F);
  Serial.println(" inMb");

  Serial.print("Approx. Altitude AGL = ");
  Serial.print(bme.readAltitude(launchzeroalt) * 3.28084);
  Serial.println(" feet");
  Serial.print("Approx. Altitude MSL = ");
  Serial.print(bme.readAltitude(ALTITUDE_inHg) * 3.28084);
  Serial.println(" feet");

  Serial.print("Humidity = ");
  Serial.print(bme.readHumidity());
  Serial.println(" %");

  digitalWrite(greenLED, LOW);
  Serial.print("CPUtemp: ");
  Serial.print(InternalTemperature.readTemperatureC(), 1);
  Serial.println("°C");

}
