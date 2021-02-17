#include <InternalTemperature.h>
///#include <I2C.h>
#include <Adafruit_MLX90393.h>
#include <TeensyThreads.h>



#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
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

Adafruit_BME280 bme;
unsigned long delayTime;
double launchzeroalt;


void setup() {
  while (!Serial);

  // make this baud rate fast enough so we aren't waiting on it
  Serial.begin(115200);

  // 9600 baud is the default rate for the Ultimate GPS
  GPSSerial.begin(9600);



  unsigned status;
  Serial.begin(115200);
  // default settings
  status = bme.begin();
  // put your setup code here, to run once:
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

  if (Serial.available()) {
    char c = Serial.read();
    GPSSerial.write(c);
  }
  if (GPSSerial.available()) {
    char c = GPSSerial.read();
    Serial.write(c);
  }
\
  Serial.println("-----------------------------------------------------");
  if ((bme.readTemperature() * 9 / 5 + 32) > 80) {
    digitalWrite(testLED, HIGH);
  }
  else {
    digitalWrite(testLED, LOW);
  }

  // put your main code here, to run repeatedly:
  digitalWrite(redLED, HIGH);

  baroData();
  Serial.println("-----------------------------------------------------");
  delay(2000);

  digitalWrite(redLED, LOW);
  delay(50);

}

/*


  void averageAlt(){
  double altArr[10];
  int i,j = 0;
  int len = sizeof(altArr);
  double avg;
  for (i; i < 9; i++){
    altArr[i] = (bme.readPressure() / 100.0F)*0.03;

  }
  for (j; j < 9; j++){
    avg = avg + altArr[j];
  }
  //Serial.print(avg/10);
  avg = 0;

  }
*/
void baroData() {

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
