/* CAMBIOS EN LA VERSION
 *   contactBounceTime cambia de volatile unsigned long a  unsigned long ya que no se usa fuera del ISR
 *   contactTime cambia de volatile unsigned long a  unsigned long ya que no se usa fuera del ISR
 *     if (calDirection < 0) calDirection = calDirection + 360; se agrego debido a que estaba duplicado para > 360
 */

#include "TimerOne.h"
#include <math.h>
#include "DHT.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SPI.h>
#include <Ethernet.h>    //Ethernet
#include <EthernetUdp.h>  //Ethernet

#define DHTPIN 5     // what digital pin we're connected to
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321

//#define Bucket_Size 0.282  // bucket size to trigger tip count (mm)
#define Bucket_Size 0.011   // bucket size to trigger tip count (in)
#define Rainmeter_Pin 3 // digital pin of the rainmeter reed switch

#define WindSensor_Pin (2) // digital pin for wind speed sensor
#define WindVane_Pin (A3) // analog pin for wind direction sensor
#define VaneOffset 0 // define the offset for caclulating wind direction

#define radSensor_Pin A1 // analog pin UV sensor is connected to
#define uvSensor_Pin A2 // analog pin solar irradiance sensor is connected to

#define TempOffset 1.9 // temperature offset from BME reading to actual ambient temperature

int DEBUG = 0;      // DEBUG flag; if set to 1, will write values back via serial
int eth = 0;        // Ethernet connectivity flag

// assign a MAC address for the ethernet controller here:
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
// assign an IP address for the controller:
/*IPAddress ip(192, 168, 2, 100);
//IPAddress gateway(192, 168, 2, 1);
//IPAddress subnet(255, 255, 255, 0);
*/
EthernetClient client;
unsigned int localPort = 8888;

IPAddress SERVER(192,168,1,6);         // Local WeeWX instance running on Raspberry Pi
//char SERVER[] = "<server>";           // Remote WeeWX instance
char WEBPAGE [] = "GET /weatherstation/updateweatherstation.php?";     // same as if it were WU for Interceptor to work
char ID [] = "<wundergroundID>";
char PASSWORD [] = "<wundergroundPWD>";
char c[10];

unsigned int connections = 0;           // number of connections
unsigned int timeout = 30000;           // Milliseconds -- 1000 = 1 Second

volatile unsigned long tipCount; // rain bucket tip counter used in interrupt routine
unsigned long contactTime; // timer to manage any rain contact bounce in interrupt routine

volatile bool isSampleRequired; // this is set every 2.5sec to generate wind speed
volatile unsigned int timerCount; // used to count ticks for 2.5sec timer count
volatile unsigned int timerMinCount; // used to determine 1-minute count
volatile unsigned long rotations; // ISR cup rotation counter
volatile unsigned int intervalRotationCount; // cup rotation counter for wind speed calcs
unsigned long contactBounceTime; // timer to avoid contact bounce in wind speed sensor

float totalRainfall; // rain inches so far today in local time
long lastTipcount; // keep track of bucket tips
long accumTipCount;  // total bucket tips measured

volatile unsigned long uvValue;
volatile unsigned long solarValue;
int solarRad;       //Solar radiation in w/m2
float uvIndex;      // UV index varies from 0 - 16

volatile float windSpeed;
volatile float lastWindGust;
volatile float windGust;
int vaneValue; // raw analog value from wind vane
int vaneDirection; // translated 0 - 360 wind direction
int calDirection; // calibrated direction after offset applied
int lastDirValue; // last recorded direction value

const int numReadings = 5;      // array size
int readings[numReadings];      // the readings from the windvane
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
int average = 0;                // the average wind direction

unsigned int windGustTimer;
unsigned int currentRFtimer;
unsigned long totalRFtimer;

// Create DHT sensor object
DHT dht (DHTPIN, DHTTYPE);

// Create Sensor Object on I2C bus
Adafruit_BME280 bme;

void setup() {

  if (Ethernet.begin(mac) ) {   // Turn the internet ON
    eth = 1;
  }

  // BME280 sensor initialization
  bool status;
  status = bme.begin();

  // DHT sensor initialization
  //dht.begin();
  delay(2000);   // allow 2 seconds for sensor module to settle down

  Serial.begin(19200);

  if (DEBUG) {

    Serial.println("Davis Vantage Pro Plus Weather Station");
    Serial.println("++ Debugging Console ++");
    Serial.println(F("\nInitializing network......\t"));

    if (eth) {

      Serial.println("Ethernet Initialization: COMPLETE!\t");
      Serial.println("Assigned IP address: "); Serial.print(Ethernet.localIP());
      Serial.println("\t");
    }
    else {
      Serial.println("Something went wrong during network initialization :'( \t");
    }

        if (!status) {
        Serial.println("Could not find a valid BME280 sensor!");
    }
  }


  // initialize all the rain sensor readings to 0
  lastTipcount = 0;
  tipCount = 0;
  accumTipCount = 0;
  totalRainfall = 0;
  currentRFtimer = 0;
  totalRFtimer = 0;

  // initialize all the anemometer readings to 0
  lastDirValue = 0;
  rotations = 0;
  intervalRotationCount = 0;
  windGust = 0;
  lastWindGust = 0;
  windGustTimer = 0;
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }

  // initialize all the timer values to 0
  timerCount = 0;
  timerMinCount = 0;


  // initialize all the solar values to 0
  solarValue = 0;
  solarRad = 0;
  uvValue = 0;
  uvIndex = 0;

  pinMode(Rainmeter_Pin, INPUT);
  pinMode(WindSensor_Pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(Rainmeter_Pin), isr_rg, FALLING);
  attachInterrupt(digitalPinToInterrupt(WindSensor_Pin), isr_rotation, FALLING);
  pinMode(uvSensor_Pin, INPUT);
  pinMode(radSensor_Pin, INPUT);

  // setup the timer for 0.5 second
  Timer1.initialize(500000);
  Timer1.attachInterrupt(isr_timer);

  // Enable Interrupts
  sei();
}

void loop() {

  // get the sensor data
  float h = bme.readHumidity();
  float t = (bme.readTemperature() - TempOffset);     // substract the temperature offset from reading to obtain actual ambient temp
  //float dhtH = dht.readHumidity();
  //float dhtT = dht.readTemperature();
  float dhtTf = (t * 9.0) / 5.0 + 32.0;               // was dht.readTemperature, need to convert native C to F
  float tempf = (t * 9.0) / 5.0 + 32.0;               // was bme.readTemperature, need to convert native C to F
  float dewptf = dewPoint(t, h);                      // Dew point calc (wunderground), replaced bme.readtemp with converted temp
  float pressure = (bme.readPressure() / 100.0);      // get barometric pressure in hPa
  float baro = (pressure * 0.02953);                  // wunderground requires pressure in inHg
  
  // update total rainfall if required
  if (tipCount != lastTipcount) {
    cli(); // disable interrupts
    lastTipcount = tipCount;
    totalRainfall = tipCount * Bucket_Size;
    sei(); // enable interrupts
  }

  // convert to mph using the formula V=P(2.25/T)
  // V = P(2.25/2.5) = P * 0.9
  windSpeed = (intervalRotationCount * 0.7);  // calibrado 08-05-20
  if (windSpeed > 7.0 ) {
    windGust = windSpeed;
    if (windGust >= lastWindGust) {
      lastWindGust = windGust;
    }
  }

  solarValue = analogRead(radSensor_Pin);
  solarRad = map(solarValue, 0, 1023, 0, 2900); // calibrado en 2900  17-04-18 mayor numero significa mas watts
  solarRad = constrain(solarRad, 0, 1800);
  if (solarRad < 16) {
    solarRad = 0;
  }

    /*uvValue = analogRead(uvSensor_Pin);
  uvIndex = (uvValue / 1024.0);
  uvIndex = constrain(uvIndex, 0, 16);
  if (uvIndex < 0.65) {
    uvIndex = 0;
  }*/

if (timerMinCount > 1) { // if 5 seconds timer is up then increment windgust timer count
    //currentRFtimer++;
    //totalRFtimer++;
    windGustTimer++;
}

  if (timerMinCount > 23) { // if 1-minute timer is up then send data
    cli(); // disable interrupts
    accumTipCount = accumTipCount + tipCount;  //keep track of total bucket tips
    timerMinCount = 0;  // reset timer
    sei(); // enable interrupts

    getWindDirection();

    // Start of debug data loop
    if (DEBUG) {

      Serial.println("----------------------------------------------");
      Serial.print("T: "); Serial.print(t); Serial.print(char(176)); Serial.print("C");
      Serial.print(" | "); Serial.print(tempf); Serial.print(char(176)); Serial.print("F\t");
      Serial.print("DP: "); Serial.print(dewptf); Serial.print(char(176)); Serial.print("F\t");
      Serial.print("H: "); Serial.print(h); Serial.print("%\t");
      Serial.print("P: "); Serial.print(pressure); Serial.print("hPa\t");
      Serial.print(" | "); Serial.print(baro); Serial.print("inHg\t");
      Serial.print("R: "); Serial.print(totalRainfall); Serial.print(" mm\t");
      Serial.print("TC: "); Serial.print(accumTipCount); Serial.print(" counts");
      Serial.print("S: "); Serial.print(windSpeed); Serial.print(" mph\t");
      Serial.print("D: "); Serial.print(calDirection); Serial.print(char(176)); Serial.print("\t");
      Serial.print("R: "); Serial.print(solarRad); Serial.print(" W/m^2\t");
      Serial.print("UV: "); Serial.print(uvIndex); Serial.print("\t");
      Serial.println(" ");
    }
    // End of debug data loop

    // Send data to WeeWX server
    if (client.connect(SERVER, 8080)) {
      if (DEBUG) {
        Serial.println("Connected!");
      }
      // Wrap the data
      client.print(WEBPAGE);
      client.print("ID=");
      client.print(ID);
      client.print("&PASSWORD=");
      client.print(PASSWORD);
      client.print("&action=updateraw");  // Standard update
      client.print("&winddir=");
      client.print(calDirection);
      client.print("&windspeedmph=");
      client.print(windSpeed);
      if (lastWindGust != 0) {
        client.print("&windgustmph=");
        client.print(lastWindGust);
      }
      client.print("&tempf=");
      client.print(tempf);
      //client.print("&tempinf=");
      //client.print(dhtTf);
      client.print("&dailyrainin=");
      client.print(totalRainfall);
      client.print("&baromin=");
      client.print(baro);
      if (isnan(dewptf) == false) {
        client.print("&dewptf=");
        client.print(dewptf);
      }
      client.print("&humidity=");
      client.print(h);
      client.print("&solarradiation=");
      client.print(solarRad);
      /*client.print("&UV=");
      client.print(uvIndex);*/
      client.print("&dateutc=");
      client.print("now");
      client.print(" HTTP/1.1");
      client.println();                  // finish HTTP request
      client.stop();      // Disconnect from the server 

      if (DEBUG) {
        Serial.println("Upload COMPLETE\t");
      }

    }
    else {
      if (DEBUG) {
        Serial.println(F("Connection FAILED\t"));
      }
      return;
    }

    // timer to clear the current rainfall to 0mm every day
    if (currentRFtimer >= 17568) {
      tipCount = 0;
      totalRainfall = 0;
      currentRFtimer = 0;
    }

    // timer to reset wind gusts to 0 every 5 minutes
    if (windGustTimer >= 61) {
      windGust = 0;
      lastWindGust = 0;
      windGustTimer = 0;
    }
  }
}

// Interrupt handler routine for timer interrupt
void isr_timer() {

  timerCount++;

  if (timerCount == 5) {
    intervalRotationCount = rotations;
    rotations = 0;
    timerCount = 0;
    timerMinCount++; // increment count
  }

}


// Interrupt handler routine to increment the rotation count for wind speed
void isr_rotation() {

  if ((millis() - contactBounceTime) > 10 ) { // debounce the switch contact
    rotations++;
    contactBounceTime = millis();
  }
}

// Interrupt handler routine that is triggered when the tipping buckets detects rain
void isr_rg() {

  if ((millis() - contactTime) > 10 ) { // debounce of sensor signal
    tipCount++;
    contactTime = millis();
  }
}

// Get Wind Direction
void getWindDirection() {

  vaneValue = analogRead(WindVane_Pin);
  vaneDirection = map(vaneValue, 0, 1023, 0, 360);
  calDirection = vaneDirection + VaneOffset;

  if (calDirection > 360)
    calDirection = calDirection - 360;

  if (calDirection < 0)
    calDirection = calDirection + 360;
    
// Average wind vane readings
  total = total - readings[readIndex];         // subtract the last reading
  readings[readIndex] = calDirection;          // read current direction
  total = total + readings[readIndex];         // add the reading to the total
  readIndex = readIndex + 1;                   // advance to the next position in the array
  if (readIndex >= numReadings) {              // if we're at the end of the array...
    readIndex = 0;                             // ...wrap around to the beginning
  }
  average = total / numReadings;               // calculate the average
  calDirection = average;

}

// Calculate Dew Point
double dewPoint(double t, double humidity) {

  double A0 = 373.15 / (273.15 + t);
  double SUM = -7.90298 * (A0 - 1);
  SUM += 5.02808 * log10(A0);
  SUM += -1.3816e-7 * (pow(10, (11.344 * (1 - 1 / A0))) - 1) ;
  SUM += 8.1328e-3 * (pow(10, (-3.49149 * (A0 - 1))) - 1) ;
  SUM += log10(1013.246);
  double VP = pow(10, SUM - 3) * humidity;
  double T = log(VP / 0.61078);
  return (((241.88 * T) / (17.558 - T)) * 1.8) + 32;

}
