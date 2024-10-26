#include "DHT.h"
#include <SPI.h>
#include <Arduino.h>
#include <Wire.h>
#include <BMP180I2C.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <nRF24L01.h>
#include <RF24.h>

//#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    //#include "Wire.h"
//#endif

#define DHTPIN 2     // Digital pin connected to the DHT sensor
#define DHTTYPE DHT11
#define I2C_ADDRESS 0x77

RF24 radio(7, 8); // CE, CSN pins
const byte address[6] = "00001";

DHT dht(DHTPIN, DHTTYPE);
BMP180I2C bmp180(I2C_ADDRESS);
MPU6050 accelgyro;
unsigned long lastTime;
int16_t ax, ay, az;
int16_t gx, gy, gz;
float dt, Humidity, Temperature, Pressure, filteredAngleX, filteredAngleY;
#define OUTPUT_READABLE_ACCELGYRO
#define LED_PIN 13
bool blinkState = false;

void setup() {
  //dht code
  Serial.begin(9600);
  Serial.println(F("DHTxx test!"));
  dht.begin();

  //bmp code
  while (!Serial);
  Wire.begin();
  if (!bmp180.begin())
  {
    Serial.println("begin() failed. check your BMP180 Interface and I2C Address.");
    while (1);
  }
  bmp180.resetToDefaults();
  bmp180.setSamplingMode(BMP180MI::MODE_UHR);

  //mpu code
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    Serial.begin(9600);
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
    pinMode(LED_PIN, OUTPUT);
    lastTime = millis();
    
  // nrf code  
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
}

void loop() {
  //dht
  delay(100);
  float h = dht.readHumidity();
  if (isnan(h)) {
    Serial.println(F("Failed to read from DHT sensor!"));
  }
  Humidity = h;
 
  // bmp code 
  if (!bmp180.measureTemperature())
  {
    Serial.println("could not start temperature measurement, is a measurement already running?");
  }
  do
  {
    delay(100);
  } while (!bmp180.hasValue());
  Temperature = bmp180.getTemperature();
  
  if (!bmp180.measurePressure())
  {
    Serial.println("could not start perssure measurement, is a measurement already running?");
  }
  do
  {
    delay(100);
  } while (!bmp180.hasValue());
  Pressure = bmp180.getPressure();
  
  //mpu code
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  unsigned long currentTime = millis();
  dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;

  // Calculate angles
  float roll = atan2(ay, az) * 180/PI;
  float pitch = atan2(-ax, sqrt((ay * ay) + (az * az))) * 180/PI;

  // Complementary filter
  filteredAngleX = 0.98 * (filteredAngleX + (gx * dt)) + (0.02 * roll);
  filteredAngleY = 0.98 * (filteredAngleY + (gy * dt)) + (0.02 * pitch);

  // Create a struct to hold the data
  struct {
    float humidity;
    float temp;
    float pressure;
    float angleX;
    float angleY;
  } data = {Humidity, Temperature, Pressure, filteredAngleX, filteredAngleY};

  // Send the data
  radio.write(&data, sizeof(data));

  // Print to check from transmitter end, only for testing time.
  Serial.print("\tHumidity: "); Serial.print(data.humidity);
  Serial.print("\tTemperature: "); Serial.print(data.temp); Serial.print(" °C");
  Serial.print("\tPressure: "); Serial.print(data.pressure);
  Serial.print("\tRoll: "); Serial.print(data.angleX);
  Serial.print("\tPitch: "); Serial.println(data.angleY);
}
