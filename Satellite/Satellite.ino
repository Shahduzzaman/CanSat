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

float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;
void gyro_signals(void) {
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission(); 
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission(); 
  Wire.requestFrom(0x68,6);
  int16_t GyroX=Wire.read()<<8 | Wire.read();
  int16_t GyroY=Wire.read()<<8 | Wire.read();
  int16_t GyroZ=Wire.read()<<8 | Wire.read();
  RateRoll=(float)GyroX/65.5;
  RatePitch=(float)GyroY/65.5;
  RateYaw=(float)GyroZ/65.5;
}

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
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68); 
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  for (RateCalibrationNumber=0;
         RateCalibrationNumber<2000; 
         RateCalibrationNumber ++) {
    gyro_signals();
    RateCalibrationRoll+=RateRoll;
    RateCalibrationPitch+=RatePitch;
    RateCalibrationYaw+=RateYaw;
    delay(1);
  }
  RateCalibrationRoll/=2000;
  RateCalibrationPitch/=2000;
  RateCalibrationYaw/=2000;
  
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

  gyro_signals();
  RateRoll-=RateCalibrationRoll;
  RatePitch-=RateCalibrationPitch;
  RateYaw-=RateCalibrationYaw;

  // Create a struct to hold the data
  struct {
    float humidity;
    float temp;
    float pressure;
    float angleX;
    float angleY;
    float RateRoll;
    float RatePitch;
    float RateYaw;
  } data = {Humidity, Temperature, Pressure, filteredAngleX, filteredAngleY, RateRoll, RatePitch, RateYaw};

  // Send the data
  radio.write(&data, sizeof(data));

  // Print to check from transmitter end, only for testing time.
  Serial.print("Humidity: "); Serial.print(data.humidity);
  Serial.print(" Temperature: "); Serial.print(data.temp); Serial.print(" °C");
  Serial.print(" Pressure: "); Serial.print(data.pressure);
  //Serial.print(" Roll: "); Serial.print(data.angleX);
  //Serial.print(" Pitch: "); Serial.println(data.angleY);
  Serial.print(" Roll rate [°/s]= "); Serial.print(data.RateRoll); 
  Serial.print(" Pitch Rate [°/s]= "); Serial.print(data.RatePitch);
  Serial.print(" Yaw Rate [°/s]= "); Serial.println(data.RateYaw);
}
