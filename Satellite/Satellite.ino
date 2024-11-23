#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "DHT.h"
#include <SPI.h>
#include <Arduino.h>
#include <Wire.h>
#include <BMP180I2C.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Adafruit_BMP085.h>
Adafruit_BMP085 bmp;

#define seaLevelPressure_hPa 1014.25
#define DHTPIN 2     // Digital pin connected to the DHT sensor
#define DHTTYPE DHT11
#define I2C_ADDRESS 0x77

RF24 radio(7, 8); // CE, CSN pins
const byte address[6] = "11010";

DHT dht(DHTPIN, DHTTYPE);
BMP180I2C bmp180(I2C_ADDRESS);
MPU6050 mpu;

#define INTERRUPT_PIN 3
#define LED_PIN 13
bool blinkState = false;

bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3];
float Humidity;
float Temperature;
float Pressure;
float Roll;
float Pitch;
float Yaw;
double AltuBaro;

volatile bool mpuInterrupt = false;

void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {
  //mpu
    Wire.begin();
    Wire.setClock(400000);
    Serial.begin(9600);
    while (!Serial);

    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788);

    if (devStatus == 0) {
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    pinMode(LED_PIN, OUTPUT);
  //dht code

  Serial.println(F("DHTxx test!"));
  dht.begin();
//bmp 180
  if (!bmp180.begin())
  {
    Serial.println("begin() failed. check your BMP180 Interface and I2C Address.");
    while (1);
  }

  //reset sensor to default parameters.
  bmp180.resetToDefaults();

  //enable ultra high resolution mode for pressure measurements
  bmp180.setSamplingMode(BMP180MI::MODE_UHR);

  // nrf code  
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
    
}

void loop() {
    if (!dmpReady) return;
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        Yaw = ypr[0] * 180/M_PI;

        Pitch = ypr[1] * 180/M_PI;

        Roll = ypr[2] * 180/M_PI;

        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
      //dht
  Humidity = dht.readHumidity();
 
  // bmp code 
  Temperature = bmp180.getTemperature();
  Pressure = bmp180.readPressure() / 100.0F;

  AltuBaro = 44330 * (1- (pow((Pressure/seaLevelPressure_hPa ), (1/5.255) ) ) );
  //AltuBaro = bmp.readAltitude();
  
    // Create a struct to hold the data
  struct {
    float humidity;
    float temp;
    float pressure;
    float RateRoll;
    float RatePitch;
    float RateYaw;
    float altitudeBarometer;
  } data = {Humidity, Temperature, Pressure, Roll, Pitch, Yaw, AltuBaro};

  // Send the data
  radio.write(&data, sizeof(data));

  Serial.print("Humid.: "); Serial.print(data.humidity);
  Serial.print("  Temp.: "); Serial.print(data.temp); Serial.print(" °C");
  Serial.print("  Pres.: "); Serial.print(data.pressure); Serial.print(" Pa");
  Serial.print("  Roll = "); Serial.print(data.RateRoll); 
  Serial.print("  Pitch = "); Serial.print(data.RatePitch);
  Serial.print("  Yaw = "); Serial.print(data.RateYaw);
  Serial.print("  Altitude [M]: "); Serial.println(data.altitudeBarometer);
}
