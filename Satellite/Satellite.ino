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
float dt, Humidity, Temperature, Pressure, filteredAngleX, filteredAngleY, AltuBaro;
#define OUTPUT_READABLE_ACCELGYRO
#define LED_PIN 13
bool blinkState = false;

float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;

uint16_t dig_T1, dig_P1;
int16_t  dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5;
int16_t  dig_P6, dig_P7, dig_P8, dig_P9; 
float AltitudeBarometer, AltitudeBarometerStartUp;
//int RateCalibrationNumber;

void barometer_signals(void){
  Wire.beginTransmission(0x76);
  Wire.write(0xF7);
  Wire.endTransmission();
  Wire.requestFrom(0x76,6);
  uint32_t press_msb = Wire.read();
  uint32_t press_lsb = Wire.read();
  uint32_t press_xlsb = Wire.read();
  uint32_t temp_msb = Wire.read();
  uint32_t temp_lsb = Wire.read();
  uint32_t temp_xlsb = Wire.read();
  unsigned long int adc_P = (press_msb << 12) | (press_lsb << 4) | (press_xlsb >>4);
  unsigned long int adc_T = (temp_msb << 12) | (temp_lsb << 4) | (temp_xlsb >>4);
  signed long int var1, var2;
  var1 = ((((adc_T >> 3) - ((signed long int )dig_T1 <<1)))* ((signed long int )dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((signed long int )dig_T1)) * ((adc_T>>4) - ((signed long int )dig_T1)))>> 12) * ((signed long int )dig_T3)) >> 14;
  signed long int t_fine = var1 + var2;
  unsigned long int p;
  var1 = (((signed long int )t_fine)>>1) - (signed long int )64000;
  var2 = (((var1>>2) * (var1>>2)) >> 11) * ((signed long int )dig_P6);
  var2 = var2 + ((var1*((signed long int )dig_P5)) <<1);
  var2 = (var2>>2)+(((signed long int )dig_P4)<<16);
  var1 = (((dig_P3 * (((var1>>2)*(var1>>2)) >> 13))>>3)+((((signed long int )dig_P2) * var1)>>1))>>18;
  var1 = ((((32768+var1))*((signed long int )dig_P1)) >>15);
  if (var1 == 0) { p=0;}    
  p = (((unsigned long int )(((signed long int ) 1048576)-adc_P)-(var2>>12)))*3125;
  if(p<0x80000000){ p = (p << 1) / ((unsigned long int ) var1);}
  else { p = (p / (unsigned long int )var1) * 2;  }
  var1 = (((signed long int )dig_P9) * ((signed long int ) (((p>>3) * (p>>3))>>13)))>>12;
  var2 = (((signed long int )(p>>2)) * ((signed long int )dig_P8))>>13;
  p = (unsigned long int)((signed long int )p + ((var1 + var2+ dig_P7) >> 4));
  double pressure=(double)p/100;
  AltitudeBarometer=44330*(1-pow(pressure
     /1013.25, 1/5.255))*100;
}

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

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x76);
  Wire.write(0xF4);
  Wire.write(0x57);
  Wire.endTransmission();   
  Wire.beginTransmission(0x76);
  Wire.write(0xF5); 
  Wire.write(0x14);
  Wire.endTransmission();   
  uint8_t data[24], i=0; 
  Wire.beginTransmission(0x76);
  Wire.write(0x88);
  Wire.endTransmission();
  Wire.requestFrom(0x76,24);      
  while(Wire.available()){
    data[i] = Wire.read();
    i++;
  }
  dig_T1 = (data[1] << 8) | data[0]; 
  dig_T2 = (data[3] << 8) | data[2];
  dig_T3 = (data[5] << 8) | data[4];
  dig_P1 = (data[7] << 8) | data[6]; 
  dig_P2 = (data[9] << 8) | data[8];
  dig_P3 = (data[11]<< 8) | data[10];
  dig_P4 = (data[13]<< 8) | data[12];
  dig_P5 = (data[15]<< 8) | data[14];
  dig_P6 = (data[17]<< 8) | data[16];
  dig_P7 = (data[19]<< 8) | data[18];
  dig_P8 = (data[21]<< 8) | data[20];
  dig_P9 = (data[23]<< 8) | data[22]; delay(250);
  for (RateCalibrationNumber=0; RateCalibrationNumber<2000; RateCalibrationNumber ++) {
    barometer_signals();
    AltitudeBarometerStartUp+=AltitudeBarometer;
    delay(1);
  }
  AltitudeBarometerStartUp/=2000;

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
  Humidity = dht.readHumidity();
 
  // bmp code 
  Temperature = bmp180.getTemperature();
  Pressure = bmp180.readPressure() / 100.0F;
  
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

  barometer_signals();
  AltitudeBarometer-=AltitudeBarometerStartUp;
  AltuBaro = AltitudeBarometer;
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
    float altitudeBarometer;
  } data = {Humidity, Temperature, Pressure, filteredAngleX, filteredAngleY, RateRoll, RatePitch, RateYaw, AltuBaro};

  // Send the data
  radio.write(&data, sizeof(data));

  // Print to check from transmitter end, only for testing time.
  Serial.print("Humid.: "); Serial.print(data.humidity);
  Serial.print(" Temp.: "); Serial.print(data.temp); Serial.print(" °C");
  Serial.print(" Pres.: "); Serial.print(data.pressure);
//  Serial.print(" Roll: "); Serial.print(data.angleX);
//  Serial.print(" Pitch: "); Serial.print(data.angleY);
  Serial.print(" Roll rate [°/s]= "); Serial.print(data.RateRoll); 
  Serial.print(" Pitch Rate [°/s]= "); Serial.print(data.RatePitch);
  Serial.print(" Yaw Rate [°/s]= "); Serial.print(data.RateYaw);
  Serial.print(" Altitude [cm]: "); Serial.println(data.altitudeBarometer);
  
}
