#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(7, 8); // CE, CSN pins
const byte address[6] = "00001";

struct SensorData {
    float humidity;
    float temp;
    float pressure;
    float angleX;
    float angleY;
    float RateRoll;
    float RatePitch;
    float RateYaw;
    float altitudeBarometer;
};

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
}

void loop() {
  if (radio.available()) {
    SensorData data;
    radio.read(&data, sizeof(data));
    Serial.print("Humid.: "); Serial.print(data.humidity);
    Serial.print(" Temp.: "); Serial.print(data.temp); Serial.print(" °C");
    Serial.print(" Pres.: "); Serial.print(data.pressure);
    //Serial.print(" Roll: "); Serial.print(data.angleX);
    //Serial.print(" Pitch: "); Serial.println(data.angleY);
    Serial.print(" Roll rate [°/s]= "); Serial.print(data.RateRoll); 
    Serial.print(" Pitch Rate [°/s]= "); Serial.print(data.RatePitch);
    Serial.print(" Yaw Rate [°/s]= "); Serial.print(data.RateYaw);
    Serial.print(" Altitude [m]: "); Serial.println(data.altitudeBarometer/100);
    
  }
}
