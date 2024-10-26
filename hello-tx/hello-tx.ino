#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// CE and CSN pin assignments
RF24 radio(7, 8); // Change these pins if needed

// Address for the communication channel
const byte address[6] = "00011";

void setup() {
    Serial.begin(9600);

    // Start the radio communication
    radio.begin();

    // Set the communication channel
    radio.openWritingPipe(address);

    // Set the power level to the minimum
    radio.setPALevel(RF24_PA_MIN);

    // Stop listening to incoming messages (transmitter mode)
    radio.stopListening();
}

void loop() {
    const char text[] = "Hello";

    // Attempt to send the message
    if (radio.write(&text, sizeof(text))) {
        Serial.println("Sent message: Hello");
    } else {
        Serial.println("Sending failed");
    }

    delay(1000);
}
