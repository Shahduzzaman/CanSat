#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// CE and CSN pin assignments
RF24 radio(9, 10); // Change these pins if needed

// Address for the communication channel
const byte address[6] = "00011";

void setup() {
    Serial.begin(9600);

    // Start the radio communication
    radio.begin();

    // Set the communication channel
    radio.openReadingPipe(0, address);

    // Set the power level to the minimum
    radio.setPALevel(RF24_PA_MIN);

    // Start listening to incoming messages (receiver mode)
    radio.startListening();
}

void loop() {
    if (radio.available()) {
        char text[32] = "";

        // Read the message from the buffer
        radio.read(&text, sizeof(text));

        Serial.print("Received message: ");
        Serial.println(text);
    } //else {
       // Serial.println("Radio not found");
    //}
}
