#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(9, 10); // CE = Pin 9, CSN = Pin 10
const byte address[6] = "CAR01";

void setup() {
  Serial.begin(9600);
  
  if (!radio.begin()) {
    Serial.println("NRF24 Init Failed!");
    while (1);
  }

  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_LOW);
  radio.stopListening();
  Serial.println("Nano Transmitter Ready.");
}

void loop() {
  if (Serial.available() > 0) {
    String incoming = Serial.readStringUntil('\n');
    incoming.trim();

    if (incoming.length() > 0) {
      char payload[16];
      incoming.toCharArray(payload, sizeof(payload));

      if (radio.write(&payload, sizeof(payload))) {
        Serial.print("RF ACK: ");
      } else {
        Serial.print("RF NO ACK: ");
      }
      Serial.println(payload);
    }
  }
}