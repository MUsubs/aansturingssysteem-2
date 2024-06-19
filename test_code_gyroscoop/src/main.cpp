#include <SPI.h>
#include <LoRa.h>

int counter = 0;

void setup() {
    Serial.begin(9600);
    while (!Serial) {
        // wait for serial port to connect. Needed for native USB port only
    }

    Serial.println("LoRa Sender");

    if (!LoRa.begin(868E6)) {
        Serial.println("Starting LoRa failed!");
        while (1);
    } else {
        Serial.println("LoRa started successfully");
    }
}

void loop() {
    Serial.print("Sending packet: ");
    Serial.println(counter);

    // begin packet
    if (LoRa.beginPacket()) {
        Serial.println("Packet begun");
    } else {
        Serial.println("Failed to begin packet");
    }

    // send packet
    LoRa.print("hello ");
    LoRa.print(counter);

    if (LoRa.endPacket()) {
        Serial.println("Packet sent successfully");
    } else {
        Serial.println("Failed to send packet");
    }

    Serial.println("Waiting for next packet...");

    counter++;

    delay(5000);
}