#include <zephyr.h>
#include <misc/printk.h>

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <stdint.h>
#include <math.h>
#include "upm.h"
#include "mraa.h"
#include "sx127x.h"
#include "upm_utilities.h"

int counter = 0;

uint8_t buffer[10] = {0};

void main() {
    printf("LoRa Receiver\n");

    // start initialization
    LoRaClass();

    // call pinset
    // ss, reset, dio0
    // 101
    //setPins(10, 3, 2);
    // c1000
    setPins(60, 82, 72);

    // complete initialization by finishing pin setup and
    // setting frequency
    //begin(915e6);
    begin(914984144);

    dumpRegisters();

    int packet_size = 0;

#if 1
    printf("Receiver\n");
    while(1) {

        // receive packet actually
        packet_size = parsePacket(0);
        if(packet_size) {
            printf("Received a packet yayy!! packet size: %d\n", packet_size);

            while(available()) {
                printf("value: %x\n", SX1276read());
            }
            printf("With RSSI: %d\n", packetRssi());
        }

        //upm_delay_us(10000000);
    }
#endif
}
/*
int counter = 0;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("LoRa Sender");

  if (!LoRa.begin(915E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
}

void loop() {
  Serial.print("Sending packet: ");
  Serial.println(counter);

  // send packet
  LoRa.beginPacket();
  LoRa.print("hello ");
  LoRa.print(counter);
  LoRa.endPacket();

  counter++;

  delay(5000);
}
*/
