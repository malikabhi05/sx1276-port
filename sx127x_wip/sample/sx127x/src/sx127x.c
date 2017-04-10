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
    printf("LoRa Sender\n");

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
    init(915e6);
    //begin(914984144);

    dumpRegisters();

#if 1
    while(1) {
        printf("sending packet: %d\n", counter);

        // send packet actually
        beginPacket(false);
        //printf("packet sent: ")
        snprintf(buffer, 10, "Ping %d", counter++);
        printf("buffer: %s\n", buffer);
        write_buf(buffer, 10);
        endPacket();

        counter++;

        upm_delay_us(10000000);
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
