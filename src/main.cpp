#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLEDescriptor.h>
#include <BLE2902.h>
#include <BLE2904.h>

/*  
Can almost certainly do without the Adafruit_GFX.h as will only be text

Characteristics -
see - https://github.com/nkolban/esp32-snippets/blob/master/Documentation/BLE%20C%2B%2B%20Guide.pdf

The read and writes should be done within the callbacks, but the space is very limited in the ESP32-C3 so
the execution has to be done within the loops. For reading json in from the phone I set up a buffer that 
the callback loads with its data. The loop then looks to see if there's data in the buffer and processes
it if there is. If the callback function is too big for the memory available then you get a stack overflow.

The loop() should only be used for preparing the data to send and processing any data sent by the client

https://openlabpro.com/guide/ble-notify-on-esp32-controller/

https://www.uuidgenerator.net/

https://embeddedtutorials.com/eps32/basic-esp32-adc-cpp-library/  ADC Library


Pin GPIO    Restriction       Usage
=== ======= ================= =====================
1   -       -                 BLE Aerial
2   -       -                 VCC
3   -       -                 VCC
4   GPIO0   -                 Port 1    ADC1_CH0
5   GPIO1   -                 Port 2    ADC1_CH1
6   GPIO2   High at reset     Port 3    ADC1_CH2
7   -       -                 Chip enable - high
8   GPIO3   -                 Port 4    ADC1_CH3
9   GPIO4   -                 Port 5    ADC1_CH4
10  GPIO5   JTAG default      Port 6    ADC1_MAX 
11  -       -                 VCC
12  GPIO6   JTAG default      -
13  GPIO7   JTAG default      -
14  GPIO8   High at reset     I2C     SCL
15  GPIO9   Low at boot       I2C     SDA
16  GPIO10  -                 -
17  -       -                 VCC
18  GPIO11  FLASH ONLY        Unavailable 
19  GPIO12  FLASH ONLY        Unavailable 
20  GPIO13  FLASH ONLY        Unavailable 
21  GPIO14  FLASH ONLY        Unavailable 
22  GPIO15  FLASH ONLY        Unavailable 
23  GPIO16  FLASH ONLY        Unavailable 
24  GPIO17  FLASH ONLY        Unavailable 
25  GPIO18  Programming       USB D- (white)
26  GPIO19  Programming       USB D+ (blue)
27  GPIO20  Default RxD       -
28  GPIO21  Default TxD       -
29  -       XTAL_N            Crystal
30  -       XTAL_P            Crystal
31  -       VDD               VDD
32  -       VDD               VDD
33  Base    GND               GND

*/

#include <SPI.h>
// ESP-IDF IOT Development Framework

#include <stdio.h>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "BLEUtilities.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

#define LED                 4    // Testing 
int ledDutyCycle = 255;

#define LED_CHANNEL 1

bool BLEStatus::deviceConnected = false;

uint8_t BLEPortValues::values[6] = {0, 0, 0, 0, 0, 0};

esp_adc_cal_characteristics_t adc_cal_chars;
bool adc_set_up = false;

void setup() {
  pinMode(LED, OUTPUT);
  ledcSetup(LED_CHANNEL, 1000, 10);
  ledcAttachPin(LED, LED_CHANNEL);
  ledcWrite(LED_CHANNEL, 1000);

  digitalWrite(LED, LOW);

  // Serial.begin(115200);
  Serial.begin(9600);
  Serial.println("Start blinky");

  Serial.println("setupBLE reached");
  setupBLE();
}

void ledON() {
//  Serial.println("LED ON:");
  digitalWrite(LED, LOW);
}

void ledOFF() {
//  Serial.println("LED OFF:");
  digitalWrite(LED, HIGH);
}

int delta = 10;

void loop() {
/*
  switch(BLEPortValues::values[0]) {
    case 1:

  }
*/
  ledDutyCycle = BLEPortValues::values[0] * 200;

  ledcWrite(LED_CHANNEL, ledDutyCycle);
  /*
  ledDutyCycle += delta;
  if (ledDutyCycle > 1000) {
    ledDutyCycle = 1000;
    delta *= -1;
  }
  if (ledDutyCycle < 0) {
    ledDutyCycle = 0;
    delta *= -1;
  }
  */
  delay(50);
 // ledOFF();
 // delay(200);
}


