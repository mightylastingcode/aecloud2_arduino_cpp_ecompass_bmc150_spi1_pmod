/*********************************************************************
This is the main thread for placing Arduino Sketch code to run on Renesas AE Cloud2 S5D9 board

Created on: September 17, 2018
First Released on: March 19, 2019
Author: Michael Li (michael.li@miketechuniverse.com)

An Arduino sketch is placed inside the section that is defined by two comment lines.  The section has the
example of setup() and loop() functions.   You can replace them with your example code.  It is very simple.
Most common functions like Serial and Wire functions are supported except of the SPI functions which need to
be modified to run.


The MIT License (MIT)


Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.


*********************************************************************/



#include "main_thread.h"
#include <Arduino.h>
#include <Wire.h>
#include <Wire1.h>
#include "SERIAL.h"
#include "SERIAL1.h"
#include "SPI.h"
#include "SPI1.h"
#include <stdio.h>

#define  CHIPIDADDR   0x00  // 1 byte
#define  XYZREGADDR   0x02  // 6 bytes
#define  TEMPADDR     0x08  // 1 bytes

#define  SPIWRITEMODE 0x80
#define  SPIREADMODE  0x00

//====================== Your Arduino Example Sketch Begin ===========//
SERIAL1 Serial  = SERIAL1();   //UART 1
SPI1    SPI = SPI1();   // SPI 1 port

uint16_t convert_acc_value (uint8_t msb, uint8_t lsb);

void setup() {
    Serial.begin(9600);
    //while(!Serial);
    Serial.println("begin uart1...");
    SPI.begin(SPI1_SSL0);
}

void loop() {
    char data[10], buf[40];
    char cmd[2];   // only the first byte is used.
    volatile uint16_t  acc_x, acc_y, acc_z;
    double  temp;

    // read chip ID
    cmd[0] = (char)(SPIWRITEMODE | CHIPIDADDR);
    SPI.readwrite_transfer(cmd, data, 1);
    Serial.print("Chip ID = ");
    Serial.print(data[0] & 0x00FF,HEX);
    Serial.println(" HEX");

    // read xyz acclerometer's values
    cmd[0] = (char)(SPIWRITEMODE | XYZREGADDR);
    SPI.readwrite_transfer(cmd, data, 6);

    acc_x = convert_acc_value (data[1], data[0]);
    acc_y = convert_acc_value (data[3], data[2]);
    acc_z = convert_acc_value (data[5], data[4]);

    Serial.print("acc_x = ");
    Serial.print(acc_x);
    Serial.print(" acc_y = ");
    Serial.print(acc_y);
    Serial.print(" acc_z = ");
    Serial.println(acc_z);

    // read temperature value
    cmd[0] = (char)(SPIWRITEMODE | TEMPADDR);
    SPI.readwrite_transfer(cmd, data, 1);

    temp = (296.15 - (data[0] * 0.5)) - 273.15;   // Kevin to degree C
    sprintf(buf,"Temp =  %5.2f C",temp);
    Serial.println(buf);


    delay(1000);                  // waits for a second

}

//  msb<7:0> =  acc<11:4>
//  lsb<7:4> =  acc<3:0>
uint16_t convert_acc_value (uint8_t msb, uint8_t lsb) {
    uint16_t acc;

    acc = 0;
    acc = msb;      // bit 11:4
    acc = (acc << 4) & 0x0FF0;
    acc = acc | ((lsb >> 4) & 0x000F); //bit 3:0

    return acc;
}
//====================== Your Arduino Example Sketch End ===========//



//============================= Renesas Main Thread entry function ====================================//
void main_thread_entry(void)
{
   system_setup();
   setup();

    while (1) {
        loop();
        delay(1);  // required for the thread if delay()does not exist in the loop() function.
    }
}

