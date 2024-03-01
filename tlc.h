/***************************************************************************
 *                                                                         *
     Mini library for particular Christmas lights
     project based on TLC59731
     (c) 2015 Jan Prihoda
     https://github.com/nathanjshaffer/TLC59731_Arduino/blob/master/tlc.cpp
 *                                                                         *
 *                                                                         *
 ***************************************************************************
 *                                                                         *
   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU License.
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU License V2 for more details.
 *                                                                         *
 ***************************************************************************/

#include "Arduino.h"

#ifndef tlc_H

#define tlc_H

#define auto_pad

#ifndef auto_pad

#define start_bit_pad 0
#define high_bit_pad 1
#define low_bit_pad 7
#define latch_length 160
#define eos_length 70

#endif

class tlc
{
  public:
  tlc(int pin = 8){pin_out = pin;}
  
  #ifdef auto_pad

  long start_bit_pad = 0;
  long low_bit_pad = 0;
  long high_bit_pad = 0;
  long latch_length = 0;
  long eos_length = 0;
  
  void configure(long freq)
    {
      long msecs = 1000000 / freq;
      unsigned long startM, stopM;
      
      startM = micros();
      startBit(1);startBit(1);startBit(1);startBit(1);
      stopM = micros();
      unsigned long start_bit_min = (stopM - startM)/4;
      
      startM = micros();
      writeBit(0);writeBit(0);writeBit(0);writeBit(0);
      stopM = micros();
      unsigned long low_bit_min = (stopM - startM)/4;
      
      startM = micros();
      writeBit(1);writeBit(1);writeBit(1);writeBit(1);
      stopM = micros();
      unsigned long high_bit_min = (stopM - startM)/4;
      
      if((start_bit_min * 2) +2 > msecs)
      {
        msecs = (start_bit_min * 2) + 2;
      }
      
      start_bit_pad = (msecs/2 - 1) - start_bit_min;
      high_bit_pad = msecs - (start_bit_pad + high_bit_min);
      low_bit_pad = msecs - (start_bit_pad + low_bit_min);
      latch_length = msecs * 8;
      eos_length = msecs * 3.5;
    }
    void startBit(int data)
    {
      digitalWrite(pin_out, HIGH);
      digitalWrite(pin_out, LOW);
      if(data){}
    }
    void printSettings()
    {
      while (!Serial) {}
      delay(10);
      
      Serial.println("settings:");
      Serial.print("#define start_bit_pad ");Serial.println(start_bit_pad);
      Serial.print("#define high_bit_pad ");Serial.println(high_bit_pad);
      Serial.print("#define low_bit_pad ");Serial.println(low_bit_pad);
      Serial.print("#define latch_length ");Serial.println(latch_length);
      Serial.print("#define eos_length ");Serial.println(eos_length);
    }
   
#endif

    int pin_out;
    void gsDataLatchSequence();
    void EOS();
    void writeBit(uint8_t data);
    void writeByte(uint8_t data);
    void writeLed(uint8_t r, uint8_t g, uint8_t b, bool latch);
};

#endif
