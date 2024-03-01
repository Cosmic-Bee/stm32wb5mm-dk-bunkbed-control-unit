/***************************************************************************
 *                                                                         *
 *   Arduino Library for the TLC59731                                      *
 *   (c) 2018 Nathan Shaffer      
 *   https://github.com/nathanjshaffer/TLC59731_Arduino/blob/master/tlc.cpp                                         *
 *                                                                         *
 *                                                                         *
 ***************************************************************************
 *                                                                         * 
 * This program is free software; you can redistribute it and/or modify    *
 * it under the terms of the GNU License.                                  *
 * This program is distributed in the hope that it will be useful,         *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of          *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the           *
 * GNU License V2 for more details.                                        *
 *                                                                         *
 ***************************************************************************/

#include "Arduino.h"
#include "tlc.h"
//#include <math.h>

#ifdef auto_pad

#endif

void tlc::writeBit(uint8_t data)
{
  
  digitalWrite(pin_out, HIGH);
  digitalWrite(pin_out, LOW);

  if(data)
  { 
    digitalWrite(pin_out, HIGH);
    digitalWrite(pin_out, LOW);
    delayMicroseconds(high_bit_pad);
  }
  else
  {
    delayMicroseconds(low_bit_pad);
  }/**/
}
void tlc::writeByte(uint8_t data)
{
  writeBit(data & (1<<7));
  writeBit(data & (1<<6));
  writeBit(data & (1<<5));
  writeBit(data & (1<<4));
  writeBit(data & (1<<3));
  writeBit(data & (1<<2));
  writeBit(data & (1<<1));
  writeBit(data & (1<<0));
}

void tlc::EOS()
{
  delayMicroseconds(eos_length );
}
void tlc::gsDataLatchSequence()
{
  delayMicroseconds(latch_length );
}

void tlc::writeLed(uint8_t r, uint8_t g, uint8_t b, bool latch = true)
{
  writeByte(B00111010);
  writeByte(r);
  writeByte(g);
  writeByte(b);
  
  if(latch)
  {
    gsDataLatchSequence();
  }
  else
  {
    EOS();
  }
}
