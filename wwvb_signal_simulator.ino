/* WWVB Test Signal v 1.0
 *
 * This code uses an Arduino to simulate the C-Max CMMR-6P TCO positive 
 * output signal.  This was written for debugging WWVB Clock 
 * Receiver code.  
 * 
 * This code is adapted from the WWVB Receiver Simulator provided by 
 * Capt.Tagon at duinolab.blogspot.com.  It expands on that code by 
 * incrementing the time and date starting from that set in the 
 * setup() routine.  This also contains debugging code to display 
 * this value on the Serial monitor.
 * 
 * This code supports the "Atomic Clock" article in the April 2010 issue
 * of Popular Science.  There is also a schematic for this project.  There
 * is also WWVB signal simulator code, to facilitate debugging and 
 * hacking on this project when the reception of the WWVB signal 
 * itself is less than stellar.
 * 
 * The code for both the clock and the WWVB simulator, and the schematic
 * are available online at:
 * http://www.popsci.com/diy/article/2010-03/build-clock-uses-atomic-timekeeping
 * 
 * and on GitHub at: http://github.com/vinmarshall/WWVB-Clock
 *
 * 
 * Copyright (c) 2010 Vin Marshall (vlm@2552.com, www.2552.com)
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 * 
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#include <stdio.h>

int txPin = 13;

/* WWVB time format struct - acts as an overlay on wwvbRxBuffer to extract time/date data.
 * All this points to a 64 bit buffer wwvbRxBuffer that the bits get read from as the
 * sample data stream is transmitted.   (Thanks to Capt.Tagon @ duinolab.blogspot.com)
 *
 */

struct wwvbBuffer {
  unsigned long long U12       :4;  // no value, empty four bits only 60 of 64 bits used
  unsigned long long Frame     :1;  // framing
  unsigned long long Dst       :2;  // dst flags
  unsigned long long Leapsec   :1;  // leapsecond
  unsigned long long Leapyear  :1;  // leapyear
  unsigned long long U11       :1;  // no value
  unsigned long long YearOne   :4;  // year (5 -> 2005)
  unsigned long long U10       :1;  // no value
  unsigned long long YearTen   :4;  // year (5 -> 2050)
  unsigned long long U09       :1;  // no value
  unsigned long long OffVal    :4;  // offset value
  unsigned long long U08       :1;  // no value
  unsigned long long OffSign   :3;  // offset sign
  unsigned long long U07       :2;  // no value
  unsigned long long DayOne    :4;  // day ones
  unsigned long long U06       :1;  // no value
  unsigned long long DayTen    :4;  // day tens
  unsigned long long U05       :1;  // no value
  unsigned long long DayHun    :2;  // day hundreds
  unsigned long long U04       :3;  // no value
  unsigned long long HourOne   :4;  // hours ones
  unsigned long long U03       :1;  // no value
  unsigned long long HourTen   :2;  // hours tens
  unsigned long long U02       :3;  // no value
  unsigned long long MinOne    :4;  // minutes ones
  unsigned long long U01       :1;  // no value
  unsigned long long MinTen    :3;  // minutes tens
  unsigned long long U00       :1;
};

// We point the struct and the unsigned long long at the 
// same memory space so we can access the bits using
// both paradigms.
struct wwvbBuffer * buffer = (struct wwvbBuffer *) malloc(sizeof(struct wwvbBuffer));
unsigned long long * timeBits = (unsigned long long *) buffer;


/*
 * setup 
 * 
 * uC Initialization
 */

void setup() {

	// Setup the Serial port out and the WWVB signal output pin
	Serial.begin(9600);
        pinMode(txPin, OUTPUT);

        // Preset the time struct 
        *timeBits = 0x0000000000000000;
        buffer->MinTen = 5;
        buffer->MinOne = 7; 
        buffer->HourTen = 2;
        buffer->HourOne = 3;
        buffer->DayHun = 3;
        buffer->DayTen = 6;
        buffer->DayOne = 5;
        buffer->OffSign = 5;  // 2 -> -  5 -> +
        buffer->OffVal = 3;
        buffer->YearTen = 1;
        buffer->YearOne = 0;
        buffer->Dst = 3; // 0 no dst, 1 dst ending, 2 dst starting, 3 dst
        buffer->Leapyear = 0;
        buffer->Leapsec = 0;
}


/*
 * loop
 *
 * Main program loop 
 */

void loop() {

	// Print the Date & Time to the Serial port for debugging.
	int year   = (buffer->YearTen * 10) + buffer->YearOne;
        int day    = (buffer->DayHun * 100) + (buffer->DayTen * 10) + buffer->DayOne;
        int hour   = (buffer->HourTen * 10) + buffer->HourOne;
        int minute = (buffer->MinTen * 10) + buffer->MinOne;
	char date[30];
	sprintf(date, "%.2i:%.2i  %.3i, 20%.2i\n", hour, minute, day, year);
	Serial.print(date);

	// Step through each bit in this frame
        int position = 0;
	for (int i = 63; i >= 4; i--) {
		// Mask off all bits but the one in question.
		// This singles out one bit, moving from MSB to LSB
		unsigned long long mask = (unsigned long long) 1 << i;
		unsigned long long masked = (*timeBits) & mask;
		
		// Determine if there was a 1 in that bit position
		int bit = ((*timeBits) & mask)?1:0;

                // Determine if we're at a Marker position
                int mark = 0;
                if ( (position == 0) || 
                     ((position + 1) % 10 == 0) ) {
                       mark = 1;
                }
                position++;
	
		// Rattle off each bit

		// Debug output to the Serial port
		// Print in groups of 4. Makes cross ref to hex easier
		if ( (i+1) % 4 == 0) {
			Serial.println("");
		}	

                if (mark) { 
                  Serial.print("M"); 
                  sendMark();
                } else if (bit == 0) {
                  Serial.print("0");
                  sendUnweighted();
                } else if (bit == 1) {
                  Serial.print("1");
                  sendWeighted();
                }

	}
        Serial.println("");

	// Increment the Time and Date
	if (++(buffer->MinOne) == 10) {
                buffer->MinOne = 0;
		buffer->MinTen++;
	}

	if (buffer->MinTen == 6) { 
		buffer->MinTen = 0;
		buffer->HourOne++;
        }

        if (buffer->HourOne == 10) {
                buffer->HourOne = 0;
                buffer->HourTen++;
        }

        if ( (buffer->HourTen == 2) && (buffer->HourOne == 4) ) {
                buffer->HourTen = 0;
                buffer->HourOne = 0;
                buffer->DayOne++;
        }
        
        if (buffer->DayOne == 10) {
                buffer->DayOne = 0;
                buffer->DayTen++;
        }
        
        if (buffer->DayTen == 10) {
                buffer->DayTen = 0;
                buffer->DayHun++;
        }
        
        if ( (buffer->DayHun == 3) && 
             (buffer->DayTen == 6) &&
             (buffer->DayOne == (6 + (int) buffer->Leapyear)) ) {
                 // Happy New Year.
                 buffer->DayHun = 0;
                 buffer->DayTen = 0;
                 buffer->DayOne = 1;
                 buffer->YearOne++;
         }
         
         if (buffer->YearOne == 10) {
           buffer->YearOne = 0;
           buffer->YearTen++;
         }
         
         if (buffer->YearTen == 10) {
           buffer->YearTen = 0;
         }
	
}


/*
 * sendMark
 * 
 * Output a Frame / Position marker bit
 */

void sendMark() {
  
  // Send low for 0.8 sec
  digitalWrite(txPin, LOW);
  delay(800);
  
  // Send high for 0.2 sec
  digitalWrite(txPin, HIGH);
  delay(200);
   
  return;
}


/*
 * sendWeighted
 *
 * Output a Weighted bit (1)
 */

void sendWeighted() {
  
  // Send low for 0.5 sec
  digitalWrite(txPin, LOW);
  delay(500);
  
  // Send high for 0.5 sec
  digitalWrite(txPin, HIGH);
  delay(500);
  
  return;
}


/*
 * sendUnweighted
 *
 * Output an Unweighted bit (0)
 */

void sendUnweighted() {
  
  // Send low for 0.2 sec
  digitalWrite(txPin, LOW);
  delay(200);
  
  // Send high for 0.8 sec
  digitalWrite(txPin, HIGH);
  delay(800);
  
  return;
}

