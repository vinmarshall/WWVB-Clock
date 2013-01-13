/* WWVB Clock v 1.0
 * 
 * A WWVB (NIST Time Code Broadcast) receiving clock powered by an 
 * Arduino (ATMega328), a DS1307 Real Time Clock, and a C-Max 
 * C-Max CMMR-6P-60 WWVB receiver module.
 *
 * This code builds on Capt.Tagon's code at duinolab.blogspot.com
 * which was a great reference.  Specifically, I used almost without change
 * his struct / unsigned long long overlay for the received WWVB Buffer, 
 * which apparently in turn draws on DCF77 decoding code by Rudi Niemeijer,
 * Mathias Dalheimer, and "Captain."  
 *
 * The RTC (DS1307) interface code builds in part on code by 
 * Jon McPhalen (www.jonmcphalen.com)
 *
 * There you have it.
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
 * Version 1.0
 * Notes: 
 *  - No timezone support in this version.  UTC only.
 *  - No explicit leapsecond (3 frame marker bits) support in this version.
 *  - 24 hour mode only
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
#include <LiquidCrystal.h>
#include <Wire.h>

// 
// Initializations

// Debugging mode - prints debugging information on the Serial port.
boolean debug = true;

// Front Panel Light
int lightPin = 13;

// WWVB Receiver Pin
int wwvbInPin = 2;

// LCD init 
LiquidCrystal lcd(12, 11, 6, 5, 4, 3);

// RTC init
// RTC uses pins Analog4 = SDA, Analog5 = SCL

// RTC I2C Slave Address
#define DS1307 0xD0 >> 1

// RTC Memory Registers
#define RTC_SECS        0
#define RTC_MINS        1
#define RTC_HRS         2
#define RTC_DAY         3
#define RTC_DATE        4
#define RTC_MONTH       5
#define RTC_YEAR        6
#define RTC_SQW         7

// Month abbreviations
char *months[12] = {
  "Jan",
  "Feb",
  "Mar",
  "Apr", 
  "May",
  "Jun",
  "Jul",
  "Aug",
  "Sep",
  "Oct",
  "Nov",
  "Dec" 
};

// Day of Year to month translation (thanks to Capt.Tagon)
// End of Month - to calculate Month and Day from Day of Year 
int eomYear[14][2] = {
  {0,0},      // Begin
  {31,31},    // Jan
  {59,60},    // Feb
  {90,91},    // Mar
  {120,121},  // Apr
  {151,152},  // May
  {181,182},  // Jun
  {212,213},  // Jul
  {243,244},  // Aug
  {273,274},  // Sep
  {304,305},  // Oct
  {334,335},  // Nov
  {365,366},  // Dec
  {366,367}   // overflow
};


//
// Globals

// Timing and error recording 
unsigned long pulseStartTime = 0;
unsigned long pulseEndTime = 0;
unsigned long frameEndTime = 0;
unsigned long lastRtcUpdateTime = 0;
boolean bitReceived = false;
boolean wasMark = false;
int framePosition = 0;
int bitPosition = 1;
char lastTimeUpdate[17];
char lastBit = ' ';
int errors[10] = { 1,1,1,1,1,1,1,1,1,1 };
int errIdx = 0;
int bitError = 0;
boolean frameError = false;

// RTC clock variables
byte second = 0x00;
byte minute = 0x00;
byte hour = 0x00;
byte day = 0x00;
byte date = 0x01;
byte month = 0x01;
byte year = 0x00;
byte ctrl = 0x00;

// WWVB time variables
byte wwvb_hour = 0;
byte wwvb_minute = 0;
byte wwvb_day = 0;
byte wwvb_year = 0;


/* WWVB time format struct - acts as an overlay on wwvbRxBuffer to extract time/date data.
 * This points to a 64 bit buffer wwvbRxBuffer that the bits get inserted into as the
 * incoming data stream is received.  (Thanks to Capt.Tagon @ duinolab.blogspot.com)
 */
struct wwvbBuffer {
  unsigned long long U12       :4;  // no value, empty four bits only 60 of 64 bits used
  unsigned long long Frame     :2;  // framing
  unsigned long long Dst       :2;  // dst flags
  unsigned long long Leapsec   :1;  // leapsecond
  unsigned long long Leapyear  :1;  // leapyear
  unsigned long long U11       :1;  // no value
  unsigned long long YearOne   :4;  // year (5 -> 2005)
  unsigned long long U10       :1;  // no value
  unsigned long long YearTen   :4;  // year (5 -> 2005)
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
};

struct wwvbBuffer * wwvbFrame;
unsigned long long receiveBuffer;
unsigned long long lastFrameBuffer;


/*
 * setup()
 *
 * uC Initialization
 */

void setup() {

  // Debugging information prints to the serial line
  if (debug) { Serial.begin(9600); }
  if (debug) { Serial.println("Ready."); }

  // Initialize the I2C Two Wire Communication for the RTC
  Wire.begin();

  // Set up the LCD's number of rows and columns: 
  lcd.begin(16, 2);
  
  // Setup the light pin
  pinMode(lightPin, OUTPUT);
  digitalWrite(lightPin, HIGH);

  // Print a message to the LCD.
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("WWVB Clock v 1.0");
  lcd.setCursor(0, 1);
  lcd.print("PopSci Apr. 2010"); 
  delay(5000);

  // Front panel light lights for 10 seconds on a successful frame rcpt.
  digitalWrite(lightPin, LOW);
  
  // Setup the WWVB Signal In Handling
  pinMode(wwvbInPin, INPUT);
  attachInterrupt(0, wwvbChange, CHANGE);

  // Setup the WWVB Buffer
  lastFrameBuffer = 0;
  receiveBuffer = 0;
  wwvbFrame = (struct wwvbBuffer *) &lastFrameBuffer;

}


/*
 * loop()
 *
 * Main program loop
 */

void loop() {

  // If we've received another bit, process it
  if (bitReceived == true) {
    processBit();
  }

  // Read from the RTC and update the display 4x per second
  if (millis() - lastRtcUpdateTime > 250) {

    // Snag the RTC time and store it locally
    getRTC();
    
    // And record the time of this last update.
    lastRtcUpdateTime = millis();

    // Update RTC if there has been a successfully received WWVB Frame
    if (frameEndTime != 0) {
      updateRTC();
      frameEndTime = 0;
    }

    // Update the display
    updateDisplay();

  }

}


/*
 * updateDisplay()
 *
 * Update the 2 line x 16 char LCD display
 */

void updateDisplay() {

  // Turn off the front panel light marking a successfully 
  // received frame after 10 seconds of being on.
  if (bcd2dec(second) >= 10) {
    digitalWrite(lightPin, LOW);
  } 

  // Update the LCD 
  lcd.clear();

  // Update the first row
  lcd.setCursor(0,0);
  char *time = buildTimeString();
  lcd.print(time);

  // Update the second row
  // Cycle through our list of status messages
  lcd.setCursor(0,1);
  int cycle = bcd2dec(second) / 10; // This gives us 6 slots for messages
  char msg[17]; // 16 chars per line on display

  switch (cycle) {

    // Show the Date
    case 0:
    {
      sprintf(msg, "%s %0.2i 20%0.2i", 
              months[bcd2dec(month)-1], bcd2dec(date), bcd2dec(year));
      break;
    }

    // Show the WWVB signal strength based on the # of recent frame errors
    case 1:
    {
      int signal = (10 - sumFrameErrors()) / 2;
      sprintf(msg, "WWVB Signal: %i", signal);
      break;
    }

    // Show LeapYear and LeapSecond Warning bits
    case 2:
    {
      const char *leapyear = ( ((byte) wwvbFrame->Leapyear) == 1)?"Yes":"No";
      const char *leapsec  = ( ((byte) wwvbFrame->Leapsec) == 1)?"Yes":"No";
      sprintf(msg, "LY: %s  LS: %s", leapyear, leapsec);
      break;
    }

    // Show our Daylight Savings Time status
    case 3: 
    {  
      switch((byte)wwvbFrame->Dst) {
        case 0:
	  sprintf(msg, "DST: No");
          break;
        case 1: 
          sprintf(msg, "DST: Ending");
          break;
        case 2: 
          sprintf(msg, "DST: Starting");
          break;
        case 3: 
          sprintf(msg, "DST: Yes");
          break;
      }
      break;
    }

    // Show the UT1 correction sign and amount
    case 4:
    {
      char sign;
      if ((byte)wwvbFrame->OffSign == 2) {
	sign = '-';
      } else if ((byte)wwvbFrame->OffSign == 5) {
	sign = '+';
      } else {
	sign = '?';
      } 
      sprintf(msg, "UT1 %c 0.%i", sign, (byte) wwvbFrame->OffVal);
      break;
    }

    // Show the time and date of the last successfully received 
    // wwvb frame
    case 5:
    { 
      sprintf(msg, "[%s]", lastTimeUpdate);
      break; 
    }
  }
  
  lcd.print(msg);

}


/*
 * buildTimeString
 *
 * Prepare the string for displaying the time on line 1 of the LCD
 */

char* buildTimeString() {
  char rv[255];
  sprintf(rv,"%0.2i:%0.2i:%0.2i UTC   %c",
        bcd2dec(hour),
        bcd2dec(minute),
        bcd2dec(second),
        lastBit);
  return rv;
}


/*
 * getRTC
 * 
 * Read data from the DS1307 RTC over the I2C 2 wire interface.
 * Data is stored in the uC's global clock variables.
 */

void getRTC() {

  // Begin the Transmission
  Wire.beginTransmission(DS1307);

  // Point the request at the first register (seconds)
  Wire.write(RTC_SECS);

  // End the Transmission and Start Listening
  Wire.endTransmission();
  Wire.requestFrom(DS1307, 8);
  second = Wire.read();
  minute = Wire.read();
  hour = Wire.read();
  day = Wire.read();
  date = Wire.read();
  month = Wire.read();
  year = Wire.read();
  ctrl = Wire.read();
}


/*
 * setRTC
 * 
 * Set the DS1307 RTC over the I2C 2 wire interface.
 * Data is read from the uC's global clock variables.
 */

void setRTC() {

  // Begin the Transmission       
  Wire.beginTransmission(DS1307);

  // Start at the beginning
  Wire.write(RTC_SECS);

  // Send data for each register in order
  Wire.write(second);
  Wire.write(minute);
  Wire.write(hour);
  Wire.write(day);
  Wire.write(date);
  Wire.write(month);
  Wire.write(year);
  Wire.write(ctrl);

  // End the transmission
  Wire.endTransmission();
}


/*
 * updateRTC
 * 
 * Update the time stored in the RTC to match the received WWVB frame.
 */

void updateRTC() {

  // Find out how long since the frame's On Time Marker (OTM)
  // We'll need this adjustment when we set the time.
  unsigned int timeSinceFrame = millis() - frameEndTime;
  byte secondsSinceFrame = timeSinceFrame / 1000;
  if (timeSinceFrame % 1000 > 500) {
    secondsSinceFrame++;
  }

  // The OTM for a minute comes at the beginning of the frame, meaning that
  // the WWVB time we have is now 1 minute + `secondsSinceFrame` seconds old.
  incrementWwvbMinute();

  // Set up data for the RTC clock write
  second = secondsSinceFrame;
  minute = ((byte) wwvbFrame->MinTen << 4) + (byte) wwvbFrame->MinOne;
  hour = ((byte) wwvbFrame->HourTen << 4) + (byte) wwvbFrame->HourOne;
  day = 0; // we're not using day of week at this time.

  // Translate wwvb day of year into a month and a day of month
  // This routine is courtesy of Capt.Tagon
  int doy = ((byte) wwvbFrame->DayHun * 100) +
             ((byte) wwvbFrame->DayTen * 10) +
             ((byte) wwvbFrame->DayOne);

  int i = 0;
  byte isLeapyear = (byte) wwvbFrame->Leapyear;
  while ( (i < 14) && (eomYear[i][isLeapyear] < doy) ) {
    i++;
   }
  if (i>0) {
    date = dec2bcd(doy - eomYear[i-1][isLeapyear]);
    month = dec2bcd((i > 12)?1:i);
  }
 
  year = ((byte) wwvbFrame->YearTen << 4) + (byte) wwvbFrame->YearOne;

  // And write the update to the RTC
  setRTC();

  // Store the time of update for the display status line
  sprintf(lastTimeUpdate, "%0.2i:%0.2i %0.2i/%0.2i/%0.2i", 
          bcd2dec(hour), bcd2dec(minute), bcd2dec(month), 
          bcd2dec(date), bcd2dec(year));

}


/*
 * processBit()
 * 
 * Decode a received pulse.  Pulses are decoded according to the 
 * length of time the pulse was in the low state.
 */

void processBit() {

  // Clear the bitReceived flag, as we're processing that bit.
  bitReceived = false;

  // determine the width of the received pulse
  unsigned int pulseWidth = pulseEndTime - pulseStartTime;

  // Attempt to decode the pulse into an Unweighted bit (0), 
  // a Weighted bit (1), or a Frame marker.

  // Pulses < 0.2 sec are an error in reception.
  if (pulseWidth < 100) {
  buffer(-2);
  bitError++;
  wasMark = false;

  // 0.2 sec pulses are an Unweighted bit (0)
  } else if (pulseWidth < 400) {
    buffer(0);
    wasMark = false;

  // 0.5 sec pulses are a Weighted bit (1)
  } else if (pulseWidth < 700) {
    buffer(1);
    wasMark = false;

  // 0.8 sec pulses are a Frame Marker
  } else if (pulseWidth < 900) {
    
    // Two Frame Position markers in a row indicate an
    // end/beginning of frame marker
    if (wasMark) {

	 // For the display update 
	 lastBit = '*';
	 if (debug) { Serial.print("*"); }
 
	 // Verify that our position data jives with this being 
	 // a Frame start/end marker
	 if ( (framePosition == 6) && 
	      (bitPosition == 60)  &&
              (bitError == 0)) {

           // Process a received frame
	   frameEndTime = pulseStartTime;
           lastFrameBuffer = receiveBuffer;
	   digitalWrite(lightPin, HIGH);
           logFrameError(false);

	   if (debug) {
             debugPrintFrame();
           }

	 } else {

           frameError = true;
	 }

	 // Reset the position counters
	 framePosition = 0;
	 bitPosition = 1;
	 wasMark = false;
         bitError = 0;
	 receiveBuffer = 0;

    // Otherwise, this was just a regular frame position marker
    } else {

	 buffer(-1);
	 wasMark = true;
	 
    }

  // Pulses > 0.8 sec are an error in reception
  } else {
    buffer(-2);
    bitError++;
    wasMark = false;
  }

  // Reset everything if we have more than 60 bits in the frame.  This means
  // the frame markers went missing somewhere along the line
  if (frameError == true || bitPosition > 60) {

        // Debugging
        if (debug) { Serial.print("  Frame Error\n"); }

        // Reset all of the frame pointers and flags
        frameError = false;
        logFrameError(true);
        framePosition = 0;
        bitPosition = 1;
        wasMark = false;
        bitError = 0;
        receiveBuffer = 0;
  }

}


/*
 * logFrameError
 *
 * Log the error in the buffer that is a window on the past 10 frames. 
 */

void logFrameError(boolean err) {

  // Add a 1 to log errors to the buffer
  int add = err?1:0;
  errors[errIdx] = add;

  // and move the buffer loop-around pointer
  if (++errIdx >= 10) { 
    errIdx = 0;
  }
}


/* 
 * sumFrameErrors
 * 
 * Turn the errors in the frame error buffer into a number useful to display
 * the quality of reception of late in the status messages.  Sums the errors
 * in the frame error buffer.
 */

int sumFrameErrors() {

  // Sum all of the values in our error buffer
  int i, rv;
  for (i=0; i< 10; i++) {
    rv += errors[i];
  }

  return rv;
} 


/*
 * debugPrintFrame
 * 
 * Print the decoded frame over the seriail port
 */

void debugPrintFrame() {

  char time[255];
  byte minTen = (byte) wwvbFrame->MinTen;
  byte minOne = (byte) wwvbFrame->MinOne;
  byte hourTen = (byte) wwvbFrame->HourTen;
  byte hourOne = (byte) wwvbFrame->HourOne;
  byte dayHun = (byte) wwvbFrame->DayHun;
  byte dayTen = (byte) wwvbFrame->DayTen;
  byte dayOne = (byte) wwvbFrame->DayOne;
  byte yearOne = (byte) wwvbFrame->YearOne;
  byte yearTen = (byte) wwvbFrame->YearTen;

  byte wwvb_minute = (10 * minTen) + minOne;
  byte wwvb_hour = (10 * hourTen) + hourOne;
  byte wwvb_day = (100 * dayHun) + (10 * dayTen) + dayOne;
  byte wwvb_year = (10 * yearTen) + yearOne;	

  sprintf(time, "\nFrame Decoded: %0.2i:%0.2i  %0.3i  20%0.2i\n", 
          wwvb_hour, wwvb_minute, wwvb_day, wwvb_year);
  Serial.print(time);

}


/*
 * buffer
 *
 * Places the received bits in the receive buffer in the order they
 * were recived.  The first received bit goes in the highest order 
 * bit of the receive buffer.
 */

void buffer(int bit) {
  
  // Process our bits 
  if (bit == 0) {
    lastBit = '0';
    if (debug) { Serial.print("0"); }

  } else if (bit == 1) {
    lastBit = '1';
    if (debug) { Serial.print("1"); }

  } else if (bit == -1) {
    lastBit = 'M';
    if (debug) { Serial.print("M"); }
    framePosition++;

  } else if (bit == -2) {
    lastBit = 'X';
    if (debug) { Serial.print("X"); }
  }

  // Push the bit into the buffer.  The 0s and 1s are the only
  // ones we care about.  
  if (bit < 0) { bit = 0; }
  receiveBuffer = receiveBuffer | ( (unsigned long long) bit << (64 - bitPosition) );

  // And increment the counters that keep track of where we are
  // in the frame.
  bitPosition++;
}


/*
 * incrementWwvbMinute
 *
 * The frame On Time Marker occurs at the beginning of the frame.  This means
 * that the time in the frame is one minute old by the time it has been fully
 * received.  Adding one to the minute can be somewhat complicated, in as much 
 * as it can roll over the successive hours, days, and years in just the right 
 * circumstances.  This handles that.
 */

void incrementWwvbMinute() {
  
  // Increment the Time and Date
  if (++(wwvbFrame->MinOne) == 10) {
	  wwvbFrame->MinOne = 0;
	  wwvbFrame->MinTen++;
  }

  if (wwvbFrame->MinTen == 6) {
	  wwvbFrame->MinTen = 0;
	  wwvbFrame->HourOne++;
  }

  if (wwvbFrame->HourOne == 10) {
	  wwvbFrame->HourOne = 0;
	  wwvbFrame->HourTen++;
  }
  
  if ( (wwvbFrame->HourTen == 2) && (wwvbFrame->HourOne == 4) ) {
	  wwvbFrame->HourTen = 0;
	  wwvbFrame->HourOne = 0;
	  wwvbFrame->DayOne++;
  }

  if (wwvbFrame->DayOne == 10) {
	  wwvbFrame->DayOne = 0;
	  wwvbFrame->DayTen++;
  }

  if (wwvbFrame->DayTen == 10) {
	  wwvbFrame->DayTen = 0;
	  wwvbFrame->DayHun++;
  }

  if ( (wwvbFrame->DayHun == 3) &&
       (wwvbFrame->DayTen == 6) &&
       (wwvbFrame->DayOne == (6 + (int) wwvbFrame->Leapyear)) ) {
	   // Happy New Year.
	   wwvbFrame->DayHun = 0;
	   wwvbFrame->DayTen = 0;
	   wwvbFrame->DayOne = 1;
           wwvbFrame->YearOne++;
  }

  if (wwvbFrame->YearOne == 10) {
    wwvbFrame->YearOne = 0;
    wwvbFrame->YearTen++;
  }

  if (wwvbFrame->YearTen == 10) {
    wwvbFrame->YearTen = 0;
  }

}


/*
 * wwvbChange
 * 
 * This is the interrupt servicing routine.  Changes in the level of the 
 * received WWVB pulse are recorded here to be processed in processBit().
 */

void wwvbChange() {

  int signalLevel = digitalRead(wwvbInPin);

  // Determine if this was triggered by a rising or a falling edge
  // and record the pulse low period start and stop times
  if (signalLevel == LOW) {
    pulseStartTime = millis();
  } else {
    pulseEndTime = millis();
    bitReceived = true;
  }

}



/*
 * bcd2dec
 * 
 * Utility function to convert 2 bcd coded hex digits into an integer
 */

int bcd2dec(int bcd) {
      return ( (bcd>>4) * 10) + (bcd % 16);
}


/*
 * dec2bcd
 * 
 * Utility function to convert an integer into 2 bcd coded hex digits
 */

int dec2bcd(int dec) {
      return ( (dec/10) << 4) + (dec % 10);
} 



