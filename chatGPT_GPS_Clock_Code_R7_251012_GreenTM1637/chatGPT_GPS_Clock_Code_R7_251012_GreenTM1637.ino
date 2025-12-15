// chatGPT_GPS_Clock_Code_R7_251012.ino
// 12 October 2025
//
// Significant change to 1 second loop in Rev 5 code -- there is no longer
// a 1 second  timer.  See comments further down in the code, in what was
// the 1-second timer loop.
//
// k6jca


#include <TM1637Display.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>

# define VERBOSE_SERIAL


// Revisions:
//
// 6 - 251006 -- Remove the 1 second timer from the gps loop and instead look for gps.time.isValid(),
//               gps.time.isUpdated, and gps.satellites.isValid() 
// 7 - 251012 -- This version Automatically adjusts for Daylight Savings Time.  It will enter this mode
//               if BOTH the nDSTPin and the nGMTPin are grounded (which should never occur if the 'time'
//               switch is connected).
//               Also, rename the nGMT and nDST signals to GMT and DST.
//
//     251024:  1.  Added some more display prompts while trying bauds
//              2.  When testing baud rate, first clear serial buffer in detectBaud()
//                  to remove data that might have been received at a different
//                  baud rate.
//
//     251028   Fix problem with auto-DST time being off by an hour if not yet receiving a valid date.
//              (i.e. don't display time until getting date with the month not equal to 0).
//              Also, streamline code for displaying of time.

int Rev = 7;  // Firmware revision


// ---------------- Pin definitions ----------------
const int nMenuPin = 7;
const int nNextPin = 8;
const int nGMTPin = 9;
const int nDSTPin = 10;
const int CLK = 2;
const int DIO = 3;
const int gpsRxPin = 6;
const int gpsTxPin = 11;

const int analogPin = A0;  // Analog input pin


// ---------------- Menu system ----------------
const int numMenuItems = 9;  // Number of menus
int currentMenuItem = 0;
int lastMenuItem = 0;

// ---------------- State tracking ----------------
bool lastMenuState = HIGH;
bool currentMenuState = HIGH;
bool lastNextState = HIGH;
bool currentNextState = HIGH;
bool GMT = false;
bool DST = false;
bool brightnessChanged = false;
bool autoDST = false;           //****for autodst
bool DSTstate = false;          //****for autodst

// ---------------- Timing ----------------
unsigned long lastSampleTime = 0;
const unsigned long sampleInterval = 40;  // in msec, for debouncing the switches
unsigned long lastGpsSample = 0;
const unsigned long gpsSampleInterval = 1000;  // 1 sec

// ---------------- Brightness Threshods (vs ADC In) ---------
// Amber:
//int hiBright = 775;    
//int midBright = 500;   
//int lowBright = 300;   
//int hyst = 25;    
// Green:
int hiBright = 500;    
int midBright = 250;   
int lowBright = 100;   
int hyst = 20;    
// Large Red:
//int hiBright = 370;    
//int midBright = 250;   
//int lowBright = 100;   
//int hyst = 15;    

     

// ---------------- Adjustable values ----------------
int Zone = 0;  // time zone delta, in hours, from GMT
int absZone = 0;
int westernmostZone = -12;
int easternmostZone = +14;
int maxBrightness = 3;  // display max brightness
int Brightness = maxBrightness;
int minBrightness = 0;
int rawAnalog = 0;  // Analog Input, referenced to 5V, 1023 max
bool Hour24 = false;
bool blink = true;           // enable or disable blinking of display's cursor
bool receivingTime = false;  // true if receiving valid time from gps
bool ambient = true;         // enable ambient light detector
long desiredBaud = 38400;
long baudDisplay = desiredBaud;
long detectedBaud = 0;
// Possible GPS baud rates (try these for detection)
const long supportedBauds[] = { 4800, 9600, 19200, 38400 };
const int numBauds = sizeof(supportedBauds) / sizeof(supportedBauds[0]);
long currentBaud = -1;


uint32_t startmS;



// ---------------- Time variables ----------------
int delayedHour = 0;
int Time = 0;
int GPSTime = 0;
bool colonState = false;
bool forceColonState = false;
int gpsHour = 0;
int gpsMinute = 0;
int gpsSecond = 0;
int gpsLastSecond = 0;
int sats = 0;  // number of satellites tracked
int gpsYear = 0;  //****for autodst
int gpsMonth = 0; //****for autodst
int gpsDay = 0;   //****for autodst
int localHour = 0;  //****for autodst
int localDay = 0;  //****for autodst
int localMonth = 0;  //****for autodst
int localYear = 0;  //****for autodst

// Flags and state
bool gotGGA = false;
char nmeaBuf[128];
int nmeaIdx = 0;
int lastSecond = -1;
int lastSats = -1;



// define "splash" screens for the TM1637
// For defining 4-letter messages
uint8_t splashRev[4] = { 0, 0, 0, 0 };
uint8_t splashText[4] = { 0, 0, 0, 0 };

// ------- 7-segment Hex codes for letters -----------
const uint8_t cblank = 0x00;
const uint8_t cdash = 0x40;
const uint8_t cunderScore = 0x08;
const uint8_t cA = 0x77;
const uint8_t cb = 0x7C;
const uint8_t cC = 0x39;
const uint8_t cc = 0x58;
const uint8_t cd = 0x5E;
const uint8_t cE = 0x79;
const uint8_t cF = 0x71;
const uint8_t cH = 0x76;
const uint8_t ci = 0x04;
const uint8_t cI = 0x30;
const uint8_t cJ = 0x1E;
const uint8_t cL = 0x38;
const uint8_t cn = 0x54;
const uint8_t co = 0x5C;
const uint8_t cO = 0x3F;
const uint8_t cP = 0x73;
const uint8_t cr = 0x50;
const uint8_t cS = 0x6D;
const uint8_t ct = 0x78;
const uint8_t cU = 0x3E;
const uint8_t cu = 0x1C;
const uint8_t cY = 0x6E;



// ---------------- Display object ----------------
TM1637Display display(CLK, DIO);


// ---------------- SoftwareSerial and GPS ----------------
SoftwareSerial gpsSerial(gpsRxPin, gpsTxPin);
TinyGPSPlus gps;

// ---------------- EEPROM Addresses ----------------
const int EEPROM_ADDR_BRIGHTNESS = 0;
const int EEPROM_ADDR_ZONE = 1;
const int EEPROM_ADDR_HOUR24 = 2;
const int EEPROM_ADDR_GPSBAUD = 3;
const int EEPROM_ADDR_BLINK = 7;
const int EEPROM_ADDR_AMBIENT = 8;


const uint8_t dash[4] = { 0x40, 0x40, 0x40, 0x40 };  // SEG_G for each digit


// ---------------- Helper functions ----------------
void writeLongToEEPROM(int addr, long value) {
  for (int i = 0; i < 4; i++) {
    EEPROM.update(addr + i, (value >> (8 * i)) & 0xFF);
  }
}

long readLongFromEEPROM(int addr) {
  long value = 0;
  for (int i = 0; i < 4; i++) {
    value |= ((long)EEPROM.read(addr + i)) << (8 * i);
  }
  return value;
}


// ---------------- Setup ----------------
void setup() {
  pinMode(nMenuPin, INPUT_PULLUP);
  pinMode(nNextPin, INPUT_PULLUP);
  pinMode(nGMTPin, INPUT_PULLUP);
  pinMode(nDSTPin, INPUT_PULLUP);

  Serial.begin(115200);  // baud rat of NANO back to PC
  Serial.println("System Started");


  // Read stored EEPROM values
  int storedBrightness = EEPROM.read(EEPROM_ADDR_BRIGHTNESS);
  if (storedBrightness >= minBrightness && storedBrightness <= 7) Brightness = storedBrightness;
  Serial.print(F("Stored Brightness: "));
  Serial.println(Brightness);

  int storedZoneRaw = EEPROM.read(EEPROM_ADDR_ZONE);
  int storedZone = (storedZoneRaw > 127) ? storedZoneRaw - 256 : storedZoneRaw;
  if (storedZone >= westernmostZone && storedZone <= easternmostZone) Zone = storedZone;
  Serial.print(F("Stored Zone: "));
  Serial.println(Zone);

  Hour24 = EEPROM.read(EEPROM_ADDR_HOUR24) != 0;
  Serial.print(F("Stored Hour24: "));
  Serial.println(Hour24 ? "24h" : "12h");

  long storedGpsBaud = readLongFromEEPROM(EEPROM_ADDR_GPSBAUD);
  if (storedGpsBaud == 4800 || storedGpsBaud == 9600 || storedGpsBaud == 19200 || storedGpsBaud == 38400 ) {
    desiredBaud = storedGpsBaud;
    Serial.print(F("Stored GPS Baud: "));
    Serial.println(desiredBaud);
  } else {
    Serial.print(F("Stored GPS Baud is garbage:"));
    Serial.println(storedGpsBaud);
  }

  blink = EEPROM.read(EEPROM_ADDR_BLINK) != 0;
  Serial.print(F("Stored Blink: "));
  Serial.println(blink ? "true" : "false");

  ambient = EEPROM.read(EEPROM_ADDR_AMBIENT) != 0;
  Serial.print(F("Stored Ambient Light Enable: "));
  Serial.println(ambient ? "true" : "false");

  display.setBrightness(Brightness);
  display.clear();


#ifndef VERBOSE_SERIAL
  {
    Serial.println("");
    Serial.println(F("Verbose Serial Mode is OFF"));
  }
#endif


  // Splash 1 
  writeText(cblank, cJ, cC, cA);
  delay(2000);                    // 251028

  int tens = (Rev % 100) / 10;
  int ones = Rev % 10;
  uint8_t digit2 = display.encodeDigit(tens);
  uint8_t digit3 = display.encodeDigit(ones);
 
  // Splash 2 (2 sec) showing revision
  writeText(cblank, cr | 0x80, digit2, digit3);
  delay(1200);

  // After splash 2, set display to all dashes
  display.setSegments(dash);
  delay(1000);

  // in autoDST mode if both DST and GMT are true (i.e. tied to gnd).
  GMT = digitalRead(nGMTPin) == LOW;  // if pin is low, set signal true //****Rev7
  DST = digitalRead(nDSTPin) == LOW; // if pin is low, set signal true  //****Rev7

  autoDST = DST & GMT;  //**** for autodst
  if (autoDST) {
    
    writeText(cA,cU,ct,cO);// write "AUtO"
    delay(1000);
    writeText(cblank,cd,cS,ct); // write " dSt"
    delay(1000);
    writeText(cdash,cO,cn,cblank); // write " On "
    delay(1000);
  } else {
    writeText(cA,cU,ct,cO);// write "AUtO"
    delay(1000);
    writeText(cblank,cd,cS,ct); // write " dSt"
    delay(1000);
    writeText(cdash,cO,cF,cF); // write " OFF"
    delay(1000);
    
  }




  // ***********************
  // Detect GPS baud rate and change receiver's rate
  // ***********************
  Serial.println(F("GPS Baud Auto-Detect & Configure"));

  // Detect current GPS baud
  for (int i = 0; i < numBauds; i++) {
    if (detectBaud(supportedBauds[i])) {
      currentBaud = supportedBauds[i];
      Serial.print(F("Detected GPS baud: "));
      Serial.println(currentBaud);
      break;
    }
  }

  if (currentBaud == -1) {
    Serial.println(F("No GPS detected at supported bauds!"));
    
    // and show on TM1637 "no SErL", as 2 sequential words.
    // First, "no"
    writeText(cdash, cdash, cn, co);
      delay(1000);
      // next, "SErL"
    writeText(cS, cE, cr, cL);
      delay(3000);

    return; // no gps found, so skip the rest of setup()
  }

  // If GPS baud not what we want → change it
  checkandUpdateBaud();

  delay(750);
}

// ---------------- Main loop ----------------
void loop() {
  unsigned long now = millis();

  // ---------------- Sample buttons at the debouncing ----------------
  if (now - lastSampleTime >= sampleInterval) {
    lastSampleTime = now;

    currentMenuState = digitalRead(nMenuPin);
    currentNextState = digitalRead(nNextPin);
    GMT = digitalRead(nGMTPin) == LOW;
    DST = digitalRead(nDSTPin) == LOW;
    autoDST = DST & GMT;  //**** for autodst

    // Menu navigation
    if (lastMenuState == LOW && currentMenuState == HIGH) {

      lastMenuItem = currentMenuItem;

      // Only update a menu item's eeprom if we are leaving that
      // menu item (e.g. desiredBaud), so that we aren't updating
      // eeprom each time we step thru each, say, the desiredBaud choice.
      if (lastMenuItem == 1) {
        EEPROM.update(EEPROM_ADDR_ZONE, (uint8_t)Zone);
        Serial.println(F("  Zone EEPROM updated"));
      }
      if (lastMenuItem == 2) {  // only update eeprom Brightness
        // if it was changed WITHIN the brightness menu, not
        // if it was changed due to automatic brightness adjust
        if (brightnessChanged == true) {
          brightnessChanged = false;
          EEPROM.update(EEPROM_ADDR_BRIGHTNESS, Brightness);
          Serial.println(F("  Brightness Level EEPROM updated"));
        }
      }
      if (lastMenuItem == 3) {
        EEPROM.update(EEPROM_ADDR_HOUR24, Hour24 ? 1 : 0);
        Serial.println(F("  Hour24 EEPROM updated"));
      }
      if (lastMenuItem == 4) {
        EEPROM.update(EEPROM_ADDR_BLINK, blink ? 1 : 0);
        Serial.println(F("  Blink Enable EEPROM updated"));
      }
      if (lastMenuItem == 5) {
        writeLongToEEPROM(EEPROM_ADDR_GPSBAUD, desiredBaud);
        Serial.println(F("  GPS Baud EEPROM updated"));
        checkandUpdateBaud(); // now update the gps receiver's baud rate.
      }
      if (lastMenuItem == 7) {
        EEPROM.update(EEPROM_ADDR_AMBIENT, ambient ? 1 : 0);
        Serial.println(F("  Ambient Enable EEPROM updated"));
      }
      if (lastMenuItem == 8) {
        // leaving analog input read, so turn on Colon to
        // disambiguate analog number from the time that
        // we are stepping to...
        forceColonState = true;
        display.setSegments(dash); // show dashes while we wait to get time
      }

      currentMenuItem++;  // step to next menu.

      if (currentMenuItem >= numMenuItems) currentMenuItem = 0;
      Serial.print(F("Menu changed to: "));
      Serial.println(currentMenuItem);
      drawmenu();  // so that get menus even if clk loop isn't working (due to failed gps comm)
    }

    // Menu actions -- step thru the selections in a menu
    // Note that menu items for 0 and 6 have no actions -- they simply
    // display gps info (time and satellites-tracked, respectively)
    if (lastNextState == LOW && currentNextState == HIGH) {
      switch (currentMenuItem) {
        case 1:  // Zone
          Zone--;
          if (Zone < westernmostZone) Zone = easternmostZone;  // cycle from -12 to +14 (yes, 26 zones!)
          Serial.print(F("Zone = "));
          Serial.println(Zone);
          break;
        case 2:  // Brightness
          Brightness--;
          brightnessChanged = true;  // brightness changed within menu.
          if (Brightness < minBrightness) Brightness = maxBrightness;
          Serial.print(F("Brightness = "));
          Serial.println(Brightness);
          break;
        case 3:  // Hour24
          Hour24 = !Hour24;
          Serial.print(F("Hour24 = "));
          Serial.println(Hour24 ? "24h" : "12h");
          break;
        case 4:  // Blink
          blink = !blink;
          Serial.print(F("Blink = "));
          Serial.println(blink ? "ON" : "OFF");
          break;
        case 5:  // GPS Baud
          switch (desiredBaud) {
            case 4800: desiredBaud = 9600; break;
            case 9600: desiredBaud = 19200; break;
            case 19200: desiredBaud = 38400; break;
            case 38400: desiredBaud = 4800; break;
            default: desiredBaud = 9600; break;
          }
          Serial.print(F("desiredBaud = "));
          Serial.println(desiredBaud);
          break;
        case 7:  // Ambient Light Detector Enable
          ambient = !ambient;
          Serial.print(F("Ambient Detector = "));
          Serial.println(ambient ? "ON" : "OFF");
          break;

        default: break;
      }
      drawmenu();  // so that get menus even if clk loop isn't working (due to failed gps comm)
    }

    lastMenuState = currentMenuState;
    lastNextState = currentNextState;
  }

  // ---------------- GPS read and display update every 1 sec ----------------

  // Read characters from GPS and look for GGA
  while (gpsSerial.available()) {
    char c = gpsSerial.read();
    gps.encode(c);

    if (nmeaIdx < (int)sizeof(nmeaBuf) - 1) nmeaBuf[nmeaIdx++] = c;
    if (c == '\n') {
      nmeaBuf[nmeaIdx] = '\0';
      if (strstr(nmeaBuf, "$GPGGA") || strstr(nmeaBuf, "$GNGGA")) {
        gotGGA = true;
      }
      // else Serial.println("   No GPGGA");  // for testing operation
      // else gotGGA = false; // chatGPT sez: don't use this
      nmeaIdx = 0;
    }
  }

  // Note that there is no 1000 ms loop for getting and parsing the gps data from the receiver.
  // Instead, we look to see if the gps.time.isUpdated flag is true (along with gps.time.isValid).
  // If it's true, then we've received time via one of two messages, per the example, below:
  //
  //  Example 1 Hz update cycle (simplified)
  //  --------------------------------------
  //
  //  Time (s) →     0.000s              0.100s           0.200s        ... → next cycle
  //                 |                   |                |
  //  NMEA sentence  $GPRMC              $GPGGA           $GPVTG ...
  //  -----------------------------------------------------------------
  //  gps.time       UPDATED (from RMC)  UPDATED (again)  unchanged
  //  gps.date       UPDATED (from RMC)  unchanged        unchanged
  //  gps.location   UPDATED (from RMC)  unchanged        unchanged
  //  gps.satellites unchanged           UPDATED (from GGA) unchanged
  //  gps.altitude   unchanged           UPDATED (from GGA) unchanged
  //  gps.speed      unchanged           unchanged        UPDATED (from VTG)
  //  gps.course     unchanged           unchanged        UPDATED (from VTG)
  //
  // From the above description, time is updated (and thus the gps.time.isUpdated() flag is true) 
  // twice in each 1-second gps message packet.  So, the following conditional statement
  // will be entered twice, and so we will see the same time, twice.
  //
  // On the other hand, gps.satellites.isUpdated() is only valid once in each message packet.
  // 
  // If we don't gate entering this code with gps.satellites.isUpdated, then we get (and display) 
  // time twice (if in Menu 0, which is the  time-display menu), but we never see any satellites,
  // maybe because that gps msg somehow gets squashed by the displaying of time twice -- unsure
  // as to cause (this doesn't seem to be true at gps baud rates slower than 38.4kbaud, e.g.
  // 9600 or 19200 -- we see satellites then, but I'd like not to limit operation to those rates).
  //
  // But if we add gps.satellites.isUpdated as a condition to entering the loop, we now only update 
  // time once a second, and we also see satellites.
  //
  // BUT will this change cause a delay in seeing time upon power up?
  //   -- Doesn't seem to.  At powerup I see both the time.isUpdated and the satellites.isUpdated
  //      flags pretty much immediately (although the satellite number == 0).
  //   -- However, at startup when satellites = 0 (& @ 38.4kbaud), we check time 4 times before seeing 
  //      satellites.isUpdated go true.  This number goes down to 3 times (sometimes 2) when the number of
  //      satellites is, say, 4 (don't know if this is true if the number of sats is 1, 2, or 3 --
  //      seems to correspond to the receiver's 1 pps light blinking).
  //
  //  Note that:
  //    o  gps.time.isUpdated() goes false when get time, e.g. do: gps.time.hours().
  //    o  gps.satellites.isUpdated() goes false after reading the number of satellites
  //       with gps.satellites.value().
  //    o  gps.satellites.isUpdated() seems to start happening pretty much immediately upon
  //       power-up (assuming satellites are in view?).  So, to prevent $GPPGA reads 
  //       from not being received by, I assume, the Nano already occupied writing to the
  //       display the time it had received via $GPRMC, delay getting time until the Nano
  //       receives a %GPPGA message.  I.e. gate getting time with gps.satellites.isValid() 
  //       being true (reading satellite value will reset this flag).
  //
  // In this section we will:
  // 1.  Parse the gps messages for time.
  // 2.  Toggle the colonState flag once a second.
  // 3.  (Test to see if we are missing seconds)
  // 4.  Calculate and format local time from UTC time.
  // 5.  If Menu '0' is selected, display time.
  // 6.  Read the analog input and adjust display brightness accordingly.
  // 7.  If not displaying time, display whichever menu has been selected.
  // 8.  If gps.satellites.isUpdated() is true, get the satellite number.

  if(gps.time.isValid() && gps.time.isUpdated() && gps.satellites.isUpdated()) { 
  
    gpsHour = gps.time.hour();
    gpsMinute = gps.time.minute();
    gpsSecond = gps.time.second();

    gpsYear = gps.date.year();  //****for autodst
    gpsMonth = gps.date.month(); //****for autodst
    gpsDay = gps.date.day(); //****for autodst

    // for testing autoDST...
    #ifdef FORCE_DATE_TIME
      {
        gpsHour = 2 - Zone; // compensate for zone (PST = -8)
        gpsMinute = 12;
        gpsSecond = 0;

        gpsYear = 2025;  // 2025
        gpsMonth = 11;    // 3 or 11 (march 9 or november 2)
        gpsDay = 2;      // 9 or 2
      }
    #endif


 
    // look for skipped seconds and flag as error to serial port
    // don't flag if second is repeated (which can happen if using gotGPA)
    if (gpsSecond == 0) {
      if ((gpsLastSecond != 59) && (gpsLastSecond != 0)) {
        Serial.print(F("Seconds error: "));
        Serial.print(gpsLastSecond);
        Serial.print(F(" "));
        Serial.println(gpsSecond);
      }
    }
    else {
      if ((gpsSecond != (gpsLastSecond + 1)) && (gpsSecond != gpsLastSecond)) {
        Serial.print(F("Seconds error: "));
        Serial.print(gpsLastSecond);
        Serial.print(F(" "));
        Serial.println(gpsSecond);
      }
    }
    gpsLastSecond = gpsSecond;
    
    GPSTime = gpsHour * 100 + gpsMinute;
    if (GPSTime != 0 && gpsMonth != 0) receivingTime = true;  // GPSTime will start out = 0 at powerup.  // 251028 add gpsMonth, too,
                                              // because auto-DST time could be wrong if date isn't being received yet.
                                              // this flag, when false,  lets us display
                                              // dashes if not getting valid time.

      localHour = gpsHour;  //****for autodst
      localDay = gpsDay;  //****for autodst
      localMonth = gpsMonth;  //****for autodst
      localYear = gpsYear;  //****for autodst

      computeLocalTime(localHour, localDay, localMonth, localYear); //****for autodst

    // Make the 1-second colon blink by
    // toggling the state of colonState
    // depending upon the seconds field
    // being even or odd.
    if (forceColonState) { 
      colonState = true;
      forceColonState = false;
    } else {
      if (gpsSecond % 2 != 0) {
        // number is odd
        colonState = true;
      } else {
        // number is even
        colonState = false;
      }
    }

    // (FOR TESTING) look for skipped seconds 
    // and flag as error to serial port.
    // don't flag if a second is repeated.
    if (gpsSecond == 0) {  // first, the wraparound case at 59 to 0 seconds.
      if ((gpsLastSecond != 59) && (gpsLastSecond != 0)) {
        Serial.print(F("Seconds error: "));
        Serial.print(gpsLastSecond);
        Serial.print(F(" "));
        Serial.println(gpsSecond);
      }
    }
    else {  // otherwise, check all other seconds...
      if ((gpsSecond != (gpsLastSecond + 1)) && (gpsSecond != gpsLastSecond)) {
        Serial.print(F("Seconds error: "));
        Serial.print(gpsLastSecond);
        Serial.print(F(" "));
        Serial.println(gpsSecond);
      }
    }
    gpsLastSecond = gpsSecond;
 

    // Time Zone, etc. calculation
    // Note that if time is formated as 12 hours, the hours 
    // will never be '00'.  Instead, they will show as '12'.
    // Also, if time is GMT or in 24 hour format, the
    //  leading zeroes are not suppressed.

    // 251028
    if (GMT && !DST) { // display 24 hour format always if GMT (and not in autoDST mode)
          delayedHour = gpsHour;
    } else { 
      delayedHour = localHour;
      if (!Hour24) { // convert 24 hours to 12 hour format (if in DST or Standard time, or in autoDST)
        if (delayedHour > 12) delayedHour -= 12;     // result is between 0 and 12.
        if (delayedHour == 0) delayedHour = 12;      // if hours are 0, display them as "12"
      }
    }

/*    // COMMENT OUT 251028
      //****for autodst...
      if (!autoDST) {  // not autoDST, so assume GMT and DST and calc time as before...
        if (GMT) {
          delayedHour = gpsHour;
        } else {
          delayedHour = localHour;
          //if (DST) {
          //  delayedHour += 1;
          //  DSTstate = true;  // added for debugging
          //}
          //else DSTstate = false; // added for debugging
          if (Hour24) {  // min possible delayedHour = 0-12 = -12, max = 23+14 = +37
            if (delayedHour < 0) delayedHour += 24;
            if (delayedHour > 24) delayedHour -= 24;
          } else {  // min possible delayedHour = 0-12 = -12, max = 23+14 = +37
            if (delayedHour < 0) delayedHour += 12;
            while (delayedHour > 12) delayedHour -= 12;  // do until result is between 0 and 12.
            if (delayedHour == 0) delayedHour = 12;      // if hours are 0, display them as "12"
          }
        }
      }
      else {  // in autodst mode, so get rid of GMT and DST stuff
        delayedHour = localHour;
        if (Hour24) {  // min possible delayedHour = 0-12 = -12, max = 23+14 = +37
          if (delayedHour < 0) delayedHour += 24;
          if (delayedHour > 24) delayedHour -= 24;
        } else {  // min possible delayedHour = 0-12 = -12, max = 23+14 = +37
          if (delayedHour < 0) delayedHour += 12;
          while (delayedHour > 12) delayedHour -= 12;  // do until result is between 0 and 12.
          if (delayedHour == 0) delayedHour = 12;
        }
      }
*/
      
      Time = delayedHour * 100 + gpsMinute;    // combine hours and minutes
    
    

    // Now that we have time, display it, but only if we aren't looking 
    // at a different menu item (i.e. currentMenuItem not equal to 0).
    if (currentMenuItem == 0) {
      if (gps.time.isValid() && receivingTime == true) {
        if ((Hour24 == true) || (GMT && !DST)) { // show leading 0 if 24 hr time or gmt.
          // Update LCD with Time and appropriate Colon state
          // do NOT suppress leading zeroes
          if (!blink == true) display.showNumberDecEx(Time, 0b01000000, true, 4, 0);
          else {
            if (colonState == false) {
              display.showNumberDecEx(Time, 0b00000000, true, 4, 0);
            } else {
              display.showNumberDecEx(Time, 0b01000000, true, 4, 0);
            }
          }
        } else {
          // Update LCD with Time and appropriate Colon state.
          // Suppress leading zeroes
          if (!blink == true) display.showNumberDecEx(Time, 0b01000000, false, 4, 0);
          else {
            if (colonState == false) {
              display.showNumberDecEx(Time, 0b00000000, false, 4, 0);
            } else {
              display.showNumberDecEx(Time, 0b01000000, false, 4, 0);
            }
          }
        }
      } else {
        if (receivingTime == false) {
          if (currentBaud < 0) {
            writeText(cE, cr, cr, cblank);
          }
          else display.setSegments(dash);  // otherwise, display 4 dashes while wait for time.
        }
      }
    }

    // Adjust Display Brightness for ambient lighting if NOT in the
    // Brightness Menu and if "ambient" is True.
    // First read analog voltage (0-1023)
    rawAnalog = analogRead(analogPin);
    if ((currentMenuItem != 2) && (ambient == true)) {
      switch (Brightness) {
        case 3:
          if (rawAnalog < (hiBright - hyst)) {
            Brightness = 2;
            display.setBrightness(Brightness);
            Serial.print(F("  New brightness: "));
            Serial.println(Brightness);
          }
          break;

        case 2:
          if (rawAnalog > (hiBright + hyst)) {
            Brightness = 3;
            display.setBrightness(Brightness);
            Serial.print(F("  New brightness: "));
            Serial.println(Brightness);
          } else if (rawAnalog < (midBright - hyst)) {
            Brightness = 1;
            display.setBrightness(Brightness);
            Serial.print(F("  New brightness: "));
            Serial.println(Brightness);
          }
          break;

        case 1:
          if (rawAnalog > (midBright + hyst)) {
            Brightness = 2;
            display.setBrightness(Brightness);
            Serial.print(F("  New brightness: "));
            Serial.println(Brightness);
          } else if (rawAnalog < (lowBright - hyst)) {
            Brightness = 0;
            display.setBrightness(Brightness);
            Serial.print(F("  New brightness: "));
            Serial.println(Brightness);
          }
          break;

        case 0:
          if (rawAnalog > (lowBright + hyst)) {
            Brightness = 1;
            display.setBrightness(Brightness);
            Serial.print(F("  New brightness: "));
            Serial.println(Brightness);
          }
          break;

        default: break;
      }
    }

    drawmenu();

    // Get number of satellites tracked
    if (gps.satellites.isUpdated()) {
      sats = gps.satellites.value();  // note that this should be the 8th field of $GPGGA
      #ifdef VERBOSE_SERIAL
        Serial.print(F("Satellites tracked: "));
        Serial.println(sats);
      #endif
    } else {
      #ifdef VERBOSE_SERIAL
        Serial.println(F("Satellite data not available."));
      #endif
    }

    #ifdef VERBOSE_SERIAL
      // Serial output
      Serial.print("  Auto DST = ");  //****for autodst
      Serial.println(autoDST);
      Serial.print("  DST State = ");  //****for autodst
      Serial.println(DSTstate);
    Serial.print("GPS Time: ");
      Serial.print(gpsHour);
      Serial.print(":");
      Serial.print(gpsMinute);
      Serial.print(":");
      Serial.println(gpsSecond);
      Serial.print("GPS Date: ");  //****for autodst
      Serial.print(gpsMonth);//****for autodst
      Serial.print("/");//****for autodst
      Serial.print(gpsDay);//****for autodst
      Serial.print("/");//****for autodst
      Serial.println(gpsYear);//****for autodst

      Serial.println("");
      Serial.print("Local Time: ");
      Serial.print(localHour);
      Serial.print(":");
      Serial.print(gpsMinute);
      Serial.print(":");
      Serial.println(gpsSecond);
      Serial.print("local Date: ");  //****for autodst
      Serial.print(localMonth);//****for autodst
      Serial.print("/");//****for autodst
      Serial.print(localDay);//****for autodst
      Serial.print("/");//****for autodst
      Serial.println(localYear);//****for autodst


      //Serial.print("Delayed Hour: "); Serial.print(delayedHour); Serial.print(" | Time (HHMM): "); Serial.println(Time);
      Serial.print("Raw Analog ");
      Serial.print(": ");
      Serial.println(rawAnalog);
    #endif

  }
}


// ... from ChatGPT...
// functions to find the gps baud rate and change it, if necessary.
//-----------------------------------------------------------------
// Build and send UBX-CFG-PRT message to set GPS baud dynamically
// -----------------------------------------------------------------
void sendUBXsetBaud(long baud) {
  uint8_t msg[] = {
    0xB5, 0x62,  // sync
    0x06, 0x00,  // CFG-PRT
    0x14, 0x00,  // payload length = 20

    // payload (20 bytes)
    0x01, 0x00, 0x00, 0x00,  // portID=1, reserved, txReady=0
    0xD0, 0x08, 0x00, 0x00,  // mode = 0x08D0 (8N1), little endian
    0x00, 0x00, 0x00, 0x00,  // baud placeholder
    0x07, 0x00,              // inProtoMask: UBX+NMEA
    0x03, 0x00,              // outProtoMask: UBX+NMEA
    0x00, 0x00,              // flags
    0x00, 0x00               // reserved
  };

  // Fill baud (little-endian) at payload[8..11]
  msg[14] = (uint8_t)(baud & 0xFF);
  msg[15] = (uint8_t)((baud >> 8) & 0xFF);
  msg[16] = (uint8_t)((baud >> 16) & 0xFF);
  msg[17] = (uint8_t)((baud >> 24) & 0xFF);

  // Fletcher checksum
  uint8_t ckA = 0, ckB = 0;
  for (int i = 2; i < sizeof(msg); i++) {
    ckA += msg[i];
    ckB += ckA;
  }

  // Send message
  for (int i = 0; i < sizeof(msg); i++) gpsSerial.write(msg[i]);
  gpsSerial.write(ckA);
  gpsSerial.write(ckB);
}

// -----------------------------------------------------------------
// Try a baud rate, return true if NMEA sentences detected
// -----------------------------------------------------------------
bool detectBaud(long baud) {
  long baudDisplay = baud / 100;

  Serial.print("   ...checking if rx's baud rate = ");
  Serial.println(baud);
  display.showNumberDecEx(baudDisplay, 0x00, false, 4, 0);

  gpsSerial.begin(baud);
  while (gpsSerial.available() > 0) {       // 251024  // first clear input buffer, otherwise might falsely detect baud rate.
    gpsSerial.read();                       // 251024
  }                                         // 251024
  unsigned long start = millis();
  String sentence = "";
  while (millis() - start < 1000) {  // 1 second timeout
    while (gpsSerial.available()) {
      char c = gpsSerial.read();
      if (c == '\n') {
        if (sentence.startsWith("$GP") || sentence.startsWith("$GN")) {
          return true;  // NMEA detected
        }
        sentence = "";
      } else {
        sentence += c;
      }
    }
  }
  return false;
}

void checkandUpdateBaud() {
  // If GPS baud not what we want, then change it
  // This should be done at the start of power up
  // (when the gps receiver's baud rate defaults to
  // 9600 baud), and whenever we step to and thru
  // the gps baud rate menu).
  uint8_t segData[4] = { 0, 0, 0, 0 };
  if (currentBaud != desiredBaud) {
    Serial.print(F("Attempting to change GPS baud to "));
    Serial.println(desiredBaud);
    //display.setSegments(dash);    // 251024
    writeText(cS, cE, ct, cblank);  // 251024  write "Set Baud"
    delay(1000);                    // 251024
    writeText(cb, cA, cU, cd);      // 251024
    delay(1000);                    // 251024

    bool success = false;
    int attempt = 1;
    //for (int attempt = 1; attempt <= 3; attempt++) {
    while (!success) {
      Serial.print(F("  Attempt "));
      Serial.println(attempt);
      segData[3] = display.encodeDigit(attempt);  // 251024  write "try" (attempt number)
      writeText(ct, cr, cY, segData[3]);          // 251024

      attempt++;

      gpsSerial.begin(currentBaud);  // talk at old baud
      sendUBXsetBaud(desiredBaud);

      delay(500);  // let GPS switch
      //gpsSerial.begin(desiredBaud);  // 251024  Doing this in detectBaud()
      if (detectBaud(desiredBaud)) {
        Serial.println(F("  ✔ Baud change verified OK!"));
        writeText(cdash, cdash, cdash, cdash);          // 251024

        delay(500); 
        success = true;
        currentBaud = desiredBaud;
        break;
      } else {
        if (attempt > 4) {
          Serial.println(F("  ✘ Verification failed"));

          // and show on TM1637 "UPd- FAIL", (i.e. UPdate FAILED, as 2 sequential words.
          // First, "UPd-"
          writeText(cU, cP, cd, cdash);
          delay(1000);
          // next, "FAIL"
          writeText(cF, cA, cI, cL);
          delay(3000);
          break;

        }
      }
    }

    if (!success) {
      Serial.println(F("❌ Could not change GPS baud."));

      // and show on TM1637 "bAUd FAIL", as 2 sequential words.
      // First, "bAUd"
      writeText(cb, cA, cU, cd);
      delay(1000);
      // next, "FAIL"
      writeText(cF, cA, cI, cL);
      delay(3000);
      return;
    }
  } else {
    Serial.println(F("GPS already at desired baud."));
  }  
}

void drawmenu() {
  uint8_t segData[4] = { 0, 0, 0, 0 };
  // ---------------- Display output for menus ----------------
  switch (currentMenuItem) {
    case 0:  // GPS Time
      break;

    case 1:  // Zone
      // First character is a 't' (using seven segments)
      // Then display the zone offset from GMT as a negative number,
      // WITHOUT Daily-Time adjustment (i.e. Standard Time only)
      if (Zone < 0) {  // display a negative number
        absZone = -Zone;
        segData[1] = 0x40;  // dash
      } else {              // display a positive number
        absZone = Zone;
        segData[1] = 0x0;  // blank
      }
      segData[2] = display.encodeDigit(absZone / 10);
      segData[3] = display.encodeDigit(absZone % 10);
      writeText(ct, segData[1], segData[2], segData[3]);
      break;

    case 2:  // Brightness
      // Display TM1637 display brightness value.
      // First two characters are "br", followed by colon
      // then brightness value.
      display.setBrightness(Brightness);

      segData[2] = display.encodeDigit(Brightness / 10);
      segData[3] = display.encodeDigit(Brightness % 10);
      writeText(cb, cr | 0x80, segData[2], segData[3]);
      break;

    case 3:  // Hour24
      // First two characters are "Hr" followed by the colon.
      // last two characters are either 24 or 12.
      segData[2] = display.encodeDigit(Hour24 ? 2 : 1);
      segData[3] = display.encodeDigit(Hour24 ? 4 : 2);
      writeText(cH, cr | 0x80, segData[2], segData[3]);
      break;

    case 4:  // Blink
      // First two characters are "bL" followed by the colon.
      // last two characters are either "On" for ON or "Of" for OFF
      if (blink) {
        writeText(cb, cL | 0x80, cO, cn);
      } else {
        writeText(cb, cL | 0x80, cO, cF);
      }
      break;

    case 5:  // GPS Baud
      // Displays gps baud rate / 100.
      baudDisplay = desiredBaud / 100;
      display.showNumberDecEx(baudDisplay, 0x00, false, 4, 0);
      break;

    case 6:
      // Displays the number of satellites tracked.
      // first characters are "SA" plus the colon
      // The second two characters are the number of satellites tracked
      // (from the eighth field of the $GPGGA message).
      // If no satellite data is available, a "-" is shown.
      if (gps.satellites.isValid()) {
        if (sats < 10) segData[2] = 0;  // blank first digit if 0
        else segData[2] = display.encodeDigit(sats / 10);
        segData[3] = display.encodeDigit(sats % 10);
        writeText(cS, cA | 0x80, segData[2], segData[3]);
      } else {
        writeText(cS, cA | 0x80, cblank, cdash);
      }
      break;

    case 7:  // First two characters are "AL" followed by the colon.
      // last two characters are either "On" for ON or "Of" for OFF
      if (ambient) {
        writeText(cA, cL | 0x80, cO, cn);
      } else {
        writeText(cA, cL | 0x80, cO, cF);
      }
      break;


    case 8:
      // display the raw analog value (referenced to 5V, 1023 max) on
      // the defined analog pin.
      display.showNumberDecEx(rawAnalog);
      break;

    default: break;
  }  
}

// ---------------------- DST & DATE FUNCTIONS ----------------------
// From chatGPT.  Added for Rev 7 -- 251010

// Check for leap year
bool isLeapYear(int year) {
  return ((year % 4 == 0) && (year % 100 != 0)) || (year % 400 == 0);
}

// Days in a given month
int daysInMonth(int month, int year) {
  static const int dim[] = { 31,28,31,30,31,30,31,31,30,31,30,31 };
  if (month == 2 && isLeapYear(year)) return 29;
  return dim[month - 1];
}

// Compute nth Sunday of a month (e.g., 2nd Sunday in March)
int nthSunday(int year, int month, int nth) {
  int q = 1, m = month, Y = year;
  if (m == 1 || m == 2) { m += 12; Y -= 1; }
  int K = Y % 100, J = Y / 100;
  int h = (q + (13 * (m + 1)) / 5 + K + K / 4 + J / 4 + 5 * J) % 7;
  int dow = ((h + 6) % 7); // 0=Sunday
  int day = 1 + ((7 - dow) % 7) + (nth - 1) * 7;
  return day;
}

// Determine if a given date/time is during U.S. DST
// Note:
// Original chatGPT code had:
//     if (month == 11) return (day < dstEndDay) || (day == dstEndDay && hour < 2);
// But this caused time in November not to fall back until 3 AM, instead of 2 AM,
// because chatGPT wasn't applying the DST 1 hour offset when calculating hour.
// Instead, it was doing this calculation *before* taking into account the DST offset.
// 
// But we can compensate for this by simply changing the statement: 
//     if (month == 11) return (day < dstEndDay) || (day == dstEndDay && hour < 2);
// to:
//     if (month == 11) return (day < dstEndDay) || (day == dstEndDay && hour < 1);
bool isDST(int year, int month, int day, int hour) {
  int dstStartDay = nthSunday(year, 3, 2);   // 2nd Sunday in March
  int dstEndDay   = nthSunday(year, 11, 1);  // 1st Sunday in November

  if (month < 3 || month > 11) return false;
  if (month > 3 && month < 11) return true;
  if (month == 3) return (day > dstStartDay) || (day == dstStartDay && hour >= 2);
  if (month == 11) return (day < dstEndDay) || (day == dstEndDay && hour < 1);     //**** was 2, but need to account for DST
  return false;
}

// Compute local time adjusted for UTC offset and optional DST
void computeLocalTime(int &hour, int &day, int &month, int &year) {
  hour += Zone;
  // Handle day rollovers
  if (hour < 0) {
    hour += 24;
    day -= 1;
    if (day < 1) {
      month -= 1;
      if (month < 1) { month = 12; year -= 1; }
      day = daysInMonth(month, year);
    }
  } else if (hour >= 24) {
    hour -= 24;
    day += 1;
    int dim = daysInMonth(month, year);
    if (day > dim) {
      day = 1;
      month += 1;
      if (month > 12) { month = 1; year += 1; }
    }
  }

  // Apply DST only if enabled
  if ((autoDST && isDST(year, month, day, hour)) || (DST && !GMT)) { //**** AND isDAT() with autoDST
    DSTstate = true;
    hour += 1;
    if (hour >= 24) {
      hour -= 24;
      day += 1;
      int dim = daysInMonth(month, year);
      if (day > dim) {
        day = 1;
        month += 1;
        if (month > 12) { month = 1; year += 1; }
      }
    }
  }
  else DSTstate = false;
}

// write text to TM1637 display...
void writeText(uint8_t a, uint8_t b, uint8_t c, uint8_t d) 
{
  uint8_t splashText[4] = {a, b, c, d};
  //splashText[0] = a;
  //splashText[1] = b;
  //splashText[2] = c;
  //splashText[3] = d;

  display.setSegments(splashText);
}
