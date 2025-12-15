// k6jca_LCD_GPS_Clock_Code_Rev_251008.ino
// 8 October 2025
//
// based on 'chatGPT_GPS_Clock_Code_R6_251006.ino',
// but with TM1637 4 digit display replaced with
// 4x20 LCD display with I2C adapter.
//
// See comments further down, in code.
//
// Rev. 251008 -- Fixed some bugs from Rev 251004
// Rev. 251013 -- This version Automatically adjusts for Daylight Savings Time.  It will enter this mode
//                 if BOTH the nDSTPin and the nGMTPin are grounded (which should never occur if the 'time'
//                 switch is connected).
//                 Also:
//                    o rename the nGMT and nDST signals to GMT and DST.
//                    o Add displaying GMT time (HH:MM) on menu line.
//
//
// k6jca 



#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include <Wire.h>

//# define VERBOSE_SERIAL
//# define VERBOSE_SERIAL2
//# define VERBOSE_NMEA
//# define FORCE_DATE_TIME


const long Rev = 251013L;  // revision


const uint8_t LCD_ADDR = 0x3F;
//const uint8_t LCD_ADDR = 0x27;
const int LCD_COLS = 20;
const int LCD_ROWS = 4;

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
const int numMenuItems = 8;  // Number of menus
//int currentMenuItem = 1;     // init to satellites menu
int currentMenuItem = 7;     // init to GMT on menu line
int lastMenuItem = currentMenuItem;

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
const unsigned long debounceInterval = 40;  // in msec, for debouncing the switches

// ---------------- LCD backlight stuff ---------
int lowBright = 100;   
int hyst = 10;         // add some hysterises to brightness switching
bool noBkliteIfDark = false;

// ---------------- Adjustable values ----------------
int Zone = 0;  // time zone delta, in hours, from GMT
int absZone = 0;
int westernmostZone = -12;
int easternmostZone = +14;
int rawAnalog = 0;  // Analog Input, referenced to 5V, 1023 max
bool Hour24 = false;
bool blink = true;           // enable or disable blinking of display's cursor
bool receivingTime = false;  // true if receiving valid time from gps
bool ambient = true;         // enable ambient light detector

long desiredBaud = 19200;
long baudDisplay = desiredBaud;
long detectedBaud = 0;
// Possible GPS baud rates (try these for detection)
const long supportedBauds[] = { 4800, 9600, 19200, 38400};
const int numBauds = sizeof(supportedBauds) / sizeof(supportedBauds[0]);
long currentBaud = -1;


uint32_t startmS;


// ---------------- Time variables ----------------
int delayedHour = 0;
int Time = 0;
int GPSTime = 0;
bool colonState = false;
int gpsHour = 0;
int gpsMinute = 0;
int gpsSecond = 0;
int gpsLastSecond = 0;
int sats = 0;  // number of satellites tracked
bool displaySeconds = true;
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

// ----------------- For Texting -------------------------

bool startup = true;

bool test = true;

// ---------------- SoftwareSerial and GPS ----------------
SoftwareSerial gpsSerial(gpsRxPin, gpsTxPin);
TinyGPSPlus gps;
LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS);

// ---------------- EEPROM Addresses ----------------
const int EEPROM_ADDR_SECONDS = 0;
const int EEPROM_ADDR_ZONE = 1;
const int EEPROM_ADDR_GPSBAUD = 3;
const int EEPROM_BACKLIGHT = 7;
//const int EEPROM_ADDR_AMBIENT = 8;


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

// Simple Zeller-like day-of-week (0=Sunday..6=Saturday)
// Valid for Gregorian calendar
int dayOfWeek(int day, int month, int year) {
  int d = 7; // use to print "--------" for day if date not valid
  if ((day == 0) && (month == 0)) d = 7;  // not a valid date, thus no day
  else {
    if (month < 3) { month += 12; year -= 1; }
    int K = year % 100;
    int J = year / 100;
    int h = (day + (13*(month+1))/5 + K + K/4 + J/4 + 5*J) % 7;
    d = ((h + 6) % 7); // convert to 0=Sunday
  }
  return d;
}

const char* dayText(int d) {
  switch(d) {
    case 0: return "Sunday";
    case 1: return "Monday";
    case 2: return "Tuesday";
    case 3: return "Wednesday";
    case 4: return "Thursday";
    case 5: return "Friday";
    case 6: return "Saturday";
    case 7: return "--------";  // use if no day
    default: return " Invalid ";
  }
}

String twoDigits(int v) {
  char buf[3];
  snprintf(buf, sizeof(buf), "%02d", v);
  return String(buf);
}


// Print centered or left, utils
void lcdPrintLeft(uint8_t row, const char* s) {
  lcd.setCursor(0, row);
  lcd.print(F("                    ")); // clear line (20 chars)
  lcd.setCursor(0, row);
  lcd.print(s);
}


// ---------------- Setup ----------------
void setup() {
  pinMode(nMenuPin, INPUT_PULLUP);
  pinMode(nNextPin, INPUT_PULLUP);
  pinMode(nGMTPin, INPUT_PULLUP);
  pinMode(nDSTPin, INPUT_PULLUP);

  Serial.begin(115200);  // baud rat of NANO back to PC
  Serial.println("System Started");

  // Start I2C and set speed to 400kHz 
  Wire.begin(); 
  Wire.setClock(400000); // set to fast speed (standard speed is 100 KHz)

  // Read stored EEPROM values
  displaySeconds = EEPROM.read(EEPROM_ADDR_SECONDS) != 0;
  Serial.print(F("Stored displaySeconds: "));
  Serial.println(displaySeconds ? "true" : "false");

  noBkliteIfDark = EEPROM.read(EEPROM_BACKLIGHT) != 0;
  Serial.print(F("Backlight Off if Dark: "));
  Serial.println(noBkliteIfDark ? "true" : "false");

  int storedZoneRaw = EEPROM.read(EEPROM_ADDR_ZONE);
  int storedZone = (storedZoneRaw > 127) ? storedZoneRaw - 256 : storedZoneRaw;
  if (storedZone >= westernmostZone && storedZone <= easternmostZone) Zone = storedZone;
  Serial.print(F("Stored Zone: "));
  Serial.println(Zone);
  
  long storedGpsBaud = readLongFromEEPROM(EEPROM_ADDR_GPSBAUD);
  if (storedGpsBaud == 4800 || storedGpsBaud == 9600 || storedGpsBaud == 19200 || storedGpsBaud == 38400 ) {
    desiredBaud = storedGpsBaud;
    Serial.print(F("Stored GPS Baud: "));
    Serial.println(desiredBaud);
  } else {
    Serial.print(F("Stored GPS Baud is garbage:"));
    Serial.println(storedGpsBaud);
  }


  #ifndef VERBOSE_SERIAL
    {
      Serial.println("");
      Serial.println("Verbose Serial Mode is OFF");
    }
  #endif


  Serial.println(F("GPS Clock starting..."));

  // in autoDST mode if both DST and GMT are true (i.e. tied to gnd).
  GMT = digitalRead(nGMTPin) == LOW;  // if pin is low, set signal true //****Rev7
  DST = digitalRead(nDSTPin) == LOW; // if pin is low, set signal true  //****Rev7

  autoDST = DST & GMT;  //**** for autodst

  lcd.init();
  lcd.backlight();

  // Display Rev of SW and state of Automatic Daylight Savings Time
  // (autoDST)
  lcd.setCursor(0,0);
  lcd.print(F("   K6JCA GPS CLock  "));
  lcd.setCursor(0,1);
  lcd.print(F("     Rev: "));
  lcd.print(Rev);
  lcd.setCursor(0,3);
  if (autoDST) {
    lcd.print(F("   Auto-DST is ON   "));
  } else {
    lcd.print(F("   Auto-DST is OFF  "));
  }
  delay(2500);

  // briefly clear menu line
  lcd.setCursor(0,3);
  lcd.print(F("                    "));
  delay(500);


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
      lcd.setCursor(0,3);
      lcd.print(F("     Detected:      "));
      if (currentBaud < 10000) lcd.setCursor(16,3);
      else lcd.setCursor(15,3);
      lcd.print(currentBaud);
      delay(1000);
      break;
    }
  }

  if (currentBaud == -1) {
    Serial.println(F("No GPS detected at supported bauds!"));
    
    lcd.setCursor(0,3);
    lcd.print(F("ERR: NO SERIAL      "));
    delay(3000);

    return; // no gps found, so skip the rest of setup()
  }

  // If GPS baud not what we want → change it
  checkandUpdateBaud();
  delay(1000);

  // Init to show these fields:
  lcd.setCursor(0,0);
  lcd.print(F("      --:--:--      "));
  lcd.setCursor(0,1);
  lcd.print(F("       ------       "));
  lcd.setCursor(0,2);
  lcd.print(F("     --/--/----     "));
  lcd.setCursor(0,3);
  lcd.print(F("                    "));

  delay (500);
}


// ---------------- Main loop ----------------
void loop() {
  while(test == true) { // *** For testing.  At end of loop set test to false for 1 pass thru

    unsigned long now = millis();

    // ---------------- Sample buttons at the debouncing ----------------
    if (now - lastSampleTime >= debounceInterval) {

      lastSampleTime = now;
      currentMenuState = digitalRead(nMenuPin);
      currentNextState = digitalRead(nNextPin);
      GMT = digitalRead(nGMTPin) == LOW;
      DST = digitalRead(nDSTPin) == LOW;
      autoDST = DST & GMT;  //**** 251018

      // Menu navigation
      if (lastMenuState == LOW && currentMenuState == HIGH) {  // step to next menu

        lastMenuItem = currentMenuItem;
        //lcd.setCursor(0,3);
        //lcd.print(F("                    ")); // clear menu line

        // Only update a menu item's eeprom if we are leaving that
        // menu item (e.g. desiredBaud), so that we aren't updating
        // eeprom each time we step thru each, say, the desiredBaud choice.
        // Note that Menu 0 is blank (no menu), and 
        // Menu 1 is satellites, which is not stored in eeprom
        if (lastMenuItem == 2) {
          EEPROM.update(EEPROM_ADDR_ZONE, (uint8_t)Zone);
          Serial.println(F("  Zone EEPROM updated"));
        }
        if (lastMenuItem == 3) {  // display seconds
          EEPROM.update(EEPROM_ADDR_SECONDS, displaySeconds);
          Serial.println(F("  Display Seconds EEPROM updated"));
        }
        if (lastMenuItem == 4) {
          writeLongToEEPROM(EEPROM_ADDR_GPSBAUD, desiredBaud);
          Serial.println(F("  GPS Baud EEPROM updated"));
          checkandUpdateBaud(); // now update the gps receiver's baud rate.
        }
        if (lastMenuItem == 5) {
          EEPROM.update(EEPROM_BACKLIGHT, noBkliteIfDark);
          Serial.println(F("  No Backlight if Dark EEPROM updated"));        
        }

        currentMenuItem++;  // step to next menu.

        if (currentMenuItem >= numMenuItems) currentMenuItem = 0;
        Serial.print(F("Menu changed to: "));
        Serial.println(currentMenuItem);

        drawMenuLine(); // so that draw menu even if not getting time (no 1 sec loop)
      }

      // Menu actions -- step thru the selections in a menu
      // Note that menu items for 0 (blank) and 1  (satellites) have no actions -- they simply
      // display gps info (time and satellites-tracked, respectively)
      if (lastNextState == LOW && currentNextState == HIGH) {
        switch (currentMenuItem) {
          case 2:  // Zone
            Zone--;
            if (Zone < westernmostZone) Zone = easternmostZone;  // cycle from -12 to +14 (yes, 26 zones!)
            Serial.print(F("Zone = "));
            Serial.println(Zone);
            break;
          case 3:  // Display Seconds
            if(displaySeconds == true) displaySeconds = false;
            else displaySeconds = true;
            Serial.print(F("Display Seconds= "));
            Serial.println(displaySeconds ? "ON" : "OFF");
            break;
          case 4:  // GPS Baud
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

          case 5:  // Display Seconds
            if(noBkliteIfDark == true) noBkliteIfDark = false;
            else noBkliteIfDark = true;
            Serial.print(F("No Backlight if Dark = "));
            Serial.println(displaySeconds ? "ON" : "OFF");
            break;

          default: break;
        }
        drawMenuLine();  // so that can see menu even if rcving no gps time
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
        #ifdef VERBOSE_NMEA 
          Serial.println(nmeaBuf); // for testing
        #endif
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
          gpsMonth = 3;    // 3 or 11 (march 9 or november 2)
          gpsDay = 9;      // 9 or 2
        }
      #endif



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


      #ifdef VERBOSE_SERIAL2 
        Serial.println(gps.time.isUpdated()); // debug: checking what clears flag
      #endif

      // Make the 1-second colon blink by
      // toggling the state of colonState
      // depending upon the seconds field
      // being even or odd.
      if (gpsSecond % 2 != 0) {
        // number is odd
        colonState = true;
      } else {
        // number is even
        colonState = false;
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
      
      drawTimeDateDay();

      // Get number of satellites tracked
      if (gps.satellites.isValid() && gps.satellites.isUpdated()) { // note that sats.isUpdated goes false after it is read.
          sats = gps.satellites.value();  // note that this should be the 8th field of $GPGGA
          #ifdef VERBOSE_SERIAL
                Serial.print("Satellites tracked: ");
                Serial.println(sats);
          #endif          
      }

      drawMenuLine(); 

      // Serial.println(" ... thru loop");  // for testing

      // Turn LCD backlight OFF if ambient lighting is dark
      // (maybe to extend lifetime of backlight leds?)
      rawAnalog = analogRead(analogPin);
      if (noBkliteIfDark == true) {
        if (rawAnalog < lowBright - hyst) lcd.noBacklight();
        if (rawAnalog > lowBright + hyst) lcd.backlight();
      }
      else lcd.backlight();


      #ifdef VERBOSE_SERIAL  // for testing
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
        //Serial.print("Raw Analog ");
        //Serial.print(": ");
        Serial.println(rawAnalog);
      # endif

    }      
    //test = false; // ***
  }  
}


// ... from ChatGPT...
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

  Serial.print(F("   ...checking if rx's baud rate = "));
  Serial.println(baud);
          
  lcd.setCursor(0,3);
  //lcd.print(F("                   "));
  lcd.setCursor(0,3);
  lcd.print(F("  Trying baud:      ")); 
    if (baud < 10000) lcd.setCursor(16,3);
    else lcd.setCursor(15,3);
  lcd.print(baud);
  
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

void drawMenuLine() {
  // 4th line: Menu selection and menu item within that menu
  lcd.setCursor(0,3);
  switch (currentMenuItem) {
    case 0:  // display blank line
      lcd.print(F("                    ")); // print blank line (menu 0)
    break;

    case 1: 
      lcd.print(F("    Satellites: "));
      lcd.print(sats);
      if (sats < 10) lcd.print(F(" "));
      break;
    
    case 2:   // zone
      lcd.print(F("   UTC Offset: "));
      if (Zone >= 10) lcd.print(" ");
      else if (Zone <10 && Zone >= 0) lcd.print("  ");
      else if (Zone < 0 && Zone > -10) lcd.print(" ");
      lcd.print(Zone);
      break;
    
    case 3:  // display seconds
      lcd.print(F("Display Seconds: "));
      lcd.print(displaySeconds ? "ON " : "OFF");
      break;
    
    case 4:   // baud rate
      lcd.print(F("GPS Baud Rate:   "));
      if (desiredBaud < 10000) lcd.setCursor(16,3);
      else lcd.setCursor(15,3);
      lcd.print(desiredBaud);
      break;

    case 5:  // display seconds
      lcd.print(F("BkLt off @ dark: "));
      lcd.print(noBkliteIfDark ? "ON " : "OFF");
      break;

    case 6: 
      lcd.print(F("Ambient Light:      "));
      lcd.setCursor(16,3);
      lcd.print(rawAnalog);
      break;

    case 7: // display GMT time
      // 24-hour format HH:MM(:SS optional) (utc)
      //if (displaySeconds) {
      //  String t = "      " + twoDigits(gpsHour) + ":" + twoDigits(gpsMinute) + ":" + twoDigits(gpsSecond);
      //  lcd.print(t);
      //  lcd.print(F("   GMT"));
      //} else {
      //  if (colonState == true) {
          String t = "        " +twoDigits(gpsHour) + ":" + twoDigits(gpsMinute) ;
          lcd.print(t);
      //  }
      //  else {
      //    String t = "        " +twoDigits(gpsHour) + " " + twoDigits(gpsMinute) ;
      //    lcd.print(t);
      //  }
        lcd.print(F("    GMT")); 
      //}
    break;

    default:  
      break;    
  }
}

// Helper to format and display time/date/day lines
void drawTimeDateDay() {

  // We'll compute displayTime fields using gps.time and gps.date
  // If GPS data not valid, show placeholders
  if (!gps.time.isValid() || !gps.date.isValid() || !receivingTime) {   // 251028 add this because sometime date is wrong, and so DST can be wrong
    // Show waiting or invalid
    lcd.setCursor(0,0);
    lcd.print(F("      --:--:--      "));
    lcd.setCursor(0,1);
    lcd.print(F("       ------       "));
    lcd.setCursor(0,2);
    lcd.print(F("     --/--/----     "));
    return;
  }

  lcd.setCursor(0,0);

  if (!autoDST) {  // Not in auto-DST mode.  Format time depending upon whether GMT or not.
    if (GMT) {
      // 24-hour format HH:MM(:SS optional) (utc)
      if (displaySeconds) {
        String t = "      " + twoDigits(gpsHour) + ":" + twoDigits(gpsMinute) + ":" + twoDigits(gpsSecond);
        lcd.print(t);
        lcd.print(F("   GMT"));
      } else {
        if (colonState == true) {
          String t = "        " +twoDigits(gpsHour) + ":" + twoDigits(gpsMinute) ;
          lcd.print(t);
        }
        else {
          String t = "        " +twoDigits(gpsHour) + " " + twoDigits(gpsMinute) ;
          lcd.print(t);
        }
        lcd.print(F("    GMT")); 
      }
    } else {  // In auto-DST mode (assume in U.S.A.).  Only display time in 12-hour format
      // local time in 12-hour format; hours 01..12
      int hour12 = localHour % 12;
      if (hour12 == 0) hour12 = 12;
      String ampm = (localHour >= 12) ? "PM" : "AM";
      if (displaySeconds) {
        String t = "      " + twoDigits(hour12) + ":" + twoDigits(gpsMinute) + ":" + twoDigits(gpsSecond);
        lcd.print(t);
        lcd.print(F("    "));
        lcd.print(ampm);
      } else {
        if (colonState == true) {
          String t = "        " + twoDigits(hour12) + ":" + twoDigits(gpsMinute);
          lcd.print(t);
        }
        else {
          String t = "        " + twoDigits(hour12) + " " + twoDigits(gpsMinute);
          lcd.print(t);
        }
        lcd.print(F("     "));
        lcd.print(ampm);
      }
    }
  } else {  //autoDST true.  show time in 12hour format
    // local time in 12-hour format; hours 01..12
    int hour12 = localHour % 12;
    if (hour12 == 0) hour12 = 12;
    String ampm = (localHour >= 12) ? "PM" : "AM";
    if (displaySeconds) {
      String t = "      " + twoDigits(hour12) + ":" + twoDigits(gpsMinute) + ":" + twoDigits(gpsSecond);
      lcd.print(t);
      lcd.print(F("    "));
      lcd.print(ampm);
    } else {
      if (colonState == true) {
        String t = "        " + twoDigits(hour12) + ":" + twoDigits(gpsMinute);
        lcd.print(t);
      }
      else {
        String t = "        " + twoDigits(hour12) + " " + twoDigits(gpsMinute);
        lcd.print(t);
      }
      lcd.print(F("     "));
      lcd.print(ampm);
    }
  }

  // Print Date
  lcd.setCursor(0,2);
  lcd.print(F(" ")); // move over one space 

  char dateBuf[22];
  snprintf(dateBuf, sizeof(dateBuf), "    %02d/%02d/%04d   ", localMonth, localDay, localYear);
  lcd.print(dateBuf);

  // Day of week line:
  int dow = dayOfWeek(localDay, localMonth, localYear); // 0..6
  lcd.setCursor(0,1);
  // print 6 spaces if dow = sunday, monday, tuesday, friday (0,1,5),
  if (dow == 0 || dow == 1 || dow == 5) {
    lcd.print(F("       "));   // seven spaces
    lcd.print(dayText(dow));   // 6 characters
    lcd.print(F("       "));   // seven spaces
  }
  else {
    if (dow == 2) {   // tuesday (7 characters)
      lcd.print(F("       "));   // seven spaces
      lcd.print(dayText(dow));   // 7 characters
      lcd.print(F("      "));    // 6 spaces
    }
    else {
      if (dow == 4 || dow == 6 || dow == 7) { //thursday, saturday (8 characters)
        lcd.print(F("      "));    // 6 spaces
        lcd.print(dayText(dow));   // 8 characters
        lcd.print(F("      "));    // 6 spaces
      }
      else {  // wednesday or invalid day (9 characters)
        lcd.print(F("      "));    // 6 spaces
        lcd.print(dayText(dow));   // 9 characters
        lcd.print(F("     "));     // 5 spaces
      }
    }
  }
}

void checkandUpdateBaud() {
  // If GPS baud not what we want, then change it
  // This should be done at the start of power up
  // (when the gps receiver's baud rate defaults to
  // 9600 baud), and whenever we step to and thru
  // the gps baud rate menu).

  if (currentBaud != desiredBaud) {
    Serial.print(F("Attempting to change GPS baud to "));
    Serial.println(desiredBaud);
    lcd.setCursor(0,3);
    lcd.print(F("Changing to:        "));
    if (desiredBaud < 10000) lcd.setCursor(16,3);
    else lcd.setCursor(15,3);
    lcd.print(desiredBaud);
    delay(1000);

    bool success = false;
    int attempt = 1;
    //for (int attempt = 1; attempt <= 3; attempt++) {
    while (!success) {
      Serial.print(F("  Attempt "));
      Serial.println(attempt);
      lcd.setCursor(0,3);
      lcd.print(F("                    "));
      lcd.setCursor(0,3);
      lcd.print(F("...Attempt "));
      lcd.print(attempt);

      attempt++;

      gpsSerial.begin(currentBaud);  // talk at old baud
      sendUBXsetBaud(desiredBaud);

      delay(750);  // let GPS switch
      //gpsSerial.begin(desiredBaud);   // 251024 done in detectBaud() instead

      if (detectBaud(desiredBaud)) {
        Serial.println(F("  ✔ Baud change verified OK!"));
        lcd.setCursor(0,3);
        lcd.print(F("  Baud Change OK!   "));
        currentBaud = desiredBaud;
        success = true;
        delay(1000);
        break;
      } else {
        if (attempt > 4) {
          Serial.println(F("  ✘ Verification failed"));
          // show error   
          lcd.setCursor(0,3);
          lcd.print(F("ERR: Did Not Verify "));
          delay(3000);
          break;
        }
      }      
    }

    if (!success) {
      Serial.println(F("❌ Could not change GPS baud."));

        // show error   
        lcd.setCursor(0,3);
        lcd.print(F("FAIL TO CHANGE BAUD "));
        delay(3000);
      }
  } else {
    Serial.println(F("GPS already at desired baud."));
    lcd.setCursor(0,3);
    lcd.print(F("    Already at     "));
    if (desiredBaud < 10000) lcd.setCursor(16,3);
    else lcd.setCursor(15,3);
    lcd.print(desiredBaud);
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
  if ((autoDST && isDST(year, month, day, hour)) || (DST && !GMT)) { //**** 251018  if DST also do this
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
