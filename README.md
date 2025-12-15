Code for Arduino Nano GPS Clock.
Rev 7 (R7) is the latest revision.  It includes code for Automatic Daylight-Savings-Time adjustment (Auto-DST).
There are different versions Rev 7 code for:
  o  4-line LCD (requires a board mod to use Arduino I2C I/F).
  o  2-line LCD (requires a board mod to use Arduino I2C I/F).
  o  Various 4-digit LED displays 
     -- There are three of these, each with different
        Ambient Light thresholds for display brightness
        adjustment, because the three displays have 
        different maximum brightnesses.

To invoke the Auto-DST function, both the nGMT and the nDST pins must be shorted to ground.  For clocks that I had built before this code modification that use a SP3T switch on the front panel to select between DST,Standard, or GMT time, a diode is added across the nGMT contact to ground (see uploaded schematic mod in this repository) and the switch left in "daylight savings time" mode.
