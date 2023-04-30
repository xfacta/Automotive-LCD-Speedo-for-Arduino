# Automotive LCD Speedo for Arduino
## Digital and bargraph display of Speed in km/hr

- AT Mega2560 and 3.5" 320x480 LCD display
- Speedometer and Trip meter with ParkBrake warning
- Large text with km travelled
- Bar style graphical meter
- Normal direction of bar to pair with tachometer
- Dim display on "lights" input
- Park brake warning
- Uses pulseIn , no interupts
- Odometer saved to EEPROM with wear-leveling
- and two redundant copies of the odometer value
- EEPROM write denied during power-down
- Offloaded sounds to external Leonardo Tiny



### Uses 
UTFT Libraries and some associated font files

Copyright (C)2015 Rinky-Dink Electronics, Henning Karlsen. All right reserved

web: http://www.RinkyDinkElectronics.com/

This is the ILI9481 display used

https://www.altronics.com.au/p/z6527a-3.5-lcd-tft-arduino-mega2560-shield/


### It is required to check and adjust

```
diff_r = 3.90;                        // diff ratio
tyre_dia = 576;                       // tyre diameter in mm
vss_rev = 4;                          // vss pulses per tailshaft revolution

Min_vspeed = 4;                       // set the minimum expected speed
Speed_Marker_1 = 39;                  // set 1st speed marker on bar graph
Speed_Marker_2 = 58;                  // set 2nd speed marker on bar graph
Speed_Marker_3 = 97;                  // set 3rd speed marker on bar graph

float kludge_factor = 0.996;          // manual scaling of km/hr if needed

const float vcc_ref = 4.92;           // measure the 5 volts DC and set it here
const float R1 = 1200.0;              // measure and set the exact voltage divider values
const float R2 = 3300.0;              // for accurate voltage measurements
```

`Kludge_Factor` is a way of adjusting timing since all Arduino crystals will be slightly different.

If the `vss sensor` is on a wheel then set `diff_r = 1.0` and `vss_rev` to the number pulses per wheel revolution.

### You can also set
- `Demo_Mode` = true or false for display of random values
- `Calibration_Mode` = true or false for display of some raw data like input frequency in Hz

Pressing the button at startup also enters calibration mode.

Pressing the button during normal operation resets the trip meter value.
Currently there is only a crude long-press detection using delay, and some debouncing in hardware is assumed.

The sounds are offloaded to a Leonardo Tiny to avoid delays and allow one Tiny and speaker to srvice multiple other functions such as Speedo and Fuel/Temperature/OilPressure gauge warning sounds.

The dimming function works by changing to using light grey and dark grey instead of white and colours, since there is no backlight control on the LCD panel I used.

