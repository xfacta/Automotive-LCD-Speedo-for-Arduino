// Speedometer and Trip meter with ParkBrake warning
// Large text with km travelled
// Removed maximum speed
// Removed gear indicator (requires RPM and extra calcs)
// Bar style graphical meter
// Dim display on parker lights
// Park brake warning
// Uses pulseIn , no interupts
// Odometer saved to EEPROM
// with EEPROM write denied during power-down
// Offloaded sounds to external Leonardo Tiny
// Updated version so all modules are using the same numbering


// UTFT Libraries
// Copyright (C)2015 Rinky-Dink Electronics, Henning Karlsen. All right reserved
// web: http://www.RinkyDinkElectronics.com/



#define Version "Speed Bar V14"



//========================== Set These Manually ==========================

float diff_r = 3.90;   // diff ratio
float tyre_dia = 576;  // tyre diameter in mm
float vss_rev = 4;     // vss pulses per tailshaft revolution

float Min_vspeed = 4;     // set the minimum expected speed
int Speed_Marker_1 = 39;  // set 1st speed marker on bar graph
int Speed_Marker_2 = 58;  // set 2nd speed marker on bar graph
int Speed_Marker_3 = 97;  // set 3rd speed marker on bar graph

float kludge_factor = 0.996;  // manual scalling of km/hr if needed

const float vcc_ref = 4.92;  // measure the 5 volts DC and set it here
const float R1 = 1200.0;     // measure and set the voltage divider values
const float R2 = 3300.0;     // for accurate voltage measurements

const float Safe_Voltage = 0.0;  // set to 11.0 for real usage

//========================================================================



//========================== Calibration mode ===========================

bool Calibration_Mode = false;
bool Demo_Mode = true;

//========================================================================



// Using the inbuilt eeprom to save the odometer value
#include <EEPROM.h>

// Using the EEWL EEPROM wear level library to spread data over the EEPROM
#include <eewl.h>
// setting 100 blocks = 100*5 bytes = 500 bytes, approx 500*100000 writes
// Changing these values will cause the EEPROM variables to be cleared
#define BUFFER_LEN 100      // number of data blocks (1 blk = 1 ulong = 4 bytes) + 1 byte overhead
#define BUFFER_START1 100   // EEPROM address where buffer1/Odo1 starts (UL)
#define BUFFER_START2 700   // EEPROM address where buffer2/Odo1 starts (UL)
#define BUFFER_START3 1300  // EEPROM address where buffer3/Check starts (int)


// Screen and font stuff
// more fonts use more memory
#include <UTFT.h>
#include <UTFT_Geometry.h>  // needed for drawing triangles

UTFT myGLCD(ILI9481, 38, 39, 40, 41);
UTFT_Geometry geo(&myGLCD);

// Declare which fonts we will be using
//extern uint8_t SmallFont[];
//extern uint8_t BigFont[];
extern uint8_t franklingothic_normal[];
//extern uint8_t SevenSegNumFont[];
extern uint8_t SevenSegmentFull[];
extern uint8_t Grotesk16x32[];
//extern uint8_t Grotesk24x48[];
//extern uint8_t Grotesk32x64[];
//extern uint8_t GroteskBold32x64[];
//extern uint8_t SevenSeg_XXXL_Num[];
extern uint8_t SevenSegment96x144Num[];

#define font0 franklingothic_normal
#define font1 Grotesk16x32
//#define font2 Grotesk24x48
//#define font3 Grotesk32x64
//#define font4 GroteskBold32x64
//#define font7B SevenSeg_XXXL_Num
#define font7L SevenSegment96x144Num
#define font7F SevenSegmentFull


// Set colours used for bright or dim mode
// this display doesnt have a controllable backlight
// so darker text colours are used for dim mode
bool dim_mode = false;
int text_colour1 = VGA_WHITE;      // or VGA_SILVER
int text_colour2 = VGA_SILVER;     // or VGA_GRAY
int block_fill_colour = VGA_GRAY;  // or VGA_BLACK
int startup_time = 10000;



// Voltage calculations
/*
   Read the Analog Input
   adc_value = analogRead(ANALOG_IN_PIN);

   Determine voltage at ADC input
   adc_voltage  = (adc_value+0.5) * ref_voltage / 1024.0 / (voltage divider)

   Calculate voltage at divider input
   in_voltage = adc_voltage / (R2/(R1+R2));

   R1 = 3300
   R2 = 1200
   vref = default = ~5.0 volts
   Vin = analogRead(ANALOG_IN_PIN) * 5 / 1024 / (3300/(3300+1200))
   Vin = analogRead(ANALOG_IN_PIN) * 0.00665838
*/
const float Input_Multiplier = vcc_ref / 1024.0 / (R2 / (R1 + R2));



// Common pin definitions
const int SD_Select = 53;


// Pin definitions for digital inputs
//const int Oil_Press_Pin = 0;              // Oil pressure digital input pin
//const int Parker_Light_Pin = 1;           // Parker lights digital input pin
const int Low_Beam_Pin = 2;  // Low beam digital input pin
//const int High_Beam_Pin = 3;              // High beam digital input pin
const int Pbrake_Input_Pin = 4;  // Park brake input pin
const int VSS_Input_Pin = 5;     // Speed frequency input pin
//const int RPM_Input_Pin = 6;              // RPM frequency input pin
//const int RPM_PWM_In_Pin = 6;             // Input PWM signal representing RPM
const int Button_Pin = 7;  // Button momentary input

// Pin definitions for analog inputs
//const int Temp_Pin = A0;                  // Temperature analog input pin - not used with OneWire sensor
//const int Fuel_Pin = A1;                  // Fuel level analog input pin
//const int Batt_Volt_Pin = A2;             // Voltage analog input pin
//const int Alternator_Pin = A3;            // Alternator indicator analog input pin
//const int Head_Light_Input = A4;          // Headlights via resistor ladder
const int Power_Good_Pin = A5;  // Power good, deny EEPROM writes when going bad

// Pin definitions for outputs
//const int RPM_PWM_Out_Pin = 10;           // Output of RPM as a PWM signal for shift light
//const int LED_Pin = 10;                   // NeoPixel LED pin
const int Warning_Pin = 11;  // Link to external Leonardo for general warning sounds
//const int OP_Warning_Pin = 12;            // Link to external Leonardo for oil pressure warning sound
//const int Relay_Pin = 13;                 // Relay for fan control


// Times of last important events
uint32_t distLoopTime;  // used to measure the next loop time
uint32_t distIntTime;   // Interval time between loops


// Speed variables
float freq, vss, distance_per_VSS_pulse, pulses_per_km;
int vspeed, last_vspeed;
unsigned long period, lowtime, hightime, pulsein_timeout;
int speed_x = 54, speed_y = 75;


// Park Brake variables
// used for position of Park Brake indicator
int PB_x = 410, PB_y = 120;


//Distance variables
float avg_vspeed, DistM, DistINTm, DistTotalm, DistKM;
int dist_x = 260, dist_y = 240;
int odo_x = 20, odo_y = 240;
unsigned long OdometerTotal, OdometerVerify, Odometer1, Odometer2;  // Odo values read and writen to eeprom in km
float OdometerTemp;                                                 // in meters
byte Check_Byte;                                                    // Byte to say which odo value was last written OK


// Meter variables
int blocks, new_val, num_segs, seg_size, x1, x2, block_colour;
int linearBarX = 15, linearBarY = 10;  // bar starting position in pixels
const int barLength = 450;             // length and width in pixels
const int barWidth = 50;
const int seg_value = 5;                 // 5km/hr segments
const int seg_gap = 2;                   // gap between segments in pixels
const int meterMin = 0, meterMax = 200;  // bar scale


// Start a Class for each Odomoter and Check_Byte counter
EEWL Odo1_EEPROM(Odometer1, BUFFER_LEN, BUFFER_START1);
EEWL Odo2_EEPROM(Odometer2, BUFFER_LEN, BUFFER_START2);
EEWL Check_EEPROM(Check_Byte, BUFFER_LEN, BUFFER_START3);


// ##################################################################################################################################################



void setup() {


  // Improved randomness for testing
  unsigned long seed = 0, count = 32;
  while (--count)
    seed = (seed << 1) | (analogRead(A6) & 1);
  randomSeed(seed);


  // Set any pins that might be used
  // after testing take out any _PULLUP
  // and rely on external interface electronics

  // Outputs
  pinMode(Warning_Pin, OUTPUT);
  digitalWrite(Warning_Pin, HIGH);

  // Digital inputs
  pinMode(Button_Pin, INPUT_PULLUP);
  pinMode(Pbrake_Input_Pin, INPUT);
  pinMode(Low_Beam_Pin, INPUT);
  pinMode(VSS_Input_Pin, INPUT);

  // Analog inputs
  pinMode(Power_Good_Pin, INPUT);


  // =======================================================
  // Calculate the distance travelled per VSS pulse
  // based on tyre size, diff ratio and VSS pulses per tailshaft revolution, in millimeters

  distance_per_VSS_pulse = tyre_dia * PI / diff_r / vss_rev;  // millimeters
  pulses_per_km = 1000000.0 / distance_per_VSS_pulse;

  // =======================================================

  // Maximum time pulsein will wait for a signal in microseconds
  // Use period of lowest expected frequency from
  // diff_r , tyre_dia , vss_rev and Min_vspeed
  pulsein_timeout = 1000000.0 / (pulses_per_km * Min_vspeed / 3600.0) * 2.0;


  // Start the wear-leveling classes for EEPROM
  // this also initiates .fastFormat() on first use or no valid values
  Odo1_EEPROM.begin();
  Odo2_EEPROM.begin();
  Check_EEPROM.begin();

  // Read Odometer values from EEPROM
  // First ever use or unreadable values sets them to 100000
  // Use the highest value available as the valid Odometer

  Check_EEPROM.get(Check_Byte);

  switch (Check_Byte) {
    // Check Byte 1 indicates ONLY Odo1 was written successfully, but not Odo2
    case 1:
      Odo1_EEPROM.get(Odometer1);
      Odometer2 = Odometer1;
      break;

    // Check Byte 2 indicates ONLY Odo2 was written successfully, but not Odo1
    case 2:
      Odo2_EEPROM.get(Odometer2);
      Odometer1 = Odometer2;
      break;

    // Check Byte 3 indicates Odo1 and Odo2 was written successfully, use either
    case 3:
      Odo1_EEPROM.get(Odometer1);
      Odo2_EEPROM.get(Odometer2);
      break;

    // Check Byte not set, maybe first run
    // There might be junk values, try to read them
    // otherwise set new default values
    default:
      Odo1_EEPROM.get(Odometer1);
      if ((Odometer1 < 100000) || (Odometer1 > 999999)) {
        Odometer1 = 100000;
      }

      Odo2_EEPROM.get(Odometer2);
      if ((Odometer2 < 100000) || (Odometer2 > 999999)) {
        Odometer2 = 100000;
      }
  }

  // Catch-all if the Switch/Case routine doesnt work as expected
  // Whichever can be read successfully from EEPROM or is greater
  // becomes the new Odo value
  if (Odometer2 > Odometer1) {
    Odometer1 = Odometer2;
  } else if (Odometer1 > Odometer2) {
    Odometer2 = Odometer1;
  }
  // but dont write any new values to EEPROM until loops have been completed

  // Set the working Odo total from a valid saved value
  OdometerTotal = Odometer1;


  // set some more values for the bar graph
  // these only need to be calculated once
  num_segs = int(0.5 + (float)(meterMax - meterMin) / (float)seg_value);  // calculate number of segments
  seg_size = int(0.5 + (float)barLength / (float)num_segs);               // calculate segment width in pixels
  linearBarX = linearBarX + (barLength - num_segs * seg_size) / 2;        // centre the bar to allow for rounding errors


  // Clear the screen and display static items
  myGLCD.InitLCD(LANDSCAPE);
  myGLCD.clrScr();
  myGLCD.setColor(VGA_GRAY);
  myGLCD.setBackColor(VGA_BLACK);
  myGLCD.setFont(font0);
  myGLCD.print(Version, CENTER, CENTER);
  delay(2000);
  myGLCD.clrScr();
  myGLCD.print("km/h", speed_x + 300, speed_y + 124);
  //myGLCD.print("km", dist_x + 210, dist_y + 32);
  //myGLCD.print("km", odo_x + 210, odo_y + 32);


  // Set calibration mode from button input
  if (digitalRead(Button_Pin) == LOW) {
    // wait and see if the button input is still low
    delay(100);
    if (digitalRead(Button_Pin) == LOW) Calibration_Mode = true;
  }


  if (Calibration_Mode == true) {
    Demo_Mode = false;  // cant have both demo mode and calibration mode at once
    myGLCD.printNumF(distance_per_VSS_pulse, 2, 0, 265, '.', 6, ' ');
    myGLCD.printNumF(pulses_per_km, 2, 0, 285, '.', 6, ' ');
  } else  // Display the stored Odometer value
  {
    myGLCD.setColor(text_colour2);
    myGLCD.setFont(font7F);
    myGLCD.printNumI(OdometerTotal, odo_x, odo_y, 6, ' ');
  }


  // Draw triangles at predetermined points
  int S1 = linearBarX + int(0.5 + (float)barLength / ((float)meterMax / (float)Speed_Marker_1));  // 1st mark
  int S2 = linearBarX + int(0.5 + (float)barLength / ((float)meterMax / (float)Speed_Marker_2));  // 2nd mark
  int S3 = linearBarX + int(0.5 + (float)barLength / ((float)meterMax / (float)Speed_Marker_3));  // 3rd mark
  myGLCD.setColor(VGA_RED);
  geo.fillTriangle(S1 - 4, linearBarY + barWidth + 8, S1 + 4, linearBarY + barWidth + 8, S1, linearBarY + barWidth + 2);
  geo.fillTriangle(S2 - 4, linearBarY + barWidth + 8, S2 + 4, linearBarY + barWidth + 8, S2, linearBarY + barWidth + 2);
  geo.fillTriangle(S3 - 4, linearBarY + barWidth + 8, S3 + 4, linearBarY + barWidth + 8, S3, linearBarY + barWidth + 2);


  // set initial timing value
  distLoopTime = millis();


}  // End void setup


// ##################################################################################################################################################
// ##################################################################################################################################################


void loop() {


  // =======================================================
  // Reset trip meter by button press
  // =======================================================

  if (digitalRead(Button_Pin) == LOW) {
    // wait and see if the button input is still low
    delay(100);
    if (digitalRead(Button_Pin) == LOW) DistTotalm = 0;
  }


  // =======================================================
  // Dim display when headlights on
  // =======================================================

  if (millis() > startup_time) {
    // Dim mode when headlights are on
    if (digitalRead(Low_Beam_Pin) == HIGH && dim_mode == false) {
      dim_mode = true;
      text_colour1 = VGA_SILVER;
      text_colour2 = VGA_GRAY;
      block_fill_colour = VGA_BLACK;
    }

    // Normal colours when headlights are off
    if (digitalRead(Low_Beam_Pin) == LOW && dim_mode == true) {
      dim_mode = false;
      text_colour1 = VGA_WHITE;
      text_colour2 = VGA_SILVER;
      block_fill_colour = VGA_GRAY;
    }
  }


  // =======================================================
  // Audible warning if the vehicle is moving with Park Brake on
  // =======================================================

  if (digitalRead(Pbrake_Input_Pin) == HIGH) {
    // display a red P when the park brake is on
    myGLCD.setFont(font7F);
    myGLCD.setColor(VGA_BLACK);
    myGLCD.setBackColor(VGA_RED);
    myGLCD.print("P", PB_x, PB_y);
    myGLCD.setBackColor(VGA_BLACK);

    // sound a warning tone if vehicle is moving
    if (vspeed > 0) digitalWrite(Warning_Pin, LOW);
  } else {
    // print a black space over the P
    if (digitalRead(Warning_Pin) == LOW) digitalWrite(Warning_Pin, HIGH);
    myGLCD.setFont(font7F);
    myGLCD.setColor(VGA_BLACK);
    myGLCD.setBackColor(VGA_BLACK);
    myGLCD.print(" ", PB_x, PB_y);
  }


  // =======================================================
  // Input from the vehicle speed sensor
  // =======================================================

  if (Demo_Mode == false) {
    // Read the real sensor
    hightime = pulseIn(VSS_Input_Pin, HIGH, pulsein_timeout);
    lowtime = pulseIn(VSS_Input_Pin, LOW, pulsein_timeout);
    period = hightime + lowtime;

    // prevent overflows or divide by zero
    if (period > 1000) {
      freq = 1000000.0 / (float)period;
    } else {
      freq = 0;
    }

    vss = 3600.0 * freq / pulses_per_km * kludge_factor;

    if (Calibration_Mode == true) {
      myGLCD.setColor(VGA_GRAY);
      myGLCD.setBackColor(VGA_BLACK);
      myGLCD.setFont(font0);
      myGLCD.printNumI(period, 0, 220, 4);
      myGLCD.printNumI(freq, 0, 240, 4);
      //myGLCD.printNumI(vss, 0, 260, 4);
    }
  } else {
    // Demo mode, invent values
    vss = random(5, 185);
    vss = constrain(vss, last_vspeed - 20, last_vspeed + 20);
    //vss = 60;
  }

  // the reluctor pickup is not accurate below 4km/hr
  if (vss < Min_vspeed || vss > 220) vss = 0;
  // round up the vspeed so it doesnt under-read
  vspeed = round(vss + 0.5);


  // =======================================================
  // Calculate and display distance travelled
  // =======================================================

  // Calc average speed for the loop
  avg_vspeed = (vss + (float)last_vspeed) / 2.0;

  // Convert km/hr to meters per second
  // avspeed * 1000 / 3600
  // Convert interval in millis to seconds
  // distIntTime / 1000

  // Calculate how long it took to get back to this point in the loop
  distIntTime = (millis() - distLoopTime);
  distLoopTime = millis();

  // Calc meters travelled in this interval, removed redunant *1000 / 1000
  DistINTm = (avg_vspeed / 3600.0) * (float)distIntTime;

  // Add up total meters
  DistTotalm += DistINTm;

  // Convert to kilometers
  DistKM = DistTotalm / 1000.0;

  // Display the trip distance each loop
  myGLCD.setColor(text_colour2);
  myGLCD.setFont(font7F);
  myGLCD.printNumF(DistKM, 2, dist_x, dist_y, '.', 6, ' ');


  // =======================================================
  // Update the Odometer and check byte
  // =======================================================

  // add to Odometer
  // keep it in meters until saving to odometer
  OdometerTemp += DistINTm;

  // If 1 km or more has been accumulated write it
  // to the permenant odo and reset the temporary value
  if (OdometerTemp >= 1000) {
    OdometerTotal += round(OdometerTemp / 1000.0);

    // Display the new Odo total only when the values is updated
    myGLCD.printNumI(OdometerTotal, odo_x, odo_y, 6, ' ');

    // Write OdometerTotal to Odometer1 and Odometer2
    // only if Power is good
    Odometer1 = OdometerTotal;
    Odometer2 = OdometerTotal;

    if ((analogRead(Power_Good_Pin) * Input_Multiplier) >= Safe_Voltage) {
      // Reset the check byte
      Check_Byte = 0;

      // Write the Odo1 to EEPROM
      Odo1_EEPROM.put(Odometer1);
      // Check it and set the Check_Byte
      if (Odo1_EEPROM.get(OdometerVerify) == Odometer1) {
        Check_Byte = 1;
      }

      // Write Odo2 to EEPROM
      Odo2_EEPROM.put(Odometer2);
      // Check it and set the Check_Byte
      if (Odo2_EEPROM.get(OdometerVerify) == Odometer2) {
        Check_Byte += 2;
      }

      // Write the check byte to EEPROM last
      Check_EEPROM.put(Check_Byte);
    }

    // Odotemp has reached a 1000m or more and been added to total
    // reset it and start counting again for the next 1km
    OdometerTemp = 0;
  }


  // =======================================================
  // Display the speed
  // =======================================================

  constrain(vspeed, 0, 200);

  myGLCD.setFont(font7L);
  if (vspeed <= 99) {
    // draw a black rectangle where a rogue digit would be
    // because this large font only has digits, no space char
    if (last_vspeed > 99) {
      myGLCD.setColor(VGA_BLACK);
      myGLCD.setBackColor(VGA_BLACK);
      myGLCD.fillRect(speed_x, speed_y, speed_x + 96, speed_y + 144);
    }
    myGLCD.setColor(text_colour1);
    myGLCD.printNumI(vspeed, speed_x + 96, speed_y, 2, '0');
  } else {
    myGLCD.setColor(text_colour1);
    myGLCD.printNumI(vspeed, speed_x, speed_y, 3, '0');
  }
  last_vspeed = vspeed;

  // =======================================================
  // Draw the barmeter
  // =======================================================

  int new_val = map(vspeed, meterMin, meterMax, 0, num_segs);

  // Draw colour blocks for the segents
  for (blocks = 0; blocks < num_segs; blocks++) {
    // Calculate pair of coordinates for segment start and end
    x1 = linearBarX + (blocks * seg_size);                  // starting X coord
    x2 = linearBarX + ((blocks + 1) * seg_size) - seg_gap;  // ending X coord allowing for gap

    if (new_val > 0 && blocks < new_val) {
      if (dim_mode == false) {
        // Choose colour from scheme using the Rainbow function
        // uncomment one line
        //block_colour = VGA_RED; // Fixed colour
        //block_colour = VGA_GREEN;  // Fixed colour
        //block_colour = VGA_NAVY; // Fixed colour
        //block_colour = rainbow(map(blocks, 0, num_segs, 0, 127)); // Blue to red
        block_colour = rainbow(map(blocks, 0, num_segs, 63, 127));  // Green to red
        //block_colour = rainbow(map(blocks, 0, num_segs, 127, 63)); // Red to green
        //block_colour = rainbow(map(blocks, 0, num_segs, 127, 0)); // Red to blue
      } else {
        block_colour = text_colour2;
      }

      // Fill in coloured blocks
      myGLCD.setColor(block_colour);
      myGLCD.fillRect(x1, linearBarY, x2, linearBarY + barWidth);
    }
    // Fill in blank segments
    else {
      myGLCD.setColor(block_fill_colour);
      myGLCD.fillRect(x1, linearBarY, x2, linearBarY + barWidth);
    }
  }


}  // End void loop


// ##################################################################################################################################################
// ##################################################################################################################################################


// Function to return a 16 bit rainbow colour

unsigned int rainbow(byte value) {
  // Value is expected to be in range 0-127
  // The value is converted to a spectrum colour from 0 = blue through to 127 = red

  byte red = 0;    // Red is the top 5 bits of a 16 bit colour value
  byte green = 0;  // Green is the middle 6 bits
  byte blue = 0;   // Blue is the bottom 5 bits

  byte quadrant = value / 32;

  if (quadrant == 0) {
    blue = 31;
    green = 2 * (value % 32);
    red = 0;
  }
  if (quadrant == 1) {
    blue = 31 - (value % 32);
    green = 63;
    red = 0;
  }
  if (quadrant == 2) {
    blue = 0;
    green = 63;
    red = value % 32;
  }
  if (quadrant == 3) {
    blue = 0;
    green = 63 - 2 * (value % 32);
    red = 31;
  }
  return (red << 11) + (green << 5) + blue;
}


// ##################################################################################################################################################
// ##################################################################################################################################################
