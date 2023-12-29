/***************************************************
*                                                  *
*   Arduino Mega2560 based LCD automotive gauge    *
*   for displaying speed and distance              *
*   An odometer values is stored permenantly       *
*   in EEPROM and SD Card                          *
*   This gauge is used in landscape mode           *
*                                                  *
***************************************************/


// Just in case you are not using the Arduino IDE
//#include <arduino.h>


/*
  Speedometer and Odometer with 
  Trip meter and ParkBrake warning
  Large text with km travelled
  Bargraph style meter
  Dim display on parker lights
  Park brake warning
  Uses pulseIn , no interupts
  Odometer saved to EEPROM wear levelling circular buffer
  and EEPROM write denied during power-down
  Odometer saved to SD Card while vehicle stationary
  Changed to using bitwise NOT as a check and improved odo tests
  Offloaded sounds to external Leonardo Tiny
  Improved power-good test

  These features used to exist but removed
  to speed up display updates
  - maximum speed
  - gear indicator with requires RPM and extra calcs
*/


/*
  UTFT Libraries and fonts
  Copyright (C)2015 Rinky-Dink Electronics, Henning Karlsen. All right reserved
  web: http://www.RinkyDinkElectronics.com/
*/

/*
  EEWL library is authored by Fabrizio Pollastri
  under the GNU Lesser General Public License version 3
*/


#define Version "Speed Bar V19"



//========================================================================
//-------------------------- Set These Manually --------------------------
//========================================================================

float diff_r         = 3.70;    // differential ratio
float tyre_dia       = 634;     // tyre diameter in mm
float vss_rev        = 4;       // vss pulses per tailshaft revolution

float Min_vspeed     = 4;      // set the minimum expected speed
float Max_vspeed     = 240;    // set maximum speed for digits
int   Max_barspeed   = 200;    // set maximum speed for bar graph
int   Speed_Marker_1 = 40;     // set 1st speed marker on bar graph
int   Speed_Marker_2 = 60;     // set 2nd speed marker on bar graph
int   Speed_Marker_3 = 80;     // set 3rd speed marker on bar graph
int   Speed_Marker_4 = 100;    // set 4th speed marker on bar graph

// Set whether digitial inputs are active low or active high
// example: active low = pulled to ground for a valid button press
const bool Digitial_Input_Active = LOW;

// Set high or low for valid warnings to be passed to external processing
const bool Valid_Warning = LOW;

// Kludge factor to allow for differing
// crystals and similar inconsistancies
float Kludge_Factor = 1.000;

// Set these to ensure correct voltage readings of analog inputs
const float vcc_ref = 4.92;      // measure the 5 volts DC and set it here
const float R1      = 1200.0;    // measure and set the voltage divider values
const float R2      = 3300.0;    // for accurate voltage measurements

// Power Good safe voltage level
// used to deny eeprom writes while supply voltage is too low
// set to about 11 or 12 for normal use
// set to 0 for demo use or testing
const float Safe_Voltage = 0.0;

//========================================================================



//========================================================================
//-------------------------- Calibration mode ----------------------------
//========================================================================

// Demo = true gives random speed values

bool Demo_Mode  = true;
bool Debug_Mode = false;

// Danger Will Robinson!
bool wipe_totals = false;
// Set back to false and upload again
// or this takes effect every reboot

//========================================================================


// Needed for SD Card operation
#include <SD.h>
#include <SPI.h>


// Using the EEWL EEPROM wear level library
// to spread data over the EEPROM with a circular buffer
#include <eewl.h>
// pulls in <EEPROM.h> too
// setting 100 blocks = 100*5 bytes = 500 bytes, approx 500*100000 writes
// ! Changing these values will cause the EEPROM variables to be cleared !
#define BUFFER_LEN    0x0080    // 128 = number of data blocks (1 blk = 1 ulong = 4 bytes) + 1 byte overhead = 128 x 5 = 640 bytes
#define BUFFER_START1 0x0100    // EEPROM address where buffer1/Odo1 starts (UL) = 256
#define BUFFER_START2 0x0400    // EEPROM address where buffer2/Odo2 starts (UL) = 1024
#define BUFFER_START3 0x0800    // EEPROM address where buffer3/Check starts (int) = 2048


// Screen and font stuff
// more fonts use more memory
#include <UTFT.h>
// needed for drawing triangles
#include <UTFT_Geometry.h>

UTFT          myGLCD(ILI9481, 38, 39, 40, 41);
UTFT_Geometry geo(&myGLCD);

/*
  My display needed the ILI8491 driver changed
  to flip the display
  LCD_Write_DATA(0x4A); <- correct
  LCD_Write_DATA(0x8A); <- was
  Possible values: 0x8A 0x4A 0x2A 0x1A
*/

/*
Available colours
  VGA_BLACK	
  VGA_WHITE	
  VGA_RED
  VGA_GREEN	
  VGA_BLUE	
  VGA_SILVER	
  VGA_GRAY	
  VGA_MAROON	
  VGA_YELLOW	
  VGA_OLIVE	
  VGA_LIME	
  VGA_AQUA	
  VGA_TEAL	
  VGA_NAVY	
  VGA_FUCHSIA
  VGA_PURPLE	
  VGA_TRANSPARENT
*/
// orange is missing from the default range
#define VGA_ORANGE 0xFD20 /* 255, 165,   0 */

// Declare which fonts will be used
// and comment out the rest
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
bool dim_mode          = false;
int  text_colour1      = VGA_WHITE;     // or VGA_SILVER
int  text_colour2      = VGA_SILVER;    // or VGA_GRAY
int  block_fill_colour = VGA_GRAY;      // or VGA_BLACK

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
  or
  Vin = analogRead(ANALOG_IN_PIN) * Input_Multiplier
*/
const float Input_Multiplier = vcc_ref / 1024.0 / (R2 / (R1 + R2));


// Common pin definitions
const int SD_Select = 53;

// Pin definitions for digital inputs
// Mega2560 Serial2 pins 17(RX), 16(TX)
const int Oil_Press_Pin    = 0;    // Oil pressure digital input pin
const int Parker_Light_Pin = 1;    // Parker lights digital input pin
const int Low_Beam_Pin     = 2;    // Low beam digital input pin
const int High_Beam_Pin    = 3;    // High beam digital input pin
const int Pbrake_Input_Pin = 4;    // Park brake input pin
const int VSS_Input_Pin    = 5;    // Speed frequency input pin
const int RPM_Input_Pin    = 6;    // RPM frequency INPUT pin
const int Button_Pin       = 7;    // Button momentary input

// Pin definitions for analog inputs
const int Temp_Pin       = A0;    // Temperature analog input pin - OneWire sensor on pin 14
const int Fuel_Pin       = A1;    // Fuel level analog input pin
const int Batt_Volt_Pin  = A2;    // Voltage analog input pin
const int Alternator_Pin = A3;    // Alternator indicator analog input pin

// Pin definitions for outputs
// Mega2560 Serial2 pins 17(RX), 16(TX)
const int LED_Pin        = 10;    // NeoPixel LED pin
const int Warning_Pin    = 11;    // Link to external Leonardo for general warning sounds
const int OP_Warning_Pin = 12;    // Link to external Leonardo for oil pressure warning sound
const int Relay_Pin      = 13;    // Relay for fan control

// Times of last important events
uint32_t distLoopTime;                   // used to measure the next loop time
uint32_t distIntTime;                    // Interval time between loops
uint32_t odoCheckTime;                   // Last time saved odometer value was verified
uint32_t odoCheckInterval   = 600000;    // minimum time between checking saved odometer values, 10 minutes
bool     Pbrake_last_status = false;     // last status of Park Brake
int      spinnerState       = 1;
int      startup_time       = 10000;

// Speed variables
float    freq, vss, distance_per_VSS_pulse, pulses_per_km, VSS_constant;
int      vspeed, last_vspeed, barspeed, last_barspeed;
uint32_t period, lowtime, hightime, pulsein_timeout, period_min;

// Battery Voltage variables for "Power Good"
int Converted_Safe_Voltage, Raw_Battery_Volts;

// Distance variables
// these in meters or kilometers as indicated
float avg_vspeed, Dist_M, Dist_Interval_M, Dist_Total_M, Dist_KM;
float Odometer_Temp;
// Odo values read and writen to eeprom
// these in kilometers
uint32_t Odometer_Total, Odometer_Verify, Odometer1, Odometer2;
// Flags to check valid eeprom reads
bool Odo1_Good, Odo2_Good, Chk_Good;
// Range of valid odometer values
const uint32_t Odometer_Min = 101428;
const uint32_t Odometer_Max = 999999;

// SD Card variables
bool     SD_Present;
char     SD_Filename[] = "Odometer.dat";
File     SD_DataFile;
uint32_t OdometerSD, Temp_SD, SD_size;

// Position on display
const int speed_x   = 480 / 2 - 195;    // horizontal, long side
const int speed_y   = 320 / 2 - 50;     // vertical, short side
const int totals_x  = 480 / 2;
const int totals_y  = 320 - 50;
const int spinner_x = 10;
const int spinner_y = 320 / 2 + 50;
const int alert_x   = 480 / 2 + 120;
const int alert_y   = 320 / 2 - 35;

// Meter variables
const int barMargin = 10;
const int barLength = 480 - (2 * barMargin);        // horizontal or long side
const int barWidth  = 320 / 5;                      // vertical or short side
const int meterMin = 0, meterMax = Max_barspeed;    // bar scale
int       blankBarValue, colouredBarValue, block_colour;
int       linearBarX = barMargin;
int       linearBarY = barMargin;    // bar starting position in pixels

// Start a Class for each Odomoter and Check
EEWL Odo1_EEPROM(Odometer1, BUFFER_LEN, BUFFER_START1);
EEWL Odo2_EEPROM(Odometer2, BUFFER_LEN, BUFFER_START2);
EEWL Check_EEPROM(Odometer_Verify, BUFFER_LEN, BUFFER_START3);



// ##################################################################################################################################################



void setup()
    {


    // Improved randomness for testing
    // Choose an unused analog pin
    unsigned long seed = 0, count = 32;
    while (--count)
        seed = (seed << 1) | (analogRead(A6) & 1);
    randomSeed(seed);

    // Outputs
    pinMode(Warning_Pin, OUTPUT);
    digitalWrite(Warning_Pin, !Valid_Warning);

    // Digital inputs
    // remove input_pullup's after testing
    // since pullups are handled by external hardware
    pinMode(Button_Pin, INPUT_PULLUP);
    pinMode(Pbrake_Input_Pin, INPUT_PULLUP);
    pinMode(Low_Beam_Pin, INPUT_PULLUP);
    pinMode(VSS_Input_Pin, INPUT);

    // Analog inputs
    pinMode(Batt_Volt_Pin, INPUT);

    //Init SD_Card
    pinMode(SD_Select, OUTPUT);

    // =======================================================
    // Recalculate the safe voltage value to raw analoge input value
    // This is to avoid performing the reverse calculation every
    // time there is a test for "power good" to improve loop rate
    Converted_Safe_Voltage = int(Safe_Voltage / Input_Multiplier + 0.5);
    // =======================================================

    // =======================================================
    // Calculate the distance travelled per VSS pulse
    // based on tyre size, diff ratio and VSS pulses per tailshaft revolution, in millimeters
    // Also calculate the VSS contant up front to avoid
    // doing so many divides every loop
    distance_per_VSS_pulse = tyre_dia * PI / diff_r / vss_rev;    // millimeters
    pulses_per_km          = 1000000.0 / distance_per_VSS_pulse;
    VSS_constant           = 3600000000.0 / pulses_per_km * Kludge_Factor;
    // based on:
    //freq = 1000000.0 / (float)period;
    //vss = 3600.0 * freq / pulses_per_km;
    //vss = 3600.0 * 1000000.0 / (float)period / pulses_per_km;
    // =======================================================

    // =======================================================
    // Calculate the pulseIn timeout in microseconds
    // Maximum time pulsein will wait for a signal
    // Use period of lowest expected frequency from
    // diff_r , tyre_dia , vss_rev and Min_vspeed
    pulsein_timeout = 1000000.0 / (pulses_per_km * Min_vspeed / 3600.0) * 2.0;
    // =======================================================

    // =======================================================
    // Calculate the minimum in microseconds
    // values beyond this would be considered noise or an error
    // this helps to prevent divide by zero errors
    // based on maximum speed and halve it
    period_min = 1000000.0 / (pulses_per_km * Max_vspeed / 3600.0) / 2.0;
    // =======================================================

    // Display important startup items
    myGLCD.InitLCD(LANDSCAPE);
    myGLCD.clrScr();
    myGLCD.setFont(font0);
    myGLCD.setColor(VGA_GRAY);
    myGLCD.setBackColor(VGA_BLACK);
     myGLCD.print(Version, CENTER, CENTER);

    // =======================================================
    // Try to access the SD Card
    if (!SD.begin(SD_Select))
        {
        SD_Present = false;
        }
    else
        {
        SD_Present = true;
        if (wipe_totals | !SD.exists(SD_Filename))
            // Write a new file
            // with the minimum value
            {
            SD_DataFile = SD.open(SD_Filename, FILE_WRITE);
            SD_DataFile.write((byte *)&Odometer_Min, sizeof(unsigned long));
            SD_DataFile.close();
            }
        else
            // Read the file
            {
            OdometerSD = read_SD();
            }
        }
    // =======================================================

    // =======================================================
    // Odometer read and verification below

    // Start the wear-leveling classes for EEPROM
    // this also initiates .fastFormat() on first use or no valid values
    // uncomment the fastformat line to force it
    Odo1_EEPROM.begin();
    if (wipe_totals)
        Odo1_EEPROM.fastFormat();
    Odo2_EEPROM.begin();
    if (wipe_totals)
        Odo2_EEPROM.fastFormat();
    Check_EEPROM.begin();
    if (wipe_totals)
        Check_EEPROM.fastFormat();

    // Try to read saved values
    // EEWL returns a bool status if the read is not successful
    if (!Odo1_EEPROM.get(Odometer1))
        {
        // odo1 read error
        Odometer1 = Odometer_Min;
        Odo1_Good = false;
        myGLCD.setColor(VGA_ORANGE);
        }
    else
        {
        // Odo1 read was good
        Odo1_Good = true;
        myGLCD.setColor(VGA_BLACK);
        if (Debug_Mode)
            {
            myGLCD.setColor(VGA_GREEN);
            }
        }
    // Print the Odo1 status with a suitable colour
     myGLCD.print("OD1 ", LEFT, 100);

    if (!Odo2_EEPROM.get(Odometer2))
        {
        // Odo2 read error
        Odometer2 = Odometer_Min;
        Odo2_Good = false;
        myGLCD.setColor(VGA_ORANGE);
        }
    else
        {
        // Odo2 read was good
        Odo2_Good = true;
        myGLCD.setColor(VGA_BLACK);
        if (Debug_Mode)
            {
            myGLCD.setColor(VGA_GREEN);
            }
        }
    // Print the Odo2 status with a suitable colour
     myGLCD.print("OD2 ", LEFT, 120);

    if (!Check_EEPROM.get(Odometer_Verify))
        {
        // Chk read error
        Odometer_Verify = ~Odometer_Min;
        Chk_Good        = false;
        myGLCD.setColor(VGA_ORANGE);
        }
    else
        {
        // Chk read was good
        Chk_Good = true;
        myGLCD.setColor(VGA_BLACK);
        if (Debug_Mode)
            {
            myGLCD.setColor(VGA_GREEN);
            }
        }
    // Print the Odo Chk status with a suitable colour
     myGLCD.print("Chk ", LEFT, 140);

    // Check all values are within acceptable range
    // even if successfully read from EEPROM
    // Reset them to minimum if required
    // and update their status
    if (Odometer1 < Odometer_Min || Odometer1 >= Odometer_Max)
        {
        Odometer1 = Odometer_Min;
        Odo1_Good = false;
        }
    if (Odometer2 < Odometer_Min || Odometer2 >= Odometer_Max)
        {
        Odometer2 = Odometer_Min;
        Odo2_Good = false;
        }
    if (~Odometer_Verify < Odometer_Min || ~Odometer_Verify >= Odometer_Max)
        {
        Odometer_Verify = ~Odometer_Min;
        Chk_Good        = false;
        }

    // All EEPROM reads successful
    if (Odo1_Good && Odo2_Good && Chk_Good)
        {
        myGLCD.setColor(VGA_GREEN);
         myGLCD.print("EEPROM good", CENTER, 140);
        // All values agree
        if (~Odometer_Verify == Odometer1 && Odometer1 == Odometer2)
            {
            Odometer_Total = ~Odometer_Verify;
             myGLCD.print("Values good", CENTER, 160);
            //------------------------------
            // Skip any further checks
            goto Odometer_Fixed;
            //------------------------------
            }
        else
            {
            myGLCD.setColor(VGA_YELLOW);
             myGLCD.print("Values mismatched", CENTER, 160);
            }
        }
    else
        {
        myGLCD.setColor(VGA_RED);
         myGLCD.print("EEPROM bad", CENTER, 160);
         myGLCD.print("Attempting", CENTER, 180);
         myGLCD.print("Recovery", CENTER, 200);
        }

    // Further Consistency checks are required
    // Choose the highest available good value
    // if there are two good values
    if (Chk_Good && Odo1_Good)
        {
        Odometer_Total = max(~Odometer_Verify, Odometer1);
        }
    if (Chk_Good && Odo2_Good)
        {
        Odometer_Total = max(~Odometer_Verify, Odometer2);
        }
    if (Odo1_Good && Odo2_Good)
        {
        Odometer_Total = max(Odometer1, Odometer2);
        }

    // Choose a good value if there is only one
    if (Chk_Good && !Odo1_Good && !Odo2_Good)
        Odometer_Total = ~Odometer_Verify;
    if (!Chk_Good && Odo1_Good && !Odo2_Good)
        Odometer_Total = Odometer1;
    if (!Chk_Good && !Odo1_Good && Odo2_Good)
        Odometer_Total = Odometer2;

    // There were no good values, choose the highest value anyway
    // This test may be not needed if all values were marked as bad
    // because they would all be reset to minimum, but try anyway
    if (!Chk_Good && !Odo1_Good && !Odo2_Good)
        {
        Odometer_Total = max(max(Odometer1, Odometer2), ~Odometer_Verify);
        }

    // Try the value stored on SD Card, if present
    if (SD_Present)
        // Read has already been successful
        {
        myGLCD.setColor(VGA_GREEN);
         myGLCD.print("SDC good", CENTER, 220);
        Odometer_Total = max(Odometer_Total, OdometerSD);
        }

Odometer_Fixed:
    // Odometer now fixed, or at least
    // contains the best/highest available good value
    // Recalc the Check value in case it doesnt match the updated Odo value
    Odometer_Verify = ~Odometer_Total;

    // Try to write the updated valid values back into EEPROM
    if (power_good())
        {
        Odo1_EEPROM.put(Odometer_Total);
        Odo2_EEPROM.put(Odometer_Total);
        Check_EEPROM.put(Odometer_Verify);
        Verify_Write();
        }
    // end of odometer section
    // =======================================================

    // Leave the important messages onscreen for a second
    delay(1500);

    // Now we are into sustained operation
    // Clear the screen and display static items
    myGLCD.clrScr();
    myGLCD.setFont(font1);
    myGLCD.setColor(VGA_GRAY);
    myGLCD.setBackColor(VGA_BLACK);
     myGLCD.print("km/h", totals_x + 120, totals_y - 60);

    // Display the stored Odometer value
    myGLCD.setFont(font7F);
    myGLCD.setColor(text_colour2);
    myGLCD.printNumI(Odometer_Total, totals_x - 210, totals_y, 6, ' ');

    // =======================================================
    // Draw small triangles at predetermined points
    // Bar goes from left to right
    // linearBarX + barLength - (Speed_Marker_1 / meterMax * barLength)
    int S1 = linearBarX + int((float)Speed_Marker_1 / (float)meterMax * (float)barLength);    // 1st Mark
    int S2 = linearBarX + int((float)Speed_Marker_2 / (float)meterMax * (float)barLength);    // 2nd Mark
    int S3 = linearBarX + int((float)Speed_Marker_3 / (float)meterMax * (float)barLength);    // 3rd Mark
    int S4 = linearBarX + int((float)Speed_Marker_4 / (float)meterMax * (float)barLength);    // 4th Mark
    myGLCD.setColor(VGA_RED);
    geo.fillTriangle(S1 - 4, linearBarY + barWidth + 10, S1 + 4, linearBarY + barWidth + 10, S1, linearBarY + barWidth + 5);
    geo.fillTriangle(S2 - 4, linearBarY + barWidth + 10, S2 + 4, linearBarY + barWidth + 10, S2, linearBarY + barWidth + 5);
    geo.fillTriangle(S3 - 4, linearBarY + barWidth + 10, S3 + 4, linearBarY + barWidth + 10, S3, linearBarY + barWidth + 5);
    geo.fillTriangle(S4 - 4, linearBarY + barWidth + 10, S4 + 4, linearBarY + barWidth + 10, S4, linearBarY + barWidth + 5);
    // =======================================================

    // Draw the bar background
    myGLCD.setColor(block_fill_colour);
    myGLCD.fillRect(linearBarX, linearBarY, linearBarX + barLength, linearBarY + barWidth);

    // Ensure parkbrake status is displayed correctly and not skipped
    Pbrake_last_status = !digitalRead(Pbrake_Input_Pin);

    // set initial timing value
    distLoopTime = millis();


    }    // End void setup



// ##################################################################################################################################################



void loop()
    {


    // =======================================================
    // Reset trip meter by button press
    // =======================================================

    if (digitalRead(Button_Pin) == Digitial_Input_Active)
        {
        // Allow time for the button pin to settle
        // this assumes some electronic/external debounce
        delay(10);
        if (digitalRead(Button_Pin) == Digitial_Input_Active)
            Dist_Total_M = 0;
        }


    // =======================================================
    // Verify the odometer was saved successfully
    // =======================================================

    // Only while the vehicle is not moving
    // and more than 10 minutes have elapsed
    if (vspeed == 0 && millis() >= odoCheckTime + odoCheckInterval)
        {
        odoCheckTime = millis();
        Verify_Write();
        }


    // =======================================================
    // Dim display when headlights on
    // =======================================================

    if (millis() > startup_time)
        {
        // Dim mode when headlights are on
        if (digitalRead(Low_Beam_Pin) == Digitial_Input_Active && !dim_mode)
            {
            dim_mode          = true;
            text_colour1      = VGA_SILVER;
            text_colour2      = VGA_GRAY;
            block_fill_colour = VGA_BLACK;
            }

        // Normal colours when headlights are off
        if (digitalRead(Low_Beam_Pin) == !Digitial_Input_Active && dim_mode)
            {
            dim_mode          = false;
            text_colour1      = VGA_WHITE;
            text_colour2      = VGA_SILVER;
            block_fill_colour = VGA_GRAY;
            }
        }


    // =======================================================
    // Audible warning if the vehicle is moving with Park Brake on
    // =======================================================

    // Only change the diplay if the Park Brake status has changed
    if (digitalRead(Pbrake_Input_Pin) != Pbrake_last_status)
        {
        if (Pbrake_last_status == !Digitial_Input_Active)
            {
            // Park Brake is ON, it just changed to "active"
            // Set the colours for a red "P"
            myGLCD.setColor(VGA_BLACK);
            myGLCD.setBackColor(VGA_RED);
            }
        else
            {
            // Park Brake is OFF
            // Set the colours for a black a "P"
            myGLCD.setColor(VGA_BLACK);
            myGLCD.setBackColor(VGA_BLACK);
            }
        // Do the actual display
        myGLCD.setFont(font7F);
         myGLCD.print("P", alert_x, alert_y);
        Pbrake_last_status = !Pbrake_last_status;
        }


    // =======================================================
    // Input from the vehicle speed sensor
    // =======================================================

    if (!Demo_Mode)
        {
        // Read the real VSS sensor
        hightime = pulseIn(VSS_Input_Pin, HIGH, pulsein_timeout);
        lowtime  = pulseIn(VSS_Input_Pin, LOW, pulsein_timeout);
        period   = hightime + lowtime;
        // For testing
        //period   = random(period_min, pulsein_timeout);

        // prevent overflows or divide by zero
        if (period > period_min)
            {
            vss = VSS_constant / (float)period;
            // Advance the spinner
            myGLCD.setFont(font1);
            myGLCD.setColor(text_colour2);
            myGLCD.setBackColor(VGA_BLACK);
            switch (spinnerState)
                {
                case 1:
                    // "|"
                    myGLCD.print("|", spinner_x, spinner_y);
                    spinnerState++;
                    break;
                case 2:
                    // "/"
                    myGLCD.print("/", spinner_x, spinner_y);
                    spinnerState++;
                    break;
                case 3:
                    // "_" 95
                    // print a space then an underscore shifted higher
                    // a minus sign is too narrow
                    myGLCD.print(" ", spinner_x, spinner_y);
                    myGLCD.print("_", spinner_x, spinner_y - 10);
                    spinnerState++;
                    break;
                case 4:
                    // "\"
                    // need to "eascape" this character for it to print
                    // and not interfere with the compiler
                    myGLCD.print("\\", spinner_x, spinner_y);
                    spinnerState++;
                    break;
                case 5:
                    // "|"
                    myGLCD.print("|", spinner_x, spinner_y);
                    spinnerState++;
                    break;
                case 6:
                    // "/"
                    myGLCD.print("/", spinner_x, spinner_y);
                    spinnerState++;
                    break;
                case 7:
                    // "_" 95
                    // print a space then an underscore shifted higher
                    // a minus sign is too narrow
                    myGLCD.print(" ", spinner_x, spinner_y);
                    myGLCD.print("_", spinner_x, spinner_y - 10);
                    spinnerState++;
                    break;
                case 8:
                    // "\"
                    // need to "eascape" this character for it to print
                    // and not interfere with the compiler
                    myGLCD.print("\\", spinner_x, spinner_y);
                    spinnerState = 1;
                    break;
                }
            }
        else
            {
            vss = 0.0;
            }
        }
    else
        {
        // Demo mode, invent values
        vss = random(5, Max_vspeed);
        vss = constrain(vss, last_vspeed - 20, last_vspeed + 20);
        //vss = 40;
        }

    // Set vspeed from the float vss
    vspeed = int(vss + 0.5);
    // Limit the vspeed range
    // A reluctor pickup is not accurate below 4km/hr
    if (vspeed < Min_vspeed)
        {
        vspeed = 0;
        }
    else if (vspeed > int(Max_vspeed))
        vspeed = int(Max_vspeed);


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
    distIntTime  = (millis() - distLoopTime);
    distLoopTime = millis();

    // Calc meters travelled in this interval
    // removed redunant *1000 / 1000
    Dist_Interval_M = (avg_vspeed / 3600.0) * (float)distIntTime;

    // Add up total meters
    Dist_Total_M += Dist_Interval_M;

    // Convert to kilometers
    Dist_KM = Dist_Total_M / 1000.0;

    // Display the trip distance each loop
    myGLCD.setFont(font7F);
    myGLCD.setColor(text_colour2);
    myGLCD.setBackColor(VGA_BLACK);
    myGLCD.printNumF(Dist_KM, 2, totals_x + 20, totals_y, '.', 6, ' ');


    // =======================================================
    // Update the Odometer and Check value
    // =======================================================

    // add to Odometer
    // keep it in meters until saving to odometer
    Odometer_Temp += Dist_Interval_M;

    // If 1 km or more has been accumulated write it
    // to the permenant odo and reset the temporary value
    if (Odometer_Temp >= 1000)
        {
        Odometer_Total += int(Odometer_Temp / 1000.0 + 0.5);

        // Display the new Odo total since the value has just updated
        myGLCD.printNumI(Odometer_Total, totals_x - 210, totals_y, 6, ' ');

        // Write Odometer_Total to EEPROM only if Power is good
        if (power_good())
            {
            Odo1_EEPROM.put(Odometer_Total);
            Odo2_EEPROM.put(Odometer_Total);
            }

        // Calculate and set the Check value
        // dont bother testing for power good a second time
        // in such a short span of time
        Odometer_Verify = ~Odometer_Total;
        Check_EEPROM.put(Odometer_Verify);

        // Only verify writes in debug mode
        // and odo value has just changed
        if (Debug_Mode)
            Verify_Write();

        // Odometertemp has reached a 1000m or more and been added to total
        // reset it and start counting again for the next 1km
        Odometer_Temp = 0;

        }    // end if update odometer +1000m


    // =======================================================
    // Display the speed
    // =======================================================

    myGLCD.setFont(font7L);
    myGLCD.setBackColor(VGA_BLACK);
    if (vspeed <= 99)
        {
        // Print a black digit 8 where a rogue digit would be
        // because this large font only has digits, no space char
        if (last_vspeed > 99)
            {
            myGLCD.setColor(VGA_BLACK);
            myGLCD.printNumI(8, speed_x, speed_y, 1, '0');
            }
        // Print the 2 digit speed
        myGLCD.setColor(text_colour1);
        myGLCD.printNumI(vspeed, speed_x + 96, speed_y, 2, '0');
        }
    else
        {
        // Print the 3 digit speed
        myGLCD.setColor(text_colour1);
        myGLCD.printNumI(vspeed, speed_x, speed_y, 3, '0');
        }

    // =======================================================
    // Draw the barmeter
    // =======================================================

    // Limit the speed to what the bar meter can handle
    // even though the printed digits might be a greater value
    barspeed         = constrain(vspeed, meterMin, meterMax);

    colouredBarValue = map(barspeed, meterMin, meterMax, 0, barLength);

    if (!dim_mode)
        {
        // All colours available fir bright mode
        // Choose colour scheme using the Rainbow function
        // uncomment one line
        //block_colour = TFT_RED;    // Fixed colour
        //block_colour = TFT_GREEN;  // Fixed colour
        //block_colour = TFT_NAVY;   // Fixed colour
        //block_colour = rainbow(map(barspeed, meterMin, meterMax, 0, 127));   // Blue to red
        block_colour = rainbow(map(barspeed, meterMin, meterMax, 63, 127));    // Green to red
        //block_colour = rainbow(map(barspeed, meterMin, meterMax, 127, 63));  // Red to green
        //block_colour = rainbow(map(barspeed, meterMin, meterMax, 127, 0));   // Red to blue
        }
    else
        {
        // Dim mode, headlingts on
        block_colour = text_colour2;
        }

    // The bar goes from left to right

    // Fill in coloured blocks
    // This is the foreground colour of the bar graph
    myGLCD.setColor(block_colour);
    myGLCD.fillRect(linearBarX, linearBarY, linearBarX + colouredBarValue, linearBarY + barWidth);

    // Fill in blank segments
    // This is the background colour of the bar graph
    if (last_barspeed > barspeed)
        {
        myGLCD.setColor(block_fill_colour);
        myGLCD.fillRect(linearBarX + colouredBarValue, linearBarY, linearBarX + barLength, linearBarY + barWidth);
        }

    last_barspeed = barspeed;
    last_vspeed   = vspeed;


    }    // End void loop



// ##################################################################################################################################################
// ##################################################################################################################################################



// =======================================================
// Reusable functions
// =======================================================


// Set the power good status
// in this case we dont need a dummy read of the analog pin to settle the value
// since this is the only analog input used in this sketch
// The raw input value is used to save calculations
// and the threshold is calculated in Setup
bool power_good()
    {
    Raw_Battery_Volts = analogRead(Batt_Volt_Pin);
    return (Raw_Battery_Volts >= Converted_Safe_Voltage);
    }


// ------------------------------------------------------


// Read from SD Card
uint32_t read_SD()
    {
    myGLCD.setFont(font0);
    myGLCD.setColor(VGA_BLACK);
    myGLCD.setBackColor(VGA_BLACK);
    SD_DataFile = SD.open(SD_Filename, FILE_READ);
    if (!SD_DataFile)
        // Unable to open file
        // Ignore the SD Card from now on
        {
        SD_Present = false;
        Temp_SD    = 0;
        myGLCD.setColor(VGA_ORANGE);
        }
    else
        {
        // Read the required number of bytes
        SD_DataFile.seek(SD_DataFile.size() - sizeof(Temp_SD));
        SD_DataFile.read((byte *)&Temp_SD, sizeof(Temp_SD));
        SD_DataFile.close();
        if (Debug_Mode)
            {
            myGLCD.setColor(VGA_GREEN);
            }
        if (Temp_SD < Odometer_Min || Temp_SD >= Odometer_Max)
            // Value out of range, fix it
            {
            OdometerSD = Odometer_Min;
            myGLCD.setColor(VGA_YELLOW);
            }
        }
    // This should only appear if there is a problem, or debug mode
     myGLCD.print("SDC ", LEFT, 160);
    return Temp_SD;
    }


// ------------------------------------------------------


// Write to SD Card
uint32_t update_SD(uint32_t Temp_SD)
    {
    myGLCD.setFont(font0);
    myGLCD.setColor(VGA_BLACK);
    myGLCD.setBackColor(VGA_BLACK);
    SD_DataFile = SD.open(SD_Filename, FILE_WRITE);
    if (!SD_DataFile)
        // Unable to open file
        // Ignore the SD Card from now on
        {
        SD_Present = false;
        myGLCD.setColor(VGA_ORANGE);
        }
    else
        {
        // Append the last four bytes
        SD_DataFile.seek(SD_DataFile.size());
        SD_DataFile.write((byte *)&Temp_SD, sizeof(unsigned long));
        SD_DataFile.close();
        if (Debug_Mode)
            {
            myGLCD.setColor(VGA_GREEN);
            }
        }
    // This should only appear if there is a problem, or debug mode
     myGLCD.print("SDC ", LEFT, 160);
    }


// ------------------------------------------------------


// Function to return a 16 bit rainbow colour
unsigned int rainbow(byte value)
    {
    // Value is expected to be in range 0-127
    // The value is converted to a spectrum colour from 0 = blue through to 127 = red

    byte red      = 0;    // Red is the top 5 bits of a 16 bit colour value
    byte green    = 0;    // Green is the middle 6 bits
    byte blue     = 0;    // Blue is the bottom 5 bits

    byte quadrant = value / 32;

    if (quadrant == 0)
        {
        blue  = 31;
        green = 2 * (value % 32);
        red   = 0;
        }
    if (quadrant == 1)
        {
        blue  = 31 - (value % 32);
        green = 63;
        red   = 0;
        }
    if (quadrant == 2)
        {
        blue  = 0;
        green = 63;
        red   = value % 32;
        }
    if (quadrant == 3)
        {
        blue  = 0;
        green = 63 - 2 * (value % 32);
        red   = 31;
        }
    return (red << 11) + (green << 5) + blue;
    }



// ##################################################################################################################################################



void Verify_Write()
    {

    // The checks that determine if this routine is called
    // are performed at the calling location
    // for example if vehicle speed = 0 or debug mode

    // only if power is good
    if (power_good())
        {

        if (SD_Present)
            {
            // Write latest Odometer total to SD Card
            update_SD(Odometer_Total);
            }

        myGLCD.setFont(font0);
        myGLCD.setBackColor(VGA_BLACK);

        // Read the values stored in EEPROM
        Odo1_EEPROM.get(Odometer1);
        Odo2_EEPROM.get(Odometer2);
        Check_EEPROM.get(Odometer_Verify);

        // Odometer_Total is still the live running total
        // to be compared against
        if (Odometer1 != Odometer_Total)
            {
            // Try to write Odo1 again
            // and indicate a warning
            Odo1_EEPROM.put(Odometer_Total);
            myGLCD.setColor(VGA_ORANGE);
            }
        else
            {
            if (Debug_Mode)
                {
                // Highlight a good result
                myGLCD.setColor(VGA_GREEN);
                }
            else
                {
                // Hide the previous result
                myGLCD.setColor(VGA_BLACK);
                }
            }
         myGLCD.print("OD1 ", LEFT, 100);

        if (Odometer2 != Odometer_Total)
            {
            // Try to write Odo2 again
            // and indicate a warning
            Odo2_EEPROM.put(Odometer_Total);
            myGLCD.setColor(VGA_ORANGE);
            }
        else
            {
            if (Debug_Mode)
                {
                // Highlight a good result
                myGLCD.setColor(VGA_GREEN);
                }
            else
                {
                // Hide the previous result
                myGLCD.setColor(VGA_BLACK);
                }
            }
         myGLCD.print("OD2 ", LEFT, 120);

        if (Odometer_Verify != ~Odometer_Total)
            {
            // Try to write Verify again
            // and indicate a warning
            Odometer_Verify = ~Odometer_Total;
            Check_EEPROM.put(Odometer_Verify);
            myGLCD.setColor(VGA_ORANGE);
            }
        else
            {
            if (Debug_Mode)
                {
                // Highlight a good result
                myGLCD.setColor(VGA_GREEN);
                }
            else
                {
                // Hide the previous result
                myGLCD.setColor(VGA_BLACK);
                }
            }
         myGLCD.print("Chk ", LEFT, 140);

        if (Debug_Mode)
            {
            // These should produce zero
            myGLCD.setColor(VGA_GRAY);
            myGLCD.printNumI((Odometer_Total - Odometer1), LEFT, 180, 3, ' ');
            myGLCD.printNumI((Odometer_Total - Odometer2), LEFT, 200, 3, ' ');
            }

        }    //  end if power good

    }    //  end void Verify_Write()



// ##################################################################################################################################################
// ##################################################################################################################################################
