/*
Over The Top (OTT) Servo Tester
===============================

Hardware Requirements
---------------------
* Arduino processor (Pro Mini, but any processor should work)
* Hitachi 1602 LCD display with IIC backpack
* Rotary encoder with built-in momentary-on switch
* Additional momentary-on switch

I/O Mapping - Arduino Pro Mini
------------------------------
+---+-----------------------+-------------------+
|Pin| Usage                 | Defined           |
+---+-----------------------+-------------------+
| 00| USB Serial            | N/A               |
| 01| USB Serial            | N/A               |
| 02| Servo Output A        | SERVO_PIN[0]      |
| 03| Servo Output B        | SERVO_PIN[1]      |
| 04| Servo Output C        | SERVO_PIN[2]      |
| 05| Servo Output D        | SERVO_PIN[3]      |
| 06| Servo Output E        | SERVO_PIN[4]      |
| 07| Servo Output F        | SERVO_PIN[5]      |
| 08|                       |                   |
| 09|                       |                   |
| 10|                       |                   |
| 11| Receiver Signal Pin   | RCV_PIN           |
| 12|                       |                   |
| 13|                       |                   |
| A0| Menu encoder B        | MENU_ENC_B        |
| A1| Menu encoder A        | MENU_ENC_A        |
| A2| Menu encoder select   | MENU_SEL          |
| A3| Run switch            | MENU_RUN          |
| A4| IIC SDA               | LCD_SDA           |
| A5| IID SCL               | LCD_SCL           |
| A6|                       |                   |
| A7|                       |                   |
+---+-----------------------+-------------------+

Summary
-------
The Application operates in one of three modes, cycled through a double press of the encoder switch.

Manual: Manual mode is shown by 'Man' in the top right of the display. Moving the encoder moves all
        the servos in unison. A single press of the Run switch resets all the servos to their home
        position, as configured in the associated profile.

Sweep:  Sweep mode display shows the same information as Manual mode display, with 'Swp' in the top right.
        In this mode the servos are swept between their associated profile start and end positions using
        the defined sweep profile (Linear, Sine or Square) and timing parameters. The sweep is toggled
        on/off by a single press of the Run switch.

Rcv Check: Rcv Check displays the signal characteristics from a servo controller connected to the RCV_PIN
        input. Signal high and low durations are displayed in real time.

The application code is configured to allow for up to MAX_SERVO servos to be attached for testing.
The actual number of active servos is set by the NUM_SERVO (<= MAX_SERVO) constant. Each of the 
Servo outputs (labeled Output A through F) is associated with a servo profile. 

Each servo has associated a profile defining the travel timing limits for the servo. Up to MAX_PROFILE
profiles are allowed for. The actual number of profiles is set by the NUM_PROFILE constant. Note the 
variable selProfile needs to be manually adjusted for menu selections if the value of NUM_PROFILE is 
changed.

The servo profiles are configurable and define the minimum and maximum values for the servo sweep 
(in microseconds), the percentage of range to set as the 'mid-point' value, the type of sweep testing 
(Linear, Sine or Square patterns), the sweep duration and the pause between sweeps. By default three 
profiles are created with range 1000 to 2000 microseconds and 'mid' point (or 'home') values at 50%, 
0% and 100% for profiles 1, 2 and 3 respectively. An output with no associated profile is considered 
as 'disabled' and will not receive control signals.

The user interface (UI) is implemented using the rotary encoder and a separate tact switch (Run switch).
In general the UI navigation operates as follows:
 * Cycle between modes by double pressing the encoder switch.
 * The Run Switch is used to run/stop execution in the mode (if applicable).
 * The Configuration Menu only uses the encoder, accessed from Manual mode using a long press of the 
   encoder switch.
     - The top line of the display is the menu/submenu name or the value being edited.
     - Below the title, Menu Selections are displayed inside angled brackets ("<" and ">").
     - Editable values are displayed inside square brackets ("[" and "]").
     - Menu options are scrolled by turning the encoder
     - Menu options are selected by pressing the encoder switch.
     - Configuration item values are changed up/down by turning the encoder switch
     - Configuration items are confirmed by pressing the encoder switch.
     - Aborting the current menu level (menu selection or value editing) is done using a long press.
     
The application stores all its configuration information in EEPROM so that it is retained between 
processor resets. Configuration items include
* Display parameters for splash screen, menu timeouts and auto paging
* Safety limits on servo timing values
* Profile definitions (as described above)
* Assignment of profile to servo

Library Dependencies
--------------------
- Rotary encoder (MD_REncoder) at https://github.com/MajicDesigns/MD_REncoder or Arduino IDE Library Manager
- Menu switches (MD_UISwitch) at https://github.com/MajicDesigns/MD_UISwitch or Arduino IDE Library Manager
- LCD Menu (MD_Menu) at https://github.com/MajicDesigns/MD_Menu or Arduino IDE Librarey Manager
- LCD Manager (HD44780) at https://github.com/duinoWitchery/hd44780 or Arduino IDE Library Manager

Version History
---------------
Nov 2020 v1.0.0
- First Release

Copyright and Licensing
-----------------------
Copyright (C) 2020 Marco Colli. All rights reserved.

This is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this software; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
*/
#include <EEPROM.h>
#include <Servo.h>

#include <Wire.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>

#include <MD_REncoder.h>
#include <MD_UISwitch.h>
#include <MD_Menu.h>

// Application options
const uint8_t MAX_SERVO = 6;   // Maximum number of servos allowed for. NUM_SERVO sets the actual number (below).

#define DEBUG  0   // enable debugging output
#define USE_RCV 1  // enable the Receiver functionality

#if DEBUG
#define PRINT(s, n)  do { Serial.print(F(s)); Serial.print(n); } while (false);
#define PRINTS(s)    do { Serial.print(F(s)); } while (false);
#define PRINTX(s, x)  do { Serial.print(F(s)); Serial.print(F("x")); Serial.print(x, HEX); }  while (false);
#else
#define PRINT(s, n)
#define PRINTS(s)
#define PRINTX(s, x)
#endif // DEBUG

//----------------------------------------------------------
// Miscellaneous definitions/variables
#define APP_NAME "OTT Servo Tester"
#define APP_VER  "v1.0.0"

typedef const char PROGMEM message_t;	// pointer to flash memory string

enum:uint8_t 
{ 
  RUN_MANUAL_INIT, RUN_MANUAL,
  RUN_SWEEP_INIT, RUN_SWEEP_WAITING, RUN_SWEEP,
#if USE_RCV
  RUN_RCHECK_INIT, RUN_RCHECK, RUN_RCHECK_PAUSE,
#endif
  RUN_MENU_INIT, RUN_MENU
} runMode = RUN_MANUAL_INIT;
bool dataChanged = true;    // flag for new data to display

//----------------------------------------------------------
// Servo Hardware
const uint16_t SERVO_UPDATE_FREQ = 50;       // standard servo update frequency in Hz
const uint16_t SERVO_LOWER_HARDLIMIT = 500;  // servo abs lower hard limit
const uint16_t SERVO_UPPER_HARDLIMIT = 2500; // servo abs upper hard limit

const uint8_t NUM_SERVO = MAX_SERVO;         // number of physical servos (can be [1..MAX_SERVO-1])
const uint8_t SERVO_PIN[MAX_SERVO] = { 2, 3, 4, 5, 6, 7 };  // only NUM_SERVO are relevant

const uint8_t NUM_PROFILE = 3;       // number of available profiles - also adjust selProfile below
message_t selProfile[] = "-|1|2|3";  // menu selection data for profile - adjust to allow NUM_PROFILE selections

message_t selSweepType[] = "Lin|Sin|Sqr";  // menu selection data - for sweep type in profile
const uint8_t SWP_LINEAR = 0;   // index 0
const uint8_t SWP_SINE = 1;     // index 1
const uint8_t SWP_SQUARE = 2;   // index 2

Servo servo[NUM_SERVO];

#if USE_RCV
//----------------------------------------------------------
// Receiver Signal Test
const uint8_t RCV_PIN = 11;
const uint32_t RCV_PAUSE_TIME = 500; // milliseconds pause between readings

struct
{
  uint32_t timeSig;  // signal duration measured (microseconds)
  uint32_t timeGap;  // space duration measured (microseconds)
  uint32_t timePause;    // receiver pause timer counter (milliseconds)
} rcvCheckData;
#endif

//----------------------------------------------------------
// Menu Rotary Encoder Interface I/O
const uint8_t MENU_SEL = A2;   // Selection switch on encoder
const uint8_t MENU_RUN = A3;   // Run start/stop
const uint8_t MENU_ENC_A = A1; // Encode Channel A
const uint8_t MENU_ENC_B = A0; // Encoder Channel B

MD_REncoder encMenu(MENU_ENC_A, MENU_ENC_B);
MD_UISwitch_Digital swMenuSel(MENU_SEL);
MD_UISwitch_Digital swMenuMode(MENU_RUN);
uint8_t displayPageBase = 0;   // base index for servo disaply page

//----------------------------------------------------------
// LCD Display definitions (2 wire display interface)
#define LCD_PRINT(c, r, s) do { lcd.setCursor(c,r); lcd.print(s); } while (false);
#define LCD_WRITE(c, r, s) do { lcd.setCursor(c,r); lcd.write(s); } while (false);

const uint8_t LCD_ROWS = 2;
const uint8_t LCD_COLS = 16;

const uint8_t LCD_SCL = A5;
const uint8_t LCD_SDA = A4;

hd44780_I2Cexp lcd;

//----------------------------------------------------------
// Auto paging management definitions
const uint32_t PAGE_TIME = 2000;   // page displayed time in ms for auto paging
uint32_t timerPage;     // keeping track of millis() time

//----------------------------------------------------------
// Configuration Information handling
//
// - Config info is made up of common data and additional data per channel.
// - This config info is saved at the start of EEPROM memory (address EEPROM_ADDR).
//
// Any changes to data structure need to be reflected in the static initializers in 
// the reset() method.
//
class cAppConfig
{
public:
  // Data fields --------------------------------
  struct
  {
    uint8_t signature[2];

    // UI options
    uint8_t		tmMenu;			// menu timeout in seconds
    uint8_t		tmSplash;		// splash screen display time in seconds
    bool      autoPage;   // automatically page through the main display

    // Safety limits for signal setting
    uint16_t  limLow;     // low limit in milliseconds
    uint16_t  limHigh;    // high limit in milliseconds

    // Servo Signal options
    struct 
    {
      bool enabled;       // true if servo is enabled for output
      uint8_t profileId;  // configured profile to use
    } servo[MAX_SERVO];
    
    // Profile information
    struct 
    {
      uint16_t low;        // setting for 0% of range
      uint16_t high;       // setting for 100% of range
      uint8_t  home;       // setting for home position as percentage
      uint8_t  sweepType;  // sweep type
      uint16_t sweepTime;  // duration in milliseconds for one low->high or high->low sweep
      uint16_t sweepPause; // pause between sweep movements in milliseconds
    } profile[NUM_PROFILE];
  } data;

  // Methods ------------------------------------
  cAppConfig(uint16_t eepromBase = 0) : _eepromBase(eepromBase) {}

  void reset(void)
  {
    PRINTS("\nConfig Reset");

    data.signature[0] = EEPROM_SIG[0];
    data.signature[1] = EEPROM_SIG[1];

    // Global config presets 
    data.tmMenu = 4;      // menu timeout in seconds
    data.tmSplash = 2;    // slapsh screen timeout in seconds

    // Safety limits 
    data.limLow = SERVO_LOWER_HARDLIMIT;    // low limit in microseconds
    data.limHigh = SERVO_UPPER_HARDLIMIT;   // high limit in microseconds
    data.autoPage = true;                   // default to autopaging

    // Servo information
    for (uint8_t i = 0; i < MAX_SERVO; i++)
    {
      data.servo[i].enabled = true;
      data.servo[i].profileId = 0;    // all use the same profile initially
    }

    // Profiles information
    for (uint8_t i = 0; i < NUM_PROFILE; i++)
    {
      data.profile[i].low = 1000;   // in microseconds
      data.profile[i].high = 2000;  // in microseconds
      data.profile[i].home = 0;     // percentage of range
      data.profile[i].sweepType = SWP_LINEAR; // one of the 
      data.profile[i].sweepTime = 2000;  // in milliseconds
      data.profile[i].sweepPause = 200;  // in milliseconds
    }
    data.profile[0].home = 50;    // set specific percentage value
    data.profile[1].home = 0;
    data.profile[2].home = 100;
  }

  void begin(void)
  {
    if (!load())
    {
      reset();
      save();
    }
  }

  void save(void) 
  { 
    PRINTS("\nConfig Save");
    EEPROM.put(_eepromBase, data);
  }
  
  bool load(void) 
  {
    PRINTS("\nConfig Load");

    EEPROM.get(_eepromBase, data);

    if (data.signature[0] != EEPROM_SIG[0] || data.signature[1] != EEPROM_SIG[1])
      return(false);

    return(true);
  }

protected:
  const uint16_t EEPROM_ADDR = 0;
  const uint8_t EEPROM_SIG[2] = { 0x11, 0xee };

  uint16_t  _eepromBase;
};

cAppConfig C;

// --------------------------------------------------------
// Servo real-time management data
struct
{
  uint16_t curV, setV;   // current and setpoit value setting
  uint32_t timeStart;    // sweep start in microseconds
  uint16_t timeDelta;    // in microseconds
  int8_t   deltaPct;     // percentage increment +/-
  int16_t  sweepPct;     // Current percentage through
  uint8_t  state;        // Current sweep ruinning state
} servoData[NUM_SERVO];

// ---------------------------------------------------------
// Miscellaneous functions
void setServoSPPct(uint8_t s, uint8_t pct)
{
  // sanity checks
  if (s >= NUM_SERVO) return;
  if (pct > 100) pct = 100;

  // do the deed
  uint8_t id = C.data.servo[s].profileId;
  uint32_t offset = C.data.profile[id].high - C.data.profile[id].low;

  offset = (offset * pct) / 100L;
  //PRINT("\nPct ", pct);
  //PRINT(" range ", C.data.profile[id].high - C.data.profile[id].low);
  //PRINT(" = set Offset ", offset);
  servoData[s].setV = C.data.profile[id].low + offset;
}

void setAllServoHome(void)
{
  for (uint8_t i = 0; i < NUM_SERVO; i++)
    if (C.data.servo[i].enabled)
      setServoSPPct(i, C.data.profile[C.data.servo[i].profileId].home);
}

void doServoSweep(uint8_t s, bool restart = false)
{
  if (restart)
  {
    servoData[s].state = 0;
    return;
  }

  uint8_t id = C.data.servo[s].profileId;

  switch (servoData[s].state)
  {
  case 0: // IDLE
    switch (C.data.profile[id].sweepType)
    {
    case SWP_LINEAR:
    case SWP_SINE:
      servoData[s].timeDelta = 1000 / SERVO_UPDATE_FREQ;
      servoData[s].deltaPct = (100 * servoData[s].timeDelta) / C.data.profile[id].sweepTime;
      break;

    case SWP_SQUARE:
      servoData[s].timeDelta = C.data.profile[id].sweepTime;
      servoData[s].deltaPct = 100;
      break;

    }
    //PRINT("\n[", s);
    //PRINT("] Time Delta ", servoData[s].timeDelta);
    //PRINT(" Delta Pct ", servoData[s].deltaPct);
    servoData[s].timeStart = 0;   // force the first update
    servoData[s].state = 1;       // start sweeping
    break;
    
  case 1: // SWEEPING
    if (millis() - servoData[s].timeStart >= servoData[s].timeDelta)
    {
      // now one deltaPCT advanced in the sweep
      servoData[s].sweepPct += servoData[s].deltaPct;
      //PRINT("\n[", s);
      //PRINT("] Current % ", servoData[s].sweepPct);
      if (servoData[s].sweepPct > 100) servoData[s].sweepPct = 100;
      if (servoData[s].sweepPct < 0) servoData[s].sweepPct = 0;
      //PRINT(" adjusted to % ", servoData[s].sweepPct);

      // Work out the servo setpoint value setting based on pct through and type of sweep
      switch (C.data.profile[C.data.servo[s].profileId].sweepType)
      {
      case SWP_LINEAR:
      case SWP_SQUARE:
        setServoSPPct(s, servoData[s].sweepPct);
        break;

      case SWP_SINE:
        servoData[s].setV = C.data.profile[id].low;
        servoData[s].setV += sin((PI * servoData[s].sweepPct) / 100) * (C.data.profile[id].high - C.data.profile[id].low);
        //PRINT("\nSINE: Current V ", servoData[s].setV);
        break;
      }

      // reached the end of the sweep?
      if ((servoData[s].sweepPct >= 100 && servoData[s].deltaPct > 0) ||
          (servoData[s].sweepPct == 0 && servoData[s].deltaPct < 0))
      {
        PRINT("\n[", s);
        PRINTS("] SWEEP COMPLETE");
        servoData[s].state = 2;    // time to wait for the pause time between sweeps
        servoData[s].deltaPct = -servoData[s].deltaPct;   // reverse direction next run
        servoData[s].timeStart = millis();
      }
      else
      {
        // next time to do something in this state
        servoData[s].timeStart = servoData[s].timeStart + servoData[s].timeDelta;
      }
    }
    break;
    
  case 2:  // PAUSING
    if (millis() - servoData[s].timeStart >= C.data.profile[id].sweepPause)
    {
      PRINT("\n[", s);
      PRINTS("] PAUSE COMPLETE");
      servoData[s].state = 1;
      servoData[s].timeStart = millis();
    }
    break;
  }
}

void nextPage(void)
// set up the display to be for the next page
{
  displayPageBase += LCD_ROWS;
  if (displayPageBase >= NUM_SERVO)
    displayPageBase = 0;      // roll over to start
  // PRINT("Display Base = ", displayPageBase);
  dataChanged = true;
}

//----------------------------------------------------------
// Menu handling data and functions prototypes
MD_Menu::value_t vBuf;        // interface buffer for value editing

// Menu callback functions
MD_Menu::value_t* saveConfig(MD_Menu::mnuId_t id, bool bGet)  { C.save(); return(nullptr); }
MD_Menu::value_t* loadConfig(MD_Menu::mnuId_t id, bool bGet)  { C.load(); return(nullptr); }
MD_Menu::value_t* resetConfig(MD_Menu::mnuId_t id, bool bGet) { C.reset(); return(nullptr); }

bool menuDisplay(MD_Menu::userDisplayAction_t action, char* msg = nullptr)
// Handles all aspects of output display to the 2 line LCD.
// Called by the menu library. 
{
  static char szLine[LCD_COLS + 1] = { '\0' };

  switch (action)
  {
  case MD_Menu::DISP_INIT:
    lcd.begin(LCD_COLS, LCD_ROWS);
    memset(szLine, ' ', LCD_COLS);
    PRINTS("\nINIT");
    break;

  case MD_Menu::DISP_CLEAR:
    lcd.clear();
    PRINTS("\nCLR");
    break;

  case MD_Menu::DISP_L0:
  case MD_Menu::DISP_L1:
  {
    uint8_t line = (action == MD_Menu::DISP_L0 ? 0 : 1);

    lcd.setCursor(0, line);
    lcd.print(szLine);
    lcd.setCursor(0, line);
    lcd.print(msg);
    PRINT("\nL", line);
    PRINT(":", msg);

  }
  break;
  }

  return(true);
}

MD_Menu::userNavAction_t menuNav(uint16_t& incDelta)
// Read the menu encoder. 
// This is called only from the menu system as a callback.
{
  MD_UISwitch::keyResult_t	k;
  MD_Menu::userNavAction_t r = MD_Menu::NAV_NULL;

  incDelta = 1;

  // check the switch first and it gets priority
  if ((k = swMenuSel.read()) != MD_UISwitch::KEY_NULL)
  {
    switch (k)
    {
    case MD_UISwitch::KEY_PRESS:  r = MD_Menu::NAV_SEL; break;
    case MD_UISwitch::KEY_LONGPRESS: r = MD_Menu::NAV_ESC;  break;
    default: break;
    }
  }
  else // check the encoder
  {
    incDelta <<= abs(encMenu.speed())/5;  // 'accelerated' encoder increment
    switch (encMenu.read())
    {
    case DIR_CCW: r = MD_Menu::NAV_DEC;  break;
    case DIR_CW:  r = MD_Menu::NAV_INC;  break;
    default: break;
    }
  }

  return(r);
}

void encoderMenuUI(void)
// Read the menu encoder. 
// This is called from user code only when MENU IS NOT ENABLED,
// to handle the main display
{
  MD_UISwitch::keyResult_t	k;
  uint8_t enc;

  // check the mode switch first
  if ((k = swMenuMode.read()) != MD_UISwitch::KEY_NULL)
  {
    switch (k)
    {
    case MD_UISwitch::KEY_LONGPRESS: break;
    case MD_UISwitch::KEY_DPRESS: break;

    case MD_UISwitch::KEY_PRESS:
    {
      PRINTS("\n* MODE PRESS -> ");
      switch (runMode)
      {
      case RUN_MANUAL:        PRINTS("Servos home"); setAllServoHome();           break;
      case RUN_SWEEP_WAITING: PRINTS("SWEEP_RUN");   runMode = RUN_SWEEP;         break;
      case RUN_SWEEP:         PRINTS("SWEEP_WAIT");  runMode = RUN_SWEEP_WAITING; break;
      default: PRINTS("UNHANDLED!!");  break;
      }
      dataChanged = true;
    }
    break;

    default: break;
    }
  }

  // check the encoder switch first and it takes priority over encoder clicks
  if ((k = swMenuSel.read()) != MD_UISwitch::KEY_NULL)
  {
    switch (k)
    {
    case MD_UISwitch::KEY_LONGPRESS:
    {
      PRINTS("\n* LONGPRESS -> ");
      switch (runMode)
      {
      case RUN_MANUAL:  PRINTS("MENU_INIT");  runMode = RUN_MENU_INIT; break;
      default: PRINTS("UNHANDLED!!");  break;
      }
    }
    break;

    case MD_UISwitch::KEY_DPRESS:
    {
      PRINTS("\n* DPRESS -> ");
      switch (runMode)
      {
      case RUN_MANUAL:        PRINTS("SWEEP_INIT");  runMode = RUN_SWEEP_INIT;  break;
#if USE_RCV
      case RUN_SWEEP_WAITING: PRINTS("RCHECK_INIT"); runMode = RUN_RCHECK_INIT; break;
      case RUN_RCHECK_PAUSE:  PRINTS("RUN_MANUAL");  runMode = RUN_MANUAL;      break;
#else
      case RUN_SWEEP_WAITING: PRINTS("RUN_MANUAL");  runMode = RUN_MANUAL;      break;
#endif
      default: PRINTS("UNHANDLED!!");  break;
      }
      dataChanged = true;
    }
    break;

    case MD_UISwitch::KEY_PRESS:
    {
      PRINTS("\n* PAGE PRESS -> ");
      nextPage();
    }
    break;

    default: break;
    }
  }
  else
  {
    // now handle the encoder
    uint16_t inc = 1 << abs(encMenu.speed() / 5);  // 'accelerated' encoder increment

    if ((enc = encMenu.read()) != DIR_NONE)
    {
      switch (enc)
      {
      case DIR_CCW: 
        if (runMode == RUN_MANUAL)   // change the timing manually
        {
          for (uint8_t s = 0; s < NUM_SERVO; s++)
          {
            if (C.data.servo[s].enabled)
              if (servoData[s].setV >= C.data.profile[C.data.servo[s].profileId].low + inc)
                servoData[s].setV -= inc;
          }
        }
        break;

      case DIR_CW:
        if (runMode == RUN_MANUAL)   // change the timing manually
        {
          for (uint8_t s = 0; s < NUM_SERVO; s++)
          {
            if (C.data.servo[s].enabled)
              if (servoData[s].setV <= C.data.profile[C.data.servo[s].profileId].high - inc)
                servoData[s].setV += inc;
          }
        }
        break;
      }
    }
  }
}

MD_Menu::value_t* mnuValueRqst(MD_Menu::mnuId_t id, bool bGet)
// Value request callback for editing variables
{
  static MD_Menu::value_t* r = &vBuf;

  switch (id)
  {
  case 20:  // C.data.tmSplash
    if (bGet)
      vBuf.value = C.data.tmSplash;
    else
    {
      C.data.tmSplash = vBuf.value;
      PRINT("\nC.data.tmSplash changed to ", C.data.tmSplash);
    }
    break;
  
  case 21:  // C.data.tmMenu
    if (bGet)
      vBuf.value = C.data.tmMenu;
    else
    {
      C.data.tmMenu = vBuf.value;
      PRINT("\nC.data.tmMenu changed to ", C.data.tmMenu);
    }
    break;
    
  case 22:  // C.data.autoPage
    if (bGet)
      vBuf.value = C.data.autoPage;
    else
    {
      C.data.autoPage = vBuf.value;
      PRINT("\nC.data.autoPage changed to ", C.data.autoPage);
    }
    break;

  case 25:  // C.data.limLow
    if (bGet)
      vBuf.value = C.data.limLow;
    else
    {
      C.data.limLow = vBuf.value;
      PRINT("\nC.data.limLow changed to ", C.data.limLow);
    }
    break;

  case 26:  // C.data.limHigh
    if (bGet)
      vBuf.value = C.data.limHigh;
    else
    {
      C.data.limHigh = vBuf.value;
      PRINT("\nC.data.limHigh changed to ", C.data.limHigh);
    }
    break;
    
  case 30 ... (30+NUM_SERVO-1):  // C.data.servo[n].enabled & C.data.servo[n].profileId
  {
    uint8_t idx = id - 30;

    if (bGet)
    {
      if (!C.data.servo[idx].enabled)
        vBuf.value = 0;
      else
        vBuf.value = C.data.servo[idx].profileId + 1;
    }
    else
    {
      C.data.servo[idx].enabled = !(vBuf.value == 0);
      if (C.data.servo[idx].enabled)
        C.data.servo[idx].profileId = vBuf.value - 1;
      PRINT("\nC.data.servo[", idx);
      PRINT("].enabled changed to ", C.data.servo[idx].enabled);
      PRINT("\nC.data.servo[", idx);
      PRINT("].profileId changed to ", C.data.servo[idx].profileId);
    }
  }
  break;

  case 40 ... (40+NUM_PROFILE-1): // C.data.profile[n].low
    {
      uint8_t idx = id - 40;

      if (bGet)
        vBuf.value = C.data.profile[idx].low;
      else
      {
        C.data.profile[idx].low = vBuf.value;
        if (C.data.profile[idx].low < C.data.limLow)
          C.data.profile[idx].low = C.data.limLow;

        PRINT("\nC.data.profile[", idx);
        PRINT("].low changed to ", C.data.profile[idx].low);
      }
    }
    break;

  case 50 ... (50+NUM_PROFILE-1): // C.data.profile[n].high
    {
      uint8_t idx = id - 50;

      if (bGet)
        vBuf.value = C.data.profile[idx].high;
      else
      {
        C.data.profile[idx].high = vBuf.value;
        if (C.data.profile[idx].high > C.data.limHigh)
          C.data.profile[idx].high = C.data.limHigh;

        PRINT("\nC.data.profile[", idx);
        PRINT("].high changed to ", C.data.profile[idx].high);
      }
    }
    break;

    case 60 ... (60 + NUM_PROFILE - 1) : // C.data.profile[n].home
    {
      uint8_t idx = id - 60;

      if (bGet)
        vBuf.value = C.data.profile[idx].home;
      else
      {
        C.data.profile[idx].home = vBuf.value;

        PRINT("\nC.data.profile[", idx);
        PRINT("].home changed to ", C.data.profile[idx].home);
      }
    }
    break;

    case 70 ... (70 + NUM_SERVO - 1) :  // C.data.profile[n].sweepType
    {
      uint8_t idx = id - 70;

      if (bGet)
        vBuf.value = C.data.profile[idx].sweepType;
      else
      {
        C.data.profile[idx].sweepType = vBuf.value;
        PRINT("\nC.data.profile[", idx);
        PRINT("] sweep type", vBuf.value);
      }
    }
    break;

    case 80 ... (80 + NUM_PROFILE - 1):  // C.data.profile[n].sweepTime
    {
      uint8_t idx = id - 80;

      if (bGet)
        vBuf.value = C.data.profile[idx].sweepTime;
      else
      {
        C.data.profile[idx].sweepTime = vBuf.value;
        PRINT("\nC.data.sweepTime changed to ", C.data.profile[idx].sweepTime);
      }
    }
    break;

    case 90 ... (90 + NUM_PROFILE - 1):  // C.data.profile[n].pauseTime
    {
      uint8_t idx = id - 90;

      if (bGet)
        vBuf.value = C.data.profile[idx].sweepPause;
      else
      {
        C.data.profile[idx].sweepPause = vBuf.value;
        PRINT("\nC.data.sweepPause changed to ", C.data.profile[idx].sweepPause);
      }
    }
    break;
  }

  return(r);
}

// Menu data structures
const PROGMEM MD_Menu::mnuHeader_t mnuHdr[] =
{
  // Starting Menu
  { 10, "SETUP MENU",  10, 16, 0 },
  { 20, "Assign",      20, 20+NUM_SERVO-1, 0 },
  { 25, "Profiles",    40, 40+NUM_PROFILE-1, 0 },
  { 30, "Sig Limit",   30, 31, 0 },
  { 35, "UI Setup",    35, 37, 0 },

  // Assign Menu
  { 40, "Profile 1",   50, 55, 0 },
  { 41, "Profile 2",   60, 65, 0 },
  { 42, "Profile 3",   70, 75, 0 },
};

const PROGMEM MD_Menu::mnuItem_t mnuItm[] =
{
  // Starting (Setup) menu
  { 10, "Assign",   MD_Menu::MNU_MENU, 20 },
  { 11, "Profiles", MD_Menu::MNU_MENU, 25 },
  { 12, "Sig Limit",MD_Menu::MNU_MENU, 30 },
  { 13, "UI Setup", MD_Menu::MNU_MENU, 35 },
  { 14, "Save",     MD_Menu::MNU_INPUT, 100 },
  { 15, "Load",     MD_Menu::MNU_INPUT, 101 },
  { 16, "Reset",    MD_Menu::MNU_INPUT, 102 },

  // Signal submenu
  { 20, "Output A", MD_Menu::MNU_INPUT, 30 },
  { 21, "Output B", MD_Menu::MNU_INPUT, 31 },
  { 22, "Output C", MD_Menu::MNU_INPUT, 32 },
  { 23, "Output D", MD_Menu::MNU_INPUT, 33 },
  { 24, "Output E", MD_Menu::MNU_INPUT, 34 },
  { 25, "Output F", MD_Menu::MNU_INPUT, 35 },

  // Safety parameters
  { 30, "Lower", MD_Menu::MNU_INPUT, 25 },
  { 31, "Upper", MD_Menu::MNU_INPUT, 26 },

  // UI submenu
  { 35, "Splash", MD_Menu::MNU_INPUT, 20 },
  { 36, "Menu",   MD_Menu::MNU_INPUT, 21 },
  { 37, "Paging", MD_Menu::MNU_INPUT, 22 },

  // Profiles submenu
  { 40, "Profile 1", MD_Menu::MNU_MENU, 40 },
  { 41, "Profile 2", MD_Menu::MNU_MENU, 41 },
  { 42, "Profile 3", MD_Menu::MNU_MENU, 42 },

  // Profile 1 submenu
  { 50, "Lower",    MD_Menu::MNU_INPUT, 40 },
  { 51, "Upper",    MD_Menu::MNU_INPUT, 50 },
  { 52, "Home",     MD_Menu::MNU_INPUT, 60 },
  { 53, "Swp Type",    MD_Menu::MNU_INPUT, 70 },
  { 54, "Swp Duration", MD_Menu::MNU_INPUT, 80 },
  { 55, "Swp Pause",    MD_Menu::MNU_INPUT, 90 },

  // Profile 2 submenu
  { 60, "Lower",    MD_Menu::MNU_INPUT, 41 },
  { 61, "Upper",    MD_Menu::MNU_INPUT, 51 },
  { 62, "Home",     MD_Menu::MNU_INPUT, 61 },
  { 63, "Swp Type",    MD_Menu::MNU_INPUT, 71 },
  { 64, "Swp Duration", MD_Menu::MNU_INPUT, 81 },
  { 65, "Swp Pause",    MD_Menu::MNU_INPUT, 91 },

  // Profile 2 submenu
  { 70, "Lower",    MD_Menu::MNU_INPUT, 42 },
  { 71, "Upper",    MD_Menu::MNU_INPUT, 52 },
  { 72, "Home",     MD_Menu::MNU_INPUT, 62 },
  { 73, "Swp Type",    MD_Menu::MNU_INPUT, 72 },
  { 74, "Swp Duration", MD_Menu::MNU_INPUT, 82 },
  { 75, "Swp Pause",    MD_Menu::MNU_INPUT, 92 },
};

const PROGMEM MD_Menu::mnuInput_t mnuInp[] =
{
  { 20, "Set (s)", MD_Menu::INP_INT,  mnuValueRqst, 2, 0, 0, 5, 0, 10, nullptr },   // UI Splash timeout
  { 21, "Set (s)", MD_Menu::INP_INT,  mnuValueRqst, 2, 0, 0, 10, 0, 10, nullptr },  // UI Menu timeout
  { 22, "Auto",    MD_Menu::INP_BOOL, mnuValueRqst, 1, 0, 0, 0, 0, 10, nullptr },   // Autopaging

  { 25, "Set (us)", MD_Menu::INP_INT, mnuValueRqst, 4, SERVO_LOWER_HARDLIMIT, 0, SERVO_UPPER_HARDLIMIT, 0, 10, nullptr },   // Safety Lower
  { 26, "Set (us)", MD_Menu::INP_INT, mnuValueRqst, 4, SERVO_LOWER_HARDLIMIT, 0, SERVO_UPPER_HARDLIMIT, 0, 10, nullptr },   // Safety Upper

  { 30, "Profile", MD_Menu::INP_LIST, mnuValueRqst, 1, 0, 0, 0, 0, 0, selProfile }, // Output A
  { 31, "Profile", MD_Menu::INP_LIST, mnuValueRqst, 1, 0, 0, 0, 0, 0, selProfile }, // Output B
  { 32, "Profile", MD_Menu::INP_LIST, mnuValueRqst, 1, 0, 0, 0, 0, 0, selProfile }, // Output C
  { 33, "Profile", MD_Menu::INP_LIST, mnuValueRqst, 1, 0, 0, 0, 0, 0, selProfile }, // Output D
  { 34, "Profile", MD_Menu::INP_LIST, mnuValueRqst, 1, 0, 0, 0, 0, 0, selProfile }, // Output E
  { 35, "Profile", MD_Menu::INP_LIST, mnuValueRqst, 1, 0, 0, 0, 0, 0, selProfile }, // Output F

  { 40, "Set (us)", MD_Menu::INP_INT, mnuValueRqst, 4, SERVO_LOWER_HARDLIMIT, 0, SERVO_UPPER_HARDLIMIT, 0, 10, nullptr },     // Profile 1 Lower
  { 50, "Set (us)", MD_Menu::INP_INT, mnuValueRqst, 4, SERVO_LOWER_HARDLIMIT, 0, 0, 0, 10, nullptr },                         // Profile 1 Higher
  { 60, "% Range",  MD_Menu::INP_INT, mnuValueRqst, 3, 0, 0, 100, 0, 10, nullptr },    // Profile 1 Home
  { 70, "Sweep",    MD_Menu::INP_LIST, mnuValueRqst, 3, 0, 0, 0, 0, 0, selSweepType }, // Profile 1 Sweep Type
  { 80, "Set (ms)", MD_Menu::INP_INT, mnuValueRqst, 4, 0, 0, 9999, 0, 10, nullptr },   // Profile 1 Sweep Duration
  { 90, "Set (ms)", MD_Menu::INP_INT, mnuValueRqst, 4, 0, 0, 9999, 0, 10, nullptr },   // Profile 1 Sweep Pause

  { 41, "Set (us)", MD_Menu::INP_INT, mnuValueRqst, 4, SERVO_LOWER_HARDLIMIT, 0, SERVO_UPPER_HARDLIMIT, 0, 10, nullptr },     // Profile 2 Lower
  { 51, "Set (us)", MD_Menu::INP_INT, mnuValueRqst, 4, SERVO_LOWER_HARDLIMIT, 0, SERVO_UPPER_HARDLIMIT, 0, 10, nullptr },     // Profile 2 Higher
  { 61, "% Range",  MD_Menu::INP_INT, mnuValueRqst, 3, 0, 0, 100, 0, 10, nullptr },    // Profile 2 Home
  { 71, "Sweep",    MD_Menu::INP_LIST, mnuValueRqst, 3, 0, 0, 0, 0, 0, selSweepType }, // Profile 2 Sweep Type
  { 81, "Set (ms)", MD_Menu::INP_INT, mnuValueRqst, 4, 0, 0, 9999, 0, 10, nullptr },   // Profile 2 Sweep Duration
  { 91, "Set (ms)", MD_Menu::INP_INT, mnuValueRqst, 4, 0, 0, 9999, 0, 10, nullptr },   // Profile 2 Sweep Pause

  { 42, "Set (us)", MD_Menu::INP_INT, mnuValueRqst, 4, SERVO_LOWER_HARDLIMIT, 0, SERVO_UPPER_HARDLIMIT, 0, 10, nullptr },     // Profile 3 Lower
  { 52, "Set (us)", MD_Menu::INP_INT, mnuValueRqst, 4, SERVO_LOWER_HARDLIMIT, 0, SERVO_UPPER_HARDLIMIT, 0, 10, nullptr },     // Profile 3 Higher
  { 62, "% Range",  MD_Menu::INP_INT, mnuValueRqst, 3, 0, 0, 100, 0, 10, nullptr },    // Profile 3 Home
  { 72, "Sweep",    MD_Menu::INP_LIST, mnuValueRqst, 3, 0, 0, 0, 0, 0, selSweepType }, // Profile 3 Sweep Type
  { 82, "Set (ms)", MD_Menu::INP_INT, mnuValueRqst, 4, 0, 0, 9999, 0, 10, nullptr },   // Profile 3 Sweep Duration
  { 92, "Set (ms)", MD_Menu::INP_INT, mnuValueRqst, 4, 0, 0, 9999, 0, 10, nullptr },   // Profiel 3 Sweep Pause

  { 100, "Confirm", MD_Menu::INP_RUN, saveConfig, 0, 0, 0, 0, 0, 10, nullptr },  // Save
  { 101, "Confirm", MD_Menu::INP_RUN, loadConfig, 0, 0, 0, 0, 0, 10, nullptr },  // Load
  { 102, "Confirm", MD_Menu::INP_RUN, resetConfig, 0, 0, 0, 0, 0, 10, nullptr }, // Reset
};

// bring it all together in the global menu object
MD_Menu M(menuNav, menuDisplay, // user navigation and display
  mnuHdr, ARRAY_SIZE(mnuHdr),   // menu header data
  mnuItm, ARRAY_SIZE(mnuItm),   // menu item data
  mnuInp, ARRAY_SIZE(mnuInp));  // menu input data

//----------------------------------------------------------
// Data Display
void showNumber(uint8_t c, uint8_t r, uint32_t n, uint8_t field)
{
  for (uint8_t i = 0; i < field; i++)   // print 3 digits for the pct
  {
    uint8_t dig = n % 10;

    if (i != 0 && dig == 0 && n == 0)   // always for first digit or not the first digit without any value remaining
    {
      LCD_PRINT(c + field - i - 1, r, ' ');
    }
    else
    {
      LCD_PRINT(c + field - i - 1, r, dig);
    }
    n /= 10;    // one less digit to process
  }
}

void displayRefresh(bool forceUpdate = false)
// Update the display. This function makes sure that it is not done too frequently
// by limiting it to once every UPDATE_TIME and then only if the data has changed.
{
  const uint32_t UPDATE_TIME = 200;    // in milliseconds
  static uint32_t lastUpdate = 0;     // millis() value

  if (!forceUpdate && millis() - lastUpdate <= UPDATE_TIME) 
    return;

  lastUpdate = millis();

  if (!forceUpdate && !dataChanged) 
    return;

  dataChanged = false;    // clear this global flag as we will now update the display

  lcd.clear(); // scrub everything

  // Show the correct display
#if USE_RCV
  if (runMode == RUN_RCHECK_PAUSE)    // Receiver check display
  {
    LCD_PRINT(0, 0, F("H:"));
    showNumber(3, 0, rcvCheckData.timeSig, 4);
    LCD_PRINT(7, 0, F("us"));
    LCD_PRINT(0, 1, F("L:"));
    showNumber(2, 1, rcvCheckData.timeGap, 5);
    LCD_PRINT(7, 1, F("us"));
    if (rcvCheckData.timeGap != 0)
    {
      showNumber(12, 1, 1000000UL / (rcvCheckData.timeGap * rcvCheckData.timeSig), 2);   // convert to Hz
      LCD_PRINT(14, 1, F("Hz"));
    }
  }
  else                          // Normal status display
#endif
  {
    for (uint8_t i = displayPageBase; i < displayPageBase + LCD_ROWS; i++)
    {
      if (i >= NUM_SERVO) break;    // more than the number of configured servo

      uint8_t r = i % LCD_ROWS;

      LCD_PRINT(0, r, (char)('A' + i));

      // enabled or attached profile indicator
      if (C.data.servo[i].enabled)
      {
        LCD_PRINT(1, r, char('1' + C.data.servo[i].profileId));

        // the current value in milliseconds
        showNumber(3, r, servoData[i].curV, 4);

        // the current value in % - printed by each digit in reverse order
        uint8_t id = C.data.servo[i].profileId;
        uint16_t pct = (100L * (servoData[i].curV - C.data.profile[id].low)) / (C.data.profile[id].high - C.data.profile[id].low);

        LCD_PRINT(11, r, '%');
        showNumber(8, r, pct, 3);
      }
      else
        LCD_PRINT(1, r, '-');   // not enabled
    }
  }

  // Show the current execution mode
  switch (runMode)
  {
  case RUN_MANUAL_INIT:
  case RUN_MANUAL:
    LCD_PRINT(LCD_COLS-3, 0, "Man");
    break;

  case RUN_SWEEP_INIT:
  case RUN_SWEEP_WAITING:
  case RUN_SWEEP:
    LCD_PRINT(LCD_COLS-3, 0, "Swp");
    break;

#if USE_RCV
  case RUN_RCHECK_INIT:
  case RUN_RCHECK:
  case RUN_RCHECK_PAUSE:
    LCD_PRINT(LCD_COLS-3, 0, "Rcv");
    break;
#endif

  default:
    //LCD_PRINT(LCD_COLS-3, 0, "???");
    break;
  }
}

// Control Code
void setup(void)
{
#if DEBUG
  Serial.begin(57600);
#endif
  PRINT("\n[", F(APP_NAME));
  PRINTS("]");

  // Application config parameters
  C.begin();

  // Initialize all objects and libraries
  for (uint8_t i = 0; i < NUM_SERVO; i++)
  {
    servo[i].attach(SERVO_PIN[i]);

    // set the current value and setpoints
    servoData[i].curV = 0;
  }
  setAllServoHome();

#if USE_RCV  
  pinMode(RCV_PIN, INPUT);
#endif

  // LCD display
  int status = lcd.begin(LCD_COLS, LCD_ROWS);
  if (status) // non zero status means it was unsuccessful
  {
    // hd44780 has a fatalError() routine that blinks an led if possible
    // begin() failed so blink error code using the on board LED if possible
    hd44780::fatalError(status); // does not return
  }

  // Switches and encoder
  swMenuMode.begin();   // Menu pager switch
  swMenuSel.begin();    // Menu rotary encoder switch
  swMenuSel.enableRepeat(false);
  swMenuSel.setLongPressTime(2000);
  encMenu.begin();      // Menu rotary encoder

  menuDisplay(MD_Menu::DISP_INIT);  // lcd display
  M.begin();        // Menu subsystem
  M.setMenuWrap(true);
  M.setTimeout(C.data.tmMenu * 1000);

  // Display splash screen
  if (C.data.tmSplash != 0)
  {
    LCD_PRINT(0, 0, F(APP_NAME));
    LCD_PRINT((LCD_COLS - strlen(APP_VER))/2, 1, F(APP_VER));
    delay(C.data.tmSplash * 1000);
    lcd.clear();
  }

  // Set up the display paging timer
  timerPage = millis();
}

void loop(void) 
{
  // check if the servo needs to be moved
  for (uint8_t i = 0; i < NUM_SERVO; i++)
  {
    if (servoData[i].curV != servoData[i].setV)
    {
      servo[i].writeMicroseconds(servoData[i].setV);
      servoData[i].curV = servoData[i].setV;
      dataChanged = true;
    }
  }

  // do any display updates?
  if (C.data.autoPage && millis() - timerPage >= PAGE_TIME)
  {
    PRINTS("\nPAGING");
    nextPage();
    timerPage = millis();
  }
  if (!M.runMenu()) 
    displayRefresh();

  // now check if we need to process anything else 
  // based on the current run mode
  switch (runMode)
  {
    // MANUAL mode --------------------------------
    case RUN_MANUAL_INIT:
      dataChanged = true;
      runMode = RUN_MANUAL;
      break;

    case RUN_MANUAL:
      encoderMenuUI();
      break;

    // SWEEP mode ---------------------------------
    case RUN_SWEEP_INIT:
      PRINTS("\n> SWEEP INIT");
      for (uint8_t i = 0; i < NUM_SERVO; i++)
        if (C.data.servo[i].enabled) doServoSweep(i, true);
      runMode = RUN_SWEEP_WAITING;
      PRINTS(" -> SWEEP WAIT");
      break;

    case RUN_SWEEP_WAITING:
      encoderMenuUI();
      break;

    case RUN_SWEEP:
      //PRINTS("\nSWEEP RUN");
      for (uint8_t i = 0; i < NUM_SERVO; i++)
        if (C.data.servo[i].enabled) doServoSweep(i);
      encoderMenuUI();
      break;

#if USE_RCV
    // RECEIVER CHECK mode ----------------------
    case RUN_RCHECK_INIT:
      PRINTS("\n> RCHECK INIT");
      rcvCheckData.timeSig = rcvCheckData.timeGap = 0;
      runMode = RUN_RCHECK;
      break;

    case RUN_RCHECK:
      rcvCheckData.timeSig = pulseIn(RCV_PIN, HIGH, 100000UL);
      rcvCheckData.timeGap = pulseIn(RCV_PIN,  LOW, 100000UL);
      dataChanged = true;
      rcvCheckData.timePause = millis();
      runMode = RUN_RCHECK_PAUSE;
      break;

    case RUN_RCHECK_PAUSE:
      if (millis() - rcvCheckData.timePause >= RCV_PAUSE_TIME)
        runMode = RUN_RCHECK;
      encoderMenuUI();
      break;
#endif

    // MENU mode --------------------------------
    case RUN_MENU_INIT:
      menuDisplay(MD_Menu::DISP_CLEAR, nullptr);
      M.runMenu(true);   // force running the menu
      runMode = RUN_MENU;
      break;

    case RUN_MENU:
      if (!M.runMenu())
      {
        M.setTimeout(C.data.tmMenu * 1000);	// in case we have changed the timeout value
        runMode = RUN_MANUAL_INIT;
      }
      break;

    default:
      runMode = RUN_MANUAL_INIT;
      break;
  }
}
