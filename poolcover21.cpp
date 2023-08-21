//  - set serial terminal to 57600 per constant below
// Commands:
// ? Show the current state of all the switches
//t Test all relays - switches all the relays on for 2 seconds
//lu Left Up
//ld Left Down
//lo Left Open
//lc Left Close
//ru Right Up
//rd Right Down
//ro Right Open
//rc Right Close
//bu Both Up
//bd Both Down
//bo Both Open
//bc Both Close
//s Stop all motors
//o Simulate the key being set to open the cover (stop with the "s" command)
//c Simulate the key being set to close the cover (stop with the "s" command)
//d Disable all motors - no motors can run until an "e" command is entered
//e Enable - cancel the "disabled" state
//q Quiet mode - disables the buzzer and battery warnings
//v Show the current battery voltage
//m Show the current memory usage

// -----------------------------------------------------------
// Purpose   : Arduino Pool Cover Controller
// Version   : 1.06
// Date      : 16 July 2015
// Copyright : Chris Gaebler 2013-2015
// Licence   : GNU General Public License
// -----------------------------------------------------------
//

#define DEBUG__INT(fmt, ...) if (Log_Level >= LOG_LEVEL_DEBUG) { sprintf(buffer, "DEBUG " fmt "%s", __VA_ARGS__); Serial.print(buffer); }
#define DEBUG(...) DEBUG__INT(__VA_ARGS__, "\n")
//#define DEBUG(...) (void)0

#define INFO__INT(fmt, ...) if (Log_Level >= LOG_LEVEL_INFO) { sprintf(buffer, fmt "%s", __VA_ARGS__); Serial.print(buffer); }
#define INFO(...) INFO__INT(__VA_ARGS__, "\n")
//#define INFO(...) (void)0

#define TRACE__INT(fmt, ...) if (Log_Level >= LOG_LEVEL_TRACE) { sprintf(buffer, fmt "%s", __VA_ARGS__); Serial.print(buffer); }
#define TRACE(...) TRACE__INT(__VA_ARGS__, "\n")
//#define TRACE(...) (void)0

//#define ERROR__INT(fmt, ...) fprintf(stderr, "ERROR: " fmt "%s", __VA_ARGS__)
//#define ERROR(...) ERROR__INT(__VA_ARGS__, "\n")

const char * VERSION = "2.02";

// include the PROGMEM library
// we were using too much RAM so we now store strings in program memory using the F() syntax
// see http://playground.arduino.cc/Learning/Memory

#include <avr/pgmspace.h>
#include <Arduino.h>

// Define all the I/O pins

const int M_LEFT_OPEN = 5;   // Ard D5 Purple to Relay IN4
                             //   Purple to Board D37 Grey to LMdb9 pin5
const int M_LEFT_CLOSE = 4;  // ARd D4 Silver to Relay IN3
const int M_LEFT_UP = 3;     // Ard D2 (D3 according to p20!) Brown to Relay IN1
                             //   Black to Board D40 Black to LMdb9 pin 2
                             // Ucha June2023 swapping updown: was 2 now 3
const int M_LEFT_DOWN = 2;   // Ard D3 but is D2 ..
                             // Ucha June2023 swapping updown: was 3 now 2


// Ucha June2023 Some time ago made this swap and it was wrong apparently
// swapped M_LEFT_UP and M_LEFT_DOWN
//const int M_LEFT_DOWN = 2;
//const int M_LEFT_UP = 3;


const int M_RIGHT_OPEN = 9;  // Ard D9 Orange
const int M_RIGHT_CLOSE = 8; // Ard D8 Yellow
//const int M_RIGHT_UP = 6;
//const int M_RIGHT_DOWN = 7; // 24sep2022 swapped left UD back and right UD
                            // Ucha 8June2023 Swapped them back after first test
                            //                right up is 7 and right down is 6
const int M_RIGHT_UP = 7;
const int M_RIGHT_DOWN = 6;
const int S_LEFT_FULLY_OPEN = 10;    // Ard D10 Orange to Board B11
const int S_LEFT_FULLY_CLOSED = 11;  // Ard D11 Red to Board B7
const int S_LEFT_FULLY_LIFTED = 12;
const int S_RIGHT_FULLY_OPEN = 14;        // A0
const int S_RIGHT_FULLY_CLOSED = 15;      // A1
const int S_RIGHT_FULLY_LIFTED = 16;      // A2
const int S_KEY_OPEN = 17;                // A3
const int S_KEY_CLOSE = 18;               // A4
const int V_INPUT = 19;                   // A5 - Battery voltage monitor
const int BUZZER = 13;                    // 13 - Arduini on-board LED and our buzzer

// Other Constants

const int STATE_OFF = 0;                  // All motors are stopped
const int STATE_OPENING = 1;              // Opening
const int STATE_OPEN_LIFTING = 2;         // Opening, lifting
const int STATE_OPEN_MOVING = 3;          // Opening, moving the cover backwards
const int STATE_OPEN_NEW_LIFT = 4;        // Opening, starting a new lift
const int STATE_CLOSING = 16;             // Closing
const int STATE_CLOSE_LOWERING = 17;      // Closing, lowering
const int STATE_CLOSE_MOVING = 18;        // Closing, moving the cover forwards
const int STATE_CLOSE_NEW_CLOSE = 19;     // Closing, starting a new close
const int STATE_CLOSE_NEW_LOWER = 20;     // Closing, starting a new lower
const int STATE_ERROR = 30;               // An error occurred.
const int STATE_DISABLED = 31;            // Disabled by command

const int COMMAND_NONE = 0;               // Not executing a command
const int COMMAND_OPEN = 1;               // Normal Open
const int COMMAND_AUTO_OPEN = 2;          // Auto-Open NUMBER_OF_SECTIONS
const int COMMAND_CLOSE = 3;              // Normal Close
const int COMMAND_AUTO_CLOSE = 4;         // Auto Close NUMBER_OF_SECTIONS
const int COMMAND_OTHER = 5;              // Processing another command

const int V_R1 = 3300L;                   // Voltage divider R1
const int V_R2 = 1000L;                   // Voltage divider R2
const long V_FACTOR = ((V_R2 * 1023L)/(V_R1 + V_R2));    // for the battery voltage calculation
const int DEBOUNCE = 5;                   // de-bounce delay
const int ON = 1;                         // Switch value for ON
const int OFF = 0;                        // Switch value for OFF
const long MAX_MOVE_TIME = 30000;         // max milli-seconds to open or close a section (normally takes about 15 seconds)
const long MAX_LIFT_TIME = 30000;         // max milli-seconds to lift or lower a section (normally takes about 15 seconds)
const long MAX_MOVE_DIFFERENCE = 2000;    // max milli-seconds difference between left and right move
const long MAX_INITIAL_LIFT_TIME = 1000;  // max milli-seconds to run the lift motors to clear the "fully lifted" switches
const long MAX_INITIAL_CLOSE_TIME = 1000; // max milli-seconds to run the close motors to clear the fully closed switches
// Ucha June 2023: This was too short and caused the opening to stop on the 2nd last panel. 180,000 is 3 mins so change
//                 it to 4 mins = 240000
//const long MAX_KEY_TIME = 180000;         // maximum time the key can be active
const long MAX_KEY_TIME = 240000;         // maximum time the key can be active

const int LIFT_OVER_RUN = 75;             // milli-seconds to run the lift after "fully lifted" is detected (to make sure it can't shake free)
const int VOLTAGE_WARNING = 100;          // below 10 volts we give three beeps Ucha: was 10.5 but seemed too high
const int VOLTAGE_ERROR = 85;            // below 8.5  volts we do not attempt to run. Ucha: was 10 but too high
const int KEY_ACTIVATE_TIME = 600;        // number of milli-seconds before the key activates the cover
const int AUTO_MODE_COUNT = 3;            // number of key activations to trigger auto-mode (-1 to disable auto-mode)
const int AUTO_KEY_TIME = 4000;           // the key must be activated AUTO_MODE_COUNT times within AUTO_KEY_TIME ms to trigger auto mode
// Ucha: change from 4 to 7
const int NUMBER_OF_SECTIONS = 7;         // number of cover sections, used for auto-open and auto-close


int S_left_open = OFF;
int S_left_closed = OFF;
int S_left_lifted = OFF;
int S_right_open = OFF;
int S_right_closed = OFF;
int S_right_lifted = OFF;
int S_key_open = OFF;                     // the real physical state of the key
int S_key_close = OFF;

// Other global variables

int State = STATE_OFF;                    // Current state
int Command = COMMAND_NONE;               // Current command (from serial port)
int Battery_Warning_Sounded = false;      // true to suppress voltage warning beeps
int Quiet_Mode = false;                   // true to disable the beeper
long Time_Left_Lift_Started = -1;         // time left lift motor started
long Time_Right_Lift_Started = -1;        // -1 = currently not running
long Time_Open_Started = -1;              // time both move motors started for an open
long Time_Left_Move_Started = -1;         // time the left move motor started
long Time_Right_Move_Started = -1;        // time the right move motor started
long Time_Separate_Move_Started = -1;     // time when one side started moving without the other
long Time_Open_Key_Activated = -1;        // time a key open started
long Time_Close_Key_Activated = -1;       // time a key close started
int Key_Open_Count = 0;                   // count of key activations less than KEY_ACTIVATE_TIME
long Time_First_Key_Open = -1;            // time of the first short key activation
int Key_Close_Count = 0;                  // count of key activations less than KEY_ACTIVATE_TIME
long Time_First_Key_Close = -1;           // time of the first short key activation
int Auto_Section_Count = 0;               // count of sections opened or closed in auto mode
char buffer[120];
uint8_t * heapptr, * stackptr;

// Ucha add logging and force mode
const int LOG_LEVEL_INFO = 100;
const int LOG_LEVEL_TRACE = 10000;
const int LOG_LEVEL_DEBUG = 1000;
int Log_Level = LOG_LEVEL_INFO;
int Force_Mode = false;                   // true to force continuation through low battery etc
const int FULLY_OPEN_OVERRUN = 1500;      // milli-seconds to rotate the lift after presumed fully open

void Motor_Stop(int pin);
void Motor_Start(int pin);
void Beep(int number);
void Debug_Beep(int number);
void ErrorCodeBeep(int number);
int Get_Voltage(boolean print);
boolean Check_Max_Times();
void ReadAllSwitches();                     // read all the switches
void HandleKeyState();                      // analyse the key state and set the Command
void All_Stop();

void ProcessCommand();
void HandleKeyState();
void KeyCloseChange();
void ReadAllSwitches();
void KeyOpenChange();

void Open();
void Close();
void Report_Error(char * message);
void Report_Numbered_Error(int number, char * message);
void Relay_Test();
void check_mem();
int availableMemory();
int freeRam();
void PrintAllSwitches();

//-------------------------------------------------
// This runs when the Arduino powers up or resets
//
void setup()
{
    pinMode(M_LEFT_UP, OUTPUT);
    pinMode(M_LEFT_DOWN, OUTPUT);
    pinMode(M_LEFT_OPEN, OUTPUT);
    pinMode(M_LEFT_CLOSE, OUTPUT);
    pinMode(M_RIGHT_UP, OUTPUT);
    pinMode(M_RIGHT_DOWN, OUTPUT);
    pinMode(M_RIGHT_OPEN, OUTPUT);
    pinMode(M_RIGHT_CLOSE, OUTPUT);
    pinMode(S_LEFT_FULLY_OPEN, INPUT);
    pinMode(S_LEFT_FULLY_CLOSED, INPUT);
    pinMode(S_LEFT_FULLY_LIFTED, INPUT);
    pinMode(S_RIGHT_FULLY_OPEN, INPUT);
    pinMode(S_RIGHT_FULLY_CLOSED, INPUT);
    pinMode(S_RIGHT_FULLY_LIFTED, INPUT);
    pinMode(S_KEY_OPEN, INPUT);
    pinMode(S_KEY_CLOSE, INPUT);
    pinMode(V_INPUT, INPUT);
    pinMode(BUZZER, OUTPUT);

    DEBUG("Setup")
    All_Stop();

    Serial.begin(57600);
    Serial.print(F("Version "));
    Serial.print(VERSION);
    Serial.println(F(" initialised"));
    delay(500);
    Get_Voltage(true);
}

//-----------------------------------------------------------------
// The main loop (called by Arduino)
// Returning from this function causes processing to re-start at the top of the function
//
void loop()
{
    ReadAllSwitches();                     // read all the switches
    HandleKeyState();                      // analyse the key state and set the Command

    TRACE("Loop:   cmd: %d  (0=None 1=Open 2=Auto_Open 3=Close 4=Auto_Close 5=Other)", Command)
    TRACE("Loop: state: %d  (30=Err 31=Dis 0=Off 1=O 2=Lf 3=OMv 4=ONLf 16=C, 17=Lw, 18=CMv 19=CNC 20=CNLw)", State)

    if (Serial.available() > 0)            // Commands from the serial port have priority
    {
        ProcessCommand();
        return;
    }

    if (State == STATE_DISABLED)           // Disabled by serial port command
        return;

    if (Command == COMMAND_NONE)
    {
        All_Stop();                         // also resets all flags and error states
        return;
    }

    Get_Voltage(false);

    if (State == STATE_ERROR)              // if an error occurred, user must switch key off to reset everything
        return;

    if (Check_Max_Times())                 // if any motor has been running too long, stop in STATE_ERROR
        return;

    if ((Command == COMMAND_OPEN) || (Command == COMMAND_AUTO_OPEN))
    {
        Open();
        return;
    }

    if ((Command == COMMAND_CLOSE) || (Command == COMMAND_AUTO_CLOSE))
    {
        Close();
        return;
    }
}

//-----------------------------------------------------------------
// This function handles the cover open logic
// Called repeatedly when the key is in the Open position
//
void Open()
{
    if ((State == STATE_OFF) || (State >= STATE_CLOSING))    // if not in any opening state ...
    {
        Serial.println(F("Opening"));
        State = STATE_OPENING;
    }

// make sure the closing motors are all off (they should be)

    Motor_Stop(M_LEFT_DOWN);
    Motor_Stop(M_RIGHT_DOWN);
    Motor_Stop(M_LEFT_CLOSE);
    Motor_Stop(M_RIGHT_CLOSE);

// Before we can open, the lift mechanism must be at the top, otherwise we could break something.
// We assume here that it is always safe to move the lift, because the logic here will never move
// the lift mechanism away from the top position unless a cover section is fully open or fully closed.
//
// It's not safe to lift the last section, because it is still engaged in the drive wheels.
// We prevent this by not lifting unless at least one Fully Closed sensor is active
//
// Ucha
// June 2023: This alert happens at the end of opening: because the lift has rotated past the stop switch
//            but there is no magnet over the sensor - therefore we must have the last panel at the back
    // IF Either is Not-at-Top and both sides have No-Magnet-on-Sensor - then something is wrong
    //  .... or we just finished opening and the last panel is in but the lift senors have moved on a bit
    if ( ((S_left_lifted == OFF) || (S_right_lifted == OFF))       // if not fully lifted
         &&   ((S_left_closed == OFF) && (S_right_closed == OFF)) )     // and no section fully closed ..
    {
        Report_Numbered_Error(3, "*** Lift not at top but no section is closed **");
        delay(FULLY_OPEN_OVERRUN);
        return;
    }

// if either side is not at the top, start it
    if (S_left_lifted == OFF)                          // if left side is not at the top
    {
        State = STATE_OPEN_LIFTING;
        // IF  Not-at-Top AND Not-Rotating THEN start Rotating UP
        if (Time_Left_Lift_Started == -1)                // if we were not already lifting
        {
            Time_Left_Lift_Started = millis();           // .. note when we started
            Serial.println(F(" Left lift start (lift to top before sliding open)"));
            Motor_Start(M_LEFT_UP);
        }
        // else we are already rotating left and counting Time_Left_Lift_Started until we get
        // back into this loop and detect S_left_lifted == ON a few lines below
    }

    if (S_right_lifted == OFF)
    {
        State = STATE_OPEN_LIFTING;
        // IF  Not-at-Top AND Not-Rotating THEN start Rotating UP
        if (Time_Right_Lift_Started == -1)              // if we were not already lifting
        {
            Time_Right_Lift_Started = millis();          // .. note when we started
            Serial.println(F(" Right lift start (lift to top before sliding open"));
            Motor_Start(M_RIGHT_UP);
        }
    }

// if either lift has reached the top, stop it

    //  IF       Lift-Rotating        AND     Hit the Top!
    if ((Time_Left_Lift_Started != -1) && (S_left_lifted == ON))
    {
        delay(LIFT_OVER_RUN);
        Motor_Stop(M_LEFT_UP);
        Time_Left_Lift_Started = -1;
        Serial.println(F(" Left lift stop"));
    }

    //  IF       Lift-Rotating        AND     Hit the Top!
    if ((Time_Right_Lift_Started != -1) && (S_right_lifted == ON))
    {
        delay(LIFT_OVER_RUN);
        Motor_Stop(M_RIGHT_UP);
        Time_Right_Lift_Started = -1;
        Serial.println(F(" Right lift stop"));
    }

// if either side is still lifting, we are done here for now
    // IF     L Not-at-Top    OR    R Not-at_Top
    if ((S_left_lifted == OFF) || (S_right_lifted == OFF))
        return; // At least 1 lift is still rotating - back to the main loop please!

//
// FULLY LIFTED  => TIME TO SLIDE OPEN
//

// can only get here if both sides are fully lifted
// we can now move the cover backwards until it hits either fully open micro-switch

    if (State != STATE_OPEN_MOVING)
    {
        Serial.println(F("Both sides fully lifted"));
        State = STATE_OPEN_MOVING;
    }

// make sure the lift motors are off (they should be)

    Motor_Stop(M_LEFT_UP);
    Motor_Stop(M_RIGHT_UP);

// if neither side is fully open, start both move motors
    // IF  L Back-switch-Free AND R Back-switch-Free
    if ((S_left_open == OFF) && (S_right_open == OFF))       // section is NOT fully open
    {
        if (Time_Open_Started == -1)                          // if move is not running
        {                                                  // .. start it
            Motor_Start(M_LEFT_OPEN);
            Motor_Start(M_RIGHT_OPEN);
            Time_Open_Started = millis();
            Serial.println(F(" Open start"));
        }
        return;                                               // nothing more to do until fully open
    }

// if either side has reached the fully open position, stop BOTH move motors
// - when opening, both fully open switches are not always triggered,
// - and running the move motors separately does not help

// Ucha note: this is why it lifts even when the L back switch is not hit  - and it lifts the panel a bit while catching it
//            because only one of them needs to be pressed for the sliding to stop

    // IF  L Back Pressed  OR   R Back Pressed
    if ((S_left_open == ON) || (S_right_open == ON))         // section is fully open
    {
        if (Time_Open_Started != -1)                          // if move is running
        {                                                  // .. stop it
            Motor_Stop(M_LEFT_OPEN);
            Motor_Stop(M_RIGHT_OPEN);
            Time_Open_Started = -1;
            Serial.println(F(" Open stop"));
            if (Command == COMMAND_AUTO_OPEN)                  // if we are in auto-open mode and all sections are now open, stop
            {
                Auto_Section_Count ++;
                DEBUG("Auto_Section_Count: %d",Auto_Section_Count)
                if (Auto_Section_Count == NUMBER_OF_SECTIONS)
                {
                    // LAST SECTION IS FULLY BACK
                    Command = COMMAND_NONE;
                    Auto_Section_Count = 0;
                    return;
                }
            }
        }
        // else at least one is pressed but we weren't sliding open
    }

// if either side is still opening, we are done here for now

    if ((S_left_open == OFF) && (S_right_open == OFF))
        return;

//
// FULLY LIFTED AND FULLY OPEN => START LIFTING THE NEXT ONE
//

// we only get here if both sides are fully lifted and fully open ..
// we must now start a "new lift", where we start both lift motors and wait for them
// to clear the fully lifted switches before we go back to the main loop

    Serial.println(F("Section is fully open, starting initial lift..."));
    State = STATE_OPEN_NEW_LIFT; // This state is pretty much ignored

// start both lift motors

    Motor_Start(M_LEFT_UP);
    Motor_Start(M_RIGHT_UP);
    long started = millis();

// wait until we clear the Fully Lifted sensors or we time out

    while (true)
    {
        // TIGHT LOOP: Read the switches and keep Rotating Up until both have cleared
        ReadAllSwitches();
        if ((S_left_lifted == OFF) && (S_right_lifted == OFF))
        {
            Serial.println(F("...initial lift done"));
            return;
        }
        if ((millis() - started) > MAX_INITIAL_LIFT_TIME)
        {
            Report_Numbered_Error(9, "*** MAX_INITIAL_LIFT_TIME exceeded ***");
            return;
        }
    }
}

//-----------------------------------------------------------------
// This function handles the cover close logic
// Called repeatedly when the key is in the Close position
//
void Close()
{
    boolean just_lifted = false;
    long started;

    if (State < STATE_CLOSING)    // if not in any closing state ...
    {
        Serial.println(F("Closing"));
        State = STATE_CLOSING;
    }

// make sure the opening motors are all off (they should be)

    Motor_Stop(M_LEFT_UP);
    Motor_Stop(M_RIGHT_UP);
    Motor_Stop(M_LEFT_OPEN);
    Motor_Stop(M_RIGHT_OPEN);

// Before we can close, the lift mechanism should be at the top, otherwise we could BREAK SOMETHING.
// We assume here that it is always safe to move the lift, because we never move
// the lift mechanism away from fully lifted unless a cover section is fully open or fully closed.
//
// if either side is not at the top, start it

    // Left lift is not at the top and we're not sliding closed...
    if ((S_left_lifted == OFF) && (State != STATE_CLOSE_MOVING))
    {
        // ... so start lowering (is this the right direction? Perhaps it should be up?)
        State = STATE_CLOSE_LOWERING;
        if (Time_Left_Lift_Started == -1)               // if we were not already lowering
        {
            Time_Left_Lift_Started = millis();           // .. note when we started
            Serial.println(F(" Left down start to prep for close"));
            Motor_Start(M_LEFT_DOWN);
        }
        TRACE("L rotating down")
    }

    // Rigth lift is not at the top and we're not sliding closed...
    if ((S_right_lifted == OFF) && (State != STATE_CLOSE_MOVING))
    {
        // .... so start lowering (is this right?)
        State = STATE_CLOSE_LOWERING;
        if (Time_Right_Lift_Started == -1)              // if we were not already lifting
        {
            Time_Right_Lift_Started = millis();          // .. note when we started
            Serial.println(F(" Right down start to prep for close"));
            Motor_Start(M_RIGHT_DOWN);
        }
        TRACE("R rotating down")
    }

// if either lift has reached the top, stop it

    // Left lift is running and now it is at top...
    if ((Time_Left_Lift_Started != -1) && (S_left_lifted == ON))
    {
        delay(LIFT_OVER_RUN);
        Motor_Stop(M_LEFT_DOWN);
        Time_Left_Lift_Started = -1;
        Serial.println(F(" Left down stop"));
        just_lifted = true;
    }

    // Right lift is running and now it at top
    if ((Time_Right_Lift_Started != -1) && (S_right_lifted == ON))
    {
        delay(LIFT_OVER_RUN);
        Motor_Stop(M_RIGHT_DOWN);
        Time_Right_Lift_Started = -1;
        Serial.println(F(" Right down stop"));
        just_lifted = true;
    }

// If we just stopped one of the lift motors, and both are now stopped, we should have a section ready to move,
// and it should be triggering at least one of the fully open sensors. If not, that's an error
// Note that this will beep, but will NOT prevent us trying to start a new lower
// If there really is a section at the bottom of the lift, lowering could cause serious damage
// - see the documentation for a discussion of this problem

    if (just_lifted && (Time_Left_Lift_Started == -1) && (Time_Right_Lift_Started == -1))
        if ((S_left_open == OFF) && (S_right_open == OFF))
        {
            // Finished lifting a section, but nothing pressing against the back (fully open) switches
            Report_Numbered_Error(5, "*** No section ready to move after lower");
            return;
        }

// if either side is still lowering, we are done here for now
    // still lowering or rotating back to top after lowering - continue to loop
    if ((S_left_lifted == OFF) || (S_right_lifted == OFF))
        return;

// can only get here if both sides are fully lifted
// we can now move the cover forwards until it hits the fully closed micro-switches

    if (State != STATE_CLOSE_MOVING)
    {
        Serial.println(F("Both sides fully lowered"));
        State = STATE_CLOSE_MOVING;
    }

// if either side is not fully closed, start it

    if (S_left_closed == OFF)                          // if left side is not fully closed
    {
        if (Time_Left_Move_Started == -1)               // if we were not already opening
        {
            Time_Left_Move_Started = millis();           // .. note when we started
            Serial.println(F(" Left close start"));
        }
        Motor_Start(M_LEFT_CLOSE);
    }

    if (S_right_closed == OFF)
    {
        if (Time_Right_Move_Started == -1)              // if we were not already closing
        {
            Time_Right_Move_Started = millis();          // .. note when we started
            Serial.println(F(" Right close start"));
        }
        Motor_Start(M_RIGHT_CLOSE);
    }

// if either side has reached the fully closed position, stop it

    if ((Time_Left_Move_Started != -1) && (S_left_closed == ON))
    {
        Motor_Stop(M_LEFT_CLOSE);
        Time_Left_Move_Started = -1;
        Serial.println(F(" Left close stop"));
        return;                                         // we only get one switch change per call
    }

    if ((Time_Right_Move_Started != -1) && (S_right_closed == ON))
    {
        Motor_Stop(M_RIGHT_CLOSE);
        Time_Right_Move_Started = -1;
        Serial.println(F(" Right close stop"));
        if (Command == COMMAND_AUTO_CLOSE)              // if we are in auto-close mode and all sections are closed, stop
        {
            Auto_Section_Count ++;
            sprintf(buffer,"Auto_Section_Count: %d",Auto_Section_Count);
            Serial.println(buffer);
            if (Auto_Section_Count == NUMBER_OF_SECTIONS)
            {
                Command = COMMAND_NONE;
                Auto_Section_Count = 0;
                return;
            }
        }
        return;
    }

// if either side is still closing, we are done here for now

    if ((S_left_closed == OFF) || (S_right_closed == OFF))
        return;

// We only get here if both sides are fully lifted AND fully closed ..
// There are now two possibilities ..

// If either fully open switch is active, there is a section lowered and ready to close
// so we start a "new close", where we start both close motors and wait for them
// to clear the fully closed switches before we go back to the main loop

    if ((S_left_open == ON) || (S_right_open == ON))
    {
        Serial.println(F("Starting initial close..."));
        State = STATE_CLOSE_NEW_CLOSE;
        Motor_Start(M_LEFT_CLOSE);    // start both close motors
        Motor_Start(M_RIGHT_CLOSE);
        started = millis();
        while (true)                  // wait until we clear the Fully Closed sensors or we run out of time
        {
            ReadAllSwitches();
            if ((S_left_closed == OFF) && (S_right_closed == OFF))
            {
                Serial.println(F("...initial close done"));
                return;
            }
            if ((millis() - started) > MAX_INITIAL_CLOSE_TIME)
            {
                Report_Numbered_Error(11, "*** MAX_INITIAL_CLOSE_TIME exceeded ***");
                return;
            }
        }
    }

// If S_LEFT_FULLY_OPEN AND S_RIGHT_FULLY_OPEN are both OFF, we need to lower the next section down
// we now start a "new lower", where we start both down motors and wait for them
// to clear the fully lifted switches before we go back to the main loop

    if ((S_left_open == OFF) && (S_right_open == OFF))
    {
        Serial.println(F("Starting initial lower..."));
        State = STATE_CLOSE_NEW_LOWER;
        Motor_Start(M_LEFT_DOWN);        // start both lower motors
        Motor_Start(M_RIGHT_DOWN);
        started = millis();
        while (true)                     // wait until we clear the Fully Lifted sensors or we run out of time
        {
            ReadAllSwitches();
            if ((S_left_lifted == OFF) && (S_right_lifted == OFF))
            {
                Serial.println(F("...initial lower done"));
                return;
            }
            if ((millis() - started) > MAX_INITIAL_LIFT_TIME)
            {
                Report_Numbered_Error(13, "*** MAX_INITIAL_LIFT_TIME exceeded ***");
                return;
            }
        }
    }
}

//-----------------------------------------------------------------
// Stop all motors and reset all running flags
//
void All_Stop()
{
    Motor_Stop(M_LEFT_UP);
    Motor_Stop(M_RIGHT_UP);
    Motor_Stop(M_LEFT_DOWN);
    Motor_Stop(M_RIGHT_DOWN);
    Motor_Stop(M_LEFT_OPEN);
    Motor_Stop(M_RIGHT_OPEN);
    Motor_Stop(M_LEFT_CLOSE);
    Motor_Stop(M_RIGHT_CLOSE);
    Time_Left_Lift_Started = -1;
    Time_Right_Lift_Started = -1;
    Time_Open_Started = -1;
    Time_Left_Move_Started = -1;
    Time_Right_Move_Started = -1;
    Time_Separate_Move_Started = -1;
    Battery_Warning_Sounded = false;
    if (State != STATE_OFF)
    {
        Serial.println(F(" All motors off"));
        State = STATE_OFF;
    }
}

//-----------------------------------------------------------------
// Reads all input switches and stores their current value.
// If a change is detected for one switch, the function returns so that it can be actioned immediately.
// It is especially important to process the fully closed switches quickly.
// A DEBOUNCE delay is only executed if a switch value has changed.
//
void ReadAllSwitches()
{
    int val1;
    int val2;

    val1 = digitalRead(S_LEFT_FULLY_CLOSED);
    if (val1 != S_left_closed)
    {
        if (val1 == ON)	// 1.06 - act on the first transition to ON immediately
        {
            S_left_closed = val1;
            sprintf(buffer,"Left closed is now %d",val1);
            Serial.println(buffer);
            Debug_Beep(1);
            return;
        }
        delay(DEBOUNCE);  // 1.06 - only debounce a transition to OFF
        val2 = digitalRead(S_LEFT_FULLY_CLOSED);
        if (val1 != val2)
            return;
        S_left_closed = val1;
        sprintf(buffer,"Left closed is now %d",val1);
        Debug_Beep(1);
        Serial.println(buffer);
        return;
    }

    val1 = digitalRead(S_RIGHT_FULLY_CLOSED);
    if (val1 != S_right_closed)
    {
        if (val1 == ON)	// 1.06 - act on the first transition to ON immediately
        {
            S_right_closed = val1;
            sprintf(buffer,"Right closed is now %d",val1);
            Serial.println(buffer);
            Debug_Beep(2);
            return;
        }
        delay(DEBOUNCE);  // 1.06 - only debounce a transition to OFF
        val2 = digitalRead(S_RIGHT_FULLY_CLOSED);
        if (val1 != val2)
            return;
        S_right_closed = val1;
        sprintf(buffer,"Right closed is now %d",val1);
        Serial.println(buffer);
        Debug_Beep(2);
        return;
    }

    val1 = digitalRead(S_LEFT_FULLY_OPEN);
    if (val1 != S_left_open)                           // has the value changed?
    {
        delay(DEBOUNCE);
        val2 = digitalRead(S_LEFT_FULLY_OPEN);
        if (val1 != val2)
            return;
        S_left_open = val1;
        sprintf(buffer,"Left open is now %d",val1);
        Serial.println(buffer);
        Debug_Beep(1);
        return;
    }

    val1 = digitalRead(S_RIGHT_FULLY_OPEN);
    if (val1 != S_right_open)
    {
        delay(DEBOUNCE);
        val2 = digitalRead(S_RIGHT_FULLY_OPEN);
        if (val1 != val2)
            return;
        S_right_open = val1;
        sprintf(buffer,"Right open is now %d",val1);
        Serial.println(buffer);
        Debug_Beep(2);
        return;
    }

    val1 = digitalRead(S_LEFT_FULLY_LIFTED);
    if (val1 != S_left_lifted)
    {
        if (val1 == ON)	// 1.06 - act on the first transition to ON immediately
        {
            S_left_lifted = val1;
            sprintf(buffer,"Left lifted is now %d",val1);
            Serial.println(buffer);
            Debug_Beep(1);
            return;
        }
        delay(DEBOUNCE);  // 1.06 - only debounce a transition to OFF
        val2 = digitalRead(S_LEFT_FULLY_LIFTED);
        if (val1 != val2)
            return;
        S_left_lifted = val1;
        sprintf(buffer,"Left lifted is now %d",val1);
        Serial.println(buffer);
        Debug_Beep(1);
        return;
    }

    val1 = digitalRead(S_RIGHT_FULLY_LIFTED);
    if (val1 != S_right_lifted)
    {
        if (val1 == ON)	// 1.06 - act on the first transition to ON immediately
        {
            S_right_lifted = val1;
            sprintf(buffer,"Right lifted is now %d",val1);
            Serial.println(buffer);
            Debug_Beep(2);
            return;
        }
        delay(DEBOUNCE);  // 1.06 - only debounce a transition to OFF
        val2 = digitalRead(S_RIGHT_FULLY_LIFTED);
        if (val1 != val2)
            return;
        S_right_lifted = val1;
        sprintf(buffer,"Right lifted is now %d",val1);
        Serial.println(buffer);
        Debug_Beep(2);
        return;
    }

    val1 = digitalRead(S_KEY_OPEN);
    if (val1 != S_key_open)
    {
        delay(DEBOUNCE);
        val2 = digitalRead(S_KEY_OPEN);
        if (val1 != val2)
            return;
        S_key_open = val1;
        if (S_key_open == ON)
        {
            sprintf(buffer,"Key open is now %d, count %d",val1,Key_Open_Count);
            Serial.println(buffer);
        }
        else
            Serial.println(F("Key open is now 0"));
        KeyOpenChange();
        return;
    }

    val1 = digitalRead(S_KEY_CLOSE);
    if (val1 != S_key_close)
    {
        delay(DEBOUNCE);
        val2 = digitalRead(S_KEY_CLOSE);
        if (val1 != val2)
            return;
        S_key_close = val1;
        if (S_key_close == ON)
        {
            sprintf(buffer,"Key close is now %d, count %d",val1,Key_Close_Count);
            Serial.println(buffer);
        }
        else
            Serial.println(F("Key close is now 0"));
        KeyCloseChange();
        return;
    }
}

//-----------------------------------------------------------------
// Handle a change in the Key open state
// When reading the key, short activations of less than KEY_ACTIVATE_TIME milli-seconds do not activate the motors,
// but do count towards a possible auto-open or auto-close.
// If we get AUTO_MODE_COUNT key opens of less than KEY_ACTIVATE_TIME within AUTO_KEY_TIME,
// we set COMMAND_AUTO_OPEN, which will execute Open() until we have opened NUMBER_OF_SECTIONS or until the key is activated again
//
void KeyOpenChange()
{
    if (S_key_open == OFF)
    {
        if (Command == COMMAND_OPEN)              // if we are doing a normal open
            Command = COMMAND_NONE;                // .. stop
        Time_Open_Key_Activated = -1;
        return;
    }

// Key Open has just changed to ON

    Command = COMMAND_NONE;                      // if a command was running, stop it

    Time_Open_Key_Activated = millis();
    Key_Open_Count ++;
    if (Key_Open_Count == 1)
    {
        Time_First_Key_Open = millis();
        return;
    }

    if (Key_Open_Count == AUTO_MODE_COUNT)       // if we got enough activations ..
    {
        Auto_Section_Count = 0;
        Command = COMMAND_AUTO_OPEN;
        Serial.println(F("*** Auto-open start"));
        Beep(1);
        return;
    }
}

//-----------------------------------------------------------------
// Handle a change in the Key close state
// When reading the key, short activations of less than KEY_ACTIVATE_TIME milli-seconds do not activate the motors,
// but do count towards a possible auto-open or auto-close.
// If we get AUTO_MODE_COUNT key closes of less than KEY_ACTIVATE_TIME within AUTO_KEY_TIME,
// we set COMMAND_AUTO_CLOSE, which will execute Close() until we have closed NUMBER_OF_SECTIONS or until the key is activated again
//
void KeyCloseChange()
{
    if (S_key_close == OFF)
    {
        if (Command == COMMAND_CLOSE)               // if we are doing a normal close
            Command = COMMAND_NONE;                   // .. stop
        Time_Close_Key_Activated = -1;
        return;
    }

// Key Close has just changed to ON

    Command = COMMAND_NONE;                         // if a command was running, stop it

    Time_Close_Key_Activated = millis();
    Key_Close_Count ++;
    if (Key_Close_Count == 1)
    {
        Time_First_Key_Close = millis();
        return;
    }

    if (Key_Close_Count == AUTO_MODE_COUNT)         // if we got enough activations ..
    {
        Auto_Section_Count = 0;
        Command = COMMAND_AUTO_CLOSE;
        Serial.println(F("*** Auto-close start"));
        Beep(1);
        return;
    }
}

//-----------------------------------------------------------------
// Analyse the state of the key switches and set the Command
//
void HandleKeyState()
{
    if ((S_key_open == ON) && (S_key_close == ON))
    {
        Report_Numbered_Error(27, "*** Key open and close should never be on at the same time");
        return;
    }

    if ((millis() - Time_First_Key_Open) > AUTO_KEY_TIME)       // after this many milli-seconds
        Key_Open_Count = 0;                                      // activations no longer count

    if ((millis() - Time_First_Key_Close) > AUTO_KEY_TIME)      // after this many milli-seconds
        Key_Close_Count = 0;                                     // activations no longer count

// if the open key is still on after KEY_ACTIVATE_TIME, it's a normal open

    if ((S_key_open == ON) && ((millis() - Time_Open_Key_Activated) > KEY_ACTIVATE_TIME))
    {
        Key_Open_Count = 0;                                      // it no longer counts towards AUTO_MODE_COUNT
        Command = COMMAND_OPEN;                                  // it's just a normal "manual" open
        return;
    }

// if the close key is still on after KEY_ACTIVATE_TIME, it's just a normal close

    if ((S_key_close == ON) && ((millis() - Time_Close_Key_Activated) > KEY_ACTIVATE_TIME))
    {
        Key_Close_Count = 0;                                     // it no longer counts towards AUTO_MODE_COUNT
        Command = COMMAND_CLOSE;                                 // it's just a normal "manual" close
        return;
    }
}

//-----------------------------------------------------------------
// Handle commands from the serial port
//
void ProcessCommand()
{
    char command, command2;
    int ram;

    command = Serial.read();

    switch (command)
    {
        case 's':
            Serial.println(F("Command: Stop"));
            All_Stop();
            Command = COMMAND_NONE;
            State = STATE_OFF;
            return;

        case 'd':
            Serial.println(F("Command: Disable"));
            All_Stop();
            State = STATE_DISABLED;
            Command = COMMAND_NONE;
            return;

        case 'e':
            Serial.println(F("Command: Enable"));
            State = OFF;
            Command = COMMAND_NONE;
            return;

        case '?':
            Serial.println(F("Command: Print all Switches"));
            PrintAllSwitches();
            return;

        case 'm':
            Serial.println(F("Command: Show Memory Usage"));
            ram = freeRam();
            sprintf(buffer,"Free RAM1 = %d",ram);
            Serial.println(buffer);
            ram = availableMemory();
            sprintf(buffer,"Free RAM2 = %d",ram);
            Serial.println(buffer);
            check_mem();
            sprintf(buffer,"heapptr = %d, stackptr = %d",heapptr, stackptr);
            Serial.println(buffer);
            return;

        case 'o':
            Serial.println(F("Command: Open"));
            Command = COMMAND_OPEN;
            return;

        case 'c':
            Serial.println(F("Command: Close"));
            Command = COMMAND_CLOSE;
            return;

        case 'v':
            Get_Voltage(true);
            return;

        case 't':
            Serial.println(F("Command: Relay Test"));
            Relay_Test();
            return;

        case 'q':                        // enable or disable the buzzer
            if (Quiet_Mode)
            {
                Serial.println(F("Command: Quiet Mode Off"));
                Quiet_Mode = false;
                Beep(2);
            }
            else
            {
                Serial.println(F("Command: Quiet Mode On"));
                Quiet_Mode = true;
            }
            return;

        case 'l':                           // start a left motor
            do {command2 = Serial.read();}
            while (command2 == -1);       // wait for a second command character
            switch (command2)
            {
                case 'o':
                    Serial.println(F("Command: Left Open"));
                    Motor_Start(M_LEFT_OPEN);
                    break;
                case 'c':
                    Serial.println(F("Command: Left Close"));
                    Motor_Start(M_LEFT_CLOSE);
                    break;
                case 'u':
                    Serial.println(F("Command: Left Up"));
                    Motor_Start(M_LEFT_UP);
                    break;
                case 'd':
                    Serial.println(F("Command: Left Down"));
                    Motor_Start(M_LEFT_DOWN);
                    break;
            }
            Command = COMMAND_OTHER;
            return;

        case 'r':                           // start a right motor
            do {command2 = Serial.read();}
            while (command2 == -1);       // wait for a second command character
            switch (command2)
            {
                case 'o':
                    Serial.println(F("Command: Right Open"));
                    Motor_Start(M_RIGHT_OPEN);
                    break;
                case 'c':
                    Serial.println(F("Command: Right Close"));
                    Motor_Start(M_RIGHT_CLOSE);
                    break;
                case 'u':
                    Serial.println(F("Command: Right Up"));
                    Motor_Start(M_RIGHT_UP);
                    break;
                case 'd':
                    Serial.println(F("Command: Right Down"));
                    Motor_Start(M_RIGHT_DOWN);
                    break;
            }
            Command = COMMAND_OTHER;
            return;

        case 'b':                           // start a pair of motors
            do {command2 = Serial.read();}
            while (command2 == -1);       // wait for a second command character
            switch (command2)
            {
                case 'o':
                    Serial.println(F("Command: Both Open"));
                    Motor_Start(M_LEFT_OPEN);
                    Motor_Start(M_RIGHT_OPEN);
                    break;
                case 'c':
                    Serial.println(F("Command: Both Close"));
                    Motor_Start(M_LEFT_CLOSE);
                    Motor_Start(M_RIGHT_CLOSE);
                    break;
                case 'u':
                    Serial.println(F("Command: Both Up"));
                    Motor_Start(M_LEFT_UP);
                    Motor_Start(M_RIGHT_UP);
                    break;
                case 'd':
                    Serial.println(F("Command: Both Down"));
                    Motor_Start(M_LEFT_DOWN);
                    Motor_Start(M_RIGHT_DOWN);
                    break;
            }
            Command = COMMAND_OTHER;
            return;

        // Ucha custom commands start with capitals
        case 'D':                        // turn on debug logging
            Serial.println(F("Command: DEBUG Mode On. Use I to return to INFO"));
            Log_Level = LOG_LEVEL_DEBUG;
            return;
        case 'T':                        // turn on debug logging
            Serial.println(F("Command: TRACE Mode On. Use I to return to INFO"));
            Log_Level = LOG_LEVEL_TRACE;
            return;
        case 'I':                        // turn on debug logging
            Serial.println(F("Command: INFO Mode On. Use D for DEBUG or T for TRACE"));
            Log_Level = LOG_LEVEL_INFO;
            return;

        case 'F':                        // enable force mode
            if (Force_Mode)
            {
                Serial.println(F("Command: Force Mode Off"));
                Force_Mode = false;
                Beep(2);
            }
            else
            {
                Serial.println(F("Command: Force Mode On, ignoring battery warnings etc"));
                Force_Mode = true;
            }
            return;

        case 'H':
            Serial.println(F("Commands:"));
            Serial.println(F(" H show this help"));
            Serial.println(F("  ? Show the current state of all the switches"));
			Serial.println(F("t Test all relays - switches all the relays on for 2 seconds"));
			Serial.println(F("lu Left Up"));
			Serial.println(F("ld Left Down"));
			Serial.println(F("lo Left Open"));
			Serial.println(F("lc Left Close"));
			Serial.println(F("ru Right Up"));
			Serial.println(F("rd Right Down"));
			Serial.println(F("ro Right Open"));
			Serial.println(F("rc Right Close"));
			Serial.println(F("bu Both Up"));
			Serial.println(F("bd Both Down"));
			Serial.println(F("bo Both Open"));
			Serial.println(F("bc Both Close"));
			Serial.println(F("s Stop all motors"));
			Serial.println(F("o Simulate the key being set to open the cover (stop with the 's' command)"));
			Serial.println(F("c Simulate the key being set to close the cover (stop with the 's' command)"));
			Serial.println(F("d Disable all motors - no motors can run until an 'e' command is entered"));
			Serial.println(F("e Enable - cancel the 'disabled' state"));
			Serial.println(F("q Quiet mode - disables the buzzer and battery warnings"));
			Serial.println(F("v Show the current battery voltage"));
			Serial.println(F("m Show the current memory usage"));
            Serial.println(F("F Toggle Force mode to ignore battery-too-low, etc"));
            Serial.println(F("D|T|I Debug|Trace|Info level logging"));
    }

}

//-----------------------------------------------------------------
// Show the value of all input switches on the serial monitor
//
void PrintAllSwitches()
{
    char str[12];
    sprintf(buffer,"Left open is    %d",S_left_open);
    Serial.println(buffer);
    sprintf(buffer,"Left closed is  %d",S_left_closed);
    Serial.println(buffer);
    sprintf(buffer,"Left lifted is  %d",S_left_lifted);
    Serial.println(buffer);
    sprintf(buffer,"Right open is   %d",S_right_open);
    Serial.println(buffer);
    sprintf(buffer,"Right closed is %d",S_right_closed);
    Serial.println(buffer);
    sprintf(buffer,"Right lifted is %d",S_right_lifted);
    Serial.println(buffer);
    sprintf(buffer,"Key open is     %d",S_key_open);
    Serial.println(buffer);
    sprintf(buffer,"Key close is    %d",S_key_close);
    Serial.println(buffer);
    sprintf(buffer,"Section count   %d",Auto_Section_Count);
    Serial.println(buffer);
    sprintf(buffer,"Command: %d",Command);
    Serial.println(buffer);
    sprintf(buffer,"State:   %d",State);
    Serial.println(buffer);
    Serial.println(F("------------------"));
}

//-----------------------------------------------------------------
// Check if any motor has been running too long
// - if so, report the error, stop all motors, and return true
// - if all ok returns false
//
boolean Check_Max_Times()
{
    long left_lift_runtime;
    long right_lift_runtime;
    long left_move_runtime;
    long right_move_runtime;
    long difference;

    if (Time_Left_Lift_Started != -1)
    {
        left_lift_runtime = millis() - Time_Left_Lift_Started;
        if (left_lift_runtime > MAX_LIFT_TIME)
        {
            Report_Numbered_Error(15, "*** Left lift exceeded MAX_LIFT_TIME");
            return true;
        }
    }

    if (Time_Right_Lift_Started != -1)
    {
        right_lift_runtime = millis() - Time_Right_Lift_Started;
        if (right_lift_runtime > MAX_LIFT_TIME)
        {
            Report_Numbered_Error(15, "*** Right lift exceeded MAX_LIFT_TIME");
            return true;
        }
    }

    if (Time_Open_Started != -1)
    {
        left_move_runtime = millis() - Time_Open_Started;
        if (left_move_runtime > MAX_MOVE_TIME)
        {
            Report_Numbered_Error(17,"*** Open exceeded MAX_MOVE_TIME");
            return true;
        }
    }

    if (Time_Left_Move_Started != -1)
    {
        left_move_runtime = millis() - Time_Left_Move_Started;
        if (left_move_runtime > MAX_MOVE_TIME)
        {
            Report_Numbered_Error(19, "*** Left close exceeded MAX_MOVE_TIME");
            return true;
        }
    }

    if (Time_Right_Move_Started != -1)
    {
        right_move_runtime = millis() - Time_Right_Move_Started;
        if (right_move_runtime > MAX_MOVE_TIME)
        {
            Report_Numbered_Error(21, "*** Right close exceeded MAX_MOVE_TIME");
            return true;
        }
    }

// if one move motor is running but not the other, we don't want that to continue for very long

    if ( ((Time_Left_Move_Started != -1) || (Time_Right_Move_Started != -1))         // if either is running ..
         &&   ((Time_Left_Move_Started == -1) || (Time_Right_Move_Started == -1)) )       // and either is not running
    {
        if (Time_Separate_Move_Started == -1)        // if it has only just started happening ..
            Time_Separate_Move_Started = millis();
        else
        {
            difference = millis() - Time_Separate_Move_Started;
            if (difference > MAX_MOVE_DIFFERENCE)
            {
                Report_Numbered_Error(23, "*** Exceeded MAX_MOVE_DIFFERENCE");
                return true;
            }
        }
    }
    else
        Time_Separate_Move_Started = -1;

// if the key has been active for too long, that's an error too

    if (Time_Open_Key_Activated != -1)
    {
        difference = millis() - Time_Open_Key_Activated;
        if (difference > MAX_KEY_TIME)
        {
            Report_Numbered_Error(25, "*** Exceeded MAX_KEY_TIME");
            return true;
        }
    }

    if (Time_Close_Key_Activated != -1)
    {
        difference = millis() - Time_Close_Key_Activated;
        if (difference > MAX_KEY_TIME)
        {
            Report_Numbered_Error(25, "*** Exceeded MAX_KEY_TIME");
            return true;
        }
    }

    return false;
}

//-----------------------------------------------------------------
// Get the 12 volt battery voltage
// returns battery voltage * 10
//
int Get_Voltage(boolean print)
{
    long val = analogRead(V_INPUT);
    int volts = (val * 50)/ V_FACTOR;

    if (print)
    {
        sprintf(buffer,"Battery voltage = %d.%d",(volts/10),(volts%10));
        Serial.println(buffer);
    }

    if (Quiet_Mode)
        return volts;

    if ((volts < VOLTAGE_WARNING) && (!Battery_Warning_Sounded))
    {
        sprintf(buffer,"*** WARNING: Battery voltage = %d.%d",(volts/10),(volts%10));
        Serial.println(buffer);
        Beep(3);
        Battery_Warning_Sounded = true;  // don't beep again until key off/on
    }

    if ((volts < VOLTAGE_ERROR) && (State != STATE_ERROR))
    {
        delay(10);                       // wait a while and check again
        val = analogRead(V_INPUT);
        volts = (val * 50)/ V_FACTOR;
        if (volts < VOLTAGE_ERROR) {
            if (Force_Mode) {
                INFO("IGNORING : Battery voltage is too low to run (%d.%d)", (volts / 10), (volts % 10))
            } else {
                sprintf(buffer, "Battery voltage is too low to run (%d.%d)", (volts / 10), (volts % 10));
                Report_Numbered_Error(7,buffer);
                Beep(100);                    // Abrisud controller gives one 3 second beep for this case
            }
        }
    }

    return volts;
}

//-----------------------------------------------------------------
// Report an error
//
void Report_Error(char * message)
{
    Report_Numbered_Error(0, message);
}

void Report_Numbered_Error(int number, char * message)
{
    Serial.print(F("*** ERROR: "));
    Serial.println(message);
    DEBUG("Report_Error => All Stop")
    All_Stop();
    if (number > 0) {
        ErrorCodeBeep(number);
    }
    else {
        Beep(4);
    }
    State = STATE_ERROR;           // do nothing until key off/on
}

void ErrorCodeBeep(int number)
{
    DEBUG("Error Beep %d", number)
    Beep(number);
    delay(800);
    Beep(number);
    delay(800);
    Beep(number);
    delay(800);
}

//-----------------------------------------------------------------
// Beep a number of times
// 100 is a special case, giving one long 3 second beep
//
void Beep(int number)
{
    TRACE("Start Beep %d", number);
    if (Quiet_Mode) {
        DEBUG("Quiet mode: no beep");
        return;
    }

    if (number == 100)
    {
        digitalWrite(BUZZER, HIGH);
        delay(3000);
        digitalWrite(BUZZER, LOW);
        TRACE("Finishing Beep 100");
        return;
    }

    for (int i = 0; i < number; i++)
    {
        digitalWrite(BUZZER, HIGH);
        delay(100);
        digitalWrite(BUZZER, LOW);
        delay(200);
    }
    TRACE("Finishing Beep %d", number);
}

void Debug_Beep(int number)
{
    if (Log_Level >= LOG_LEVEL_DEBUG) {
        Beep(number);
    }
}

//-----------------------------------------------------------------
// Test all relays - this is never called during live running
//
void Relay_Test()
{
    Motor_Start(M_LEFT_UP);
    Motor_Start(M_RIGHT_UP);
    Motor_Start(M_LEFT_DOWN);
    Motor_Start(M_RIGHT_DOWN);
    Motor_Start(M_LEFT_OPEN);
    Motor_Start(M_RIGHT_OPEN);
    Motor_Start(M_LEFT_CLOSE);
    Motor_Start(M_RIGHT_CLOSE);
    delay(2000);
    Motor_Stop(M_LEFT_UP);
    Motor_Stop(M_RIGHT_UP);
    Motor_Stop(M_LEFT_DOWN);
    Motor_Stop(M_RIGHT_DOWN);
    Motor_Stop(M_LEFT_OPEN);
    Motor_Stop(M_RIGHT_OPEN);
    Motor_Stop(M_LEFT_CLOSE);
    Motor_Stop(M_RIGHT_CLOSE);
}

//-----------------------------------------------------------------
// Switch a motor On
//
void Motor_Start(int pin)
{
    TRACE("Motor Start: %d", pin);
    digitalWrite(pin,LOW);
}

//-----------------------------------------------------------------
// Switch a motor Off
//
void Motor_Stop(int pin)
{
    TRACE("Motor Stop: %d", pin);
    digitalWrite(pin,HIGH);
}

//-----------------------------------------------------------------
// Various functions to get the amount of remaining RAM
// From http://playground.arduino.cc/Code/AvailableMemory
//
int freeRam()
{
    extern int __heap_start, *__brkval;
    int v;

    return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

int availableMemory()
{
    // Ucha changed from 1024: my Arduino CPI is ATmega328p = 2k sram
    int size = 2048; // Use 2048 with ATmega328
    byte *buf;

    while ((buf = (byte *) malloc(--size)) == NULL);
    free(buf);
    return size;
}

void check_mem()
{
    stackptr = (uint8_t *)malloc(4);          // use stackptr temporarily
    heapptr = stackptr;                     // save value of heap pointer
    free(stackptr);      // free up the memory again (sets stackptr to 0)
    stackptr =  (uint8_t *)(SP);           // save value of stack pointer
}

