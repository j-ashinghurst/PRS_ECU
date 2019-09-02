



/**TO-DO:
   Better RPM Measurement during shift RPM change verification (minimum time period required to get within RPMTOLERANCE)
   Implement datalogging
   Auto Shift (shift up at 100% throttle, shift down at 50% throttle[requires bias throttle system TBD])
   bias throttle: for motor speeds greater then 1mph, apply very low current limit when commanded throttle is less than motor speed
   reverse to enter gear 1 from gear 0



*/
#define DEBUGMODE   false  //CAUTION, IF TURNED ON (true) THE CODE WILL WAIT FOR REVERSE PRESS ON BOOT BEFORE DOING ANYTHING ELSE!!  make this true to turn on serial port print statements.
#define DATALOGGING true
//This is a pre-made library from sparkfun
#include <PulseServo.h>
#include <FreqMeasureMulti.h>
#include <SimpleKalmanFilter.h>

#define DPIN_RX             0
#define DPIN_TX             1
#define DPIN_ENCODER_1A     3
#define DPIN_ENCODER_1B     4
#define DPIN_ESCOUT         6
#define DPIN_BRAKE          7
//shift up button
#define DPIN_SHIFTUP        9
//shift down button
#define DPIN_SHIFTDOWN      10
#define DPIN_REVERSE        11
#define DPIN_ENCODER_2A     25//wire to pin 8
#define DPIN_ENCODER_2APASS 8
#define DPIN_ENCODER_2B     32//wire to pin 17 (A3)
#define DPIN_ENCODER_2BPASS 17
#define DPIN_MOTOR_1        22
#define DPIN_MOTOR_2        23

//analog
#define APIN_THROTTLE       A1//15
#define APIN_CURRENT        A0//14
#define APIN_BATTERY        A2//16
//----------------------------LOOP AND SAMPLE TIMING--------------------
#define MINPERIOD             1000   //LOOP AND SAMPLE TIMING  //minimum time between sending out pulses.  this enures we do not send out a new pulse before one is finished.
#define SAMPLEBUFFERSIZE      100
unsigned long prevmicros =    0;                //LOOP AND SAMPLE TIMING  //the last time we SHOULD HAVE completed a loop
unsigned long prevperiod =    0;                //LOOP AND SAMPLE TIMING  //This is the last time we ACTUALLY completed a loop
unsigned long prevsample =    0;                //the millis duration of the last sampling period.  used to calculate variable I2P
unsigned long thistime =      0;
unsigned long serialtime =    1;
#include "CMD_Defs.h"
#include "QuadDecode_def.h"
#include "Datalog_Defs.h"
//--------------------------------THROTTLE INPUT AND OUTPUT----------------------------
#define THROTTLEMIN               20
#define THROTTLEMAX               600
#define SERVODEAD                 16                         //THROTTLE INPUT AND OUTPUT  //space between this and the zero band won't be sent because it will cause odd behavior with the ESC
#define SERVOHYSTERESIS           11                          //wont turn off below SERVODEAD until we're this much below the dead zone
#define SERVOOFFSET               0
#define SERVOMIN                  74              //LOOP AND SAMPLE TIMING  //absolute minimum value writable to the servo
#define SERVOMAX                  926              //LOOP AND SAMPLE TIMING  //absolute maximum value writable to the servo
#define SERVOZERO                 (SERVOMAX+SERVOMIN)/2       //THROTTLE INPUT AND OUTPUT  //'Zero' point for servo library.
#define BRAKEPOWER                0                   //THROTTLE INPUT AND OUTPUT  //amount of negative throttle to apply when we hit the brakes.  right now, it's NOTHING.  Because regen be bad for the fuse!
#define DEBOUNCE                  50                          //debounce time for the reverse switch because I'm too lazy to do good wire routing.  also the 'pause' time that ramp up is held at the minimum rate
#define MINIMUM_DUTY              6
#define MAXIMUM_DUTY              1000
unsigned int THROTTLERANGE =      THROTTLEMAX - THROTTLEMIN;  //THROTTLE INPUT AND OUTPUT  //the total ADC counts difference in our min and max throttle readings
boolean reversing =               0;
int traveldirection =             0;
#define GOFORWARD                 1
#define GOREVERSE                 -1
float throttlepos =               0;    //the throttle we calculate.  Initialize with a throttle output of zero
unsigned int maxthrottle =              SERVOMAX;
boolean atmaxthrottle =                 false;
long maxthrottlestart =                 0;
int thisthrottle = 0;
float coastthrottlepos = throttlepos;
float thisvoltage = 0;
#define FULLSCALEVOLTAGE 67.7f
boolean thisbrake = false;
boolean thisreverse = false;
boolean rpmavailableflag = false;
#define RAMPUPRATECONST  0.4f  //ramp up 1uS of throttle output per each 1mS of time
#define RAMPUPRATESCALE  0.6f  //ramp up 1uS of throttle output per each 1mS of time
#define GEAR0RAMPSCALER 2.0f
#define GEAR1RAMPSCALER 1.33f
#define GEAR2RAMPSCALER 1.0f
#define GEAR3RAMPSCALER 0.75f
float gearrampscalers[] = {GEAR0RAMPSCALER, GEAR1RAMPSCALER, GEAR2RAMPSCALER, GEAR3RAMPSCALER};
float lastthrottle = 0;


//------------------------CURRENT LIMITING VARIABLES----------------

#define FULLSCALECURRENT      1391    //139.1A full scale
#define CURRENTOFFSET     -3       //subtract 0.9A from current reading
#define CURRENTTHRESHOLD  2      //anything beluow 0.1A is interpreted as 0A
#define MAXCURRENTLIMIT   500     //50A through a 30A fuse is roughly equivalent to our overcurrent level on a 50A fuse
#define MINCURRENTLIMIT   30
#define MINLIMITCROSSOVERRPM  700
#define MAXLIMITCROSSOVERRPM  2500
#define FUSENOMINAL       .0035f  //nominal fuse resistance during operation: 3.5mOhm
#define COASTCURRENT3TERM 0.0000000000285f
#define COASTCURRENT2TERM -0.000000122f
#define COASTCURRENT1TERM 0.000857f
#define COASTCURRENT0TERM 1.185f //data shows the average is 0.185.  have added 1A  to ensure we always stay below this limit

int currents[SAMPLEBUFFERSIZE];
float thiscurrent = 0;
long thiscurrentlimit = 0;

//------------------------TRANSMISSION VARIABLES--------------------
#define GEARUNDEFINED -1 //when we are shifting
#define GEARNEUTRAL   0
#define GEARONE       1
#define GEARTWO       2
#define GEARTHREE     3

#define SHIFTINGDOWN -1
#define NOTSHIFTING   0
#define SHIFTINGUP    1

#define SHIFTIDLE             0
#define SHIFTSTAGE_OFFGAS     1
#define SHIFTSTAGE_SERVOSLEW  2
#define SHIFTSTAGE_ONGAS      3

#define UPSHIFT_OFFGAS_TIMEOUT  2000000   //2s
#define UPSHIFT_SLEW_TIMEOUT    200000    //200ms
#define UPSHIFT_ONGAS_TIMEOUT   250000     //250ms

#define DOWNSHIFT_OFFGAS_TIMEOUT  10000   //10ms
#define DOWNSHIFT_SLEW_TIMEOUT    200000  //200ms
#define DOWNSHIFT_ONGAS_TIMEOUT   1000000 //1s

#define REVMATCHSCALER 0.9995f             //exponential decay

#define AUTOSHIFTLATENCY  1500000
unsigned long lastautoshift = 0;
unsigned long shifttimeout = 0;
#define RPMTOLERANCE    50
int startRPMs = 0;
int gear = 1;
int targetgear = 1;
int currentposition = 0;
boolean shiftmask = true;
boolean justshifted = false;
int gearaction = NOTSHIFTING;
int shiftaction = SHIFTIDLE;
boolean thisshiftup = false;
boolean thisshiftdown = false;
int targetRPMs = 0;
unsigned long shifttime = 0;
boolean autoshift = false;
boolean autoshiftuptrigger = false;
boolean gearboxreverse = false;
float thisshiftthrottle = 0;
// Factory default of Dynamixel servos is ID=1, Baudrate= 1 MegaBaud
// Factory default of Dynamixel servos is ID=1, Baudrate= 1 MegaBaud
#define SERVO_ID_L  2
#define SERVO_ID_R  2
#define SERVO_ID_BROADCAST  254
#define BAUDRATE  19200
#define GEAR0POSITION 558
#define GEAR1POSITION 300
#define GEAR2POSITION 474
#define GEAR3POSITION 668
#define CWDIR false
#define CCWDIR true
#define ECHOTIMEOUT 10
// For debugging in Serial Monitor
#define BAUD1 1
#define BAUD2 2
#define BAUD3 3
#define BAUD4 4
#define BAUD5 5
#define BAUD6 6
#define BAUD7 7
#define BAUD8 8
#define BAUD9 9
unsigned int Gearpositions[] = {GEAR0POSITION, GEAR1POSITION, GEAR2POSITION, GEAR3POSITION};
byte Servobaud[] = {207, 207, 103, 34, 16, 9, 7, 4, 3, 1};
long Serialbaud[] = {9600, 9600, 19200, 57600, 115200, 200000, 250000, 400000, 500000, 1000000};

// The Dynamixel instructions
enum eInstruction
{
  I_Ping      = 1,
  I_ReadData  = 2,
  I_WriteData = 3,
  I_RegWrite  = 4,
  I_Action    = 5,
  I_Reset     = 6,
  I_SyncWrite = 0x83,
};

// The Dynamixel registers
enum eRegister
{
  // ---------- EEPROM ------------

  R_ModelNumber             = 0x00, // 2 Byte
  R_FirmwareVersion         = 0x02, //
  R_ServoID                 = 0x03, //         Write
  R_BaudRate                = 0x04, //         Write
  R_ReturnDelayTime         = 0x05, //         Write
  R_CW_AngleLimit           = 0x06, // 2 Byte  Write
  R_CCW_AngleLimit          = 0x08, // 2 Byte  Write
  R_HighestLimitTemperature = 0x0B, //         Write
  R_LowestLimitVoltage      = 0x0C, //         Write
  R_HighestLimitVoltage     = 0x0D, //         Write
  R_MaxTorque               = 0x0E, // 2 Byte  Write
  R_StatusReturnLevel       = 0x10, //         Write
  R_AlarmLED                = 0x11, //         Write
  R_AlarmShutdown           = 0x12, //         Write
  R_DownCalibration         = 0x14, // 2 Byte
  R_UpCalibration           = 0x16, // 2 Byte

  // ----------- RAM -------------

  R_TorqueEnable            = 0x18, //         Write
  R_LED                     = 0x19, //         Write
  R_CW_ComplianceMargin     = 0x1A, //         Write
  R_CCW_ComplianceMargin    = 0x1B, //         Write
  R_CW_ComplianceSlope      = 0x1C, //         Write
  R_CCW_ComplianceSlope     = 0x1D, //         Write
  R_GoalPosition            = 0x1E, // 2 Byte  Write
  R_MovingSpeed             = 0x20, // 2 Byte  Write
  R_TorqueLimit             = 0x22, // 2 Byte  Write
  R_PresentPosition         = 0x24, // 2 Byte
  R_PresentSpeed            = 0x26, // 2 Byte
  R_PresentLoad             = 0x28, // 2 Byte
  R_PresentVoltage          = 0x2A, //
  R_PresentTemperature      = 0x2B, //
  R_RegisteredInstruction   = 0x2C, //         Write
  R_Moving                  = 0x2E, //
  R_Lock                    = 0x2F, //         Write
  R_Punch                   = 0x30, // 2 Byte  Write
};

// Dynamixel errors returned in Status packet
enum eError
{
  E_InputVoltage   = 0x01,
  E_AngleLimit     = 0x02,
  E_Overheating    = 0x04,
  E_ParameterRange = 0x08,
  E_Checksum       = 0x10,
  E_Overload       = 0x20,
  E_Instruction    = 0x40,
};


// Global variables
bool gb_LED = false;

//------------------------ENCODER VARIABLES--------------------

long xpos = 0;
long carpos = 0;
long rpms = 0;
float curdist = 0.0;
float velocity = 0.0;
long prevxpos = 0;
long motorpositions[SAMPLEBUFFERSIZE];
long axlepositions[SAMPLEBUFFERSIZE];
unsigned long times[SAMPLEBUFFERSIZE];
int thisindex = 0;
boolean cyclecomplete = false;
float prevdist = 0;
float prevvelocity = 0.0;
boolean goingforward = true;
long lastspeedloopmillis = 0;
float axlerpms = 0;
float expectedmotorrpms = 0;
boolean coasting = false;
unsigned long lastrpmreading = 0;

#define RPMLOWCUTOFF  100     //the speed below which we will attempt to go into reverse to shift (since shifting a nonmoving gearbox can damage the gearbox, servo, or other mechanism components
#define LOWRPMTHRESHOLD  550
#define LOWRPMTHROTTLE   0.22f
#define RPMHIGHLIMIT 8000.0f
#define LOOKBACKTIME_RPM  100 //milliseconds, how far back to look to get a baseline for motor RPM.
#define LOOKBACKTIME_SPEED  60 //milliseconds, how far back to look to get a baseline for speed.
#define LOOPTIME 1  //ENCODER SHOULD BE ANSWERED EVERY 8K COUNTS *max*.  For motors, at 7100RPM/4096 counts per rev, this is ~16ms.  10 should be OK.  For axles, at 10ms this works out to 229 counts per interval per 10mph
#define COUNTSPERREV 42 //14 poles * 3 commutations = 42
#define COUNTSPERRPM COUNTSPERREV * LOOPTIME / 60000.0f
#define COUNTSPERMILE 4130429.8f
#define COUNTSPERFOOT 750.8f      //assumes that dynapar is reduced 53.9/25.9 from axle, 4096 counts/dynapar rev, 10" tires with 
#define FEETPERMILE 5280
#define SPEEDRPMFACTOR 33.61f
#define FREQTORPM 2.85714f//1.4286f   //returning 1Hz results in ~1.4RPM, with 42 increments per rotation it takes 42 seconds to make one rotation at 1Hz between commutations
#define MOTORKV 130
#define MOTORSPROCKET       9.0f
#define GEARHUBINSPROCKET   29.0f
#define GEARHUBOUTSPROCKET  19.0f
#define AXLESPROCKET        59.0f
#define GEAR2RATIO  AXLESPROCKET*GEARHUBINSPROCKET/(MOTORSPROCKET*GEARHUBOUTSPROCKET)   //defined this way (95% of third) so we can jump to *any* gear from neutral and the motor will always be spinning slow enough
#define GEAR1RATIO  GEAR2RATIO*1.33f
#define GEAR3RATIO  GEAR2RATIO*0.75f
#define GEAR0RATIO  GEAR3RATIO*0.95f

float gearratios[] = {GEAR0RATIO, GEAR1RATIO, GEAR2RATIO, GEAR3RATIO};

boolean speedsensing = false;

#define THROTTLERPM4TERM  -0.000009035f
#define THROTTLERPM3TERM  0.002797f
#define THROTTLERPM2TERM  -0.2821f
#define THROTTLERPM1TERM  13.75f
#define THROTTLERPM0TERM  -166.6f
#define MAPPINGLOWCUTOFF  40.0f     //RPM/VOLTAGE can't go below this number, otherwise our trendline is invalid



#define THROTTLERPMCALPOINT 53.4f
#define COASTPERCENTAGE 0.90f
#define COASTBRAKEOFFSET 50
#define MINCOASTRPM  1000
#define COASTHYSTERESIS 250
//------------------------GENERIC VARIABLES--------------------

#define BUTTONDEBOUNCETIME 20000  //20ms
boolean shiftupdebounce = false;
boolean shiftdowndebounce = false;
unsigned long updebouncetime = 0;
unsigned long downdebouncetime = 0;
float thrdivisor = (SERVOMAX - SERVOZERO) / THROTTLERANGE;
#define ADCBITS           10
#define FULLSCALEADC      1024          //CURRENT LIMITING  //why yes, that is a 10-bit ADC in my pocket
#define PERCENT           100.0f               //GENERIC VARIABLES  
//
//INSTANTIATIONS
PulseServo myservo;          //the output is basically just a servo output, specially-controlled

FreqMeasureMulti Lmotor;
QuadDecode<1> LaxlePosn;  // Template using FTM2


//------------------------ANALOG SAMPLING-------------------------------------
//
//  takes 8 samples of all our analog inputs,
//  and constrains them to our relevant data ranges
//
//------------------------ANALOG SAMPLING-------------------------------------
void sample()
{
  //get our next sample number.
  thisindex++;
  if (thisindex >= SAMPLEBUFFERSIZE)
  {
    thisindex = 0;
  }
  //get our sample time.  the system tries to iterate at a certain rate, but everything is timed so it is relatively sample rate agnostic
  thistime = micros();
  times[thisindex] = thistime;
  //get our encoder positions.  at 100RPM in low gear, axle encoder is ~200pulses/sec and motor encoder is ~70pulses/sec
  motorpositions[thisindex] = 0;//THISISBROKEN: use freqmeasuremulti;
  axlepositions[thisindex] = LaxlePosn.calcPosn();

  //take our digital samples
  thisbrake = !digitalRead(DPIN_BRAKE);
  //thisreverse = !digitalRead(DPIN_REVERSE);
  autoshift = false;
  thisshiftup = false;
  thisshiftdown = false;
  if (!digitalRead(DPIN_SHIFTUP))
  {
    if (!digitalRead(DPIN_SHIFTDOWN))
    { //if both are pressed/switched together, auto shift.  best not to actuate this during operation, only when off.
      shiftupdebounce = false;
      shiftdowndebounce = false;
    }
    else
    {
      if (!shiftupdebounce)
      {
        shiftupdebounce = true;
        shiftdowndebounce = false;
        updebouncetime = micros();
      }
      else
      {
        if (micros() - updebouncetime > BUTTONDEBOUNCETIME)
        {
          thisshiftup = true;
        }
      }
    }
  }
  else
  {
    shiftupdebounce = false;
    if (!digitalRead(DPIN_SHIFTDOWN))
    {
      if (!shiftdowndebounce)
      {
        shiftdowndebounce = true;
        shiftupdebounce = false;
        downdebouncetime = micros();
      }
      else
      {
        if (micros() - downdebouncetime > BUTTONDEBOUNCETIME)
        {
          thisshiftdown = true;
        }
      }
    }
    else
    {
      shiftdowndebounce = false;
    }
  }

  //reset our analog readings
  thiscurrent = 0;
  thisthrottle = 0;
  thisvoltage = 0;
  //take a number of samples and decimate to filter
  int divisor = 8;
  for (int i = 0; i < divisor; i++)
  {
    thiscurrent += analogRead(APIN_CURRENT);
    thisthrottle += analogRead(APIN_THROTTLE);
    thisvoltage += analogRead(APIN_BATTERY);
  }
  thiscurrent /= divisor;
  thisthrottle /= divisor;
  thisvoltage /= divisor;

  //scale our voltage to a readable range
  thisvoltage *= FULLSCALEVOLTAGE;
  thisvoltage /= FULLSCALEADC;

  //scale our current to a readable range and store
  thiscurrent *= FULLSCALECURRENT;
  thiscurrent /= FULLSCALEADC;
  thiscurrent += CURRENTOFFSET;
  if (abs(thiscurrent) < CURRENTTHRESHOLD)
  {
    thiscurrent = 0;
  }
  currents[thisindex] = int(thiscurrent);

  //convert the throttle ADC readings to a usable range
  thisthrottle = thisthrottle - THROTTLEMIN;                //invert the range (so that a disconnected pot = zero throttle)
  thisthrottle = constrain( thisthrottle, 0, THROTTLERANGE); //make sure we stay in bounds

  //applies an exponential curve to give more sensitivity at lower throttle inputs.  may not be required with 3-speed gearbox
  thisthrottle *= thisthrottle;
  thisthrottle /= THROTTLERANGE;

  //run our divider to get us in the uS range rather than ADC range for our control loop
  throttlepos = float( thisthrottle) * thrdivisor;
  /*
    if (thisbrake)
    {
    //apply braking power to the RC line
    throttlepos = BRAKEPOWER;
    }
    Serial.print(throttlepos);
  */
  //change our log rate  and detect an event when the throttle is pulled and also when it is released
  if (thisthrottle > 0)
  {
    if (logratematrix[LOGMATRIX_ONTHROTTLE] != LOGRATE_ONTHROTTLE)
    {
      logratematrix[LOGMATRIX_ONTHROTTLE] = LOGRATE_ONTHROTTLE;
      oneshotcounter = 0;
    }
  }
  else
  {
    if (logratematrix[LOGMATRIX_ONTHROTTLE] == LOGRATE_ONTHROTTLE)
    {
      logratematrix[LOGMATRIX_ONTHROTTLE] = LOGRATE_BACKGROUND;
      oneshotcounter = 0;
    }
  }
}

//------------------------CURRENT AND THROTTLE LIMTING-------------------------------------
//
//  uses our current values and the contents of the I2P
//  buckets to determine what our limit should be.  Then
//  we alter the input throttle to hit our current limit.
//
//------------------------CURRENT AND THROTTLE LIMTING-------------------------------------
void control()
{

  //this catches a throttle of technically less than zero (outside SERVODEAD-SERVOMAX range) and tells it to be zero for our display
  if (throttlepos < SERVODEAD)
  {
    //if our last throttle input was greater than or equal to the deadband, we want to check it to give some hysteresis
    if (lastthrottle >= SERVODEAD)
    {
      //if our input really is low enough, turn the throttle off
      if (throttlepos < (SERVODEAD - SERVOHYSTERESIS))
      {
        throttlepos = 0;
      }
      //otherwise, keep it on a bit.  this gives us some ADC counts of hysteresis so ADC noise doesn't have us dither the ESC back and forth across zero
      else
      {
        throttlepos = SERVODEAD;
      }
    }
    else
    {
      throttlepos = 0;
    }
  }
  //if our throttle is in a valid range, let's calculate a ramping rate for it to limit how fast our output can climb
  else
  {
    //the test ramp rate is governed by our current gear, a constatnt ramp rate, and a ramp rate scalar that varies based on how close to full throttle we are
    float testramp = 0;
    if (gear != GEARUNDEFINED)
    {
      testramp = gearrampscalers[gear] * (RAMPUPRATECONST + RAMPUPRATESCALE * (1 - lastthrottle / (SERVOZERO - SERVOMIN)));
    }
    else
    {
      if (gearaction == SHIFTINGUP && shiftaction == SHIFTSTAGE_SERVOSLEW)
      {
        testramp = 2 * gearrampscalers[targetgear] * (RAMPUPRATECONST + RAMPUPRATESCALE * (1 - lastthrottle / (SERVOZERO - SERVOMIN)));
      }
    }
    //scale the ramp rate by how long it's been since we last went through the loop.
    if (thisindex == 0)
    {
      testramp = (testramp * float(times[thisindex] - times[(SAMPLEBUFFERSIZE - 1)] ) / 1000);
    }
    else
    {
      testramp = (testramp * float(times[thisindex] - times[(thisindex - 1)]) / 1000);
    }
    //base our max on the last throttle plus the ramp rate
    testramp += lastthrottle;
    throttlepos = min(throttlepos, testramp);
    //this seems like it never turns off our throttle, but it's in a part of the code you can't get to if the throttle psition is less than SERVODEAD to begin with
    throttlepos = max(throttlepos, SERVODEAD);
    if (autoshift && targetgear < 3 && throttlepos >= ((SERVOZERO - SERVOMIN) - 2))
    {
      autoshiftuptrigger = true;
    }
    else
    {
      autoshiftuptrigger = false;
    }
  }

  //make sure to save this as our last throttle position
  lastthrottle = throttlepos;

  //calculate reverse (required even if we don't use reverse)
  if (thisreverse || gearboxreverse)                          //if the driver is holding the reverse button
  {
    throttlepos = SERVOZERO + (throttlepos / 2); //we don't invert our throttle direction
  }
  else
  {
    throttlepos = SERVOZERO - throttlepos; //we invert our throttle direction (smaller duty cycle = faster response)
  }

  //write our signal out, and apply an offset to compensate for electrical components with different turn on/turn off delay times
  myservo.writeMicroseconds(throttlepos + SERVOOFFSET); //finally, write the throttle output
}

//------------------------CALCULATION-------------------------------------
//
//  takes our inputs and runs calculations on them
//  current limiting
//  rpm/speed determination
//  gearshift delegation
//
//------------------------CALCULATION-------------------------------------
void calculatevalues()
{
  /**
    int oldrpmindex = thisindex + SAMPLEBUFFERSIZE - 1;
    while (times[thisindex] - times[(oldrpmindex % SAMPLEBUFFERSIZE)] < (1000 * LOOKBACKTIME_RPM))
    {
    oldrpmindex--;
    if (oldrpmindex <= thisindex)
    {
      oldrpmindex++;
      break;
    }
    }
    while (oldrpmindex >= SAMPLEBUFFERSIZE)
    {
    oldrpmindex -= SAMPLEBUFFERSIZE;       //get a valid index inside our sample buffer size
    }
    long minuterpmmultiplier = 60000000 / (times[thisindex] - times[oldrpmindex]);
    prevxpos = motorpositions[oldrpmindex];
    xpos = motorpositions[thisindex];
  */

  if (Lmotor.available() > 0)
  {
    rpmavailableflag = true;
    float lastrpms = rpms;
    rpms = 0;
    int rpmcount = 0;
    lastrpmreading = millis();
    while (Lmotor.available() > 0)
    {
      rpms += Lmotor.countToFrequency(Lmotor.read()) * FREQTORPM; //(xpos - prevxpos) * minuterpmmultiplier / COUNTSPERREV;
      rpmcount++;
    }
    rpms /= rpmcount;
    if (rpms > RPMHIGHLIMIT)
    {
      rpms = lastrpms;
    }
  }
  else
  {
    if ((millis() - LOOKBACKTIME_RPM) > lastrpmreading)
    {
      rpmavailableflag = false;
      //lastrpmreading = millis();
      rpms = 0;
    }
  }

  if (rpms != 0)
  {
    if (logratematrix[LOGMATRIX_MOVING] != LOGRATE_MOVING)
    {
      logratematrix[LOGMATRIX_MOVING] = LOGRATE_MOVING;
      oneshotcounter = 0;
    }
  }
  else
  {
    if (logratematrix[LOGMATRIX_MOVING] == LOGRATE_MOVING)
    {
      logratematrix[LOGMATRIX_MOVING] = LOGRATE_BACKGROUND;
      oneshotcounter = 0;
    }
  }

  int oldspeedindex = thisindex + SAMPLEBUFFERSIZE - 1;
  while (times[thisindex] - times[(oldspeedindex % SAMPLEBUFFERSIZE)] < (1000 * LOOKBACKTIME_SPEED))
  {
    oldspeedindex--;
    if (oldspeedindex <= thisindex)
    {
      oldspeedindex++;
      break;
    }
  }
  while (oldspeedindex >= SAMPLEBUFFERSIZE)
  {
    oldspeedindex -= SAMPLEBUFFERSIZE;       //get a valid index inside our sample buffer size
  }
  long minutespeedmultiplier = 60000000 / (times[thisindex] - times[oldspeedindex]); //timeincrements per minute
  prevdist = axlepositions[oldspeedindex] / COUNTSPERFOOT; //
  curdist = axlepositions[thisindex] / COUNTSPERFOOT;
  velocity = 60 * minutespeedmultiplier / (FEETPERMILE); // mile*timeincrements/foot*hour
  velocity *= (curdist - prevdist); //(foot/timeincrements)->mile/hour
  axlerpms = velocity * SPEEDRPMFACTOR;
  if (gear >= 0 && gearaction == NOTSHIFTING)
  {
    expectedmotorrpms = axlerpms * gearratios[gear];

  }
  else
  {
    if (gearaction != NOTSHIFTING)
    {
      expectedmotorrpms = axlerpms * gearratios[targetgear];
    }
    else
    {
      expectedmotorrpms = 0;
    }
  }
  if (velocity != 0)
  {
    coasting = coastthrottlepos > 0;
    int RPMadder = COASTHYSTERESIS;
    if (coasting)
    {
      RPMadder = 0;
    }
    
    coastthrottlepos = getThrottlefromRPM(expectedmotorrpms, thisvoltage);
    if (thisbrake)
    {
      coastthrottlepos *= COASTPERCENTAGE;
      coastthrottlepos -= COASTBRAKEOFFSET;
      
      if (expectedmotorrpms < (2 * (MINCOASTRPM + RPMadder)))
      {
        coastthrottlepos = 0;
      }
    }
    else
    {
      coastthrottlepos *= COASTPERCENTAGE;
      if (expectedmotorrpms < (MINCOASTRPM + RPMadder))
      {
        coastthrottlepos = 0;
      }
    }
    if (logratematrix[LOGMATRIX_MOVING] != LOGRATE_MOVING)
    {
      logratematrix[LOGMATRIX_MOVING] = LOGRATE_MOVING;
      oneshotcounter = 0;
    }
  }
  else
  {
    coastthrottlepos = 0;
    if (logratematrix[LOGMATRIX_MOVING] == LOGRATE_MOVING)
    {
      logratematrix[LOGMATRIX_MOVING] = LOGRATE_BACKGROUND;
      oneshotcounter = 0;
    }
  }

  if (DEBUGMODE && (thisindex == 0))
  {

    byte inbuffert[10];
    for (int i = 0; i < sizeof(inbuffert); i++)
    {
      inbuffert[i] = 0;
    }
    GetServoPosition(SERVO_ID_R);
    Serial1.readBytes(inbuffert, 8);
    currentposition = (inbuffert[6] << 8) + inbuffert[5];
    Serial.println(currentposition);

    float tempcurrent = float(currents[thisindex]) / 10.0;
    Serial.print(thisthrottle);
    Serial.print("%, ");
    Serial.print(tempcurrent);
    Serial.print("A, RPM: ");
    Serial.println(velocity);//motorPosn.calcPosn());//

    Serial.print(times[thisindex]);
    Serial.print(": ");
    Serial.print(xpos);
    Serial.print(": RPM: ");
    if (rpmavailableflag)
    {
      Serial.print("+");
    }
    Serial.println(rpms);//motorPosn.calcPosn());//

  }
  if (gearaction == NOTSHIFTING)
  {
    throttlepos = max(throttlepos, coastthrottlepos);
  }
  if (rpms < LOWRPMTHRESHOLD && rpms >= 0)
  {
    float tempfloat = LOWRPMTHROTTLE * (SERVOZERO - SERVOMIN);
    throttlepos = min(throttlepos, tempfloat);
  }
  long rpmtemplong = rpms;
  thiscurrentlimit = map(rpmtemplong, MINLIMITCROSSOVERRPM, MAXLIMITCROSSOVERRPM, MINCURRENTLIMIT, MAXCURRENTLIMIT);
  thiscurrentlimit = constrain(thiscurrentlimit, MINCURRENTLIMIT, MAXCURRENTLIMIT);
  if (currents[thisindex] > thiscurrentlimit)
  {
    throttlepos = lastthrottle - 1;
    if (logratematrix[LOGMATRIX_OVERCURRENT] != LOGRATE_OVERCURRENT)
    {
      logratematrix[LOGMATRIX_OVERCURRENT] = LOGRATE_OVERCURRENT;
      oneshotcounter = 0;
    }
  }
  else
  {
    if (logratematrix[LOGMATRIX_OVERCURRENT] == LOGRATE_OVERCURRENT)
    {
      logratematrix[LOGMATRIX_OVERCURRENT] = LOGRATE_BACKGROUND;
      oneshotcounter = 0;
    }
  }

  if (autoshiftuptrigger || (autoshift && gear == 0 && rpms > LOWRPMTHRESHOLD))
  {
    int testcurrent = axlerpms * gearratios[targetgear] * 3 / 4;
    testcurrent = map(testcurrent, MINLIMITCROSSOVERRPM, MAXLIMITCROSSOVERRPM, MINCURRENTLIMIT, MAXCURRENTLIMIT);
    testcurrent = constrain(testcurrent, MINCURRENTLIMIT, MAXCURRENTLIMIT);
    if (testcurrent >= thiscurrent && thistime - AUTOSHIFTLATENCY > lastautoshift)
    {
      thisshiftup = true;
    }
    autoshiftuptrigger = false;
  }
  else
  {
    if (autoshift && ((axlerpms * gearratios[targetgear]) < 2500) && gear > 1 && thistime - AUTOSHIFTLATENCY > lastautoshift)
    {
      thisshiftdown = true;
    }
  }
  shiftcheck();

  if (gearaction != NOTSHIFTING)
  {
    if (logratematrix[LOGMATRIX_SHIFTING] != LOGRATE_SHIFTING)
    {
      logratematrix[LOGMATRIX_SHIFTING] = LOGRATE_SHIFTING;
      oneshotcounter = 0;
    }
  }
  else
  {
    if (logratematrix[LOGMATRIX_SHIFTING] == LOGRATE_SHIFTING)
    {
      logratematrix[LOGMATRIX_SHIFTING] = LOGRATE_BACKGROUND;
      oneshotcounter = 0;
    }
  }
}


//------------------------DATALOGGING-------------------------------------
//
//  takes our signals and writes them to the serial port
//
//------------------------DATALOGGING-------------------------------------
void datalog()
{
  int lograte = LOGRATE_BACKGROUND;
  if (oneshotcounter < COUNTS_ONESHOT)
  {
    logratematrix[LOGMATRIX_ONESHOT] = LOGRATE_ONESHOT;
  }
  else
  {
    logratematrix[LOGMATRIX_ONESHOT] = lograte;
  }
  for (int i = 0; i < (sizeof(logratematrix) / sizeof(int)); i++)
  {
    lograte = min(lograte, logratematrix[i]);
  }
  boolean amilogging = false;
  unsigned long thislogtime = millis();
  boolean triplog = (thislogtime - RECEIVER_TIMEOUT >= lastpacket);
  if (thislogtime % lograte == 0)
  {
    amilogging = true;
    oneshotcounter++;
  }
  if (logtrigger && amilogging)
  {
    if (!datalogging)
    {
      datalogging = true;
      logstring = "TIME,THROTTLE IN,THROTTLE OUT,CURRENT,LIMIT,VOLTAGE,RPM,SPEED,GEAR,SERVOPOS\n";
      //logstring = "TIME,THROTTLE IN,THROTTLE OUT,CURRENT,LIMIT,RPM,SPEED,GEAR,SERVOPOS\n";
    }
    else
    {
      unsigned long millistime = times[thisindex] / 1000;
      String tempstring = String(String(millistime) + ","
                                 + String(100.0 * float(thisthrottle) / float(THROTTLERANGE)) + ","
                                 + String(float(SERVOZERO - throttlepos)) + ","
                                 //+ String(thisshiftthrottle) + ","
                                 + String(float(currents[thisindex]) / 10.0) + ","
                                 + String(float(thiscurrentlimit) / 10.0) + ","
                                 + String(thisvoltage) + ","
                                 + String(rpms) + ","
                                 + String(velocity) + ","
                                 + String(gear) + ","
                                 + String(currentposition))
                          + String("\n");
      if (logstring.length() + tempstring.length() > MAXLOGSTRING)
      {
        String sendstring = logstring;
        logstring = tempstring;
        Serial.print(sendstring);
        Serial.send_now();
        lastpacket = thislogtime;
      }
      else
      {
        //keep buffering
        logstring += tempstring;
        if (triplog)
        {
          Serial.print(logstring);
          Serial.send_now();
          logstring = "";
          lastpacket = thislogtime;
        }
      }
    }
  }
  else
  {
    if (datalogging && !logtrigger)
    {
      datalogging = false;
      logstring = "";
    }
  }
}

void setup()                                             // run once, when the sketch starts
{

  byte outpins[] = {DPIN_TX, DPIN_ESCOUT};      //all output pins
  for ( int i = 0; i < sizeof(outpins); i++) {
    pinMode(outpins[i], OUTPUT);                             //turn to output
    digitalWrite(outpins[i], LOW);
  }
  //set our inputs
  byte inpins[] = {DPIN_RX, DPIN_ENCODER_2APASS, DPIN_ENCODER_2BPASS};         //all input pins
  for ( int i = 0; i < sizeof(inpins); i++) {
    pinMode(inpins[i], INPUT);                            //set tham all to input
  }
  byte pulluppins[] = {DPIN_BRAKE, DPIN_REVERSE, DPIN_SHIFTUP, DPIN_SHIFTDOWN};        //all input pins with pullups
  for ( int i = 0; i < sizeof(pulluppins); i++) {
    pinMode(pulluppins[i], INPUT_PULLUP);                            //set tham all to input with pullup
  }
  //Most important thing is making sure the controller is set to zero throttle when it boots.  seriously, there are protections in the ESC but redundancy is always good for safety
  myservo.attach(DPIN_ESCOUT, SERVOMIN, SERVOMAX);
  myservo.writeMicroseconds(SERVOZERO);
  if (DEBUGMODE || DATALOGGING)
  {
    Serial.begin(2000000);
    serialtime = millis();
  }
  if (DEBUGMODE)
  {
    while (digitalRead(DPIN_BRAKE))
    {
      Serial.print(millis(), DEC);
      Serial.print(" : ");
      Serial.println("Waiting for BRAKE Press");
      delay(1000);
    }
    while (!digitalRead(DPIN_BRAKE))
    {
      Serial.print(millis(), DEC);
      Serial.print(" : ");
      Serial.println("Waiting for BRAKE Release");
      delay(1000);
    }
    Serial.println("Begin");
  }
  //update our divisors
  //motorPosn.setup();      // Start Quad Decode position count
  //motorPosn.start();      // Start Quad Decode position count
  LaxlePosn.setup();      // Start Quad Decode position count
  LaxlePosn.start();      // Start Quad Decode position count

  thrdivisor = float(SERVOMAX - SERVOZERO) / float(THROTTLERANGE);

  Serial1.begin(Serialbaud[BAUD9], SERIAL_8N1);
  Serial1EnableOpenDrain(true);
  Serial1.setTimeout(ECHOTIMEOUT);
  //assume that we shouldnt be logging, unless we can open a file

  /**
    SetBaudRate(SERVO_ID_BROADCAST, BAUD2);
    delay(10);
    SetBaudRate(SERVO_ID_BROADCAST, BAUD3);
    delay(10);
    SetBaudRate(SERVO_ID_BROADCAST, BAUD4);
    delay(10);
    SetBaudRate(SERVO_ID_BROADCAST, BAUD5);
    delay(10);
    SetBaudRate(SERVO_ID_BROADCAST, BAUD6);
    delay(10);
    SetBaudRate(SERVO_ID_BROADCAST, BAUD7);
    delay(10);
    SetBaudRate(SERVO_ID_BROADCAST, BAUD8);
    delay(10);
  */
  //SetBaudRate(SERVO_ID_BROADCAST, BAUD9);

  delay(10);
  SetResponseDelay(SERVO_ID_BROADCAST, 5);
  delay(10);
  SetLimit(SERVO_ID_BROADCAST, 1, CWDIR);
  delay(10);
  SetLimit(SERVO_ID_BROADCAST, 1023, CCWDIR);
  delay(10);
  SetMaxTorque(SERVO_ID_BROADCAST, 1023);
  delay(10);
  SetReturnLevel(SERVO_ID_BROADCAST, 1);
  delay(10);
  SetAlarmLED(SERVO_ID_BROADCAST, 0b00000100);
  delay(10);
  SetComplianceMargin(SERVO_ID_BROADCAST, 1, CWDIR);
  delay(10);
  SetComplianceMargin(SERVO_ID_BROADCAST, 1, CCWDIR);
  delay(10);
  SetComplianceSlope(SERVO_ID_BROADCAST, 2, CWDIR);
  delay(10);
  SetComplianceSlope(SERVO_ID_BROADCAST, 2, CCWDIR);
  delay(10);
  SetMovingSpeed(SERVO_ID_BROADCAST, 1023);
  delay(10);
  SetTorqueLimit(SERVO_ID_BROADCAST, 1023);
  delay(10);
  SetTorqueEnable(SERVO_ID_BROADCAST, true);
  delay(10);
  SetGoalPosition(SERVO_ID_R, Gearpositions[gear]);


  //if the controller boots up and the throttle is held down, wait here until it returns to zero
  if (analogRead(APIN_THROTTLE) > THROTTLEMIN)
  {
    if (DEBUGMODE) {
      Serial.print(millis(), DEC);
      Serial.print(" : ");
      Serial.println("Waiting for safe throttle");
    }
    while (0)//analogRead(APIN_THROTTLE) > THROTTLEMIN)
    {

    }
  }

  if (DEBUGMODE) {
    Serial.print(millis(), DEC);
    Serial.print(" : ");
    Serial.println("GO!");
  }
  delay(250);
  Lmotor.begin(DPIN_MOTOR_1, FREQMEASUREMULTI_ALTERNATE);
  byte inbuffert[10];
  GetServoPosition(SERVO_ID_R);
  if (Serial1.readBytes(inbuffert, 8))
  {
    currentposition = (inbuffert[6] << 8) + inbuffert[5];
  }
  for (int i = 0; i < SAMPLEBUFFERSIZE; i++)
  {
    motorpositions[i] = 0;
    times[i] = 0;
  }
  times[0] = micros();
  thisindex = 1;
  //Our loop timing reference starts now.  GO!
  prevmicros = micros();
}


void loop()
{ //main loop
  //take a sample
  //SetGoalPosition(SERVO_ID_R, Gearpositions[gear]);
  thistime = micros();
  sample(); //80uS
  calculatevalues();
  control();
  byte tempchar = -1;
  tempchar = Serial.read();
  if (tempchar != -1)
  {
    if (tempchar == 'G')
    {
      logtrigger = true;
      datalogging = false;
      logratematrix[LOGMATRIX_ONESHOT] = LOGRATE_ONESHOT;
      oneshotcounter = 0;
    }
    else
    {
      if (tempchar == 'E')
      {
        logtrigger = false;
      }
    }
    Serial.clear();
  }
  if (DATALOGGING)
  {
    datalog();
  }
  while ((micros() - prevperiod) < MINPERIOD) {}
  prevmicros = prevperiod;
  prevperiod = micros();
}
