#define VAR_TIME      0
#define VAR_CURRENT_L 1
#define VAR_THROT_L   2
#define VAR_MOTOR_L   3
#define VAR_AXLE_L    4
#define VAR_AXLERPM_L 5
#define VAR_CURRENT_R 6
#define VAR_THROT_R   7
#define VAR_MOTOR_R   8
#define VAR_AXLE_R    9
#define VAR_AXLERPM_R 10

String logstring = "";
#define MAXLOGSTRING 1024
struct Samplestruct {
  unsigned int ID;
  long sample[SAMPLEBUFFERSIZE];
  long val;
};

struct Samplestruct mysamples[] = {
{ VAR_TIME,       0, 0},
{ VAR_CURRENT_L,  0, 0},
{ VAR_THROT_L,    0, 0},
{ VAR_MOTOR_L,    0, 0},
{ VAR_AXLE_L,     0, 0},
{ VAR_AXLERPM_L,  0, 0},
{ VAR_CURRENT_R,  0, 0},
{ VAR_THROT_R,    0, 0},
{ VAR_MOTOR_R,    0, 0},
{ VAR_AXLE_R,     0, 0},
{ VAR_AXLERPM_R,  0, 0}
};

//----------------------------DATALOGGING AND DISPLAY------------
struct Datastruct {
  unsigned int ID;
  String dataname;
  String dataunits;
  boolean thislogging;
  boolean averaging;
  byte decimals;
  unsigned int diffthreshold;
  long val;
  long lastval;
  long avgval;
};
//THESE DO NOT NEED TO BE IN ANY SPECIFIC ORDER SINCE THEY GET THEIR INDICES FROM THE CMDMESSENGER TABLE
//Column ID           | Column Label      | Units     | Log  | Avg  | Dec | Diff | Val | Lastval | Avgval //
struct Datastruct mydata[] =  { 
  {LOG_TIME,          "TIME",             "s",        true , true , 3,    99,    0,    0,        0},
  {LOG_THROTIN,       "THROTTLE IN",      "%",        true , false, 1,    9,     0,    0,       0},
  {LOG_CURRENT_T,     "CURRENT",          "A",        true , false, 1,    9,     0,    0,      0},
  {LOG_CAPACITY,      "CAPACITY",         "Ahr",      false, false, 2,    9,     0,    0,     0},
  {LOG_VOLTAGE,       "BATTERY",          "V",        true , false, 1,    9,     0,    0,    0}, 
  {LOG_TRIP,          "TRIP",             "ft",       true , false, 0,    9,     0,    0,   0},
  {LOG_REVERSE,       "REVERSE",          " ",        true , false, 0,    0,     0,    0,  0},
  {LOG_BRAKE,         "BRAKE",            " ",        true , false, 0,    0,     0,    0, 0},
  {LOG_FUSEIMP,       "FUSE IMPEDANCE",   "mOhm",     false, true , 3,    99,    0,   0, 0},
  
  {LOG_THROTOUT_L,    "L THROTTLE OUT",   "%",        true , false, 1,    9,     0, 0, 0},
  {LOG_RAMPRATE_L,    "L RAMP RATE",      "us/4ms",   false, false, 1,    9,    0, 0, 0},
  {LOG_CURRENT_L,     "L CURRENT",        "A",        false, true , 1,    9,   0, 0, 0},
  {LOG_POWER_L,       "L MOTOR POWER",    "W",        false, false, 0,    99, 0, 0, 0},
  {LOG_LIMIT_L,       "L LIMIT",          "A",        false, false, 0,    0, 0, 0, 0},
  {LOG_OVERCURRENT_L, "L OVERCURRENT",    "A",        false, false, 1,   9, 0, 0, 0},
  {LOG_ENERGY_L,      "L ENERGY",         "Whr",      false, false, 0,  0, 0, 0, 0},
  {LOG_RPM_L,         "L RPM",            "RPM",      true , true , 0, 0, 0, 0, 0},
  {LOG_SPEED_L,       "L SPEED",          "mph",      true , true , 1, 0, 0, 0, 0},
  {LOG_GEAR_L,        "L GEAR",           "mph",      true , true , 1, 0, 0, 0, 0},
  {LOG_SERVO_L,       "L SERVO",          "mph",      true , true , 1, 0, 0, 0, 0},
  {LOG_MOTORCOUNT_L,  "L nMOTOR",         "#",        true , true , 1, 0, 0, 0, 0},
  {LOG_AXLECOUNT_L,   "L nAXLE",          "#",        true , true , 1, 0, 0, 0, 0},
  {LOG_AXLERPM_L,     "L AXLE RPM",       "#",        true , true , 1, 0, 0, 0, 0},
  
  {LOG_ACCX,          "ACCEL-X",          "g",        false, false, 2, 9, 0, 0, 0},
  {LOG_ACCY,          "ACCEL-Y",          "g",        false, false, 2, 9, 0, 0, 0},
  {LOG_ACCZ,          "ACCEL-Z",          "g",        false, false, 2, 9, 0, 0, 0},
  {LOG_ROTX,          "ROTATION-X",       "°/s",      false, false, 1, 9, 0, 0, 0},
  {LOG_ROTY,          "ROTATION-Y",       "°/s",      false, false, 1, 9, 0, 0, 0},
  {LOG_ROTZ,          "ROTATION-Z",       "°/s",      false, false, 1, 9, 0, 0, 0},
  {LOG_STEER,         "STEERING ANGLE",   "°",        false, false, 1, 9, 0, 0, 0},
  {LOG_CPU,           "CPU USAGE",        "%",        false, false, 0, 4, 0, 0, 0}
};

const long digitsize_table[] = {0, 1, 10, 100, 1000, 10000, 100000, 1000000, 10000000, 100000000, 1000000000};
#define ECUTEMPOFFSET         250
#define ECUADCOFFSET          2463
#define ECUTEMPBIT            -1.7f
#define LOGBUFFERSIZE         2047      //DATALOGGING AND DISPLAY 
#define LOGINTERVAL           100        //DATALOGGING AND DISPLAY  //number of main loops we run through before submitting a datapoint for our log
#define LOGTIMEOUT            1000                                //maximum amount of time between logs, when logs are omitted for space saving.
#define PGOODADJUSTEDLIMIT    160       //DATALOGGING AND DISPLAY  //below this adjusted stack voltage, we should close the file and turn off logging because we've probably been turned off
#define PGOODLIMIT            90       //DATALOGGING AND DISPLAY  //below this unadjusted voltage, we should close the file and turn off logging because we're probably going to brownout
#define MAXDIGITS             10
#define STEERCENTER           2052
#define STEERRATE             1
#define IMPEDANCECURRENT      250


boolean logtrigger =          false;     //DATALOGGING AND DISPLAY
boolean datalogging =         false;     //DATALOGGING AND DISPLAY
char filebuffer[LOGBUFFERSIZE];         //DATALOGGING AND DISPLAY
unsigned int bufflength =     0;        //DATALOGGING AND DISPLAY
unsigned long wattpulses =    0;        //DATALOGGING AND DISPLAY
boolean skiplog =             false;    //DATALOGGING AND DISPLAY
unsigned long amppulses =     0;        //DATALOGGING AND DISPLAY  //running counter we use for amphours
unsigned int paranumber =     1;        //DATALOGGING AND DISPLAY  //current parameter we're writing out on the serial port.  right now we have 11, so they each update at ~44Hz
unsigned int loopssincelog =  0;        //number of times we've looped without sending a datalog
#define IMUVARIABLES 7
byte IMUcounter = 0;

#define LOGMATRIX_SHIFTING    0 //while we're shifting
#define LOGMATRIX_ONTHROTTLE  1 //while we're on the throttle
#define LOGMATRIX_OVERCURRENT 2 //while we're over our current limit
#define LOGMATRIX_MOVING      3 //while we're moving at all
#define LOGMATRIX_BACKGROUND  4 //default rate
#define LOGMATRIX_ONESHOT     5 //to request an immediate log

#define LOGRATE_SHIFTING      1
#define LOGRATE_ONTHROTTLE    10
#define LOGRATE_OVERCURRENT   1
#define LOGRATE_MOVING        100
#define LOGRATE_BACKGROUND    1000
#define LOGRATE_ONESHOT       1000
#define COUNTS_ONESHOT        2
unsigned int oneshotcounter = 0;
#define RECEIVER_TIMEOUT      990
unsigned long lastpacket = 0;

int logratematrix [] = {LOGRATE_BACKGROUND,LOGRATE_BACKGROUND,LOGRATE_BACKGROUND,LOGRATE_BACKGROUND,LOGRATE_BACKGROUND,LOGRATE_BACKGROUND};
