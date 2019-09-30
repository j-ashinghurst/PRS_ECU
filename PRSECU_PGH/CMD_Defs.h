
#define TEST                    0

#define COMMAND_HEADER_START    10
#define COMMAND_HEADER_STOP     11
#define COMMAND_LOG_START       12
#define COMMAND_LOG_STOP        13
#define COMMAND_LOG_NEWLINE     14
#define COMMAND_LOG_FULLINE     15
#define COMMAND_RECEIVER_OPEN   16
#define COMMAND_RECEIVER_CLOSED 17

#define COLUMN_HEADER           20

//40-99: Events
  #define   EVENT_START           40
  #define   EVENT_STOP            41
  #define   EVENT_THROTTLE_ON     42
  #define   EVENT_THROTTLE_OFF    43
  #define   EVENT_OVERAMP_START   44
  #define   EVENT_OVERAMP_STOP    45
  #define   EVENT_SHIFTUP_START   46
  #define   EVENT_SHIFTUP_STOP    47
  #define   EVENT_SHIFTDOWN_START 48
  #define   EVENT_SHIFTDOWN_STOP  49
  #define   EVENT_BRAKES_ON       50
  #define   EVENT_BRAKES_OFF      51
  

//100-254: Logging variables
  //100-119: General
  #define   LOG_TIME              100  //
  #define   LOG_THROTIN           101  //
  #define   LOG_CURRENT_T         102  //
  #define   LOG_CAPACITY          103  //
  #define   LOG_VOLTAGE           104 //
  #define   LOG_TRIP              105  //
  #define   LOG_REVERSE           106  //
  #define   LOG_BRAKE             107  //
  #define   LOG_FUSEIMP           108  //
  
  //120-139: Rear Left
  #define   LOG_THROTOUT_L        120  //
  #define   LOG_RAMPRATE_L        121  //
  #define   LOG_CURRENT_L         122  //
  #define   LOG_POWER_L           123  //
  #define   LOG_LIMIT_L           124  //
  #define   LOG_OVERCURRENT_L     125  //
  #define   LOG_ENERGY_L          126  //
  #define   LOG_RPM_L             127  //
  #define   LOG_SPEED_L           128  //
  #define   LOG_GEAR_L            129  //
  #define   LOG_SERVO_L           130  //
  #define   LOG_MOTORCOUNT_L      131  //
  #define   LOG_AXLECOUNT_L       132  //
  #define   LOG_AXLERPM_L         127  //
  
  //140-159: Rear Right
  
  //160-179: Front Left
  
  //180-199: Front Right
  
  //200-219: Kinematics
  #define   LOG_ACCX              200  //
  #define   LOG_ACCY              201  //
  #define   LOG_ACCZ              202  //
  #define   LOG_ROTX              203  //
  #define   LOG_ROTY              204  //
  #define   LOG_ROTZ              205  //
  #define   LOG_SPEED_C           206  //CENTER
  
  //220-249: ABS/TCS/Torque Vectoring
  #define   LOG_STEER             220  //
  
  //250-254: Debug
  #define   LOG_CPU               253  //
  
