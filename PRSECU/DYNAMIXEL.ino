
// Turns the LED at the bottom side of the servo on or off
bool SetServoLED(byte s_ID, bool b_On)
{
  byte u8_Data[2];
  u8_Data[0] = R_LED;
  u8_Data[1] = b_On ? 1 : 0;
  return SendInstruction(s_ID, I_WriteData, u8_Data, sizeof(u8_Data));
}
bool SetServoID(byte s_ID, byte b_ID)
{
  byte u8_Data[2];
  u8_Data[0] = R_ServoID;
  u8_Data[1] = b_ID;
  return SendInstruction(s_ID, I_WriteData, u8_Data, sizeof(u8_Data));
}
bool SetBaudRate(byte s_ID, long b_Baud)
{
  byte newbaud = byte(Servobaud[b_Baud]);
  byte u8_Data[2];
  u8_Data[0] = R_BaudRate;
  u8_Data[1] = newbaud;
  bool b_response = (SendInstruction(s_ID, I_WriteData, u8_Data, sizeof(u8_Data)));
  if (b_response)
  {
    Serial1.end();
    Serial1.begin(Serialbaud[b_Baud], SERIAL_8N1);
    Serial1EnableOpenDrain(true);
    Serial1.setTimeout(ECHOTIMEOUT);
    //pinMode(LED_BUILTIN, OUTPUT);
  }
  return b_response;
}
bool SetResponseDelay(byte s_ID, byte s_delay)
{
  byte u8_Data[2];
  u8_Data[0] = R_ReturnDelayTime;
  u8_Data[1] = s_delay;
  return SendInstruction(s_ID, I_WriteData, u8_Data, sizeof(u8_Data));
}
bool SetLimit(byte s_ID, int limit, bool b_Dir)
{
  byte u8_Data[3];
  u8_Data[0] = b_Dir ? R_CCW_AngleLimit : R_CW_AngleLimit;
  u8_Data[1] = limit & 0xFF;
  u8_Data[2] = (limit >> 8) & 0xFF;
  return SendInstruction(s_ID, I_WriteData, u8_Data, sizeof(u8_Data));
}
bool SetMaxTorque(byte s_ID, int maxtorque)
{
  byte u8_Data[3];
  u8_Data[0] = R_MaxTorque;
  u8_Data[1] = maxtorque & 0xFF;
  u8_Data[2] = (maxtorque >> 8) & 0xFF;
  return SendInstruction(s_ID, I_WriteData, u8_Data, sizeof(u8_Data));
}
bool SetReturnLevel(byte s_ID, byte level)
{
  byte u8_Data[2];
  u8_Data[0] = R_StatusReturnLevel;
  u8_Data[1] = level;
  return SendInstruction(s_ID, I_WriteData, u8_Data, sizeof(u8_Data));
}
bool SetAlarmLED(byte s_ID, byte b_LED)
{
  byte u8_Data[2];
  u8_Data[0] = R_AlarmLED;
  u8_Data[1] = b_LED;
  return SendInstruction(s_ID, I_WriteData, u8_Data, sizeof(u8_Data));
}

//RAM areas

bool SetComplianceMargin(byte s_ID, byte b_Margin, bool b_Dir)
{
  byte u8_Data[2];
  u8_Data[0] = b_Dir ? R_CCW_ComplianceMargin : R_CW_ComplianceMargin;
  u8_Data[1] = b_Margin;
  return SendInstruction(s_ID, I_WriteData, u8_Data, sizeof(u8_Data));
}
bool SetComplianceSlope(byte s_ID, byte b_Slope, bool b_Dir)
{
  byte u8_Data[2];
  u8_Data[0] = b_Dir ? R_CCW_ComplianceSlope : R_CW_ComplianceSlope;
  u8_Data[1] = b_Slope;
  return SendInstruction(s_ID, I_WriteData, u8_Data, sizeof(u8_Data));
}
bool SetGoalPosition(byte s_ID, int b_Position)
{
  byte u8_Data[3];
  u8_Data[0] = R_GoalPosition;
  u8_Data[1] = b_Position & 0xFF;
  u8_Data[2] = (b_Position >> 8) & 0xFF;
  return SendInstruction(s_ID, I_WriteData, u8_Data, sizeof(u8_Data));
}
bool SetMovingSpeed(byte s_ID, int b_Speed)
{
  byte u8_Data[3];
  u8_Data[0] = R_MovingSpeed;
  u8_Data[1] = b_Speed & 0xFF;
  u8_Data[2] = (b_Speed >> 8) & 0xFF;
  return SendInstruction(s_ID, I_WriteData, u8_Data, sizeof(u8_Data));
}
bool SetTorqueLimit(byte s_ID, int b_Limit)
{
  byte u8_Data[3];
  u8_Data[0] = R_TorqueLimit;
  u8_Data[1] = b_Limit & 0xFF;
  u8_Data[2] = (b_Limit >> 8) & 0xFF;
  return SendInstruction(s_ID, I_WriteData, u8_Data, sizeof(u8_Data));
}
bool SetTorqueEnable(byte s_ID, bool b_On)
{
  byte u8_Data[2];
  u8_Data[0] = R_TorqueEnable;
  u8_Data[1] = b_On ? 1 : 0;
  return SendInstruction(s_ID, I_WriteData, u8_Data, sizeof(u8_Data));
}

long GetServoPosition(byte s_ID)
{
  byte u8_Data[2];
  u8_Data[0] = R_PresentPosition;
  u8_Data[1] = 2;
  return SendInstruction(s_ID, I_ReadData, u8_Data, sizeof(u8_Data));
}

bool SendInstruction(byte u8_ServoID, byte u8_Instruction, byte* u8_Params, int s32_ParamCount)
{
  byte u8_TxPacket[100];

  // -------- SEND INSTRUCTION ------------

  int S = 0;
  u8_TxPacket[S++] = 0xFF;
  u8_TxPacket[S++] = 0xFF;
  u8_TxPacket[S++] = u8_ServoID;
  u8_TxPacket[S++] = s32_ParamCount + 2;
  u8_TxPacket[S++] = u8_Instruction;

  for (int i = 0; i < s32_ParamCount; i++)
  {
    u8_TxPacket[S++] = u8_Params[i];
  }

  byte u8_Checksum = 0;
  for (int i = 2; i < S; i++)
  {
    u8_Checksum += u8_TxPacket[i];
  }
  u8_TxPacket[S++] = ~u8_Checksum;

  //PrintHex("Instruction: ", u8_TxPacket, S);
  Serial1.clear();

  Serial1EnableOpenDrain(false);
  Serial1.write(u8_TxPacket, S);
  Serial1.flush(); // waits until the last data byte has been sent
  Serial1EnableOpenDrain(true);

  // -------- READ ECHO (timeout = 1 second) ------------

  uint32_t u32_Start = millis();
  int R = 0;
  while (R < S)
  {
    if ((millis() - u32_Start) > ECHOTIMEOUT)
    {
      if (DEBUGMODE) {
        Serial.println("ERROR: Timeout waiting for Echo (Rx and Tx must be tied togteher)");
      }
      return false;
    }

    if (!Serial1.available())
      continue;

    if (Serial1.read() != u8_TxPacket[R++])
    {
      if (DEBUGMODE) {
        Serial.println("ERROR: Invalid echo received");
      }
      return false;
    }
  }
  if (DEBUGMODE) {
    //Serial.println("Valid echo received.");
  }
  // ATTENTION: You can NOT read the servo response HERE. You must return from loop() to receive more RX data!

  return true;
}

// Toggle Pin1 (Serial 1 TX pin) between Open Drain and Slew Rate.
// ATTENTION: OUTPUT_OPENDRAIN in Teensyduino 1.28 does not work with UART. Do not use it !
void Serial1EnableOpenDrain(bool bEnable)
{
  if (bEnable) CORE_PIN1_CONFIG = PORT_PCR_DSE | PORT_PCR_ODE | PORT_PCR_MUX(3); // set Open Drain Enabled
  else         CORE_PIN1_CONFIG = PORT_PCR_DSE | PORT_PCR_SRE | PORT_PCR_MUX(3); // set Slew Rate Enabled
}

void PrintHex(const char* s8_Text, const byte* data, int count)
{
  if (DEBUGMODE) {
    Serial.print(s8_Text);

    for (int i = 0; i < count; i++)
    {
      if (i > 0)
        Serial.print(" ");

      if (data[i] <= 0xF)
        Serial.print("0");

      Serial.print(data[i], HEX);
    }
    Serial.println();
  }
}
