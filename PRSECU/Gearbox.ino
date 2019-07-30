#define SERVOCHECKDELAY 1
unsigned long lastservocheck = 0;
int throttlebeforeshift = 0;
int throttleaftershift = 0;
void shiftcheck()
{
  thisshiftthrottle = lastthrottle;
  if (shiftmask && (gearaction == NOTSHIFTING))
  {
    Serial1.clear();
    GetServoPosition(SERVO_ID_R);
    if (thisshiftup && gear == targetgear) //the shift up button has been pressed or an autoshift triggered
    {
      shiftmask = false;
      justshifted = true;
      gearaction = SHIFTINGUP;
      switch (gear) {
        case GEARNEUTRAL:
          gear = GEARUNDEFINED;
          targetgear = GEARONE;
          break;
        case GEARONE:
          gear = GEARUNDEFINED;
          targetgear = GEARTWO;
          break;
        case GEARTWO:
          gear = GEARUNDEFINED;
          targetgear = GEARTHREE;
          break;
        case GEARTHREE:
          justshifted = false;
          gearaction = NOTSHIFTING;
          break;
      }

    }
    else
    {
      if (thisshiftdown && gear == targetgear) //the shift down button has been pressed
      {
        shiftmask = false;
        justshifted = true;
        gearaction = SHIFTINGDOWN;
        switch (gear) {
          case GEARNEUTRAL:
            justshifted = false;
            gearaction = NOTSHIFTING;
            break;
          case GEARONE:
            justshifted = false;
            gearaction = NOTSHIFTING;

            //gear = GEARUNDEFINED;
            //targetgear = GEARNEUTRAL;
            break;
          case GEARTWO:
            gear = GEARUNDEFINED;
            targetgear = GEARONE;
            break;
          case GEARTHREE:
            gear = GEARUNDEFINED;
            targetgear = GEARTWO;
            break;
        }
      }
    }
  }
  else
  {
    if (!thisshiftup && !thisshiftdown)
    {
      shiftmask = true;
    }
  }

  switch (shiftaction)
  {
    case SHIFTIDLE:
      if (justshifted)
      {
        justshifted = false;
        if (gearaction == SHIFTINGUP)
        {
          shiftaction = SHIFTSTAGE_OFFGAS;
          targetRPMs = axlerpms * gearratios[targetgear] * 0.95f;

          if (targetRPMs < RPMLOWCUTOFF)
          {
            targetRPMs = 0;
            gearboxreverse = true;
            throttleaftershift = 0;
            throttlepos = 0;
          }
          else
          {
            throttleaftershift = lastthrottle * 65 / 100;
            throttlepos = 50;
          }

          justshifted = true;
        }
        else
        {
          if (gearaction == SHIFTINGDOWN)
          {
            shiftaction = SHIFTSTAGE_OFFGAS;
            targetRPMs = rpms * 95 / 100;
            throttlepos = lastthrottle * 95 / 100;
            lastthrottle = throttlepos;
            justshifted = true;
          }
        }
        if (DEBUGMODE)
        {
          Serial.print("Gear Shift to ");
          Serial.print(targetgear);
          Serial.print(", Start: ");
          Serial.println(millis());
        }
      }
      break;

    case SHIFTSTAGE_OFFGAS:
      if (DEBUGMODE && justshifted) {
        Serial.print("Gear Shift to ");
        Serial.print(targetgear);
        Serial.print(", Letting off gas: ");
        Serial.println(millis());
      }
      if ((gearaction == SHIFTINGUP) && justshifted)
      {
        justshifted = false;
        shifttime = thistime;
        shifttimeout = UPSHIFT_OFFGAS_TIMEOUT;
        /*
          if (targetgear == GEARTHREE)
          {
          if (GEAR3POSITION > GEAR2POSITION)
          {
            SetGoalPosition(SERVO_ID_R, Gearpositions[GEARNEUTRAL] - 25);
          }
          else
          {
            SetGoalPosition(SERVO_ID_R, Gearpositions[GEARNEUTRAL] + 25);
          }
          }*/
      }
      else
      {
        if ((gearaction == SHIFTINGDOWN) && justshifted)
        {
          justshifted = false;
          shifttime = thistime;
          shifttimeout = DOWNSHIFT_OFFGAS_TIMEOUT;
        }
      }
      if ((abs(rpms) > targetRPMs /*+ RPMTOLERANCE*/) && (gearaction == SHIFTINGUP))
      {
        if ((thistime - shifttime) > shifttimeout)
        {
          //if we've spent too long in this stage, fall through to servo slewing; as long as we've started to get off the gas
          shiftaction = SHIFTSTAGE_SERVOSLEW;
          justshifted = true;
          if (gearaction == SHIFTINGUP)
          {
            throttlepos = throttleaftershift;
            thisshiftthrottle = throttlepos;
          }
        }
        else
        {
          //throttlepos = lastthrottle * REVMATCHSCALER;
        }
      }
      else
      {
        if (gearboxreverse && targetRPMs == 0)
        {
          targetRPMs = RPMLOWCUTOFF;
          throttlepos = 75;
          thisshiftthrottle = throttlepos;
        }
        else
        {
          //if shifting down, fall through to servo slewing; as long as we've started to get off the gas
          shiftaction = SHIFTSTAGE_SERVOSLEW;
          if (gearaction == SHIFTINGUP)
          {
            throttlepos = throttleaftershift;
            thisshiftthrottle = throttlepos;
          }
          justshifted = true;
        }
      }
      break;
    case SHIFTSTAGE_SERVOSLEW:
      if (DEBUGMODE && justshifted) {
        Serial.print("Gear Shift to ");
        Serial.print(targetgear);
        Serial.print(", Servo Slewing: ");
        Serial.println(millis());
      }
      if (gearaction == SHIFTINGUP)
      {
        throttlepos = throttleaftershift;
        thisshiftthrottle = throttlepos;
      }
      else
      {
        throttlepos = lastthrottle;
        thisshiftthrottle = throttlepos;
      }
      if (justshifted)
      {
        justshifted = false;
        gearboxreverse = false;
        if (gearaction == SHIFTINGUP)
        {
          SetGoalPosition(SERVO_ID_R, Gearpositions[targetgear]);
          shifttime = thistime;
          shifttimeout = UPSHIFT_SLEW_TIMEOUT;

          break;
        }
        else
        {
          if (gearaction == SHIFTINGDOWN)
          {
            SetGoalPosition(SERVO_ID_R, Gearpositions[targetgear]);
            shifttime = thistime;
            shifttimeout = DOWNSHIFT_SLEW_TIMEOUT;
          }
          break;
        }
      }
      else
      {
        if ((millis() - SERVOCHECKDELAY) > lastservocheck)
        {
          byte inbuffer1[10];
          for (int i = 0; i < sizeof(inbuffer1); i++)
          {
            inbuffer1[i] = 0;
          }
          Serial1.clear();
          GetServoPosition(SERVO_ID_R);
          Serial1.readBytes(inbuffer1, 8);
          currentposition = (inbuffer1[6] << 8) + inbuffer1[5];
          int tempint = currentposition - Gearpositions[targetgear];
          if ((abs(tempint) < 3) || ((thistime - shifttime) > shifttimeout))
          {
            shiftaction = SHIFTSTAGE_ONGAS;
            justshifted = true;
            gear = targetgear;

          }
          lastservocheck = millis();
        }
      }
      break;
    case SHIFTSTAGE_ONGAS:
      if (justshifted)
      {
        justshifted = false;
        if (DEBUGMODE)
        {
          Serial.print("End Shift: ");
          Serial.println(millis());
        }
        if (gearaction == SHIFTINGUP)
        {
          shifttime = thistime;
          shifttimeout = UPSHIFT_ONGAS_TIMEOUT;
          gearaction = NOTSHIFTING;
          shiftaction = SHIFTIDLE;
        }
        else
        {
          if (gearaction == SHIFTINGDOWN)
          {
            shifttime = thistime;
            shifttimeout = DOWNSHIFT_ONGAS_TIMEOUT;
          }
        }
      }
      else
      {
        gearaction = NOTSHIFTING;
        shiftaction = SHIFTIDLE;
      }
      break;
  }
}
