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
    GetServoPosition(SERVO_ID_SHIFT);
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
        default:
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
          case GEARNEUTRAL:
          default:
            justshifted = false;
            gearaction = NOTSHIFTING;
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
          targetRPMs = axlerpms * gearratios[targetgear] * 0.90f;

          if (targetRPMs < RPMLOWCUTOFF)
          {
            targetRPMs = 0;
            gearboxreverse = true;
            throttlepos = 100;
            throttleaftershift = throttlepos;
            thisshiftthrottle = throttlepos;
          }
          else
          {
            throttleaftershift = getThrottlefromRPM(targetRPMs, thisvoltage);
            throttlepos = throttleaftershift;
          }

          justshifted = true;
        }
        else
        {
          if (gearaction == SHIFTINGDOWN)
          {
            shiftaction = SHIFTSTAGE_OFFGAS;
            targetRPMs = axlerpms * gearratios[targetgear+1] * 0.95f;
            if (targetRPMs < RPMLOWCUTOFF)
            {
              targetRPMs = 0;
              gearboxreverse = true;
              throttlepos = 100;
              throttleaftershift = throttlepos;
              thisshiftthrottle = throttlepos;
            }
            else
            {
              
              throttleaftershift = getThrottlefromRPM(targetRPMs, thisvoltage);
              lastthrottle * 0.95f;
              throttlepos = throttleaftershift;
            }
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
      if (gearboxreverse && targetRPMs == 0)
      {
        throttlepos = 100;
        thisshiftthrottle = throttlepos;
      }

      //if shifting down, fall through to servo slewing; as long as we've started to get off the gas
      shiftaction = SHIFTSTAGE_SERVOSLEW;
      if (gearaction == SHIFTINGUP)
      {
        throttlepos = throttleaftershift;
        thisshiftthrottle = throttlepos;
      }
      justshifted = true;


      break;
    case SHIFTSTAGE_SERVOSLEW:
      if (DEBUGMODE && justshifted) {
        Serial.print("Gear Shift to ");
        Serial.print(targetgear);
        Serial.print(", Servo Slewing: ");
        Serial.println(millis());
      }

      throttlepos = throttleaftershift;
      thisshiftthrottle = throttlepos;

      if (justshifted)
      {
        justshifted = false;

        if (gearaction == SHIFTINGUP)
        {
          SetGoalPosition(SERVO_ID_SHIFT, Gearpositions[targetgear]);
          shifttime = thistime;
          shifttimeout = UPSHIFT_SLEW_TIMEOUT;

          break;
        }
        else
        {
          if (gearaction == SHIFTINGDOWN)
          {
            SetGoalPosition(SERVO_ID_SHIFT, Gearpositions[targetgear]);
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
          GetServoPosition(SERVO_ID_SHIFT);
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
        gearboxreverse = false;
        if (DEBUGMODE)
        {
          Serial.print("End Shift: ");
          Serial.println(millis());
        }
        shifttime = thistime;
        shifttimeout = UPSHIFT_ONGAS_TIMEOUT;
        gearaction = NOTSHIFTING;
        shiftaction = SHIFTIDLE;
      }
      else
      {
        gearaction = NOTSHIFTING;
        shiftaction = SHIFTIDLE;
      }
      refractoryscalar = POSTSHIFTSCALAR / gearrampscalers[gear];
      break;
  }
}

float getThrottlefromRPM(float inRPM, float inVoltage)
{
  inRPM /= inVoltage;
  if (inRPM < MAPPINGLOWCUTOFF)
  {
    return 0.0;
  }
  else
  {
    if (inRPM > MAPPINGHIGHCUTOFF)
    {
      inRPM = MAPPINGHIGHCUTOFF;
    }
    float term4 = pow(inRPM, 4) * THROTTLERPM4TERM;
    float term3 = pow(inRPM, 3) * THROTTLERPM3TERM;
    float term2 = pow(inRPM, 2) * THROTTLERPM2TERM;
    float term1 = inRPM * THROTTLERPM1TERM;
    float term0 = THROTTLERPM0TERM;

    float outputThrottle = (term4 + term3 + term2 + term1 + term0);
    return outputThrottle;
  }
}
