# PRS_ECU

An ECU for system control on PRS karts. Includes functions for current limiting, datalogging, and shifting a Sturmey-Archer AW gearhub via dynamixel servo.

Takes the following inputs:
*brake signal
*throttle signal
*axle encoder
*motor encoder
*current sensor
*battery voltage divider

Has the following outputs:
*PWM to RC ESC (modified, 75uS-925uS, positive throttle is <500uS
*datalogging on USB-Serial port


powerracingseries.org
