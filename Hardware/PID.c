#include "Common_headfile.h"

volatile float kp = 15.0f;
volatile float kd = 5.0f;
volatile int16_t Speed_Delta = 0;

void PID_Calculate(void)
{
    Speed_Delta = (int16_t)(kp * Error + kd * (Error - Error_Last));
}
