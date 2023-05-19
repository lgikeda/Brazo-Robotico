#include <Servo.h>
#include <PID_v1.h>
#include <Arduino.h>

const int PIN_INPUT = 0;
const int PIN_OUTPUT = 13;

double Setpoint, Input, Output;
double Kp = 0, Ki = 0, Kd = 0;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup()
{
  Input = analogRead(PIN_INPUT);
  Setpoint = 100;

  myPID.SetMode(AUTOMATIC);
}

void loop()
{
  Input = analogRead(PIN_INPUT);
  myPID.Compute();

  analogWrite(PIN_OUTPUT, Output);
}