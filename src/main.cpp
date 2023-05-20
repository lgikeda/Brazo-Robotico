
#include <Arduino.h>
#include <Servo.h>
#include <PID_v1.h>

// Constantes del control PID
double Kp = 0.2;  // Constante proporcional
double Ki = 0.48; // Constante integral
double Kd = 0.1;  // Constante derivativa

// Variables del control PID
double setpoint = 90.0; // Valor deseado del servo
double input = 0.0;     // Valor actual del servo
double output = 0.0;    // Salida del control PID

// Configuración del servo
const int PIN_INPUT = 2;
Servo servos[4];
const int servoPin[4] = {15, 4, 5, 18};
// Crea el objeto PID
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

int cont = 0;
int inputValue = 0;

void control();

void setup()
{
  // myservo.attach(servoPin); // Pin del servo

  // Configura los límites del control PID
  myPID.SetOutputLimits(45, 130);

  // Inicializa el control PID
  myPID.SetMode(AUTOMATIC);

  pinMode(PIN_INPUT, INPUT);

  for (int i = 0; i < 4; i++)
  {
    servos[i].attach(servoPin[i]);
  }
  servos[1].write(0);

  Serial.begin(9600);
}

void loop()
{
  setpoint = 90.0;
  if (cont == 0)
  {
    servos[0].write(45);
    // servos[1].write(45);
    servos[2].write(45);
    servos[3].write(45);
    cont = 1;
  }
  else
  {
    servos[0].write(75);
    // servos[1].write(75);
    servos[2].write(75);
    servos[3].write(75);
    cont = 0;
  }
  // delay(1000);
  //  Actualiza el valor actual del servo
  control();
}

void control()
{
  Serial.println("//////");
  Serial.println(input);
  double output0 = output;
  while (not((output >= (setpoint - 1.0)) and (output <= (setpoint + 1.0))))
  {
    inputValue = analogRead(PIN_INPUT);
    input = map(inputValue, 0, 4095, 45, 130);
    myPID.Compute();
    if (output0 < output)
    {
      for (auto i = output0; i <= output; i += 1.0)
      {
        servos[1].write(i);
        delay(20);
      }
    }
    else
    {
      for (auto i = output0; i >= output; i -= 1.0)
      {
        servos[1].write(i);
        delay(20);
      }
    }

    output0 = output;
    Serial.println(output);
  }
  Serial.println(output);
}