// Control PID de brazo robotico utilizando microcontrolador ESP32
// Para controlar el robot utilizando el microcontrolador ESP32
#include "Servo.h"
#include "QuickPID.h"
#include "math.h"
#include <vector>

// Definicion de los servomotores
const int servoPins[6] = {2, 3, 4, 5, 6, 7};
Servo servos[6];

// Definicion de los limites de movimiento de los servos
const int servoMinAngle = 0;
const int servoMaxAngle = 180;

// Definicion de los parametros de contro PID
const double Kp[6] = {1, 1, 1, 1, 1, 1};
const double Ki[6] = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
const double Kd[6] = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
const int sampleTime = 10;
QuickPID pid[6];

// Definir los valores objetivo de cada servo
int servoTarget[] = {90, 90, 90, 90, 90, 90};

// Definicion de las dimensiones segun el diseño del brazo robotico
const int L1 = 10; // Dimension del eslabon 1
const int L2 = 10; // Dimension del eslabon 2
const int H = 5;   // Dimension de la herramienta (pinza)

// Definicion del punto fijo para la prueba PCR
const int x = 10;
const int y = 10;
const int z = 10;

// Definicion de la orientacion de la herramienta
const int orientation = 35;

void setup()
{
  // Inicializacion de los servos
  for (int i = 0; i < 6; i++)
  {
    servos[i].attach(servoPins[i]);
  }
  // Inicializacion del PID
  for (int i = 0; i < 6; i++)
  {
    pid[i].SetTunings(Kp[i], Ki[i], Kd[i]);
    pid[i].SetSampleTimeUs(sampleTime);
    pid[i].SetOutputLimits(servoMinAngle, servoMaxAngle);
    pid[i].SetMode(1U);
  }

  // Posicion inicial
  for (int i = 0; i < 6; i++)
  {
    servos[i].write(servoTarget[i]);
  }
}

void loop()
{
  // Calculo de posicion del extremo del brazo
  double xEnd = x;
  double yEnd = y;
  double zEnd = z + H;

  // calculo de posicion de las articulaciones

  // articulacion 1
  double theta1 = atan2(yEnd, xEnd);
  double x1 = L1 * cos(theta1);
  double y1 = L1 * sin(theta1);
  double z1 = zEnd;

  // articulación 2
  double x2 = x - L1 * cos(theta1);
  double y2 = y - L1 * sin(theta1);
  double z2 = z;

  // Cálculo de los ángulos de los servos para mover el brazo a la posición deseada
  double servoAngles[6] = {0, 0, 0, 0, 0, 0};

  servoAngles[0] = atan2(y1, x1) * 180 / PI;
  servoAngles[1] = atan2(y2, x2) * 180 / PI;
  servoAngles[2] = atan2(y3, x3) * 180 / PI;
  servoAngles[3] = atan2(y4, x4) * 180 / PI;
  servoAngles[4] = atan2(y5, x5) * 180 / PI;
  servoAngles[5] = theta5 * 180 / PI;

  // Control PID de los servos
  for (int i = 0; i < 6; i++)
  {
    pid[i].setSetpoint(servoAngles[i]);
    pid[i].setProcessValue(servos[i].read());
    pid[i].compute();
    servos[i].write(pid[i].getOutput());
  }
  // Cierre de la pinza
  servoPinza.write(minServoAngle);

  // Espera hasta el próximo ciclo de control
  delay(sampleTime);
}

void moveArmToPosition(int servoAngles[6])
{
  // Mueve los servos a las posiciones deseadas utilizando el control PID
  for (int i = 0; i < 6; i++)
  {
    pid[i].setSetpoint(servoAngles[i]);
    servos[i].write(pid[i].compute());
  }
}

void loop()
{
  // Movemos el brazo a la posición deseada
  int desiredAngles[6] = {90, 90, 90, 90, 90, 90}; // Por ejemplo, aquí usamos la posición inicial
  moveArmToPosition(desiredAngles);
}

void matrizHomogenea()
{
  int px, py, pz;
  std::vector<std::vector<int>> T, res, P;
  P = {px, py, pz, 0};
  int n, m;
  n = m = 0;
  for (int i; i < n; i++)
  {
    for (int j; j < m; j++)
    {
      res[i][j] = T[i][j] * P[i][j];
    }
  }
}