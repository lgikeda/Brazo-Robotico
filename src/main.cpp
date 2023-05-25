
#include <Arduino.h>
#include <Servo.h>
#include <PID_v1.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <math.h>

// /////////////////////////////////////////////////////////////// BLYNK ////////////////////////////////////////////////////////////////////////
#define BLYNK_TEMPLATE_ID "TMPLnngYVr9K"
#define BLYNK_TEMPLATE_NAME "Quickstart Device"
#define BLYNK_AUTH_TOKEN "wmwfA7jZpFscUvKPsR3mnUnWaRoWcHXE"

// #define BLYNK_PRINT Serial

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "Mired";
char pass[] = "Mired.123456";

// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// // Constantes del control PID
// double Kp = 0.2;  // Constante proporcional
// double Ki = 0.48; // Constante integral
// double Kd = 0.1;  // Constante derivativa

double Kp = 0.2;  // Constante proporcional
double Ki = 0.2;  // Constante integral
double Kd = 0.03; // Constante derivativa

// // Variables del control PID
double setpoint = 90.0; // Valor deseado del servo
double input = 0.0;     // Valor actual del servo
double output = 0.0;    // Salida del control PID

// // Configuración del servo
const int PIN_INPUT = 34;
Servo servos[3];
Servo servo1;
const int servoPin[3] = {12, 14, 27};
const int servo1Pin = 13;
// // Crea el objeto PID

PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

int cont = 0;
int inputValue = 0;

int /*px = 50.0, py = 30.0,*/ L1 = 8.0, L2 = 11.0, L3 = 13.0, L4 = 2.0, L5 = 13.0;
double theta1, theta2, theta3, theta4;
const double pi = 3.1415;
unsigned long tiempoInicial;
unsigned long intervalo = 20;
bool flag = false;
bool flag1 = false;
bool flag2 = false;
bool flag3 = false;

double thetaAnterior[3];

enum State
{
  STANDBY,
  MOVE_SERVOS,
  WAIT_SERVOS,
  FINISH
};
State currentState = STANDBY;

// State machine timings
unsigned long stateStartTime;
const unsigned long moveServosTime = 1000;
const unsigned long waitServosTime = 50;

void control();
void establecerSetpoint();
void inversa(int px, int py);
void movimiento();
void handleTerminalInput();
void manual(int a, int b, int c, int d);

String mensaje;

BLYNK_WRITE(V1)
{
  mensaje = param.asStr();
  flag = true;
  if (mensaje == "m")
    flag3 = true;
  if (mensaje == "a")
    flag3 = false;
  // Envia una respuesta al terminal
}

void setup()
{
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  //   // Configura los límites del control PID
  myPID.SetOutputLimits(10, 170);

  //   // Inicializa el control PID
  myPID.SetMode(AUTOMATIC);

  pinMode(PIN_INPUT, INPUT);

  for (int i = 0; i < 3; i++)
  {
    servos[i].attach(servoPin[i]);
    // servos[i].write(90);
    // thetaAnterior[i]=45;
  }
  thetaAnterior[0] = 90;
  thetaAnterior[1] = 45;
  servos[0].write(90);
  servos[1].write(45);
  servos[2].write(0);

  servo1.attach(servo1Pin);
  servo1.write(110);
  thetaAnterior[2] = 110;
  tiempoInicial = millis();

  Serial.begin(9600);
}

void loop()
{
  Blynk.run();
  ////////////////////////////////
  unsigned long currentTime = millis();
  switch (currentState)
  {
  case (STANDBY):
    Blynk.virtualWrite(V1, "Ingresa Posicion de este modo: x,y ");
    Blynk.virtualWrite(V1, "theta1: " + String(theta1));
    Blynk.virtualWrite(V1, "theta2: " + String(theta2));
    Blynk.virtualWrite(V1, "theta3: " + String(theta3));
    // if(mensaje!=NULL){
    handleTerminalInput();
    // mensaje = " ";
    //}
    currentState = MOVE_SERVOS;
    stateStartTime = currentTime;
    break;
  case MOVE_SERVOS:
    movimiento();
    control();
    currentState = WAIT_SERVOS;
    stateStartTime = currentTime;
    if (flag1 and flag2)
    {
      theta4 = 90;
      // if (currentTime - stateStartTime >= moveServosTime) {
      // }
    }
    break;
  case WAIT_SERVOS:
    if (currentTime - stateStartTime >= waitServosTime)
    {
      stateStartTime = currentTime;
      currentState = STANDBY;
    }
    break;
  }
  // inversa();
  // establecerSetpoint();
  // control();
  // movimiento();
}

void control()
{
  unsigned long tiempoActual = millis();
  // double output0 = output;
  if ((output <= (setpoint - 1.0)) or (output >= (setpoint + 1.0)))
  {
    inputValue = analogRead(PIN_INPUT);
    input = map(inputValue, 0, 4095, 10, 170);
    myPID.Compute();
    if (theta3 < output)
    {
      // for (auto i = output0; i <= output; i += 1.0)
      // {
      servo1.write(theta3++);
      //   while (tiempoActual - tiempoInicial <= intervalo)
      //   {
      //     tiempoActual = millis();
      //   }
      //   tiempoInicial = tiempoActual;
      // }
    }
    else if (theta3 > output)
    {
      // for (auto i = output0; i >= output; i -= 1.0)
      // {
      servo1.write(theta3--);
      // while (tiempoActual - tiempoInicial <= intervalo)
      // {
      //   tiempoActual = millis();
      // }
      // tiempoInicial = tiempoActual;
      // delay(100);
      //}
    }
    flag1 = false;
  }
  else
    flag1 = true;
}

void establecerSetpoint()
{
  setpoint = theta3;
  // if (mensaje != "")
  // {
  //   setpoint = mensaje.toDouble();
  //   Blynk.virtualWrite(V1, "setpoint = " + String(setpoint));
  //   mensaje = ""; // Limpia el mensaje después de utilizarlo
  // }
}

void inversa(int px, int py)
{
  if (px == 30 and py == 30)
  {
    theta1 = 0;
    theta2 = 45;
    theta3 = 90;
  }
  else
  {

    auto cosq3 = (px ^ 2 + 0 + py ^ 2 - L2 ^ 2 - L3 ^ 2) / (2 * L2 * L3);
    auto q3 = atan(sqrt(1 - cosq3 * cosq3) / cosq3);

    theta1 = (atan2(py, px) * 180) / pi;

    theta2 = abs(atan(py / px) - atan((L3 * sin(q3)) / (L2 + L3 * cosq3)) * 180 / pi);

    theta3 = q3 * 180 / pi;
  }
  // theta1 = atan(py / px)*(180)/3.1415;
  // double num = px*px + py*py - L1*L1 - L2*L2;
  // double denom = 2 * L1 * L2;
  // theta2 = acos(num / denom)*(-180)/3.1415;
  // double num1 = px*px + py*py - L1*L1 - L2*L2 - L3*L3;
  // double denom1 = 2 * L2 * L3;
  // theta3 = acos(num1 / denom1)*(180)/3.1415;

  // Blynk.virtualWrite(V1,"theta1: " + String(theta1));
  // Blynk.virtualWrite(V1,"theta2: " + String(theta2));
  // Blynk.virtualWrite(V1,"theta3: " + String(theta3));
}

void movimiento()
{
  if (thetaAnterior[0] < theta1)
  {
    servos[0].write(thetaAnterior[0]);
    thetaAnterior[0]++;
    flag2 = false;
  }
  else
  {
    servos[0].write(thetaAnterior[0]);
    thetaAnterior[0]--;
    flag2 = false;
  }
  if (thetaAnterior[1] < theta2)
  {
    servos[1].write(thetaAnterior[1]);
    thetaAnterior[1]++;
    flag2 = false;
  }
  else
  {
    servos[1].write(thetaAnterior[1]);
    thetaAnterior[1]--;
    flag2 = false;
  }
  servos[2].write(theta4);
  if ((thetaAnterior[0] == theta1) and (thetaAnterior[1] == theta2))
    flag2 = true;
}

void handleTerminalInput()
{
  if (flag)
  {
    flag = false;
    String input = mensaje;
    input.trim();

    if (input.length() > 0)
    {
      if (input.indexOf(',') != -1)
      {
        // Coordinates input
        int commaIndex = input.indexOf(',');
        int secondCommaIndex = input.indexOf(",", commaIndex + 1);
        int thirdCommaIndex = input.indexOf(",", secondCommaIndex + 1);
        String xStr = input.substring(0, commaIndex);
        String yStr = input.substring(commaIndex + 1, secondCommaIndex);
        String zStr = input.substring(secondCommaIndex + 1, thirdCommaIndex);
        String qStr = input.substring(thirdCommaIndex + 1);
        int x = xStr.toInt();
        int y = yStr.toInt();
        int z = zStr.toInt();
        int q = qStr.toInt();
        Blynk.virtualWrite(V1, "Punto final: " + String(x) + ";" + String(y));
        if (flag3)
        {
          manual(x, y, z, q);
        }
        else
        {
          inversa(x, y);
        }
      } // else {
      //   // Servo angles input
      //   String anglesStr = input;
      //   for (int i = 0; i < 4; i++) {
      //     int spaceIndex = anglesStr.indexOf(' ');
      //     if (spaceIndex != -1) {
      //       String angleStr = anglesStr.substring(0, spaceIndex);
      //       servoAngles[i] = angleStr.toInt();
      //       anglesStr = anglesStr.substring(spaceIndex + 1);
      //     } else {
      //       servoAngles[i] = anglesStr.toInt();
      //     }
      //   }
      //   moveServosToAngles();
      // }
    }
  }
}

void manual(int a, int b, int c, int d)
{
  theta1 = a;
  theta2 = b;
  theta3 = c;
  theta4 = d;
  movimiento();
  control();
}