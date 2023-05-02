#include "QTRSensors.h"

// Valores de ajustes para o seguidor de linha PID
#define PID_TRESHOLD 600                      // Valor de referencia para cor da linha branca
#define PID_SPEED0 255                         // Valor de 0 a 255 para velocidade com a seguinte leitura do sensor (0 0 0 1 1 0 0 0)
#define PID_SPEED1 150                        // Valor de 0 a 255 para velocidade com a seguinte leitura do sensor (0 0 0 0 0 0 0 1)
#define SETPOINT 2500                     // Valor dos sensores quando está no centro da linha
#define KP 2                              // Valor proporcional para calculo do PID
#define KI 0                              // Valor integral para calculo do PID
#define KD 25                             // Valor derivativo para calculo do PID
#define MAXSPEED 255                      // Valor máximo para aplicação da velocidade do PID
#define BASESPEED 150                     // Valor base para aplicação da velocidade do PID
#define MINSPEED 80                        // Valor mínimo para aplicação da velocidade do PID
#define PID_RUNTIME 25000                     // Valor para executar o percurso 

// Portas driver motor
#define PININ1 2
#define PININ2 4
#define PININ3 5
#define PININ4 7
#define PINENA 3
#define PINENB 6

// Portas sensor QTR
#define SENSOR1 A0
#define SENSOR2 A1
#define SENSOR3 A2
#define SENSOR4 A3
#define SENSOR5 A4
#define SENSOR6 A5

// Definições sobre o sensor QTR
#define NUM_SENSORS             6
#define NUM_SAMPLES_PER_SENSOR  4
#define EMITTER_PIN             13

#define TRESHOLD                600
#define MAXSPEEDCALIBRATE       60

void readSensors(void);
void calibrateSensors(void);

QTRSensorsAnalog qtra((unsigned char[]) {
  SENSOR1, SENSOR2, SENSOR3, SENSOR4, SENSOR5, SENSOR6
}, NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];

void motorControl(int speedLeft, int speedRight);
void motorOption(char optin, int speedLeft, int speedRight);
bool motorStop(long runtime, long currentTime);

void followLinePID(void);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  followLinePID();

}

void followLinePID(void) {
  // Função para controle do seguidor de linha em modo de proporcional, integral e derivativo
  calibrateSensors();

  float lastError = 0, sumError = 0;
  bool flag = true;
  long currentTime = millis();

  while (flag) {
    flag = motorStop(PID_RUNTIME, currentTime);

    float position = qtra.readLine(sensorValues);
    float error = position - SETPOINT;
    sumError += error;

    float motorSpeed = (KP * error) + (KD * (error - lastError) + (KI * sumError));
    lastError = error;

    float m1Speed = BASESPEED + motorSpeed;
    float m2Speed = BASESPEED - motorSpeed;

    // leitura do sensor (1 1 1 1 1 1)
    if (sensorValues[0] <= PID_TRESHOLD && sensorValues[1] <= PID_TRESHOLD && sensorValues[2] <= PID_TRESHOLD && sensorValues[3] <= PID_TRESHOLD && sensorValues[4] <= PID_TRESHOLD && sensorValues[5] <= PID_TRESHOLD) {

      motorOption('8', PID_SPEED0, PID_SPEED0);
      //      motorControl(constrain(m1Speed, MINSPEED, MAXSPEED), constrain(m2Speed, MINSPEED, MAXSPEED));
      // leitura do sensor (0 1 1 1 1 0)
    } else if ( sensorValues[0] >= PID_TRESHOLD && sensorValues[1] <= PID_TRESHOLD && sensorValues[2] <= PID_TRESHOLD && sensorValues[3] <= PID_TRESHOLD && sensorValues[4] <= PID_TRESHOLD && sensorValues[5] >= PID_TRESHOLD) {

      motorOption('8', PID_SPEED0, PID_SPEED0);
      //      motorControl(constrain(m1Speed, MINSPEED, MAXSPEED), constrain(m2Speed, MINSPEED, MAXSPEED));
      // leitura do sensor (0 0 1 1 0 0)
    } else if ( sensorValues[0] >= PID_TRESHOLD && sensorValues[1] >= PID_TRESHOLD && sensorValues[2] <= PID_TRESHOLD && sensorValues[3] <= PID_TRESHOLD && sensorValues[4] >= PID_TRESHOLD && sensorValues[5] >= PID_TRESHOLD) {

      motorOption('8', PID_SPEED0, PID_SPEED0);
      //      motorControl(constrain(m1Speed, MINSPEED, MAXSPEED), constrain(m2Speed, MINSPEED, MAXSPEED));

      // leitura do sensor (0 1 1 1 0 0)
    } else if (sensorValues[0] >= PID_TRESHOLD && sensorValues[1] <= PID_TRESHOLD && sensorValues[2] <= PID_TRESHOLD && sensorValues[3] <= PID_TRESHOLD && sensorValues[4] >= PID_TRESHOLD && sensorValues[5] >= PID_TRESHOLD) {

      motorControl(constrain(m1Speed, MINSPEED, MAXSPEED), constrain(m2Speed, MINSPEED, MAXSPEED));
      // leitura do sensor (0 0 1 1 1 0)
    } else if (sensorValues[0] >= PID_TRESHOLD && sensorValues[1] >= PID_TRESHOLD && sensorValues[2] <= PID_TRESHOLD && sensorValues[3] <= PID_TRESHOLD && sensorValues[4] <= PID_TRESHOLD && sensorValues[5] >= PID_TRESHOLD ) {

      motorControl(constrain(m1Speed, MINSPEED, MAXSPEED), constrain(m2Speed, MINSPEED, MAXSPEED));

      // leitura do sensor (0 0 1 0 0 0)
    } else if (sensorValues[0] >= PID_TRESHOLD && sensorValues[1] >= PID_TRESHOLD && sensorValues[2] <= PID_TRESHOLD && sensorValues[3] >= PID_TRESHOLD && sensorValues[4] >= PID_TRESHOLD && sensorValues[5] >= PID_TRESHOLD) {

      motorControl(constrain(m1Speed, MINSPEED, MAXSPEED), constrain(m2Speed, MINSPEED, MAXSPEED));
      // leitura do sensor (0 0 0 1 0 0)
    } else if (sensorValues[0] >= PID_TRESHOLD && sensorValues[1] >= PID_TRESHOLD && sensorValues[2] >= PID_TRESHOLD && sensorValues[3] <= PID_TRESHOLD && sensorValues[4] >= PID_TRESHOLD && sensorValues[5] >= PID_TRESHOLD ) {

      motorControl(constrain(m1Speed, MINSPEED, MAXSPEED), constrain(m2Speed, MINSPEED, MAXSPEED));

      // leitura do sensor (0 1 1 0 0 0)
    } else if (sensorValues[0] >= PID_TRESHOLD && sensorValues[1] <= PID_TRESHOLD && sensorValues[2] <= PID_TRESHOLD && sensorValues[3] >= PID_TRESHOLD && sensorValues[4] >= PID_TRESHOLD && sensorValues[5] >= PID_TRESHOLD) {

      motorControl(constrain(m1Speed, MINSPEED, MAXSPEED), constrain(m2Speed, MINSPEED, MAXSPEED));
      // leitura do sensor (0 0 0 1 1 0)
    } else if (sensorValues[0] >= PID_TRESHOLD && sensorValues[1] >= PID_TRESHOLD && sensorValues[2] >= PID_TRESHOLD && sensorValues[3] <= PID_TRESHOLD && sensorValues[4] <= PID_TRESHOLD && sensorValues[5] >= PID_TRESHOLD) {

      motorControl(constrain(m1Speed, MINSPEED, MAXSPEED), constrain(m2Speed, MINSPEED, MAXSPEED));

      // leitura do sensor (1 1 1 0 0 0)
    } else if (sensorValues[0] <= PID_TRESHOLD && sensorValues[1] <= PID_TRESHOLD && sensorValues[2] <= PID_TRESHOLD && sensorValues[3] >= PID_TRESHOLD && sensorValues[4] >= PID_TRESHOLD && sensorValues[5] >= PID_TRESHOLD) {

      motorControl(constrain(m1Speed, MINSPEED, MAXSPEED), constrain(m2Speed, MINSPEED, MAXSPEED));
      // leitura do sensor (0 0 0 1 1 1)
    } else if (sensorValues[0] >= PID_TRESHOLD && sensorValues[1] >= PID_TRESHOLD && sensorValues[2] >= PID_TRESHOLD && sensorValues[3] <= PID_TRESHOLD && sensorValues[4] <= PID_TRESHOLD && sensorValues[5] <= PID_TRESHOLD) {

      motorControl(constrain(m1Speed, MINSPEED, MAXSPEED), constrain(m2Speed, MINSPEED, MAXSPEED));

      // leitura do sensor (0 1 0 0 0 0)
    } else if (sensorValues[0] >= PID_TRESHOLD && sensorValues[1] <= PID_TRESHOLD && sensorValues[2] >= PID_TRESHOLD && sensorValues[3] >= PID_TRESHOLD && sensorValues[4] >= PID_TRESHOLD && sensorValues[5] >= PID_TRESHOLD) {

      motorControl(constrain(m1Speed, MINSPEED, MAXSPEED), constrain(m2Speed, MINSPEED, MAXSPEED));
      // leitura do sensor (0 0 0 0 1 0)
    } else if (sensorValues[0] >= PID_TRESHOLD && sensorValues[1] >= PID_TRESHOLD && sensorValues[2] >= PID_TRESHOLD && sensorValues[3] >= PID_TRESHOLD && sensorValues[4] <= PID_TRESHOLD && sensorValues[5] >= PID_TRESHOLD) {

      motorControl(constrain(m1Speed, MINSPEED, MAXSPEED), constrain(m2Speed, MINSPEED, MAXSPEED));

      // leitura do sensor (1 1 0 0 0 0)
    } else if (sensorValues[0] <= PID_TRESHOLD && sensorValues[1] <= PID_TRESHOLD && sensorValues[2] >= PID_TRESHOLD && sensorValues[3] >= PID_TRESHOLD && sensorValues[4] >= PID_TRESHOLD && sensorValues[5] >= PID_TRESHOLD) {

      motorControl(constrain(m1Speed, MINSPEED, MAXSPEED), constrain(m2Speed, MINSPEED, MAXSPEED));
      // leitura do sensor (0 0 0 0 1 1)
    } else if (sensorValues[0] >= PID_TRESHOLD && sensorValues[1] >= PID_TRESHOLD && sensorValues[2] >= PID_TRESHOLD && sensorValues[3] >= PID_TRESHOLD && sensorValues[4] <= PID_TRESHOLD && sensorValues[5] <= PID_TRESHOLD) {

      motorControl(constrain(m1Speed, MINSPEED, MAXSPEED), constrain(m2Speed, MINSPEED, MAXSPEED));

      // leitura do sensor (1 0 0 0 0 0)
    } else if (sensorValues[0] <= PID_TRESHOLD && sensorValues[1] >= PID_TRESHOLD && sensorValues[2] >= PID_TRESHOLD && sensorValues[3] >= PID_TRESHOLD && sensorValues[4] >= PID_TRESHOLD && sensorValues[5] >= PID_TRESHOLD) {

      motorOption('6', PID_SPEED1, PID_SPEED1);
      // leitura do sensor (0 0 0 0 0 1)
    } else if (sensorValues[0] >= PID_TRESHOLD && sensorValues[1] >= PID_TRESHOLD && sensorValues[2] >= PID_TRESHOLD && sensorValues[3] >= PID_TRESHOLD && sensorValues[4] >= PID_TRESHOLD && sensorValues[5] <= PID_TRESHOLD) {

      motorOption('4', PID_SPEED1, PID_SPEED1);

    }
  }
  motorOption('0', 0, 0);
}

void motorControl(int speedLeft, int speedRight) {
  // Função para controle do driver de motor

  // Definições das portas digitais
  pinMode(PININ1, OUTPUT);
  pinMode(PININ2, OUTPUT);
  pinMode(PININ3, OUTPUT);
  pinMode(PININ4, OUTPUT);
  pinMode(PINENA, OUTPUT);
  pinMode(PINENB, OUTPUT);

  // Ajustes motor da esquerda
  if (speedLeft < 0) {
    speedLeft = -speedLeft;
    digitalWrite (PININ3, HIGH);
    digitalWrite (PININ4, LOW);
  } else {
    digitalWrite (PININ3, LOW);
    digitalWrite (PININ4, HIGH);
  }

  // Ajustes motor da direita
  if (speedRight < 0) {
    speedRight = -speedRight;
    digitalWrite (PININ1, LOW);
    digitalWrite (PININ2, HIGH);
  } else {
    digitalWrite (PININ1, HIGH);
    digitalWrite (PININ2, LOW);
  }

  analogWrite (PINENA, speedLeft);
  analogWrite (PINENB, speedRight);
}

void motorOption(char option, int speedLeft, int speedRight) {
  // Função para controle de motor com pre definições
  switch (option) {
    case '6': // Esquerda
      motorControl(-speedLeft, speedRight);
      break;
    case '4': // Direita
      motorControl(speedLeft, -speedRight);
      break;
    case '2': // Trás
      motorControl(-speedLeft, -speedRight);
      break;
    case '8': // Frente
      motorControl(speedLeft, speedRight);
      break;
    case '0': // Parar
      motorControl(0, 0);
      break;
  }
}

bool motorStop(long runtime, long currentTime) {
  if (millis() >= (runtime + currentTime)) {
    motorOption('0', 0, 0);
    int cont = 0;
    while (true) {
    }
    return false;
  }
  return true;
}

void readSensors(void) {
  // Função para leitura dos sensores
  qtra.read(sensorValues);
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(sensorValues[i]);
    Serial.print(' ');
  }
  Serial.print(";");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(qtra.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  Serial.print(";");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(qtra.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
  Serial.print(";");

  // Processo de leitura da possição da linha
  int position = qtra.readLine(sensorValues);
  Serial.print("POSITION");
  Serial.print("=");
  Serial.print(position);
  Serial.println(";");
}

void calibrateSensors(void) {
  // Função para calibrar os sensores para poder operar em modo seguidor de linha
  qtra.read(sensorValues);
  while (sensorValues[0] > TRESHOLD) {

    qtra.calibrate();
    motorOption('4', MAXSPEEDCALIBRATE, MAXSPEEDCALIBRATE);
    qtra.read(sensorValues);
  }
  delay(10);
  motorOption('0', 0, 0);
  qtra.read(sensorValues);
  while (sensorValues[5] > TRESHOLD) {

    qtra.calibrate();
    motorOption('6', MAXSPEEDCALIBRATE, MAXSPEEDCALIBRATE);
    qtra.read(sensorValues);
  }
  delay(10);
  motorOption('0', 0, 0);
  qtra.read(sensorValues);
  while (sensorValues[2] > TRESHOLD) {

    qtra.calibrate();
    motorOption('4', MAXSPEEDCALIBRATE, MAXSPEEDCALIBRATE);
    qtra.read(sensorValues);
  }
  motorOption('0', 0, 0);

  // Impressão dos valores obtidos para calibração dos sensores
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(qtra.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  Serial.print(";");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(qtra.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
}
