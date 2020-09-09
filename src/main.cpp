#include <Arduino.h>
#include <QTRSensors.h>

//Perifericos
#define led1 4
#define led2 3
#define pul1 2
#define pul2 12
//Motores
//Derecho
#define BIN1 9
#define BIN2 10
#define PWMB 11
//Izquierdo
#define AIN1 7
#define AIN2 6
#define PWMA 5
//
#define STBY 8

QTRSensors qtr;

const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];

float proporcional_anterior;
float integral;


// Calibración

int velocidadBase = 30; // Velocidad inicial de los motores cuando esta en recta 0-255

int velocidadGiro = 80;

const int referencia = 0; //En caso de que nuestro sensor no sea 0 en el centro de la linea se ajusta este valor -255 a +255

const float kp=1, kd=0, ki=0; //Valores constantes de nuestro controlador PID

/*
1. Bajar todas las constantes de Kp, Kd, Ki a 0
2. Subir solamente la Kp hasta que el robot termine la pista y no cabecea
3. Subir solamente la Kd hasta que el robot termine la pista y no cabecea
4. Paso 3 pero con Ki

Para cada velocidad base va a existir diferentes valores de KP, KD, KI!!!
*/


// Declaracion de funciones
void menu();
void motores(boolean,boolean,int,int);


void setup()
{
  // configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5}, SensorCount);
  qtr.setEmitterPin(13);

  delay(500);
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  
  menu();
  
  digitalWrite(led1, HIGH);
  digitalWrite(led2, HIGH);// turn on Arduino's LED to indicate we are in calibration mode
  
  pinMode(PWMB, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  
  digitalWrite(STBY,LOW);
  

  // Calibracion y otros metodos de inicio
  for (uint16_t i = 0; i < 100; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(led1, LOW);
  digitalWrite(led2, LOW); // turn off Arduino's LED to indicate we are through with calibration
  
  digitalWrite(STBY,HIGH);
  delay(1000);
  proporcional_anterior = 0;
  integral = 0;
}

void loop()
{
  int16_t position = qtr.readLineBlack(sensorValues);
  
  position = map(position,0,5000,-255,255);
  
  int errorSensores =  position - referencia;
  
  float proporcional = errorSensores * kp;
  
  integral = (integral + proporcional) * ki;
  
  float derivativo = (proporcional - proporcional_anterior) * kd;
  
  int errorControlador = proporcional + integral + derivativo;
  
  if (errorControlador > velocidadGiro) errorControlador = velocidadGiro;
	else if (errorControlador < -velocidadGiro) errorControlador = -velocidadGiro;
  
  motores(true,true,velocidadBase+errorControlador,velocidadBase-errorControlador),
  
  proporcional_anterior = proporcional;
}

void motIzq(bool dir, int vel)
{
  if (vel>0)
  {
    digitalWrite(BIN1,HIGH);
    digitalWrite(BIN2,LOW);
  }  
  else
  {
    digitalWrite(BIN1,LOW);
    digitalWrite(BIN2,HIGH);
  }
  analogWrite(PWMB,abs(vel));
}

void motDer(bool dir, int vel)
{
  if (vel>0)
  {
    digitalWrite(AIN1,HIGH);
    digitalWrite(AIN2,LOW);
  }  
  else
  {
    digitalWrite(AIN1,LOW);
    digitalWrite(AIN2,HIGH);
  }
  analogWrite(PWMA,abs(vel));
}

void motores(bool dirIzq, bool dirDer, int velIzq, int velDer)
{
  motIzq(dirIzq,velIzq);
  motDer(dirDer,velDer);
}

void menu()
{
  boolean bandera = true;
  int opcion=0;
  while(bandera==true)
  {
    if(digitalRead(pul1) && digitalRead(pul2))
    {
      bandera=false;
    }
    else if(digitalRead(pul1))
    {
      delay(250);
      opcion++;
      if(opcion>3)
      {
        opcion = 0;
      }
    }
    else if(digitalRead(pul2))
    {
      delay(150);
      for(int i=0; i<=opcion;i++)
      {
        digitalWrite(led1,HIGH);
        delay(100);
        digitalWrite(led1,LOW);
        delay(100);
      }
    }
  }
}