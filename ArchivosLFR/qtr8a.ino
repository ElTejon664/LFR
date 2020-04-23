#include <QTRSensors.h>




//Mapeo de pines
#define STBY 4
#define AIN1   3
#define AIN2  2
#define PWMB  6
#define PWMA  5
#define BIN1  8
#define BIN2  7
#define NUM_SENSORS             8
#define NUM_SAMPLES_PER_SENSOR  4
#define EMITTER_PIN             11
#define LED     13


// Constantes para PID
float KP = 0.08;
float KD = 1.0;
float Ki = 0.00;

// Regulación de la velocidad Máxima}}
int Velmax = 150;
int v1 = 110;
int v2 = -90;

// Data para intrgal
int error1 = 0;
int error2 = 0;
int error3 = 0;
int error4 = 0;
int error5 = 0;
int error6 = 0;

// Configuración de la librería QTR-8A
QTRSensorsAnalog qtra((unsigned char[])
{
  A0, A1, A2, A3, A4, A5, A6, A7 //# de sensores utilizados
}
, NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);

unsigned int sensorValues[NUM_SENSORS];

// Función accionamiento motor izquierdo
void Motoriz(int value)
{
  if ( value >= 0 )
  {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  }
  else
  {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    value *= -1;
  }
  analogWrite(PWMB, value);
}

// Función accionamiento motor derecho
void Motorde(int value)
{
  if ( value >= 0 )
  {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  }
  else
  {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    value *= -1;
  }
  analogWrite(PWMA, value);
}



//Accionamiento de motores
void Motor(int left, int righ)
{
  digitalWrite(STBY, HIGH);
  Motoriz(left);
  Motorde(righ);
}



//función de freno
void freno(boolean left, boolean righ, int value)
{
  digitalWrite(STBY, HIGH);
  if ( left )
  {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite (PWMB, value);
  }
  if ( righ )
  {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite (PWMA, value);
  }
}







void setup()
{
  // Declaramos como salida los pines utilizados
  pinMode(LED   , OUTPUT);
  pinMode(BIN2  , OUTPUT);
  pinMode(STBY  , OUTPUT);
  pinMode(BIN1  , OUTPUT);
  pinMode(PWMB  , OUTPUT);
  pinMode(AIN1  , OUTPUT);
  pinMode(AIN2  , OUTPUT);
  pinMode(PWMA  , OUTPUT);
  digitalWrite(STBY  , 1);


  // Calibramos con la función qtra.calibrate();, y dejamos parpadeando el led, mientras se produce la calibración.

  for ( int i = 0; i < 70; i++)
  {
    digitalWrite(LED, HIGH); delay(20);
    qtra.calibrate();
    digitalWrite(LED, LOW);  delay(20);
  }
  delay(3000);
}


unsigned int position = 0;

//declaraos variables para utilizar PID
int proporcional = 0;         // Proporcional
int integral = 0;           //Intrgral
int derivativo = 0;          // Derivativo




int diferencial = 0;   // Diferencia aplicada a los motores
int last_prop;         // Última valor del proporcional (utilizado para calcular la derivada del error)
int Target = 3500; // Setpoint (Como utilizamos 6 sensores, la línea debe estar entre 0 y 5000, por lo que el ideal es que esté en 2500)

void loop()
{

  if (position < 50) {
    Motor(v2, v1);
  }
  if (position > 6550) {
    Motor(v1, v2);
  }
  position = qtra.readLine(sensorValues);
  proporcional = ((int)position) - 3500;



  derivativo = proporcional - last_prop;
  integral = error1 + error2 + error3 + error4 + error5 + error6;
  last_prop = proporcional;


  error6 = error5;
  error5 = error4;
  error4 = error3;
  error3 = error2;
  error2 = error1;
  error1 = proporcional;


  int diferencial = ( proporcional * KP ) + ( derivativo * KD ) + (integral * Ki) ;

  if ( diferencial > Velmax ) diferencial = Velmax;
  else if ( diferencial < -Velmax ) diferencial = -Velmax;

  ( diferencial < 0 ) ?
  Motor(Velmax + diferencial, Velmax) : Motor(Velmax, Velmax - diferencial);
  //imprimir valores de lectura del sensor
    Serial.begin(9600);
    //mapeo para que los valores sean de 1 a 100 y 100 a 0
    position = map(position, 1, 100, 100, 1);
    
    Serial.print(position);
  //imprimir valores de lectura del sensor
}
