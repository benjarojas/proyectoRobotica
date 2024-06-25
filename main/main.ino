#include <MQUnifiedsensor.h>
#include <SoftwareSerial.h>
#include <DHT.h>
#include <NewPing.h>

// motores
int motor1pin1 = 2;
int motor1pin2 = 3;
int motor2pin1 = 4;
int motor2pin2 = 5;
int enableMotor1 = 9;
int enableMotor2 = 10;

// constantes y objeto sensor MQ-135
#define board "Arduino UNO"
#define Voltage_Resolution 5
#define mqPin A2
#define type "MQ-135"
#define ADC_Bit_Resolution 10
#define RatioMQ135CleanAir 3.6 // 3.6 ppm
float CO = 0; // monoxido de carbono
float Alcohol = 0; // alcohol
float CO2 = 0; // dioxido de carbono
float Toluene = 0; // tolueno
float NH4 = 0; // amonio
float Acetone = 0; // acetona
MQUnifiedsensor MQ135(board, Voltage_Resolution, ADC_Bit_Resolution, mqPin, type);

// módulo bluetooth
// Arduino RX (A4) <- Module TX
// Arduino TX (A5) -> Module RX
SoftwareSerial bluetooth(18,19); // A4, A5

// pines sensor ultrasonido
#define TRIGGER_PIN  15 // A1
#define ECHO_PIN     14 // A0
#define MAX_DISTANCE 200
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

// sensor temperatura y humedad
#define DHTPIN 2 // D2
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// variables para timing
unsigned long startMillisDht;
unsigned long startMillisDist;
unsigned long startMillisGas;
unsigned long startMillisJson;
unsigned long currentMillis;

// prototipos funciones
void dhtMeasure(float *);
void gasMeasure();
void setSpeedMotor1(int);
void setSpeedMotor2(int);
void forward();
void backward();
void turn_right();
void turn_left();

void setup() {
    // motores
    pinMode(motor1pin1, OUTPUT);
    pinMode(motor1pin2, OUTPUT);
    pinMode(motor2pin1, OUTPUT);
    pinMode(motor2pin2, OUTPUT);
    pinMode(enableMotor1, OUTPUT);
    pinMode(enableMotor2, OUTPUT);
  
    // variables para timing
    startMillisDht = millis();
    startMillisDist = millis();
    startMillisGas = millis();
    startMillisJson = millis();
    
    Serial.begin(9600); 
    bluetooth.begin(38400);

    // modelo matemático para calcular la concentracion de ppm
    MQ135.setRegressionMethod(1);
    MQ135.setA(102.2); MQ135.setB(-2.473);
    MQ135.init();
    
    pinMode(TRIGGER_PIN, OUTPUT); 
    pinMode(ECHO_PIN, INPUT);
    
    pinMode(DHTPIN, INPUT);
    dht.begin();

    MQ135.setR0(148.45/10);
}

float dhtArray[2] = {0,0};
long distance = 0;

void loop() {
  // leemos el sensor DHT11 cada 5000 milisegundos (recomendado en el datasheet del sensor)
  currentMillis = millis(); 
  if(currentMillis-startMillisDht >= 5000)
  {
    dhtMeasure(dhtArray);
    startMillisDht = millis();
  }

  // leemos la distancia cada 500 milisegundos
  if(currentMillis-startMillisDist >= 500)
  {
    distance = sonar.ping_cm();
    startMillisDist = millis();
  }

  // leemos los gases cada 500 milisegundos
  if(currentMillis-startMillisGas >= 500)
  {
    gasMeasure();
    startMillisGas = millis();
  }

  // enviaremos las lecturas de los sensores en formato json cada 300 milisegundos mediante el módulo Bluetooth HC05
  if(currentMillis-startMillisJson >= 300)
  {
    String sensorValues = (String) "hmd:"+dhtArray[0]+",tmp:"+dhtArray[1]+",CO:"+CO+",alc:"+Alcohol+",CO2:"+CO2+",tol:"+Toluene+",NH4:"+NH4+",ace:"+Acetone;
    bluetooth.print(sensorValues);
    bluetooth.print(";"); // separador de mensajes
    startMillisJson = millis();
  }
}

void gasMeasure()
{
  MQ135.update();
  MQ135.setA(605.18); MQ135.setB(-3.937); // configuramos la ecuacion para calcular la concentracion de monóxido de carbono
  CO = MQ135.readSensor();

  MQ135.setA(77.255); MQ135.setB(-3.18); // configuramos la ecuación para calcular la concentración de alcohol
  Alcohol = MQ135.readSensor(); 

  MQ135.setA(110.47); MQ135.setB(-2.862); // configuramos la ecuación para calcular la concentración de dióxido de carbono
  CO2 = MQ135.readSensor();

  MQ135.setA(44.947); MQ135.setB(-3.445); // configuramos la ecuación para calcular la concentración de tolueno
  Toluene = MQ135.readSensor();
    
  MQ135.setA(102.2 ); MQ135.setB(-2.473); // configuramos la ecuación para calcular la concentración de amonio
  NH4 = MQ135.readSensor(); 
  
  MQ135.setA(34.668); MQ135.setB(-3.369); // configuramos la ecuación para calcular la concentración de acetona
  Acetone = MQ135.readSensor();
}

void dhtMeasure(float *arr)
{
  // humidity and temp
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  if(isnan(h) || isnan(t))
  { 
    // llenamos el array con -1 para debug
    arr[0] = -1;
    arr[1] = -1;
    arr[2] = -1;
    return;
  }
  // llenamos el array
  arr[0] = h;
  arr[1] = t;
}

void setSpeedMotor1(int speed){
  analogWrite(enableMotor1, speed);
}

void setSpeedMotor2(int speed){
  analogWrite(enableMotor2, speed);  
}

void forward(){
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);

  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);
}

void backward(){
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, HIGH);

  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, HIGH);
}
