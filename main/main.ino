#include <MQUnifiedsensor.h>
#include <ArduinoJson.h>
#include <SoftwareSerial.h>
#include <DHT.h>

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
MQUnifiedsensor MQ1 35(board, Voltage_Resolution, ADC_Bit_Resolution, mqPin, type);

// módulo bluetooth
// Arduino RX (A4) <- Module TX
// Arduino TX (A5) -> Module RX
SoftwareSerial bluetooth(18,19); // A4, A5

// pines sensor ultrasonido
#define TRIGGER_PIN  15 // A1
#define ECHO_PIN     14 // A0

// sensor temperatura y humedad
#define DHTPIN 2 // D2
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// json que se enviará a la aplicación
JsonDocument sensors;

// variables para timing
unsigned long startMillisDht;
unsigned long startMillisDist;
unsigned long startMillisGas;
unsigned long startMillisJson;
unsigned long currentMillis;

// prototipos funciones
void dhtMeasure(float *);
void gasMeasure();
long readDistance(int, int);

void setup() {
    // variables para timing
    startMillisDht = millis();
    startMillisDist = millis();
    startMillisGas = millis();
    startMillisJson = millis();
    
    Serial.begin(9600); 
    bluetooth.begin(38400);

    // modelo matemático para calcular la concentracion de ppm
    MQ135.setRegressionMethod(1);
    MQ135.setA(102.2);
    MQ135.setB(-2.473);
    MQ135.init();
    
    pinMode(TRIGGER_PIN, OUTPUT); 
    pinMode(ECHO_PIN, INPUT);
    
    pinMode(DHTPIN, INPUT);
    dht.begin();

    // calibración del sensor MQ-135
    float calcR0 = 0;
    for(int i = 1; i<=10; i ++)
    {
      MQ135.update(); // actualizamos los datos, arduino leerá el voltage en el pin analógico
      calcR0 += MQ135.calibrate(RatioMQ135CleanAir);
    }
    MQ135.setR0(calcR0/10);
  
    if(isinf(calcR0)) {Serial.println("R0 es infinito, revisar conexiones y fuente de alimentación"); while(1);}
    if(calcR0 == 0){Serial.println("R0 es 0, revisar conexiones y fuente de alimentación"); while(1);}
}

float dhtArray[3] = {0,0,0};
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
    distance = readDistance(TRIGGER_PIN, ECHO_PIN);
    startMillisDist = millis();
  }

  // leemos los gases cada 500 milisegundos
  if(currentMillis-startMillisGas >= 500)
  {
    gasMeasure();
    startMillisGas = millis();
  }

  // enviaremos las lecturas de los sensores en formato json cada 300 milisegundos mediante el puerto serial
  if(currentMillis-startMillisJson >= 300)
  {
    sensors["sensores"]["temperatura"]["unidad"] = "celsius";
    sensors["sensores"]["temperatura"]["valor"] = dhtArray[1];
    sensors["sensores"]["humedad"]["unidad"] = "Porcentaje";
    sensors["sensores"]["humedad"]["valor"] = dhtArray[0];
    sensors["sensores"]["CO"]["unidad"] = "ppm";
    sensors["sensores"]["CO"]["valor"] = CO;
    sensors["sensores"]["CO2"]["unidad"] = "ppm";
    sensors["sensores"]["CO2"]["valor"] = CO2;
    sensors["sensores"]["alcohol"]["unidad"] = "ppm";
    sensors["sensores"]["alcohol"]["valor"] = Alcohol;
    sensors["sensores"]["NH4"]["unidad"] = "ppm";
    sensors["sensores"]["NH4"]["valor"] = NH4;
    sensors["sensores"]["acetona"]["unidad"] = "ppm ";
    sensors["sensores"]["acetona"]["valor"] = Acetone;
    sensors["sensores"]["tolueno"]["unidad"] = "ppm";
    sensors["sensores"]["tolueno"]["valor"] = Toluene;
    sensors["sensores"]["distancia"]["unidad"] = "cm";
    sensors["sensores"]["distancia"]["valor"] = distance;
    serializeJsonPretty(sensors, Serial);
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

  // indice de calor
  float hic = dht.computeHeatIndex(t, h, false);

  // llenamos el array
  arr[0] = h;
  arr[1] = t;
  arr[2] = hic;
}

long readDistance(int trigger, int echo) {
  long duration; // tiempo que demora en llegar la onda emitida 
  long distance; // distancia en centimetros

  // medición según datasheet del sensor
  digitalWrite(trigger, LOW); 
  delayMicroseconds(2); 
  digitalWrite(trigger, HIGH); 
  delayMicroseconds(10); 
  digitalWrite(trigger, LOW); 
 
  duration = pulseIn(echo, HIGH); // obtenemos la duración del viaje del pulso
  distance = (duration * 0.0343) / 2; // convertimos a centimetros
  return distance; 
}
