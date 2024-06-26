#include <MQUnifiedsensor.h>
#include <SoftwareSerial.h>
#include <DHT.h>
#include <NewPing.h>

// constantes y objeto sensor MQ-135
#define board "Arduino UNO"
#define Voltage_Resolution 5
#define mqPin A1
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

const int trigPin = 11;
const int echoPin = 12;
const int motorAForward = 3;
const int motorABackward = 2;
const int motorBForward = 5;
const int motorBBackward = 4;
int enableMotor1 = 9;
int enableMotor2 = 10;

bool pathEnded = false;

// módulo bluetooth
SoftwareSerial bluetooth(7, 6); // 7, 6

// sensor temperatura y humedad
#define DHTPIN 14 // A0
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

const int mapSize = 4; // Ajustar según el tamaño del entorno
int mapGrid[mapSize][mapSize];
int closedSet[mapSize][mapSize];
bool openSet[mapSize][mapSize];
float gCost[mapSize][mapSize];
float hCost[mapSize][mapSize];
float fCost[mapSize][mapSize];
int parentX[mapSize][mapSize];
int parentY[mapSize][mapSize];

int startX = 0, startY = 0;  // Posición inicial
int goalX = 3, goalY = 3;  // Posición objetivo
int x = startX, y = startY;  // Posición actual del robot
int direction = 0;  // Dirección inicial (0: norte, 1: este, 2: sur, 3: oeste)

// variables para timing
unsigned long startMillisDht;
unsigned long startMillisDist;
unsigned long startMillisGas;
unsigned long startMillisJson;
unsigned long currentMillis;

NewPing sonar(trigPin, echoPin, 200);  // Máxima distancia de 200 cm

void setup() {
  Serial.begin(9600);
  pinMode(motorAForward, OUTPUT);
  pinMode(motorABackward, OUTPUT);
  pinMode(motorBForward, OUTPUT);
  pinMode(motorBBackward, OUTPUT);
  pinMode(enableMotor1, OUTPUT);
  pinMode(enableMotor2, OUTPUT);

  pinMode(mqPin, INPUT); // sensor de gas

  // variables para timing
  startMillisDht = millis();
  startMillisDist = millis();
  startMillisGas = millis();
  startMillisJson = millis();

  bluetooth.begin(38400);

  pinMode(trigPin, OUTPUT); 
  pinMode(echoPin, INPUT);
    
  pinMode(DHTPIN, INPUT);
  dht.begin();

  Serial.print("Calibrating please wait.");
  float calcR0 = 0;
  for(int i = 1; i <= 10; i++) { 
    MQ135.update(); // Update data, the arduino will read the voltage from the analog pin
    calcR0 += MQ135.calibrate(RatioMQ135CleanAir);
    Serial.print(".");
  }
  Serial.println(calcR0);
  MQ135.setR0(calcR0 / 10);

  // Inicializar el mapa y los conjuntos
  for (int i = 0; i < mapSize; i++) {
    for (int j = 0; j < mapSize; j++) {
      mapGrid[i][j] = 0;
      closedSet[i][j] = 0;
      openSet[i][j] = false;
      gCost[i][j] = hCost[i][j] = fCost[i][j] = 999999999;
    }
  }
  gCost[startX][startY] = 0;
  hCost[startX][startY] = heuristic(startX, startY);
  fCost[startX][startY] = hCost[startX][startY];
  openSet[startX][startY] = true;
}

float dhtArray[2] = {0, 0};

void loop() {
  currentMillis = millis();

  if(!pathEnded){
    if (x == goalX && y == goalY) {
      // Hemos llegado al objetivo
      stopMotors();
      Serial.println("Objetivo alcanzado");
      pathEnded = true;
    }

    // Realizar la búsqueda A*
    int currentX = -1, currentY = -1;
    float lowestF = 999999999;
    for (int i = 0; i < mapSize; i++) {
      for (int j = 0; j < mapSize; j++) {
        if (openSet[i][j] && fCost[i][j] < lowestF) {
          lowestF = fCost[i][j];
          currentX = i;
          currentY = j;
        }
      }
    }

    if (currentX == -1 || currentY == -1) {
      // No se encontró un camino
      stopMotors();
      Serial.println("No se encontró un camino");
      pathEnded = true;
    }

    openSet[currentX][currentY] = false;
    closedSet[currentX][currentY] = 1;

    for (int i = 0; i < 4; i++) {
      int newX = currentX;
      int newY = currentY;
      switch (i) {
        case 0: newY -= 1; break;  // Norte
        case 1: newX += 1; break;  // Este
        case 2: newY += 1; break;  // Sur
        case 3: newX -= 1; break;  // Oeste
      }

      if (isValid(newX, newY) && !closedSet[newX][newY] && mapGrid[newX][newY] == 0) {
        float tentativeG = gCost[currentX][currentY] + 1;
        if (tentativeG < gCost[newX][newY]) {
          parentX[newX][newY] = currentX;
          parentY[newX][newY] = currentY;
          gCost[newX][newY] = tentativeG;
          hCost[newX][newY] = heuristic(newX, newY);
          fCost[newX][newY] = gCost[newX][newY] + hCost[newX][newY];
          openSet[newX][newY] = true;
        }
      }
    }

    // Verificar si hay un obstáculo antes de moverse
    if (getDistance() > 15) {
      moveRobotTo(currentX, currentY);
    } else {
      // Marcar la celda como bloqueada si se detecta un obstáculo
      mapGrid[currentX][currentY] = 1;
      openSet[currentX][currentY] = false;
      closedSet[currentX][currentY] = true;
    }
  }

  if(currentMillis - startMillisDht >= 5000) {
    dhtMeasure(dhtArray);
    startMillisDht = millis();
  }
  // leemos los gases cada 500 milisegundos
  if(currentMillis - startMillisGas >= 500) {
    gasMeasure();
    startMillisGas = millis();
  }
  // enviaremos las lecturas de los sensores en formato json cada 300 milisegundos mediante el módulo Bluetooth HC05
  if(currentMillis - startMillisJson >= 300) {
    String sensorValues = (String) "hmd:" + dhtArray[0] + ",tmp:" + dhtArray[1] + ",CO:" + CO + ",alc:" + Alcohol + ",CO2:" + CO2 + ",tol:" + Toluene + ",NH4:" + NH4 + ",ace:" + Acetone;
    bluetooth.println(sensorValues);
    Serial.println(sensorValues);
    Serial.println(sonar.ping_cm());
    startMillisJson = millis();
  }
}

void gasMeasure() {
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

void dhtMeasure(float *arr) {
  // humidity and temp
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  if(isnan(h) || isnan(t)) {
    arr[0] = 0.1;
    arr[1] = 0.1;
    return;
  }
  arr[0] = h;
  arr[1] = t;
}

float heuristic(int x, int y) {
  return abs(goalX - x) + abs(goalY - y);
}

bool isValid(int x, int y) {
  return x >= 0 && x < mapSize && y >= 0 && y < mapSize;
}

void moveRobotTo(int newX, int newY) {
  int dx = newX - x;
  int dy = newY - y;

  // Calcular la dirección a moverse
  int newDirection = 0;
  if (dx == 1) newDirection = 1;  // Este
  else if (dx == -1) newDirection = 3;  // Oeste
  else if (dy == 1) newDirection = 2;  // Sur
  else if (dy == -1) newDirection = 0;  // Norte

  // Girar el robot hacia la nueva dirección
  while (direction != newDirection) {
    turnRight();
    delay(500);
  }

  // Mover hacia adelante
  moveForward();
  delay(500);

  // Actualizar la posición del robot
  x = newX;
  y = newY;
}

void moveForward() {
  stopMotors();
  setMotorSpeed1(225);
  setMotorSpeed2(225);
  digitalWrite(motorAForward, HIGH);
  digitalWrite(motorBForward, HIGH);
  digitalWrite(motorABackward, LOW);
  digitalWrite(motorBBackward, LOW);
  delay(600);
  stopMotors();
}

void turnRight() {
  stopMotors();
  setMotorSpeed1(225);
  setMotorSpeed2(225);
  digitalWrite(motorAForward, LOW);
  digitalWrite(motorABackward, HIGH);
  digitalWrite(motorBForward, HIGH);
  digitalWrite(motorBBackward, LOW);
  delay(450);
  direction = (direction + 1) % 4;
  stopMotors();
}

int getDistance() {
  unsigned int distance = sonar.ping_cm();
  return distance == 0 ? 200 : distance;  // Si no hay ping, devolver 200 cm (máxima distancia)
}

void stopMotors() {
  digitalWrite(motorAForward, LOW);
  digitalWrite(motorABackward, LOW);
  digitalWrite(motorBForward, LOW);
  digitalWrite(motorBBackward, LOW);
}

void setMotorSpeed1(int speed) {
  analogWrite(enableMotor1, speed);
}

void setMotorSpeed2(int speed) {
  analogWrite(enableMotor2, speed);
}
