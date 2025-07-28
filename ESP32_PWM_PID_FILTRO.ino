// Projeto ESP32: Controle de temperatura com PID + controle de motor de passo via A4988 + LCD I2C

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <AccelStepper.h>
#include <math.h>

// === LCD I2C ===
LiquidCrystal_I2C lcd(0x27, 16, 2); // Endereço do display, ajuste se necessário

// === Pinos ===
const int pwmMosfetPin = 25;  // PWM para controle do MOSFET
const int ntcPin = 34;        // NTC
const int potVelPin = 35;     // Potenciômetro para velocidade
const int potTempPin = 39;    // VN (GPIO39) - potenciômetro para temperatura alvo

const int dirPin = 27;         // Direção do A4988
const int stepPin = 14;        // Passos do A4988

// === NTC ===
const float seriesResistor = 10000.0;
const float nominalResistance = 100000.0;
const float nominalTemperature = 25.0;
const float bCoefficient = 3950.0;
const float vRef = 3.3;  // ESP32 usa 3.3V

// === PID ===
float kp = 20;
float ki = 3.6;
float kd = 120;

float integral = 0;
float lastError = 0;
unsigned long lastTime = 0;

float setpoint = 230.0;  // Atualizado dinamicamente pelo potenciômetro

// === Motor de passo ===
#define InterfaceMotor 1
AccelStepper Stepper(InterfaceMotor, stepPin, dirPin);

// === TaskHandle ===
TaskHandle_t motorTaskHandle = NULL;

// === Filtro EMA para NTC ===
float alphaNTC = 0.1;
float filteredTemp = 0;
bool ntcFirstRead = true;

// === Média móvel para potenciômetros ===
#define POT_FILTER_SIZE 10
int potVelBuffer[POT_FILTER_SIZE] = {0};
int potTempBuffer[POT_FILTER_SIZE] = {0};
int potIndex = 0;

void motorTask(void *parameter) {
  while (true) {
    Stepper.runSpeed();
    delayMicroseconds(50);
  }
}

// Função para pré-carregar filtro EMA do NTC
void preloadNTCFilter(int n) {
  float sum = 0;
  for (int i = 0; i < n; i++) {
    sum += readTemperature();
    delay(5);
  }
  filteredTemp = sum / n;
  ntcFirstRead = false;
}

float readFilteredTemperature() {
  float tempRaw = readTemperature();
  if (ntcFirstRead) {
    filteredTemp = tempRaw;
    ntcFirstRead = false;
  } else {
    filteredTemp = alphaNTC * tempRaw + (1 - alphaNTC) * filteredTemp;
  }
  return filteredTemp;
}

int readFilteredPot(int rawValue, int* buffer) {
  buffer[potIndex] = rawValue;
  int sum = 0;
  for (int i = 0; i < POT_FILTER_SIZE; i++) {
    sum += buffer[i];
  }
  return sum / POT_FILTER_SIZE;
}

void setup() {
  Serial.begin(115200);

  ledcSetup(0, 5000, 12);  // Canal 0, 5kHz, 12 bits
  ledcAttachPin(pwmMosfetPin, 0);

  Wire.begin(21, 22); // SDA, SCL
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Sistema PID...");
  delay(1000);
  lcd.clear();

  Stepper.setMaxSpeed(3200);
  Stepper.setSpeed(0);
  pinMode(dirPin, OUTPUT);
  digitalWrite(dirPin, HIGH);

  xTaskCreatePinnedToCore(
    motorTask,
    "MotorTask",
    1000,
    NULL,
    1,
    &motorTaskHandle,
    0);

  analogReadResolution(12);

  // Pré-carregamento do filtro EMA do NTC
  preloadNTCFilter(1000);

  // Inicializa buffers dos potenciômetros para média móvel
  for (int i = 0; i < POT_FILTER_SIZE; i++) {
    potVelBuffer[i] = analogRead(potVelPin);
    potTempBuffer[i] = analogRead(potTempPin);
  }
}

void loop() {
  unsigned long loopStart = micros();

  int potVelRaw = analogRead(potVelPin);
  int potTempRaw = analogRead(potTempPin);

  potIndex = (potIndex + 1) % POT_FILTER_SIZE;

  int filteredPotVelRaw = readFilteredPot(potVelRaw, potVelBuffer);
  int filteredPotTempRaw = readFilteredPot(potTempRaw, potTempBuffer);

  float velMap = map(filteredPotVelRaw, 0, 4095, 0, 3200);
  setpoint = map(filteredPotTempRaw, 0, 4095, 0, 275);

  float temp = readFilteredTemperature();

  if (velMap <= 0) {
    Stepper.setSpeed(0);
  } else {
    Stepper.setSpeed(velMap);
  }

  float pwm = 0;
  bool aquecimentoLigado = true;

  if (setpoint <= 40) {
    ledcWrite(0, 0);
    aquecimentoLigado = false;
  } else {
    pwm = computePID(temp);
    ledcWrite(0, (int)pwm);
  }

  Serial.print(temp); Serial.print(",");
  Serial.println(pwm);

  lcd.setCursor(0, 0);
  lcd.print("T:"); lcd.print(temp, 1);
  lcd.print((char)223); lcd.print(" PWM:");
  if (aquecimentoLigado) {
    lcd.print((int)(pwm * 100 / 4095)); lcd.print("% ");
  } else {
    lcd.print("OFF   ");
  }

  lcd.setCursor(0, 1);
  lcd.print("Set:"); lcd.print(setpoint, 0);
  lcd.print((char)223); lcd.print(" V:");
  lcd.print((int)velMap); lcd.print("    ");

  unsigned long loopTime = micros() - loopStart;
  if (loopTime < 4000)
    delayMicroseconds(4000 - loopTime);
}

float readTemperature() {
  int adc = analogRead(ntcPin);
  float voltage = adc * vRef / 4095.0;
  float resistance = seriesResistor * (vRef / voltage - 1.0);

  float steinhart;
  steinhart = resistance / nominalResistance;
  steinhart = log(steinhart);
  steinhart /= bCoefficient;
  steinhart += 1.0 / (nominalTemperature + 273.15);
  steinhart = 1.0 / steinhart;
  steinhart -= 273.15;

  return steinhart;
}

float computePID(float input) {
  unsigned long now = millis();
  float deltaTime = (now - lastTime) / 1000.0;
  if (deltaTime <= 0) deltaTime = 0.001;
  lastTime = now;

  float error = setpoint - input;
  integral += error * deltaTime;
  integral = constrain(integral, -100, 100);

  float derivative = (error - lastError) / deltaTime;
  lastError = error;

  float output = kp * error + ki * integral + kd * derivative;
  return constrain(output * 16, 0, 4095);
}
