#include <math.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Inicializa LCD no endereço I2C (ajuste se necessário)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Pinos
const int ntcPin = A0;
const int mosfetPin = 9;

// Constantes do NTC 100k B=3950
const float seriesResistor = 10580.0;
const float nominalResistance = 100000.0;
const float nominalTemperature = 25.0;
const float bCoefficient = 3950.0;
const float adcMax = 1023.0;

// Controle PID
float setpoint = 230.0;
float kp = 25;
float ki = 3.3;
float kd = 80;

float integral = 0;
float lastError = 0;
unsigned long lastTime = 0;

void setup() {
  Serial.begin(9600);
  pinMode(mosfetPin, OUTPUT);

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Controle PID...");
  delay(1000);
  lcd.clear();
}

void loop() {
  float temp = readTemperature();
  Serial.print("Temperatura: ");
  Serial.println(temp);

  float output = computePID(temp);
  analogWrite(mosfetPin, output);

  // LCD: Linha 1 - Temperatura e PWM
  lcd.setCursor(0, 0);
  lcd.print("T:");
  lcd.print(temp, 1);
  lcd.print("C PWM:");
  lcd.print((int)output);
  if (output < 100) lcd.print(" ");  // Apaga lixo visual se PWM < 100

  // LCD: Linha 2 - Setpoint
  lcd.setCursor(0, 1);
  lcd.print("Setpoint: ");
  lcd.print(setpoint, 1);
  lcd.print("C ");

  delay(50);  // Atualização do display
}

float readTemperature() {
  int adc = analogRead(ntcPin);
  float voltage = adc * 5.0 / adcMax;
  float resistance = seriesResistor * (5.0 / voltage - 1.0);
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
  float pwm = constrain(output, 0, 255);

  Serial.print("PWM: ");
  Serial.println(pwm);

  return pwm;
}
