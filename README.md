# Temp_PID
Controle de temperatura com PID usando ESP 32

# 🧠 Projeto ESP32: Controle de Temperatura com PID + Motor de Passo + Display LCD I2C

Este projeto realiza o **controle de temperatura de uma extrusora de impressora 3D** utilizando um **sensor NTC 100k**, controle **PID**, acionamento por **MOSFET IRF520**, exibição de dados em um **display LCD I2C** e controle de um **motor de passo com driver A4988**.

---

## 📌 Objetivo

Desenvolver um sistema embarcado usando **ESP32** capaz de:

- Controlar a temperatura de uma extrusora com precisão utilizando um **PID digital**
- Acionar um **MOSFET IRF520** via PWM
- Controlar a **velocidade de um motor de passo** usando o **driver A4988**
- Exibir informações como **temperatura**, **setpoint**, **PWM** e **velocidade** em um **display LCD 16x2 I2C**
- Permitir ajuste de temperatura e velocidade por **potenciômetros analógicos**
- Utilizar **filtros digitais** para eliminar ruídos das leituras dos sensores

---

## 🧰 Componentes utilizados

- ESP32 (DevKit v1)
- Sensor de temperatura NTC 100k B=3950
- Resistor de 10kΩ para divisor com o NTC
- Potenciômetros de 10kΩ (2x)
- Driver de motor A4988
- Motor de passo (ex: NEMA 17)
- Display LCD 16x2 com módulo I2C (PCF8574)
- MOSFET IRF520
- Fonte 12V
- Componentes discretos (capacitores, jumpers, etc.)

---

## 🛠️ Pinos utilizados na ESP32

| Função                  | Pino (GPIO) |
|------------------------|-------------|
| PWM para MOSFET        | 25          |
| NTC (entrada analógica)| 34 (ADC1)   |
| Potenciômetro - Veloc. | 35 (ADC1)   |
| Potenciômetro - Temp.  | 39 (VN - ADC1) |
| A4988 - DIR            | 27          |
| A4988 - STEP           | 14          |
| LCD I2C - SDA          | 21          |
| LCD I2C - SCL          | 22          |

---

## ⚙️ Como funciona

### 🔥 Controle de temperatura (PID)

- A leitura da temperatura é feita com um **NTC 100k**, convertida usando a equação de Steinhart-Hart.
- Um **PID digital** calcula o valor de PWM necessário para acionar o **MOSFET** que aquece a extrusora.
- O valor de temperatura desejado (**setpoint**) é ajustado por um potenciômetro.

### 🚀 Controle de motor de passo (A4988)

- Um potenciômetro define a **velocidade** do motor em passos por segundo.
- O controle é feito usando a biblioteca `AccelStepper`.
- O movimento é executado em uma `Task` dedicada no **núcleo 0 da ESP32**, garantindo que o motor não trave mesmo com o loop principal carregado.

### 📊 Exibição no LCD

- LCD 16x2 I2C mostra:
  - Linha 1: Temperatura atual e PWM (% ou "OFF")
  - Linha 2: Setpoint (ºC) e velocidade do motor (passos/s)

---

## 🧪 Filtros digitais aplicados

| Sensor         | Filtro         | Observação                              |
|----------------|----------------|-----------------------------------------|
| NTC            | EMA (α = 0.1)  | Com pré-carregamento de 1000 amostras   |
| Potenciômetros | Média móvel (10 amostras) | Suaviza variações indesejadas     |

---

## 🕹️ Versão anterior (Arduino Uno)

A versão inicial do projeto foi desenvolvida para **Arduino Uno**, com as seguintes limitações:

- Controlava apenas a temperatura com PID e PWM
- Utilizava `analogRead()` com resolução de 10 bits
- Não havia controle de motor, display LCD, nem filtragem

### ✅ Avanços na versão ESP32

- **Multitarefa** para manter o motor de passo girando de forma contínua
- PWM de **12 bits**
- Leituras analógicas mais precisas (ADC de 12 bits)
- Inclusão do **display I2C**
- Adição de **filtros digitais**
- Atualização a **250 Hz** com sincronização por `micros()`

---

## 📈 Plotagem serial

O código realiza a plotagem serial no formato:
Temperatura,PWM


Você pode visualizar os gráficos com o **Plotter Serial** da IDE do Arduino ou ferramentas como o **SerialPlot**.
---

## 📌 Melhorias futuras

- Interface Web para ajuste remoto dos parâmetros PID
- Armazenamento dos parâmetros em EEPROM
- Segurança por temperatura máxima e watchdog de travamento
- Inclusão de sensor de corrente ou encoder para feedback do motor

---

