/*
========================================================================================================
TESTE 1 - RESISTENCIAS
Objetivo: ligar/desligar cada resistencia manualmente (via botao) e confirmar no LED + Serial + LCD
que o estagio certo esta acionando. Sem sensores, sem logica automatica, sem wifi - so o basico
pra validar fiacao e reles antes de juntar o resto.

Como usar:
  BTN1 -> liga/desliga Resistencia Estagio 1
  BTN2 -> liga/desliga Resistencia Estagio 2
  BTN3 -> liga/desliga Resistencia Auxiliar (caixa eletronica)
========================================================================================================
*/

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ==============================================================================
// --- Pinos (conforme esquematico "Secador de Ar V1.1", pag. 6) ---
// ==============================================================================
// Saidas TRIAC (RES_1 a RES_4) - RES_4 sobra livre por enquanto
const int resistenciaEstagio1 = 14;  // RES_1
const int resistenciaEstagio2 = 13;  // RES_2
const int resistenciaAuxiliar = 12;  // RES_3

const int ledResistencia1 = 15;  // LED1
const int ledResistencia2 = 16;  // LED2

const int botao01 = 4;  // BT1
const int botao02 = 5;  // BT2
const int botao03 = 6;  // BT3

// ==============================================================================
// --- Objetos e Variaveis ---
// ==============================================================================
LiquidCrystal_I2C lcd(0x27, 20, 4);

bool estagio1Ligado = false;
bool estagio2Ligado = false;
bool auxiliarLigado = false;

// Estado anterior de cada botao, pra detectar so a borda de descida (evita ficar
// ligando/desligando repetidamente enquanto o botao continua pressionado)
bool botao01Anterior = HIGH;
bool botao02Anterior = HIGH;
bool botao03Anterior = HIGH;

unsigned long tempoAtualizaLCD = 0;

// ==============================================================================
void atualizarLCD() {
  if (millis() - tempoAtualizaLCD < 300) return;
  tempoAtualizaLCD = millis();

  lcd.setCursor(0, 0);
  lcd.print("TESTE RESISTENCIAS");

  lcd.setCursor(0, 1);
  lcd.print("Est1: ");
  lcd.print(estagio1Ligado ? "LIGADO " : "desligado");

  lcd.setCursor(0, 2);
  lcd.print("Est2: ");
  lcd.print(estagio2Ligado ? "LIGADO " : "desligado");

  lcd.setCursor(0, 3);
  lcd.print("Aux:  ");
  lcd.print(auxiliarLigado ? "LIGADO " : "desligado");
}

// Le um botao e retorna true so no instante em que ele acabou de ser pressionado
bool botaoPressionado(int pino, bool &estadoAnterior) {
  bool estadoAtual = digitalRead(pino);
  bool pressionadoAgora = (estadoAnterior == HIGH && estadoAtual == LOW);
  estadoAnterior = estadoAtual;
  return pressionadoAgora;
}

// ==============================================================================
void setup() {
  Serial.begin(115200);
  Wire.begin(40, 41);  // SDA=IO40, SCL=IO41

  pinMode(resistenciaEstagio1, OUTPUT);
  pinMode(resistenciaEstagio2, OUTPUT);
  pinMode(resistenciaAuxiliar, OUTPUT);
  pinMode(ledResistencia1, OUTPUT);
  pinMode(ledResistencia2, OUTPUT);

  pinMode(botao01, INPUT_PULLUP);
  pinMode(botao02, INPUT_PULLUP);
  pinMode(botao03, INPUT_PULLUP);

  digitalWrite(resistenciaEstagio1, LOW);
  digitalWrite(resistenciaEstagio2, LOW);
  digitalWrite(resistenciaAuxiliar, LOW);
  digitalWrite(ledResistencia1, LOW);
  digitalWrite(ledResistencia2, LOW);

  lcd.init();
  lcd.backlight();
  lcd.clear();

  Serial.println("=== TESTE DE RESISTENCIAS ===");
  Serial.println("BTN1: Estagio 1 | BTN2: Estagio 2 | BTN3: Auxiliar");
}

// ==============================================================================
void loop() {
  if (botaoPressionado(botao01, botao01Anterior)) {
    estagio1Ligado = !estagio1Ligado;
    digitalWrite(resistenciaEstagio1, estagio1Ligado ? HIGH : LOW);
    digitalWrite(ledResistencia1, estagio1Ligado ? HIGH : LOW);
    Serial.print("Resistencia Estagio 1 -> ");
    Serial.println(estagio1Ligado ? "LIGADA" : "DESLIGADA");
  }

  if (botaoPressionado(botao02, botao02Anterior)) {
    estagio2Ligado = !estagio2Ligado;
    digitalWrite(resistenciaEstagio2, estagio2Ligado ? HIGH : LOW);
    digitalWrite(ledResistencia2, estagio2Ligado ? HIGH : LOW);
    Serial.print("Resistencia Estagio 2 -> ");
    Serial.println(estagio2Ligado ? "LIGADA" : "DESLIGADA");
  }

  if (botaoPressionado(botao03, botao03Anterior)) {
    auxiliarLigado = !auxiliarLigado;
    digitalWrite(resistenciaAuxiliar, auxiliarLigado ? HIGH : LOW);
    Serial.print("Resistencia Auxiliar -> ");
    Serial.println(auxiliarLigado ? "LIGADA" : "DESLIGADA");
  }

  atualizarLCD();
}
