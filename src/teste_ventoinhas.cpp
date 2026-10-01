/*
========================================================================================================
TESTE 3 - VENTOINHAS (reles RL1-RL3 / VENT_1-VENT_3)
Objetivo: ligar/desligar cada rele manualmente (via botao) e confirmar no LED + Serial + LCD.
Hoje so tem 1 ventoinha de verdade instalada (a do 1o estagio, que comanda a ventilacao geral
do sistema) -> VENT_1. VENT_2 e VENT_3 ficam de reserva/futuro, mas ja testaveis aqui.

Como usar:
  BTN1 -> liga/desliga Ventoinha 1 (VENT_1 - a que existe de verdade hoje)
  BTN2 -> liga/desliga Ventoinha 2 (VENT_2 - reserva/futuro)
  BTN3 -> liga/desliga Ventoinha 3 (VENT_3 - reserva/futuro)

Esse arquivo NAO compila junto com os outros testes - use o environment "teste_ventoinhas" do
platformio.ini pra compilar/gravar so este:
  pio run -e teste_ventoinhas -t upload
========================================================================================================
*/

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ==============================================================================
// --- Pinos (conforme esquematico "Secador de Ar V1.1", pag. 6) ---
// ==============================================================================
const int ventoinha1 = 10;  // VENT_1 - ventoinha real do 1o estagio (ventilacao geral)
const int ventoinha2 = 9;   // VENT_2 - reserva/futuro
const int ventoinha3 = 46;  // VENT_3 - reserva/futuro

const int ledVentoinha1 = 15;  // LED1 - indica a ventoinha real (VENT_1) ligada

const int botao01 = 4;  // BT1
const int botao02 = 5;  // BT2
const int botao03 = 6;  // BT3

// ==============================================================================
// --- Objetos e Variaveis ---
// ==============================================================================
LiquidCrystal_I2C lcd(0x27, 20, 4);

bool ventoinha1Ligada = false;
bool ventoinha2Ligada = false;
bool ventoinha3Ligada = false;

// Estado anterior de cada botao, pra detectar so a borda de descida
bool botao01Anterior = HIGH;
bool botao02Anterior = HIGH;
bool botao03Anterior = HIGH;

unsigned long tempoAtualizaLCD = 0;

// ==============================================================================
void atualizarLCD() {
  if (millis() - tempoAtualizaLCD < 300) return;
  tempoAtualizaLCD = millis();

  lcd.setCursor(0, 0);
  lcd.print("TESTE VENTOINHAS");

  lcd.setCursor(0, 1);
  lcd.print("V1(real): ");
  lcd.print(ventoinha1Ligada ? "LIGADA " : "desligada");

  lcd.setCursor(0, 2);
  lcd.print("V2(fut):  ");
  lcd.print(ventoinha2Ligada ? "LIGADA " : "desligada");

  lcd.setCursor(0, 3);
  lcd.print("V3(fut):  ");
  lcd.print(ventoinha3Ligada ? "LIGADA " : "desligada");
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

  pinMode(ventoinha1, OUTPUT);
  pinMode(ventoinha2, OUTPUT);
  pinMode(ventoinha3, OUTPUT);
  pinMode(ledVentoinha1, OUTPUT);

  pinMode(botao01, INPUT_PULLUP);
  pinMode(botao02, INPUT_PULLUP);
  pinMode(botao03, INPUT_PULLUP);

  digitalWrite(ventoinha1, LOW);
  digitalWrite(ventoinha2, LOW);
  digitalWrite(ventoinha3, LOW);
  digitalWrite(ledVentoinha1, LOW);

  lcd.init();
  lcd.backlight();
  lcd.clear();

  Serial.println("=== TESTE DE VENTOINHAS (RELES) ===");
  Serial.println("BTN1: Ventoinha 1 (real) | BTN2: Ventoinha 2 (futuro) | BTN3: Ventoinha 3 (futuro)");
}

// ==============================================================================
void loop() {
  if (botaoPressionado(botao01, botao01Anterior)) {
    ventoinha1Ligada = !ventoinha1Ligada;
    digitalWrite(ventoinha1, ventoinha1Ligada ? HIGH : LOW);
    digitalWrite(ledVentoinha1, ventoinha1Ligada ? HIGH : LOW);
    Serial.print("Ventoinha 1 (real) -> ");
    Serial.println(ventoinha1Ligada ? "LIGADA" : "DESLIGADA");
  }

  if (botaoPressionado(botao02, botao02Anterior)) {
    ventoinha2Ligada = !ventoinha2Ligada;
    digitalWrite(ventoinha2, ventoinha2Ligada ? HIGH : LOW);
    Serial.print("Ventoinha 2 (futuro) -> ");
    Serial.println(ventoinha2Ligada ? "LIGADA" : "DESLIGADA");
  }

  if (botaoPressionado(botao03, botao03Anterior)) {
    ventoinha3Ligada = !ventoinha3Ligada;
    digitalWrite(ventoinha3, ventoinha3Ligada ? HIGH : LOW);
    Serial.print("Ventoinha 3 (futuro) -> ");
    Serial.println(ventoinha3Ligada ? "LIGADA" : "DESLIGADA");
  }

  atualizarLCD();
}
