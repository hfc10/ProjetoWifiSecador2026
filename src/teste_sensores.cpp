/*
========================================================================================================
TESTE 2 - SENSORES (AHT20 x6 via multiplexador TCA9548A + BMP180)
Objetivo: varrer os canais do multiplexador I2C (0 a 7), identificar o que esta conectado em
cada um (AHT20 = 0x38, BMP180 = 0x77) e mostrar as leituras no Serial e no LCD. Sem logica de
controle, sem resistencias/ventoinhas - so leitura.

Esse arquivo NAO compila junto com Secador_Novo2026.cpp (teste de resistencias) - use o
environment "teste_sensores" do platformio.ini pra compilar/gravar so este:
  pio run -e teste_sensores -t upload
========================================================================================================
*/

#include <Arduino.h>
#include <Wire.h>
#include <AHT20.h>
#include <BMP180I2C.h>
#include <LiquidCrystal_I2C.h>

// ==============================================================================
// --- Enderecos I2C ---
// ==============================================================================
#define ENDERECO_MUX 0x70    // TCA9548APWR (A0/A1/A2 aterrados)
#define ENDERECO_AHT20 0x38
#define ENDERECO_BMP180 0x77
#define ENDERECO_SHT2X 0x40  // TEMPORARIO: sensor SHT20/SHT25 que esta no canal 0 pra teste - remover depois
#define TOTAL_CANAIS 8

// ==============================================================================
// --- Objetos ---
// ==============================================================================
LiquidCrystal_I2C lcd(0x27, 20, 4);
AHT20 sensorAHT(ENDERECO_AHT20);
BMP180I2C sensorBMP(ENDERECO_BMP180);

// ==============================================================================
unsigned long tempoUltimaVarredura = 0;
int canalMostradoLCD = 0;

// Seleciona um unico canal no multiplexador (desliga os outros)
bool selecionarCanalMux(uint8_t canal) {
  Wire.beginTransmission(ENDERECO_MUX);
  Wire.write(1 << canal);
  return Wire.endTransmission() == 0;
}

// Checa rapido se tem alguem respondendo no endereco antes de chamar a lib do sensor
bool dispositivoPresente(uint8_t endereco) {
  Wire.beginTransmission(endereco);
  return Wire.endTransmission() == 0;
}

// Le e imprime o BMP180 (temperatura + pressao) no canal ja selecionado
void lerImprimirBMP180() {
  if (!sensorBMP.begin()) {
    Serial.println("    BMP180 nao respondeu ao begin()");
    return;
  }
  sensorBMP.resetToDefaults();
  sensorBMP.setSamplingMode(BMP180MI::MODE_UHR);

  unsigned long inicio = millis();
  sensorBMP.measureTemperature();
  do { delay(50); } while (!sensorBMP.hasValue() && millis() - inicio < 2000);
  float temperatura = sensorBMP.getTemperature();

  inicio = millis();
  sensorBMP.measurePressure();
  do { delay(50); } while (!sensorBMP.hasValue() && millis() - inicio < 2000);
  float pressao = sensorBMP.getPressure();

  Serial.print("    BMP180 -> T: "); Serial.print(temperatura, 2);
  Serial.print(" C | P: "); Serial.print(pressao, 2); Serial.println(" Pa");
}

// Le e imprime o AHT20 (temperatura + umidade) no canal ja selecionado
void lerImprimirAHT20() {
  if (!sensorAHT.begin()) {
    Serial.println("    AHT20 nao respondeu ao begin()");
    return;
  }

  unsigned long inicio = millis();
  while (!sensorAHT.available() && millis() - inicio < 2000) {
    delay(50);
  }

  float temperatura = sensorAHT.getTemperature();
  float umidade = sensorAHT.getHumidity();

  Serial.print("    AHT20 -> T: "); Serial.print(temperatura, 2);
  Serial.print(" C | U: "); Serial.print(umidade, 2); Serial.println(" %");
}

// TEMPORARIO: leitura do SHT20/SHT25 via comandos "no hold master" (sem lib pronta no projeto).
// Comandos: 0xF3 = mede temperatura, 0xF5 = mede umidade. Retorna false se o sensor nao respondeu.
bool lerSHT2x(float &temperatura, float &umidade) {
  Wire.beginTransmission(ENDERECO_SHT2X);
  Wire.write(0xF3);
  if (Wire.endTransmission() != 0) return false;
  delay(90);  // tempo max de conversao (14 bits) segundo o datasheet
  if (Wire.requestFrom((int)ENDERECO_SHT2X, 3) != 3) return false;
  uint16_t rawTemp = (Wire.read() << 8) | Wire.read();
  Wire.read();  // CRC (nao verificado aqui, so pra teste)
  rawTemp &= 0xFFFC;
  temperatura = -46.85 + 175.72 * ((float)rawTemp / 65536.0);

  Wire.beginTransmission(ENDERECO_SHT2X);
  Wire.write(0xF5);
  if (Wire.endTransmission() != 0) return false;
  delay(30);  // tempo max de conversao (12 bits) segundo o datasheet
  if (Wire.requestFrom((int)ENDERECO_SHT2X, 3) != 3) return false;
  uint16_t rawUmid = (Wire.read() << 8) | Wire.read();
  Wire.read();
  rawUmid &= 0xFFFC;
  umidade = -6 + 125 * ((float)rawUmid / 65536.0);
  return true;
}

void lerImprimirSHT2x() {
  float temperatura, umidade;
  if (!lerSHT2x(temperatura, umidade)) {
    Serial.println("    SHT2x nao respondeu a leitura");
    return;
  }
  Serial.print("    SHT2x -> T: "); Serial.print(temperatura, 2);
  Serial.print(" C | U: "); Serial.print(umidade, 2); Serial.println(" %");
}

// Varre os 8 canais do mux, mostrando no Serial o que foi encontrado em cada um
void varrerCanais() {
  Serial.println();
  Serial.println("========== VARREDURA I2C (MUX) ==========");

  for (uint8_t canal = 0; canal < TOTAL_CANAIS; canal++) {
    Serial.print("Canal "); Serial.print(canal); Serial.print(": ");

    if (!selecionarCanalMux(canal)) {
      Serial.println("MUX nao respondeu (0x70) - confira alimentacao/fiacao do multiplexador.");
      continue;
    }

    bool temAHT = dispositivoPresente(ENDERECO_AHT20);
    bool temBMP = dispositivoPresente(ENDERECO_BMP180);
    bool temSHT = dispositivoPresente(ENDERECO_SHT2X);  // TEMPORARIO

    if (!temAHT && !temBMP && !temSHT) {
      Serial.println("nada conectado.");
      continue;
    }

    Serial.println();
    if (temAHT) lerImprimirAHT20();
    if (temBMP) lerImprimirBMP180();
    if (temSHT) lerImprimirSHT2x();  // TEMPORARIO
  }
}

// Mostra no LCD, em rodizio, o resultado de um canal por vez (atualiza junto com a varredura)
void atualizarLCD(uint8_t canal) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("TESTE SENSORES I2C");

  lcd.setCursor(0, 1);
  lcd.print("Canal: ");
  lcd.print(canal);

  if (!selecionarCanalMux(canal)) {
    lcd.setCursor(0, 2);
    lcd.print("MUX (0x70) falhou");
    return;
  }

  bool temAHT = dispositivoPresente(ENDERECO_AHT20);
  bool temBMP = dispositivoPresente(ENDERECO_BMP180);
  bool temSHT = dispositivoPresente(ENDERECO_SHT2X);  // TEMPORARIO

  if (!temAHT && !temBMP && !temSHT) {
    lcd.setCursor(0, 2);
    lcd.print("Nada conectado");
    return;
  }

  if (temSHT) {  // TEMPORARIO
    float temperatura, umidade;
    lcd.setCursor(0, 2);
    if (lerSHT2x(temperatura, umidade)) {
      lcd.print("SHT2x T:");
      lcd.print(temperatura, 1);
      lcd.print(" U:");
      lcd.print(umidade, 0);
      lcd.print("%");
    } else {
      lcd.print("SHT2x sem resposta");
    }
  }

  if (temAHT && sensorAHT.begin()) {
    unsigned long inicio = millis();
    while (!sensorAHT.available() && millis() - inicio < 2000) delay(50);
    lcd.setCursor(0, 2);
    lcd.print("AHT20 T:");
    lcd.print(sensorAHT.getTemperature(), 1);
    lcd.print(" U:");
    lcd.print(sensorAHT.getHumidity(), 0);
    lcd.print("%");
  }

  if (temBMP && sensorBMP.begin()) {
    sensorBMP.resetToDefaults();
    sensorBMP.setSamplingMode(BMP180MI::MODE_UHR);
    unsigned long inicio = millis();
    sensorBMP.measureTemperature();
    do { delay(50); } while (!sensorBMP.hasValue() && millis() - inicio < 2000);
    lcd.setCursor(0, 3);
    lcd.print("BMP180 T:");
    lcd.print(sensorBMP.getTemperature(), 1);
  }
}

// ==============================================================================
void setup() {
  Serial.begin(115200);
  Wire.begin(40, 41);  // SDA=IO40, SCL=IO41 (confirmado no esquematico)

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("TESTE SENSORES I2C");
  lcd.setCursor(0, 1);
  lcd.print("Iniciando...");

  if (!dispositivoPresente(ENDERECO_MUX)) {
    Serial.println("ALERTA: TCA9548A (0x70) nao respondeu no barramento - confira a fiacao do mux.");
  }

  Serial.println("=== TESTE DE SENSORES (AHT20 + BMP180 via MUX) ===");
}

// ==============================================================================
void loop() {
  // Varredura completa no Serial a cada 3s
  if (millis() - tempoUltimaVarredura > 3000) {
    tempoUltimaVarredura = millis();
    varrerCanais();

    atualizarLCD(canalMostradoLCD);
    canalMostradoLCD = (canalMostradoLCD + 1) % TOTAL_CANAIS;
  }
}
