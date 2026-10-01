/*
 * Secador de Ar - monitoramento e controle via WiFi (ESP32-S3)
 *
 *  - Le 4x SHT40 (topo/base dos 2 estagios) e 1x BMP180 (ambiente) pelo multiplexador I2C TCA9548A.
 *  - Modo AUTOMATICO: liga a resistencia de UM estagio por vez quando a umidade do topo passa do
 *    limite e desliga quando a umidade da base cai; a ventoinha acompanha. Ha corte de seguranca
 *    por temperatura, com histerese e prioridade de retomada pro estagio interrompido.
 *  - Watchdog (30s): reinicia o ESP sozinho se o loop() travar por qualquer motivo.
 *  - Modo MANUAL: liga/desliga estagios e ventoinha direto pela pagina e ajusta os limites.
 *  - Pagina web com leituras, graficos de tendencia e atualizacao de firmware por OTA (/update).
 *
 * Uso: ajuste a rede/senha em include/credenciais.h e grave por USB na primeira vez
 * (pio run -e teste_wifi_servidor -t upload). Depois, atualize pelo navegador em /update com o .bin
 * gerado por "pio run -e teste_wifi_servidor". O IP aparece no Monitor Serial (115200).
 */

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>
#include <LiquidCrystal_I2C.h>
#include <BMP180I2C.h>
#include <Preferences.h>
#include <esp_task_wdt.h>
#include "credenciais.h"

// ============================================================================
// Configuracao
// ============================================================================

// Rede e senha ficam em include/credenciais.h (fora do Git; modelo em credenciais_exemplo.h)
const char* SSID_HOTSPOT = WIFI_SSID;
const char* SENHA_HOTSPOT = WIFI_SENHA;

// Enderecos I2C
#define ENDERECO_MUX 0x70    // TCA9548A (A0/A1/A2 aterrados)
#define ENDERECO_SHT40 0x44
#define ENDERECO_BMP180 0x77

// Pinos de saida (esquematico "Secador de Ar V1.1")
#define PINO_RES_ESTAGIO1 14  // RES_1 (TRIAC)
#define PINO_RES_ESTAGIO2 13  // RES_2 (TRIAC)
#define PINO_VENTOINHA 12     // RES_3 (TRIAC, Out3) - ventoinha AC
#define PINO_VENTOINHA2 11    // RES_4 (TRIAC, Out4) - segunda ventoinha AC (so manual)
#define PINO_LED1 15          // indica Estagio 1 aquecendo
#define PINO_LED2 16          // indica Estagio 2 aquecendo
#define PINO_TEMP_PLACA 3     // LM35 embutido na placa (analogico)

#define TOTAL_ESTAGIOS 2
const int PINOS_RESISTENCIA[TOTAL_ESTAGIOS] = { PINO_RES_ESTAGIO1, PINO_RES_ESTAGIO2 };
const int PINOS_LED[TOTAL_ESTAGIOS] = { PINO_LED1, PINO_LED2 };
const char* CHAVE_CICLO_INCOMPLETO[TOTAL_ESTAGIOS] = { "cicloIncE1", "cicloIncE2" };

// Intervalos (ms)
#define INTERVALO_LEITURA_MS 2000  // le um sensor e executa o controle
#define INTERVALO_LCD_MS 500

// Limites do controle automatico. Os valores em uso ficam nas variaveis abaixo: sao ajustaveis
// pela pagina (modo manual) e salvos na NVS; os PADRAO_* valem so enquanto nada foi salvo.
#define PADRAO_LIMITE_LIGAR_UMIDADE 55       // umidade no topo que dispara o aquecimento
#define PADRAO_LIMITE_DESLIGAR_UMIDADE 8     // umidade na base que confirma que secou
#define PADRAO_TEMPERATURA_MAXIMA_MANUAL 50  // corte de seguranca no modo manual: temperatura do topo

// Corte de temperatura do modo automatico: fixo no firmware (nao editavel pela pagina), com
// histerese pra nao ficar ligando/desligando toda hora perto do limite - desliga acima de
// TEMPERATURA_DESLIGAR_AUTO, so liga de novo quando cair abaixo de TEMPERATURA_RELIGAR_AUTO.
#define TEMPERATURA_DESLIGAR_AUTO 84.0f
#define TEMPERATURA_RELIGAR_AUTO 67.0f

// ============================================================================
// Estado do sistema
// ============================================================================

WebServer server(80);
LiquidCrystal_I2C lcd(0x27, 20, 4);
BMP180I2C sensorBMP(ENDERECO_BMP180);
Preferences preferencias;

// false = automatico (controlarEstagios() decide); true = manual (a pagina decide)
bool modoManual = false;

float limiteLigarUmidade = PADRAO_LIMITE_LIGAR_UMIDADE;
float limiteDesligarUmidade = PADRAO_LIMITE_DESLIGAR_UMIDADE;
float temperaturaMaximaManual = PADRAO_TEMPERATURA_MAXIMA_MANUAL;

bool estagioAquecendo[TOTAL_ESTAGIOS] = { false, false };
unsigned long ultimaMudancaEstagioMs[TOTAL_ESTAGIOS] = { 0, 0 };  // millis() da ultima vez que ligou/desligou
bool ventoinhaLigada = false;
bool ventoinha2Ligada = false;   // so o modo manual liga; no automatico fica sempre desligada

// true quando o estagio foi desligado pelo corte de temperatura maxima antes da base terminar de
// secar (ciclo interrompido, nao concluido). Da prioridade pra retomar esse estagio assim que a
// temperatura cair, sem depender do topo estar umido de novo (o topo seca primeiro que a base e
// nunca mais voltaria a pedir sozinho). Volta a false quando o estagio termina de verdade (base
// seca). Salvo na NVS: se o ESP reiniciar no meio de um ciclo incompleto (queda de energia, etc.),
// a prioridade nao se perde - so a resistencia em si nao volta ligada sozinha (estagioAquecendo
// comeca sempre desligado no boot, de proposito, por seguranca).
bool cicloIncompleto[TOTAL_ESTAGIOS] = { false, false };

unsigned long ultimaLeituraMs = 0;
unsigned long ultimoLcdMs = 0;
unsigned long ultimoHistoricoMs = 0;

// Se ficar sem WiFi por mais que isso, corta as resistencias por seguranca (ninguem estaria
// supervisionando pela pagina). No dia que trocar pra Modbus, e so trocar a condicao "sem WiFi"
// por "mestre Modbus sem perguntar nada ha um tempo" - a logica de corte continua a mesma.
#define WIFI_TIMEOUT_SEGURANCA_MS 60000
unsigned long ultimoWifiOkMs = 0;

// ============================================================================
// Sensores
// ============================================================================

enum TipoSensor { TIPO_SHT40, TIPO_BMP180, TIPO_LM35 };

struct Sensor {
  const char* nome;
  uint8_t canal;            // canal do multiplexador
  TipoSensor tipo;
  float temperatura;        // C
  float umidadeOuPressao;   // umidade (%) no SHT40, pressao (Pa) no BMP180
  bool ok;                  // ultima leitura valida
};

// Posicao dos sensores dos estagios em sensores[] (usada pela logica de controle)
enum IndiceSensor { E1_TOPO, E1_BASE, E2_TOPO, E2_BASE };

// O canal 3 do multiplexador nao esta disponivel na placa.
// LM35 (GPIO3) removido: confirmado que trava o servidor. Fica desligado ate mudar de pino/investigar.
Sensor sensores[] = {
  {"Estagio 1 - Topo",      1, TIPO_SHT40,  NAN, NAN, false},
  {"Estagio 1 - Base",      0, TIPO_SHT40,  NAN, NAN, false},
  {"Estagio 2 - Topo",      4, TIPO_SHT40,  NAN, NAN, false},
  {"Estagio 2 - Base",      2, TIPO_SHT40,  NAN, NAN, false},
  {"Ambiente - BMP180",     6, TIPO_BMP180, NAN, NAN, false},
  {"Placa Eletronica - LM35", 0, TIPO_LM35, NAN, NAN, false},
};
const int TOTAL_SENSORES = sizeof(sensores) / sizeof(sensores[0]);
int indiceProximoSensor = 0;

// Liga somente o canal informado no multiplexador (o registrador e sobrescrito, os demais desligam).
bool selecionarCanalMux(uint8_t canal) {
  Wire.beginTransmission(ENDERECO_MUX);
  Wire.write(1 << canal);
  return Wire.endTransmission() == 0;
}

// Confere se algum dispositivo responde no endereco, antes de chamar a biblioteca do sensor.
bool dispositivoPresente(uint8_t endereco) {
  Wire.beginTransmission(endereco);
  return Wire.endTransmission() == 0;
}

// SHT40: comando de alta precisao (0xFD); temperatura e umidade voltam juntas em 6 bytes.
bool lerSHT40(float &temperatura, float &umidade) {
  Wire.beginTransmission(ENDERECO_SHT40);
  Wire.write(0xFD);
  if (Wire.endTransmission() != 0) return false;
  delay(10);  // tempo maximo de conversao (~8,2 ms)
  if (Wire.requestFrom((int)ENDERECO_SHT40, 6) != 6) return false;

  uint16_t rawTemp = (Wire.read() << 8) | Wire.read();
  Wire.read();  // CRC da temperatura (nao verificado)
  uint16_t rawUmid = (Wire.read() << 8) | Wire.read();
  Wire.read();  // CRC da umidade (nao verificado)

  temperatura = -45.0 + 175.0 * ((float)rawTemp / 65535.0);
  umidade = -6.0 + 125.0 * ((float)rawUmid / 65535.0);
  if (umidade < 0) umidade = 0;
  if (umidade > 100) umidade = 100;
  return true;
}

// BMP180 (pela biblioteca, modo de maxima resolucao): temperatura (C) e pressao (Pa).
void lerBMP180(float &temperatura, float &pressao) {
  sensorBMP.setSamplingMode(BMP180MI::MODE_UHR);

  unsigned long inicio = millis();
  sensorBMP.measureTemperature();
  do { delay(20); server.handleClient(); } while (!sensorBMP.hasValue() && millis() - inicio < 800);
  temperatura = sensorBMP.getTemperature();

  inicio = millis();
  sensorBMP.measurePressure();
  do { delay(20); server.handleClient(); } while (!sensorBMP.hasValue() && millis() - inicio < 800);
  pressao = sensorBMP.getPressure();
}

// LM35 da propria placa: 10 mV/C, leitura direta no ADC (sem passar pelo mux).
void lerLM35(float &temperatura) {
  float tensao = analogRead(PINO_TEMP_PLACA) * 3.3f / 4095.0f;
  temperatura = tensao * 100.0f;
}

// Le um sensor pelo seu canal do mux. Sensor ausente ou sem resposta fica com ok = false.
void lerSensor(Sensor &s) {
  if (s.tipo == TIPO_LM35) {
    lerLM35(s.temperatura);
    s.umidadeOuPressao = 0;  // LM35 nao tem segundo valor; sem isso ficava NAN e quebrava o JSON
    s.ok = true;
    return;
  }

  bool ok = false;

  if (selecionarCanalMux(s.canal)) {
    if (s.tipo == TIPO_SHT40) {
      ok = dispositivoPresente(ENDERECO_SHT40) && lerSHT40(s.temperatura, s.umidadeOuPressao);
    } else if (dispositivoPresente(ENDERECO_BMP180) && sensorBMP.begin()) {
      lerBMP180(s.temperatura, s.umidadeOuPressao);
      ok = true;
    }
  }

  s.ok = ok;
}

// Le um sensor por vez, em rodizio, para nao travar o loop lendo todos de uma vez.
void atualizarProximoSensor() {
  lerSensor(sensores[indiceProximoSensor]);
  indiceProximoSensor = (indiceProximoSensor + 1) % TOTAL_SENSORES;
}

// ============================================================================
// Historico (graficos de tendencia)
// ============================================================================

// 1 ponto a cada 2 min, 360 pontos = 12h de historico (RAM: 6 sensores * 360 * 2 valores * 4 bytes
// = ~17 KB, tranquilo pro ESP32-S3). Cobre um teste de bancada de um dia inteiro sem perder pontos.
#define HIST_TAMANHO 360
#define HIST_INTERVALO_MS 120000

float histTemperatura[TOTAL_SENSORES][HIST_TAMANHO];
float histUmidadePressao[TOTAL_SENSORES][HIST_TAMANHO];
int histQuantidade[TOTAL_SENSORES] = {0};

// Guarda um ponto no historico do sensor; quando enche, descarta o mais antigo.
void registrarPontoHistorico(int idx, float temperatura, float umidadeOuPressao) {
  int &n = histQuantidade[idx];
  if (n == HIST_TAMANHO) {
    for (int i = 1; i < HIST_TAMANHO; i++) {
      histTemperatura[idx][i - 1] = histTemperatura[idx][i];
      histUmidadePressao[idx][i - 1] = histUmidadePressao[idx][i];
    }
    n = HIST_TAMANHO - 1;
  }
  histTemperatura[idx][n] = temperatura;
  histUmidadePressao[idx][n] = umidadeOuPressao;
  n++;
}

// Registra a ultima leitura de cada sensor com leitura valida.
void registrarHistoricoSensores() {
  for (int i = 0; i < TOTAL_SENSORES; i++) {
    if (sensores[i].ok) {
      registrarPontoHistorico(i, sensores[i].temperatura, sensores[i].umidadeOuPressao);
    }
  }
}

// ============================================================================
// Controle dos estagios e da ventoinha
// ============================================================================

// Muda o estado de um estagio e marca quando mudou (pra "ha Xmin" na pagina). So mexe no
// timestamp se o valor realmente mudou - chamar de novo com o mesmo valor nao reseta o relogio.
void definirEstagio(int i, bool valor) {
  if (estagioAquecendo[i] != valor) {
    estagioAquecendo[i] = valor;
    ultimaMudancaEstagioMs[i] = millis();
  }
}

bool algumEstagioAquecendo() {
  for (int i = 0; i < TOTAL_ESTAGIOS; i++) {
    if (estagioAquecendo[i]) return true;
  }
  return false;
}

bool algumCicloIncompleto() {
  for (int i = 0; i < TOTAL_ESTAGIOS; i++) {
    if (cicloIncompleto[i]) return true;
  }
  return false;
}

// Escreve nos pinos o estado atual dos estagios (resistencia + LED) e da ventoinha.
void aplicarSaidas() {
  for (int i = 0; i < TOTAL_ESTAGIOS; i++) {
    digitalWrite(PINOS_RESISTENCIA[i], estagioAquecendo[i] ? HIGH : LOW);
    digitalWrite(PINOS_LED[i], estagioAquecendo[i] ? HIGH : LOW);
  }
  digitalWrite(PINO_VENTOINHA, ventoinhaLigada ? HIGH : LOW);
  digitalWrite(PINO_VENTOINHA2, ventoinha2Ligada ? HIGH : LOW);
}

// Corte de seguranca por temperatura do topo, roda em QUALQUER modo (inclusive manual - evita
// que uma resistencia ligada manualmente pelo operador passe do limite se ele esquecer de
// desligar durante um teste de bancada). No automatico usa o limite fixo TEMPERATURA_DESLIGAR_AUTO;
// no manual usa o limite ajustavel temperaturaMaximaManual. Retorna true se desligou algo.
bool aplicarCorteSeguranca() {
  Sensor* sensorTopo[TOTAL_ESTAGIOS] = { &sensores[E1_TOPO], &sensores[E2_TOPO] };
  float limite = modoManual ? temperaturaMaximaManual : TEMPERATURA_DESLIGAR_AUTO;
  bool desligou = false;

  for (int i = 0; i < TOTAL_ESTAGIOS; i++) {
    if (!estagioAquecendo[i]) continue;
    const Sensor &topo = *sensorTopo[i];
    if (topo.ok && topo.temperatura > limite) {
      definirEstagio(i, false);
      if (!modoManual) {  // corte no manual e teste de bancada: nao vira prioridade no automatico
        cicloIncompleto[i] = true;
        preferencias.putBool(CHAVE_CICLO_INCOMPLETO[i], true);
      }
      desligou = true;
    }
  }
  return desligou;
}

// Controle automatico, sempre UM estagio por vez: enquanto um regenera, o outro fica de guarda.
//  - Desliga: fim normal (umidade da base abaixo do limite) ou corte de seguranca por temperatura
//    (tratado em aplicarCorteSeguranca, chamada antes desta).
//  - Liga: da prioridade a um estagio com ciclo incompleto (foi cortado por temperatura antes da
//    base terminar de secar) assim que a temperatura dele cair abaixo de TEMPERATURA_RELIGAR_AUTO -
//    nao depende do topo estar umido de novo, ja que o topo seca primeiro que a base e nao voltaria
//    a pedir sozinho. So se nenhum estagio tiver ciclo incompleto pronto pra retomar, escolhe um
//    novo pelo topo mais umido (o mesmo criterio de sempre). Um estagio que acabou de secar a base
//    nao religa no mesmo ciclo.
//  - A ventoinha liga enquanto qualquer estagio estiver aquecendo.
// So considera sensores com leitura valida (ok): sensor ausente nunca liga nada.
// No modo manual, so roda o corte de seguranca acima; os dois estagios podem ficar ligados juntos.
void controlarEstagios() {
  bool segurancaDesligou = aplicarCorteSeguranca();

  if (modoManual) {
    if (segurancaDesligou) aplicarSaidas();
    return;
  }

  Sensor* sensorTopo[TOTAL_ESTAGIOS] = { &sensores[E1_TOPO], &sensores[E2_TOPO] };
  Sensor* sensorBase[TOTAL_ESTAGIOS] = { &sensores[E1_BASE], &sensores[E2_BASE] };
  bool desligadoAgora[TOTAL_ESTAGIOS] = { false, false };

  // 1) Desligamentos (base seca - temperatura ja tratada acima)
  for (int i = 0; i < TOTAL_ESTAGIOS; i++) {
    if (!estagioAquecendo[i]) continue;
    const Sensor &base = *sensorBase[i];

    bool baseSeca = base.ok && base.umidadeOuPressao < limiteDesligarUmidade;

    if (baseSeca) {
      definirEstagio(i, false);
      desligadoAgora[i] = true;
      cicloIncompleto[i] = false;  // terminou de verdade - nao e mais prioridade
      preferencias.putBool(CHAVE_CICLO_INCOMPLETO[i], false);
    }
  }

  // 2) Partida: so se nenhum estagio estiver aquecendo
  if (!algumEstagioAquecendo()) {
    int escolhido = -1;

    // 2a) Prioridade: retomar um ciclo incompleto assim que a temperatura tiver caido o suficiente.
    for (int i = 0; i < TOTAL_ESTAGIOS; i++) {
      if (!cicloIncompleto[i] || desligadoAgora[i]) continue;
      const Sensor &base = *sensorBase[i];
      if (base.ok && base.umidadeOuPressao < limiteDesligarUmidade) {
        // a base ja esta seca (ex: reboot quase no fim do ciclo): nada a retomar
        cicloIncompleto[i] = false;
        preferencias.putBool(CHAVE_CICLO_INCOMPLETO[i], false);
        continue;
      }
      const Sensor &topo = *sensorTopo[i];
      if (topo.ok && topo.temperatura <= TEMPERATURA_RELIGAR_AUTO) {
        escolhido = i;
        break;  // so um estagio aquece por vez
      }
    }

    // 2b) Sem ciclo incompleto pronto pra retomar: escolhe um novo pelo topo mais umido
    if (escolhido < 0) {
      float maiorUmidade = -1;
      for (int i = 0; i < TOTAL_ESTAGIOS; i++) {
        if (cicloIncompleto[i]) continue;  // esses so entram pela prioridade acima
        const Sensor &topo = *sensorTopo[i];
        bool quer = topo.ok && topo.umidadeOuPressao > limiteLigarUmidade &&
                    topo.temperatura <= TEMPERATURA_RELIGAR_AUTO;
        if (quer && !desligadoAgora[i] && topo.umidadeOuPressao > maiorUmidade) {
          maiorUmidade = topo.umidadeOuPressao;
          escolhido = i;
        }
      }
    }

    if (escolhido >= 0) {
      definirEstagio(escolhido, true);
      // Ciclo em andamento: se o ESP reiniciar (queda de energia) no meio, retoma com prioridade.
      // Fica marcado ate a base secar. So no automatico - o manual nunca grava nada.
      if (!cicloIncompleto[escolhido]) {
        cicloIncompleto[escolhido] = true;
        preferencias.putBool(CHAVE_CICLO_INCOMPLETO[escolhido], true);
      }
    }
  }

  // 3) Aplica nas saidas
  // A ventoinha tambem fica ligada durante a espera de um ciclo incompleto esfriar ate poder
  // retomar (mesmo com os dois estagios desligados nesse intervalo) - ajuda a esfriar mais rapido
  // e mantem o ar circulando em vez de parar tudo.
  ventoinhaLigada = algumEstagioAquecendo() || algumCicloIncompleto();
  aplicarSaidas();
}

// ============================================================================
// Pagina web principal (HTML/CSS/JS fixo; os valores chegam via /dados e /historico)
// ============================================================================

const char* PAGINA_HTML =
  "<!DOCTYPE html><html lang='pt'><head><meta charset='UTF-8'>"
  "<meta name='viewport' content='width=device-width, initial-scale=1.0'>"
  "<title>Secador - Sensores</title>"
  "<style>"
  "  :root { color-scheme: dark; }"
  "  * { box-sizing: border-box; }"
  "  body {"
  "    margin: 0; min-height: 100vh;"
  "    font-family: 'Segoe UI', Arial, sans-serif;"
  "    background: linear-gradient(160deg, #101a2b 0%, #1b2c4a 100%);"
  "    color: #eaf1ff;"
  "  }"
  "  .wrap { max-width: 1700px; margin: 0 auto; padding: 10px 16px 16px; }"
  "  h1 { margin: 0 0 8px; font-size: 19px; font-weight: 600; text-align: center; color: #9fc4ff; }"
  "  .secao-titulo { font-size: 13px; font-weight: 700; text-transform: uppercase; letter-spacing: 1.3px;"
  "    color: #7e91b8; margin: 10px 4px 6px; }"
  "  .hist-info { font-size: 12px; color: #5f7093; text-transform: none; letter-spacing: 0; font-weight: 400; }"
  ""
  /* --- Esquema fisico do secador (estatico, so pra mostrar a montagem) --- */
  "  .esquema-card { background: rgba(255,255,255,0.05); border: 1px solid rgba(255,255,255,0.1);"
  "    border-radius: 16px; padding: 14px; text-align: center; flex: 1; display: flex;"
  "    flex-direction: column; align-items: stretch; justify-content: center; min-height: 0; }"
  "  .esquema-card svg { width: 100%; max-width: 780px; height: auto; display: block; margin: 0 auto; }"
  "  @keyframes girarVentoinha { from { transform: rotate(0deg); } to { transform: rotate(360deg); } }"
  "  @keyframes fluirAr { to { stroke-dashoffset: -32; } }"
  "  .layout-duas-colunas { display: flex; flex-wrap: wrap; gap: 16px; align-items: stretch; margin-top: 4px; }"
  "  .coluna-esquema { flex: 0 1 400px; max-width: 420px; min-width: 240px; display: flex; flex-direction: column; }"
  "  .coluna-sensores { flex: 1 1 600px; min-width: 300px; min-height: 520px; }"
  ""
  /* --- Legenda de cores e cartoes de estagio (saturacao + aquecendo), abaixo do esquema --- */
  "  .legenda-cor { display: flex; gap: 18px; flex-wrap: wrap; justify-content: center;"
  "    font-size: 15px; font-weight: 600; color: #c4d3ef; margin-top: 6px; }"
  "  .legenda-cor span { display: inline-flex; align-items: center; gap: 7px; }"
  "  .legenda-cor i { width: 13px; height: 13px; border-radius: 4px; display: inline-block; }"
  "  .estagios-linha { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; margin-top: 64px; }"
  "  .estagio-card { border: 1px solid rgba(255,255,255,0.12); border-radius: 14px; padding: 16px 18px;"
  "    background: rgba(255,255,255,0.04); }"
  "  .estagio-card .cabecalho { display: flex; flex-wrap: wrap; align-items: center; justify-content: space-between; gap: 8px; margin-bottom: 12px; }"
  "  .estagio-card .nome { font-size: 15px; font-weight: 700; }"
  "  .estado-chip { font-size: 12px; font-weight: 700; text-transform: uppercase; letter-spacing: .4px;"
  "    padding: 4px 11px; border-radius: 999px; background: rgba(255,255,255,0.06); white-space: nowrap;"
  "    border: 1px solid rgba(255,255,255,0.12); color: #8ea2c7; }"
  "  .estado-chip.aquecendo { color: #ff9a62; border-color: rgba(255,154,98,0.45); }"
  "  .ciclo-pendente-chip { font-size: 11px; font-weight: 700; text-transform: uppercase; letter-spacing: .3px;"
  "    padding: 3px 10px; border-radius: 999px; white-space: nowrap; color: #ff9a62;"
  "    border: 1px solid rgba(255,154,98,0.45); background: rgba(255,154,98,0.12); }"
  "  .barra-saturacao { height: 9px; border-radius: 999px; background: rgba(255,255,255,0.06);"
  "    border: 1px solid rgba(255,255,255,0.12); overflow: hidden; }"
  "  .barra-saturacao > i { display: block; height: 100%; border-radius: 999px;"
  "    transition: width 1.1s ease, background 1.1s ease; }"
  "  .estagio-card .valores { display: flex; justify-content: space-between; margin-top: 10px; font-size: 13.5px; color: #8ea2c7; }"
  "  .estagio-card .valores b { color: #eaf1ff; font-weight: 700; font-size: 15px; }"
  ""
  /* --- Blocos por sensor: um cartao unico com cabecalho + valor + grafico de cada medida --- */
  "  .sensores-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(250px, 1fr)); gap: 18px; }"
  "  .sensor-bloco {"
  "    background: rgba(255,255,255,0.06); border: 1px solid rgba(255,255,255,0.12);"
  "    border-left: 4px solid var(--cor, #62c4ff); border-radius: 14px; padding: 16px 18px;"
  "  }"
  "  .sensor-bloco-titulo {"
  "    font-size: 15px; font-weight: 700; text-transform: uppercase; letter-spacing: 1px;"
  "    color: #eaf1ff; margin-bottom: 12px; display: flex; align-items: center; gap: 7px;"
  "  }"
  "  .sensor-bloco-titulo .pt { width: 8px; height: 8px; border-radius: 50%; }"
  "  .metric-secao + .metric-secao { border-top: 1px solid rgba(255,255,255,0.08); margin-top: 12px; padding-top: 12px; }"
  ""
  /* --- Chips de ventoinha --- */
  "  .chips-vent { display: flex; flex-wrap: wrap; gap: 10px; }"
  "  .chip-vent {"
  "    display: inline-flex; align-items: center; gap: 8px; padding: 10px 16px; border-radius: 999px;"
  "    font-size: 15px; font-weight: 700; border: 1px solid rgba(255,255,255,0.1);"
  "  }"
  "  .chip-vent .pt { width: 8px; height: 8px; border-radius: 50%; }"
  "  .chip-vent.on { background: rgba(74,222,128,0.12); color: #4ade80; }"
  "  .chip-vent.on .pt { background: #4ade80; box-shadow: 0 0 6px #4ade80; }"
  "  .chip-vent.off { background: rgba(95,112,147,0.12); color: #8496b8; }"
  "  .chip-vent.off .pt { background: #5f7093; }"
  "  button.chip-vent {"
  "    font-family: inherit; cursor: pointer; transition: transform .1s ease;"
  "  }"
  "  button.chip-vent:hover { transform: translateY(-1px); }"
  "  button.chip-vent:active { transform: translateY(0); }"
  "  button.chip-vent:disabled { opacity: .4; cursor: not-allowed; }"
  ""
  /* --- Formulario de limites de controle --- */
  "  .limites-form { display: flex; flex-wrap: wrap; align-items: flex-end; gap: 16px; }"
  "  .limites-form label { display: flex; flex-direction: column; gap: 6px; font-size: 13px; font-weight: 700; color: #8ea2c7; }"
  "  .limites-form .campo { display: flex; align-items: center; gap: 8px; }"
  "  .limites-form input {"
  "    width: 90px; padding: 10px 12px; border-radius: 10px; font-size: 16px; font-weight: 700;"
  "    background: rgba(255,255,255,0.06); border: 1px solid rgba(255,255,255,0.15); color: #eaf1ff;"
  "  }"
  "  .limites-form input:disabled { opacity: .5; cursor: not-allowed; }"
  ""
  /* --- Faixa de controles compacta no topo (modo, estagios e ventoinhas lado a lado; limites embaixo) --- */
  "  .controles-topo { background: rgba(255,255,255,0.03); border-radius: 10px; padding: 6px 12px 6px;"
  "    margin-bottom: 4px; position: relative; }"
  "  .controles-divisor { position: absolute; top: 0; bottom: 0; width: 1px; background: rgba(255,255,255,0.08); }"
  "  .controles-linha { display: flex; flex-wrap: wrap; }"
  "  .controle-col { flex: 1; min-width: 160px; padding: 0 16px; }"
  "  .controle-col:first-child { padding-left: 0; }"
  "  .modo-linha { display: flex; flex-wrap: wrap; align-items: center; gap: 10px; }"
  "  .controles-topo .secao-titulo { font-size: 13px; margin: 4px 4px 5px; }"
  "  .controles-topo .chips-vent { gap: 7px; }"
  "  .controles-topo .chip-vent { padding: 6px 13px; font-size: 14px; gap: 6px; }"
  "  .controles-topo .limites-form { gap: 10px; }"
  "  .controles-topo .limites-form label { font-size: 13px; gap: 4px; }"
  "  .controles-topo .limites-form input { width: 68px; padding: 6px 9px; font-size: 15px; }"
  ""
  /* --- Secao de tendencia dentro do sensor-bloco (numero + grafico + min/med/max) --- */
  "  .metric-secao .cabecalho { display: flex; justify-content: space-between; align-items: center; gap: 8px; }"
  "  .metric-secao .titulo { font-size: 14px; color: #c4d3ef; font-weight: 700; display: flex; align-items: center; gap: 6px; }"
  "  .metric-secao .periodo { font-size: 11px; font-weight: 600; color: #5f7093; white-space: nowrap; }"
  "  .metric-secao .valor-grande { font-size: 25px; font-weight: 700; color: var(--cor2, #62c4ff); margin: 5px 0 7px; }"
  "  .metric-secao .valor-grande.off { font-size: 15px; font-weight: 600; font-style: italic; color: #5f7093; }"
  "  .metric-secao canvas { width: 100%; height: 64px; display: block; }"
  "  .metric-secao .stats { display: flex; gap: 12px; margin-top: 9px; font-size: 12px; font-weight: 600; color: #8ea2c7; }"
  "  .metric-secao .stats span { display: flex; align-items: center; gap: 3px; }"
  "  .metric-secao .stats .dot { width: 6px; height: 6px; border-radius: 50%; }"
  ""
  "  .infobar {"
  "    margin-top: 24px; display: flex; flex-wrap: wrap; justify-content: center; gap: 24px;"
  "    font-size: 16px; font-weight: 600; color: #7e91b8; background: rgba(255,255,255,0.04); border-radius: 12px;"
  "    padding: 14px 20px; border: 1px solid rgba(255,255,255,0.08);"
  "  }"
  "  .infobar span { display: flex; align-items: center; gap: 8px; }"
  "  .infobar b { color: #c4d3ef; font-weight: 600; }"
  "  .ota { text-align: center; margin-top: 14px; }"
  "  .ota a {"
  "    color: #eaf1ff; font-size: 16px; font-weight: 600; text-decoration: none;"
  "    background: linear-gradient(135deg, #3b82f6, #7c3aed); padding: 11px 22px; border-radius: 10px;"
  "    display: inline-flex; align-items: center; gap: 8px; box-shadow: 0 8px 20px rgba(59,130,246,0.35);"
  "  }"
  "</style></head><body>"
  "<div class='wrap'>"
  "  <h1>&#127808; Monitoramento do Secador</h1>"
  "  <div class='controles-topo'>"
  "  <div class='controles-divisor' style='left:33.333%'></div>"
  "  <div class='controles-divisor' style='left:66.666%'></div>"
  "  <div class='controles-linha'>"
  "    <div class='controle-col'>"
  "      <div class='secao-titulo'>&#9881;&#65039; Modo de Operacao</div>"
  "      <div class='modo-linha'>"
  "        <div class='chips-vent' id='modo'></div>"
  "        <button class='chip-vent on' id='btnSalvarLimites' onclick='salvarLimites()'>&#128190; Salvar</button>"
  "      </div>"
  "    </div>"
  "    <div class='controle-col'>"
  "      <div class='secao-titulo'>&#128293; Estagios de Aquecimento</div>"
  "      <div class='chips-vent' id='estagios'></div>"
  "    </div>"
  "    <div class='controle-col'>"
  "      <div class='secao-titulo'>&#128168; Ventoinhas</div>"
  "      <div class='chips-vent' id='ventoinhas'></div>"
  "    </div>"
  "  </div>"
  "  <div class='secao-titulo'>&#127919; Limites de Controle <span class='hist-info' id='limitesInfo'></span></div>"
  "  <div class='limites-form'>"
  "    <label>Umidade p/ LIGAR (topo)"
  "      <span class='campo'><input type='number' id='limLigar' min='0' max='100' step='1'> %</span></label>"
  "    <label>Umidade p/ DESLIGAR (base)"
  "      <span class='campo'><input type='number' id='limDesligar' min='0' max='100' step='1'> %</span></label>"
  "    <label>Temperatura maxima (so no modo manual - automatico usa 85/75 fixo)"
  "      <span class='campo'><input type='number' id='limTemp' min='0' max='125' step='1'> C</span></label>"
  "  </div>"
  "  </div>"
  "  <div class='layout-duas-colunas'>"
  "  <div class='coluna-esquema'>"
  "  <div class='secao-titulo'>&#128208; Esquema da Montagem</div>"
  "  <div class='esquema-card'>"
  "    <svg viewBox='0 0 410 420' xmlns='http://www.w3.org/2000/svg'>"
  "      <defs>"
  "        <clipPath id='clipZonaE2'><rect x='45' y='16' width='150' height='179' rx='12'/></clipPath>"
  "        <clipPath id='clipZonaE1'><rect x='45' y='205' width='150' height='201' rx='12'/></clipPath>"
  "      </defs>"
  "      <ellipse cx='120' cy='16' rx='75' ry='14' fill='#1b2c4a' stroke='#2b3a52' stroke-width='2'/>"
  "      <rect x='45' y='16' width='150' height='179' rx='12' fill='#1b2c4a' stroke='#2b3a52' stroke-width='2'/>"
  "      <g clip-path='url(#clipZonaE2)'><rect id='saturacaoE2' x='45' y='195' width='150' height='0' fill='#4fa8e8' opacity='0.55'/></g>"
  "      <rect x='45' y='16' width='150' height='179' rx='12' fill='none' stroke='#2b3a52' stroke-width='2'/>"
  "      <line x1='45' y1='195' x2='195' y2='195' stroke='#eaf1ff' stroke-width='2' opacity='0.9'/>"
  "      <line id='fluxoSuperior' x1='120' y1='38' x2='120' y2='158' stroke='#8ea2c7' stroke-width='2.5' stroke-dasharray='5 8' stroke-linecap='round' opacity='0'/>"
  "      <line id='fluxoInferior' x1='120' y1='228' x2='120' y2='378' stroke='#8ea2c7' stroke-width='2.5' stroke-dasharray='5 8' stroke-linecap='round' opacity='0'/>"
  "      <g id='grupoVentoinha' style='transform-origin: 120px 195px;'>"
  "      <circle cx='120' cy='195' r='22' fill='#1b2c4a' stroke='#eaf1ff' stroke-width='2.5'/>"
  "      <circle cx='120' cy='195' r='5' fill='#eaf1ff'/>"
  "      <line x1='120' y1='177' x2='120' y2='186' stroke='#eaf1ff' stroke-width='3'/>"
  "      <line x1='120' y1='204' x2='120' y2='213' stroke='#eaf1ff' stroke-width='3'/>"
  "      <line x1='102' y1='195' x2='111' y2='195' stroke='#eaf1ff' stroke-width='3'/>"
  "      <line x1='129' y1='195' x2='138' y2='195' stroke='#eaf1ff' stroke-width='3'/>"
  "      </g>"
  "      <rect id='brasaE2' x='64' y='163' width='112' height='8' rx='4' fill='#8ea2c7'/>"
  "      <rect x='45' y='205' width='150' height='201' rx='12' fill='#1b2c4a' stroke='#2b3a52' stroke-width='2'/>"
  "      <g clip-path='url(#clipZonaE1)'><rect id='saturacaoE1' x='45' y='406' width='150' height='0' fill='#4fa8e8' opacity='0.55'/></g>"
  "      <rect x='45' y='205' width='150' height='201' rx='12' fill='none' stroke='#2b3a52' stroke-width='2'/>"
  "      <rect id='brasaE1' x='64' y='382' width='112' height='8' rx='4' fill='#8ea2c7'/>"
  "      <ellipse cx='120' cy='406' rx='75' ry='14' fill='#1b2c4a' stroke='#2b3a52' stroke-width='2'/>"
  "      <text x='120' y='101' text-anchor='middle' font-size='15' font-weight='700' fill='#eaf1ff'>Estagio 2</text>"
  "      <text x='120' y='301' text-anchor='middle' font-size='15' font-weight='700' fill='#eaf1ff'>Estagio 1</text>"
  "      <circle cx='120' cy='4' r='5' fill='#a78bfa'/><line x1='120' y1='4' x2='200' y2='8' stroke='#8ea2c7' stroke-width='1.5'/>"
  "      <text x='204' y='12' font-size='17' font-weight='400' fill='#eaf1ff'>Ambiente - BMP180</text>"
  "      <circle cx='120' cy='61' r='5' fill='#4ade80'/><line x1='120' y1='61' x2='200' y2='65' stroke='#8ea2c7' stroke-width='1.5'/>"
  "      <text x='204' y='69' font-size='17' font-weight='400' fill='#eaf1ff'>Estagio 2 - Topo</text>"
  "      <circle cx='120' cy='150' r='5' fill='#4ade80'/><line x1='120' y1='150' x2='200' y2='154' stroke='#8ea2c7' stroke-width='1.5'/>"
  "      <text x='204' y='158' font-size='17' font-weight='400' fill='#eaf1ff'>Estagio 2 - Base</text>"
  "      <circle cx='138' cy='195' r='5' fill='#fbbf24'/><line x1='138' y1='195' x2='200' y2='199' stroke='#8ea2c7' stroke-width='1.5'/>"
  "      <text x='204' y='203' font-size='17' font-weight='400' fill='#eaf1ff'>Ventoinha</text>"
  "      <circle cx='120' cy='255' r='5' fill='#4ade80'/><line x1='120' y1='255' x2='200' y2='259' stroke='#8ea2c7' stroke-width='1.5'/>"
  "      <text x='204' y='263' font-size='17' font-weight='400' fill='#eaf1ff'>Estagio 1 - Topo</text>"
  "      <circle cx='120' cy='356' r='5' fill='#4ade80'/><line x1='120' y1='356' x2='200' y2='360' stroke='#8ea2c7' stroke-width='1.5'/>"
  "      <text x='204' y='364' font-size='17' font-weight='400' fill='#eaf1ff'>Estagio 1 - Base</text>"
  "    </svg>"
  "    <div class='legenda-cor'>"
  "      <span><i style='background:#4fa8e8'></i>Silica seca</span>"
  "      <span><i style='background:#e8709f'></i>Silica saturada</span>"
  "      <span><i style='background:#ff9a62'></i>Aquecendo</span>"
  "    </div>"
  "    <div class='estagios-linha'>"
  "      <div class='estagio-card'>"
  "        <div class='cabecalho'><span class='nome'>Estagio 1</span><span class='estado-chip' id='chipE1'>--</span>"
  "          <span class='ciclo-pendente-chip' id='pendenteE1' hidden>ciclo pendente</span></div>"
  "        <div class='barra-saturacao'><i id='barraE1' style='width:0%;background:#4fa8e8'></i></div>"
  "        <div class='valores'><span>Saturacao</span><b id='valorSatE1'>--%</b></div>"
  "      </div>"
  "      <div class='estagio-card'>"
  "        <div class='cabecalho'><span class='nome'>Estagio 2</span><span class='estado-chip' id='chipE2'>--</span>"
  "          <span class='ciclo-pendente-chip' id='pendenteE2' hidden>ciclo pendente</span></div>"
  "        <div class='barra-saturacao'><i id='barraE2' style='width:0%;background:#4fa8e8'></i></div>"
  "        <div class='valores'><span>Saturacao</span><b id='valorSatE2'>--%</b></div>"
  "      </div>"
  "    </div>"
  "  </div>"
  "  </div>"
  "  <div class='coluna-sensores'>"
  "  <div class='secao-titulo'>&#127777; Sensores <span class='hist-info' id='histInfo'></span></div>"
  "  <div class='sensores-grid' id='sensoresGrid'></div>"
  "  </div>"
  "  </div>"
  "  <div class='infobar'>"
  "    <span>&#127760; IP: <b id='ip'>--</b></span>"
  "    <span>&#128246; Sinal: <b id='rssi'>--</b></span>"
  "    <span>&#9200; Ligado ha: <b id='uptime'>--</b></span>"
  "  </div>"
  "  <div class='ota'>"
  "    <a href='/update'>&#11014;&#65039; Atualizar firmware (OTA)</a>"
  "    &nbsp;&middot;&nbsp;"
  "    <a href='/historico.csv'>&#128190; Exportar historico (CSV)</a>"
  "  </div>"
  "</div>"
  "<script>"
  // Sem isso, uma unica requisicao emperrada trava o ciclo de atualizacao pra sempre (fetch nao tem timeout por padrao).
  "async function fetchComTimeout(url, ms) {"
  "  const controle = new AbortController();"
  "  const id = setTimeout(() => controle.abort(), ms);"
  "  try {"
  "    return await fetch(url, { signal: controle.signal });"
  "  } finally {"
  "    clearTimeout(id);"
  "  }"
  "}"
  "const CORES = { sht40: '#ff9a62', bmp: '#a78bfa' };"
  "function corTipo(tipo) { return tipo === 'BMP180' ? CORES.bmp : CORES.sht40; }"
  ""
  "function chipStatusHTML(nome, ativo, rotuloOn, rotuloOff, onclick) {"
  "  const tag = onclick ? 'button' : 'div';"
  "  const clique = onclick ? `onclick='${onclick}'` : '';"
  "  return `<${tag} class='chip-vent ${ativo ? 'on' : 'off'}' ${clique}>"
  "    <span class='pt'></span>${nome}: ${ativo ? rotuloOn : rotuloOff}</${tag}>`;"
  "}"
  "function chipModoHTML(manual) {"
  "  return `<button class='chip-vent ${manual ? 'on' : 'off'}' onclick='toggleModo(${manual})'>"
  "    <span class='pt'></span>Modo: ${manual ? 'MANUAL' : 'AUTOMATICO'} (clique pra trocar)</button>`;"
  "}"
  "async function toggleModo(manualAtual) {"
  "  try {"
  "    editandoLimites = false;"
  "    await fetchComTimeout('/modo?manual=' + (manualAtual ? 0 : 1), 4000);"
  "    atualizar();"
  "  } catch (e) {}"
  "}"
  "let editandoLimites = false;"
  "['limLigar', 'limDesligar', 'limTemp'].forEach(id =>"
  "  document.getElementById(id).addEventListener('input', () => { editandoLimites = true; }));"
  "function atualizarLimites(d) {"
  "  const ids = ['limLigar', 'limDesligar', 'limTemp'];"
  "  ids.forEach(id => { document.getElementById(id).disabled = !d.modoManual; });"
  "  document.getElementById('btnSalvarLimites').disabled = !d.modoManual;"
  "  document.getElementById('limitesInfo').textContent ="
  "    d.modoManual ? '(editavel - modo manual)' : '(troque pra modo manual pra editar)';"
  "  if (!editandoLimites) {"
  "    document.getElementById('limLigar').value = d.limites.ligar;"
  "    document.getElementById('limDesligar').value = d.limites.desligar;"
  "    document.getElementById('limTemp').value = d.limites.temp;"
  "  }"
  "}"
  "async function salvarLimites() {"
  "  const ligar = document.getElementById('limLigar').value;"
  "  const desligar = document.getElementById('limDesligar').value;"
  "  const temp = document.getElementById('limTemp').value;"
  "  try {"
  "    const r = await fetchComTimeout(`/limites?ligar=${ligar}&desligar=${desligar}&temp=${temp}`, 4000);"
  "    if (!r.ok) { alert(await r.text()); return; }"
  "    editandoLimites = false;"
  "    atualizar();"
  "  } catch (e) { alert('sem conexao com o ESP32'); }"
  "}"
  "async function toggleSaidaManual(id, estadoAtual) {"
  "  try {"
  "    await fetchComTimeout('/saida-manual?id=' + id + '&estado=' + (estadoAtual ? 0 : 1), 4000);"
  "    atualizar();"
  "  } catch (e) {}"
  "}"
  ""
  "let ultimoDados = null, ultimoHistorico = null;"
  ""
  "function sensoresCombinados(d) {"
  "  return d.extras.map(s => ({ nome: s.nome, tipo: s.tipo, ok: s.ok, valor1: s.valor1, valor2: s.valor2 }));"
  "}"
  ""
  "function desenharSparkline(canvas, dados, cor) {"
  "  const w = canvas.width = canvas.clientWidth;"
  "  const h = canvas.height = 64;"
  "  const ctx = canvas.getContext('2d');"
  "  ctx.clearRect(0, 0, w, h);"
  "  if (dados.length < 2) {"
  "    ctx.fillStyle = '#5f7093'; ctx.font = '11px Arial'; ctx.textAlign = 'center';"
  "    ctx.fillText('aguardando mais pontos...', w / 2, h / 2);"
  "    return;"
  "  }"
  "  const min = Math.min(...dados), max = Math.max(...dados);"
  "  const margem = (max - min) * 0.15 || 1;"
  "  const minY = min - margem, maxY = max + margem;"
  "  const passoX = w / (dados.length - 1);"
  "  ctx.beginPath();"
  "  dados.forEach((v, i) => {"
  "    const x = i * passoX;"
  "    const y = h - ((v - minY) / (maxY - minY)) * h;"
  "    if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);"
  "  });"
  "  ctx.strokeStyle = cor; ctx.lineWidth = 2.5; ctx.lineJoin = 'round'; ctx.stroke();"
  "  ctx.lineTo(w, h); ctx.lineTo(0, h); ctx.closePath();"
  "  const grad = ctx.createLinearGradient(0, 0, 0, h);"
  "  grad.addColorStop(0, cor + '99'); grad.addColorStop(1, cor + '05');"
  "  ctx.fillStyle = grad; ctx.fill();"
  "}"
  "function metricCardHTML(id, titulo, icone, unidade, cor, corMin, corMax, valorTxt, ok, serie) {"
  "  let statsHtml = '';"
  "  if (serie && serie.length) {"
  "    const min = Math.min(...serie), max = Math.max(...serie);"
  "    const med = serie.reduce((a, b) => a + b, 0) / serie.length;"
  "    statsHtml = `<div class='stats'>"
  "      <span><span class='dot' style='background:${corMin}'></span>Min ${min.toFixed(1)}${unidade}</span>"
  "      <span><span class='dot' style='background:#4ade80'></span>Med ${med.toFixed(1)}${unidade}</span>"
  "      <span><span class='dot' style='background:${corMax}'></span>Max ${max.toFixed(1)}${unidade}</span>"
  "    </div>`;"
  "  }"
  "  return `<div class='metric-secao' style='--cor2:${cor}'>"
  "    <div class='cabecalho'><span class='titulo'>${icone} ${titulo}</span>"
  "      <span class='periodo'>ultimos ${(serie || []).length} pts</span></div>"
  "    <div class='valor-grande ${ok ? '' : 'off'}' id='${id}_valor'>${ok ? valorTxt : 'sem sensor'}</div>"
  "    <canvas id='${id}'></canvas>"
  "    ${statsHtml}"
  "  </div>`;"
  "}"
  ""
  "function sensorBlocoHTML(s, idx, serie) {"
  "  const cor = corTipo(s.tipo);"
  "  const cor2 = s.tipo === 'BMP180' ? '#a78bfa' : '#62c4ff';"
  "  const rotulo2 = s.tipo === 'BMP180' ? 'Pressao' : 'Umidade';"
  "  const unidade2 = s.tipo === 'BMP180' ? ' Pa' : ' %';"
  "  const min2 = s.tipo === 'BMP180' ? 90000 : 0, max2 = s.tipo === 'BMP180' ? 110000 : 100;"
  "  const somenteTemp = s.tipo === 'LM35';"
  "  const idT = 'graf_' + idx + '_t', idU = 'graf_' + idx + '_u';"
  ""
  "  const cardTemp = metricCardHTML(idT, 'Temperatura', '&#127777;', ' C', cor,"
  "    '#22d3ee', '#f87171', s.valor1.toFixed(1) + ' C', s.ok, serie.valor1);"
  "  const cardUmid = somenteTemp ? '' : metricCardHTML(idU, rotulo2, s.tipo === 'BMP180' ? '&#127788;' : '&#128167;', unidade2, cor2,"
  "    '#e2e8f0', '#a78bfa', s.valor2.toFixed(s.tipo === 'BMP180' ? 0 : 1) + unidade2, s.ok, serie.valor2);"
  ""
  "  return `<div class='sensor-bloco' style='--cor:${cor}'>"
  "    <div class='sensor-bloco-titulo'><span class='pt' style='background:${cor}'></span>${s.nome}</div>"
  "    ${cardTemp}${cardUmid}"
  "  </div>`;"
  "}"
  ""
  "function renderSensores() {"
  "  if (!ultimoDados) return;"
  "  const sensores = sensoresCombinados(ultimoDados);"
  "  const series = (ultimoHistorico && ultimoHistorico.series) || [];"
  "  const html = sensores.map((s, idx) => sensorBlocoHTML(s, idx, series[idx] || { valor1: [], valor2: [] })).join('');"
  "  document.getElementById('sensoresGrid').innerHTML = html;"
  "  const manual = ultimoDados.modoManual;"
  "  document.getElementById('modo').innerHTML = chipModoHTML(manual);"
  "  atualizarLimites(ultimoDados);"
  "  document.getElementById('ventoinhas').innerHTML ="
  "    ultimoDados.ventoinhas.map((v, i) => chipStatusHTML(v.nome, v.ligada, 'LIGADA', 'desligada',"
  "      (manual || i === 1) ? `toggleSaidaManual(${i === 0 ? 2 : 3}, ${v.ligada})` : null)).join('');"
  "  document.getElementById('estagios').innerHTML ="
  "    ultimoDados.estagios.map((e, i) => chipStatusHTML(e.nome, e.aquecendo, 'AQUECENDO', 'ocioso',"
  "      manual ? `toggleSaidaManual(${i}, ${e.aquecendo})` : null)).join('');"
  "  sensores.forEach((s, idx) => {"
  "    const serie = series[idx] || { valor1: [], valor2: [] };"
  "    const cor = corTipo(s.tipo);"
  "    const cor2 = s.tipo === 'BMP180' ? '#a78bfa' : '#62c4ff';"
  "    const ct = document.getElementById('graf_' + idx + '_t');"
  "    const cu = document.getElementById('graf_' + idx + '_u');"
  "    if (ct) desenharSparkline(ct, serie.valor1 || [], cor);"
  "    if (cu) desenharSparkline(cu, serie.valor2 || [], cor2);"
  "    const unidade2 = s.tipo === 'BMP180' ? ' Pa' : ' %';"
  "    if (s.ok) animarValor('graf_' + idx + '_t_valor', s.valor1, ' C', 1);"
  "    if (s.ok && s.tipo !== 'LM35') animarValor('graf_' + idx + '_u_valor', s.valor2, unidade2, s.tipo === 'BMP180' ? 0 : 1);"
  "  });"
  "  const ventoinhaLigada = ultimoDados.ventoinhas[0] && ultimoDados.ventoinhas[0].ligada;"
  "  document.getElementById('grupoVentoinha').style.animation = ventoinhaLigada ? 'girarVentoinha 1.1s linear infinite' : 'none';"
  "  ['fluxoSuperior', 'fluxoInferior'].forEach(id => {"
  "    const el = document.getElementById(id);"
  "    el.style.opacity = ventoinhaLigada ? '0.7' : '0';"
  "    el.style.animation = ventoinhaLigada ? 'fluirAr 0.8s linear infinite' : 'none';"
  "  });"
  "  const pendentes = ultimoDados.cicloIncompleto || [false, false];"
  "  document.getElementById('pendenteE1').hidden = !pendentes[0];"
  "  document.getElementById('pendenteE2').hidden = !pendentes[1];"
  "  atualizarEstagio('E2', sensores.find(s => s.nome === 'Estagio 2 - Base'),"
  "    ultimoDados.estagios[1] && ultimoDados.estagios[1].aquecendo, ultimoDados.estagios[1] && ultimoDados.estagios[1].desdeSeg,"
  "    'saturacaoE2', 195, 179, 'brasaE2');"
  "  atualizarEstagio('E1', sensores.find(s => s.nome === 'Estagio 1 - Base'),"
  "    ultimoDados.estagios[0] && ultimoDados.estagios[0].aquecendo, ultimoDados.estagios[0] && ultimoDados.estagios[0].desdeSeg,"
  "    'saturacaoE1', 406, 201, 'brasaE1');"
  "}"
  ""
  "const ultimosValores = {};"
  "function animarValor(elId, valorNovo, sufixo, casas) {"
  "  const el = document.getElementById(elId);"
  "  if (!el) return;"
  "  const valorAntigo = elId in ultimosValores ? ultimosValores[elId] : valorNovo;"
  "  ultimosValores[elId] = valorNovo;"
  "  if (valorAntigo === valorNovo) { el.textContent = valorNovo.toFixed(casas) + sufixo; return; }"
  "  const inicio = performance.now(), duracao = 500;"
  "  function passo(agora) {"
  "    const t = Math.min(1, (agora - inicio) / duracao);"
  "    el.textContent = (valorAntigo + (valorNovo - valorAntigo) * t).toFixed(casas) + sufixo;"
  "    if (t < 1) requestAnimationFrame(passo);"
  "  }"
  "  requestAnimationFrame(passo);"
  "}"
  ""
  "function corSaturacao(pct) {"
  "  const seco = [79, 168, 232], saturado = [232, 112, 159];"
  "  const t = Math.max(0, Math.min(1, pct / 100));"
  "  const c = seco.map((v, i) => Math.round(v + (saturado[i] - v) * t));"
  "  return `rgb(${c[0]},${c[1]},${c[2]})`;"
  "}"
  ""
  "function formatarDuracao(segundos) {"
  "  if (segundos < 60) return segundos + 's';"
  "  if (segundos < 3600) return Math.floor(segundos / 60) + 'min';"
  "  return (segundos / 3600).toFixed(1) + 'h';"
  "}"
  "function atualizarEstagio(prefixo, sensorBase, aquecendo, desdeSeg, idFill, baseY, alturaMax, idBrasa) {"
  "  const chip = document.getElementById('chip' + prefixo);"
  "  const barra = document.getElementById('barra' + prefixo);"
  "  const fillEl = document.getElementById(idFill);"
  "  const brasaEl = document.getElementById(idBrasa);"
  "  brasaEl.setAttribute('fill', aquecendo ? '#ff9a62' : '#8ea2c7');"
  "  if (!sensorBase || !sensorBase.ok) {"
  "    chip.textContent = 'sem leitura'; chip.className = 'estado-chip';"
  "    animarValor('valorSat' + prefixo, 0, '%', 0);"
  "    barra.style.width = '0%'; fillEl.setAttribute('height', 0);"
  "    return;"
  "  }"
  "  const pct = Math.max(0, Math.min(100, sensorBase.valor2));"
  "  const cor = corSaturacao(pct);"
  "  const desde = desdeSeg != null ? ' (ha ' + formatarDuracao(desdeSeg) + ')' : '';"
  "  chip.textContent = (aquecendo ? 'aquecendo' : 'ocioso') + desde;"
  "  chip.className = 'estado-chip' + (aquecendo ? ' aquecendo' : '');"
  "  barra.style.width = pct.toFixed(0) + '%';"
  "  barra.style.background = cor;"
  "  animarValor('valorSat' + prefixo, pct, '%', 0);"
  "  const alturaFill = (pct / 100) * alturaMax;"
  "  fillEl.setAttribute('y', baseY - alturaFill);"
  "  fillEl.setAttribute('height', alturaFill);"
  "  fillEl.setAttribute('fill', cor);"
  "}"
  ""
  "async function atualizar() {"
  "  try {"
  "    const r = await fetchComTimeout('/dados', 4000);"
  "    const d = await r.json();"
  "    ultimoDados = d;"
  "    document.getElementById('ip').textContent = d.ip;"
  "    document.getElementById('rssi').textContent = d.rssi + ' dBm';"
  "    document.getElementById('uptime').textContent = d.uptime + ' s';"
  "    renderSensores();"
  "  } catch (e) {}"
  "  setTimeout(atualizar, 2000);"
  "}"
  "async function atualizarHistorico() {"
  "  try {"
  "    const r = await fetchComTimeout('/historico', 8000);"
  "    const h = await r.json();"
  "    ultimoHistorico = h;"
  "    const pontos = (h.series[0] && h.series[0].valor1.length) || 0;"
  "    document.getElementById('histInfo').textContent = pontos + ' pontos, a cada ' + h.intervaloSeg + 's';"
  "    renderSensores();"
  "  } catch (e) {}"
  "  setTimeout(atualizarHistorico, 15000);"
  "}"
  "atualizar();"
  "setTimeout(atualizarHistorico, 1000);"
  "window.addEventListener('resize', renderSensores);"
  "</script>"
  "</body></html>";

// ============================================================================
// Rotas da pagina
// ============================================================================

void handleRaiz() {
  server.send(200, "text/html", PAGINA_HTML);
}

const char* boolJson(bool valor) {
  return valor ? "true" : "false";
}

const char* nomeTipoSensor(const Sensor &s) {
  if (s.tipo == TIPO_BMP180) return "BMP180";
  if (s.tipo == TIPO_LM35) return "LM35";
  return "SHT40";
}

// GET /dados - estado atual em JSON (as chaves sao o contrato com o JavaScript da pagina).
// Sensor sem leitura valida vai com 0, pois NaN nao e JSON valido.
void handleDados() {
  char json[1500];
  int pos = 0;

  pos += snprintf(json + pos, sizeof(json) - pos,
    "{\"modoManual\":%s,\"limites\":{\"ligar\":%.1f,\"desligar\":%.1f,\"temp\":%.1f},\"extras\":[",
    boolJson(modoManual), limiteLigarUmidade, limiteDesligarUmidade, temperaturaMaximaManual);
  for (int i = 0; i < TOTAL_SENSORES; i++) {
    const Sensor &s = sensores[i];
    pos += snprintf(json + pos, sizeof(json) - pos,
      "%s{\"nome\":\"%s\",\"tipo\":\"%s\",\"ok\":%s,\"valor1\":%.2f,\"valor2\":%.2f}",
      (i > 0 ? "," : ""), s.nome, nomeTipoSensor(s), boolJson(s.ok),
      s.ok ? s.temperatura : 0.0f,
      s.ok ? s.umidadeOuPressao : 0.0f);
  }
  pos += snprintf(json + pos, sizeof(json) - pos, "],");

  pos += snprintf(json + pos, sizeof(json) - pos,
    "\"ventoinhas\":[{\"nome\":\"Ventoinha 1\",\"ligada\":%s},{\"nome\":\"Ventoinha 2\",\"ligada\":%s}],",
    boolJson(ventoinhaLigada), boolJson(ventoinha2Ligada));

  pos += snprintf(json + pos, sizeof(json) - pos,
    "\"estagios\":[{\"nome\":\"Estagio 1\",\"aquecendo\":%s,\"desdeSeg\":%lu},"
    "{\"nome\":\"Estagio 2\",\"aquecendo\":%s,\"desdeSeg\":%lu}],",
    boolJson(estagioAquecendo[0]), (millis() - ultimaMudancaEstagioMs[0]) / 1000,
    boolJson(estagioAquecendo[1]), (millis() - ultimaMudancaEstagioMs[1]) / 1000);

  pos += snprintf(json + pos, sizeof(json) - pos,
    "\"cicloIncompleto\":[%s,%s],",
    boolJson(cicloIncompleto[0]), boolJson(cicloIncompleto[1]));

  pos += snprintf(json + pos, sizeof(json) - pos,
    "\"ip\":\"%s\",\"rssi\":%d,\"uptime\":%lu}",
    WiFi.localIP().toString().c_str(), WiFi.RSSI(), millis() / 1000);

  server.send(200, "application/json", json);
}

// GET /historico - series de temperatura e umidade/pressao de cada sensor, para os graficos.
void handleHistorico() {
  static char json[9000];
  int pos = 0;

  pos += snprintf(json + pos, sizeof(json) - pos,
    "{\"intervaloSeg\":%d,\"series\":[", HIST_INTERVALO_MS / 1000);

  for (int idx = 0; idx < TOTAL_SENSORES; idx++) {
    pos += snprintf(json + pos, sizeof(json) - pos,
      "%s{\"nome\":\"%s\",\"tipo\":\"%s\",\"valor1\":[",
      (idx > 0 ? "," : ""), sensores[idx].nome, nomeTipoSensor(sensores[idx]));
    for (int i = 0; i < histQuantidade[idx]; i++) {
      pos += snprintf(json + pos, sizeof(json) - pos, "%s%.2f", (i > 0 ? "," : ""), histTemperatura[idx][i]);
    }
    pos += snprintf(json + pos, sizeof(json) - pos, "],\"valor2\":[");
    for (int i = 0; i < histQuantidade[idx]; i++) {
      pos += snprintf(json + pos, sizeof(json) - pos, "%s%.2f", (i > 0 ? "," : ""), histUmidadePressao[idx][i]);
    }
    pos += snprintf(json + pos, sizeof(json) - pos, "]}");
  }

  pos += snprintf(json + pos, sizeof(json) - pos, "]}");

  server.send(200, "application/json", json);
}

// GET /historico.csv - o mesmo historico do /historico (RAM, ultimos HIST_TAMANHO pontos de cada
// sensor), em CSV pra abrir no Excel/planilha. So cobre a janela em memoria (30 min no padrao
// atual: HIST_TAMANHO * HIST_INTERVALO_MS) - nao ha cartao SD nem RTC neste firmware de teste,
// entao "segundos_atras" e relativo a agora, nao um horario real.
void handleExportarCSV() {
  server.sendHeader("Content-Disposition", "attachment; filename=historico.csv");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/csv", "");
  server.sendContent("sensor;tipo;segundos_atras;temperatura_C;umidade_pct_ou_pressao_Pa\n");

  char linha[96];
  for (int idx = 0; idx < TOTAL_SENSORES; idx++) {
    int n = histQuantidade[idx];
    for (int i = 0; i < n; i++) {
      unsigned long segAtras = (unsigned long)(n - 1 - i) * (HIST_INTERVALO_MS / 1000);
      snprintf(linha, sizeof(linha), "%s;%s;%lu;%.2f;%.2f\n",
        sensores[idx].nome, nomeTipoSensor(sensores[idx]), segAtras,
        histTemperatura[idx][i], histUmidadePressao[idx][i]);
      server.sendContent(linha);
    }
  }
}

// GET /saida-manual?id=<0|1|2|3>&estado=<0|1> - liga/desliga Estagio 1 (0), Estagio 2 (1),
// Ventoinha 1 (2) ou Ventoinha 2 (3). So funciona no modo manual.
void handleSaidaManual() {
  if (!server.hasArg("id") || !server.hasArg("estado")) {
    server.send(400, "text/plain", "faltando id ou estado");
    return;
  }

  int id = server.arg("id").toInt();
  // Ventoinha 2 (teste) tambem responde no automatico; o resto so no manual
  if (!modoManual && id != 3) {
    server.send(409, "text/plain", "sistema em modo automatico");
    return;
  }
  bool ligar = server.arg("estado").toInt() != 0;

  switch (id) {
    case 0:
    case 1:
      definirEstagio(id, ligar);
      break;
    case 2:
      ventoinhaLigada = ligar;
      break;
    case 3:
      ventoinha2Ligada = ligar;
      break;
    default:
      server.send(400, "text/plain", "id invalido");
      return;
  }

  aplicarSaidas();
  server.send(200, "text/plain", "ok");
}

// GET /modo?manual=<0|1> - troca entre automatico (0) e manual (1).
void handleModo() {
  if (!server.hasArg("manual")) {
    server.send(400, "text/plain", "faltando manual");
    return;
  }
  bool novoManual = server.arg("manual").toInt() != 0;

  // Ao voltar para o automatico, parte de um estado limpo (tudo desligado): assim nunca sobram os
  // dois estagios ligados do manual, e nenhum cronometro antigo.
  if (modoManual && !novoManual) {
    for (int i = 0; i < TOTAL_ESTAGIOS; i++) definirEstagio(i, false);
    ventoinhaLigada = false;
    aplicarSaidas();  // a ventoinha 2 (teste) nao e mexida pela troca de modo
  }

  // Entrar no manual descarta ciclos pendentes: quem mexe na bancada assume o controle, e o
  // automatico nao deve retomar sozinho um ciclo antigo ao voltar.
  if (!modoManual && novoManual) {
    for (int i = 0; i < TOTAL_ESTAGIOS; i++) {
      cicloIncompleto[i] = false;
      preferencias.putBool(CHAVE_CICLO_INCOMPLETO[i], false);
    }
  }

  modoManual = novoManual;
  server.send(200, "text/plain", "ok");
}

// GET /limites?ligar=&desligar=&temp= - altera e salva os limites do controle automatico.
// So no modo manual, para nao mexer em limite de seguranca com o automatico rodando.
void handleLimites() {
  if (!modoManual) {
    server.send(409, "text/plain", "so e possivel alterar os limites no modo manual");
    return;
  }
  if (!server.hasArg("ligar") || !server.hasArg("desligar") || !server.hasArg("temp")) {
    server.send(400, "text/plain", "faltando ligar, desligar ou temp");
    return;
  }

  float ligar = server.arg("ligar").toFloat();
  float desligar = server.arg("desligar").toFloat();
  float temp = server.arg("temp").toFloat();

  if (ligar < 0 || ligar > 100 || desligar < 0 || desligar > 100) {
    server.send(400, "text/plain", "umidade deve estar entre 0 e 100");
    return;
  }
  if (desligar >= ligar) {
    server.send(400, "text/plain", "umidade de desligar precisa ser menor que a de ligar");
    return;
  }
  if (temp < 0 || temp > 125) {
    server.send(400, "text/plain", "temperatura deve estar entre 0 e 125 C");
    return;
  }

  limiteLigarUmidade = ligar;
  limiteDesligarUmidade = desligar;
  temperaturaMaximaManual = temp;
  preferencias.putFloat("limLigar", ligar);
  preferencias.putFloat("limDesl", desligar);
  preferencias.putFloat("tempMax", temp);
  server.send(200, "text/plain", "ok");
}

// ============================================================================
// Atualizacao de firmware por OTA
// ============================================================================

// Pagina de upload do .bin (OTA)
const char* PAGINA_UPDATE =
  "<!DOCTYPE html><html lang='pt'><head><meta charset='UTF-8'>"
  "<meta name='viewport' content='width=device-width, initial-scale=1.0'>"
  "<title>Atualizar firmware</title>"
  "<style>"
  "  body { font-family: Arial, sans-serif; background: #101a2b; color: #eaf1ff;"
  "    display: flex; align-items: center; justify-content: center; min-height: 100vh; margin: 0; }"
  "  .card { background: rgba(255,255,255,0.06); border: 1px solid rgba(255,255,255,0.12);"
  "    border-radius: 20px; padding: 48px 40px; width: 420px; text-align: center; }"
  "  h2 { font-size: 26px; margin: 0 0 12px; }"
  "  input[type='file'] { width: 100%; margin: 20px 0; color: #eaf1ff; font-size: 16px; }"
  "  input[type='submit'] { background: #3b82f6; color: #fff; border: none; border-radius: 10px;"
  "    padding: 16px; width: 100%; font-size: 20px; font-weight: 600; cursor: pointer; }"
  "  a { color: #62c4ff; font-size: 16px; }"
  "</style></head><body>"
  "<div class='card'>"
  "  <h2>Atualizar firmware</h2>"
  "  <p style='font-size:16px; color:#8ea2c7;'>Escolha o .bin gerado pelo PlatformIO</p>"
  "  <form method='POST' action='/update' enctype='multipart/form-data'>"
  "    <input type='file' name='firmware' accept='.bin'>"
  "    <input type='submit' value='Enviar e atualizar'>"
  "  </form>"
  "  <p style='margin-top:20px;'><a href='/'>&larr; voltar</a></p>"
  "</div></body></html>";

void handleUpdatePagina() {
  server.send(200, "text/html", PAGINA_UPDATE);
}

// Recebe o .bin em pedacos e grava na particao OTA
void handleUpdateUpload() {
  HTTPUpload& upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    Serial.printf("OTA: iniciando gravacao de %s\n", upload.filename.c_str());
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
      Update.printError(Serial);
    }
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
      Update.printError(Serial);
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (Update.end(true)) {
      Serial.printf("OTA: gravacao concluida (%u bytes) - reiniciando\n", upload.totalSize);
    } else {
      Update.printError(Serial);
    }
  }
}

// Resposta final do upload: mostra o resultado e, se deu certo, reinicia o ESP e volta para a
// pagina principal depois de 5 s.
void handleUpdateResultado() {
  bool ok = !Update.hasError();

  String pagina =
    "<!DOCTYPE html><html lang='pt'><head><meta charset='UTF-8'>"
    "<meta name='viewport' content='width=device-width, initial-scale=1.0'>"
    "<title>Atualizacao " + String(ok ? "concluida" : "falhou") + "</title>"
    "<style>"
    "  body { font-family: Arial, sans-serif; background: #101a2b; color: #eaf1ff;"
    "    display: flex; align-items: center; justify-content: center; min-height: 100vh; margin: 0; }"
    "  .card { background: rgba(255,255,255,0.06); border: 1px solid rgba(255,255,255,0.12);"
    "    border-radius: 20px; padding: 48px 40px; width: 420px; text-align: center; }"
    "  .icone { font-size: 64px; margin-bottom: 16px; }"
    "  h2 { margin: 0 0 12px; font-size: 26px; }"
    "  p { color: #8ea2c7; font-size: 16px; }"
    "  a { color: #62c4ff; font-size: 16px; }"
    "</style></head><body>"
    "<div class='card'>";

  if (ok) {
    pagina +=
      "  <div class='icone'>&#9989;</div>"
      "  <h2>Atualizacao concluida!</h2>"
      "  <p>Reiniciando...</p>";
  } else {
    pagina +=
      "  <div class='icone'>&#10060;</div>"
      "  <h2>Falha na atualizacao</h2>"
      "  <p>O firmware nao foi gravado. Tente novamente.</p>"
      "  <p style='margin-top:20px;'><a href='/update'>&larr; voltar</a></p>";
  }

  pagina += "</div>";

  if (ok) {
    pagina +=
      "<script>"
      "setTimeout(() => { window.location = '/'; }, 5000);"
      "</script>";
  }

  pagina += "</body></html>";

  server.send(200, "text/html", pagina);
  delay(500);
  if (ok) ESP.restart();
}

// ============================================================================
// LCD 20x4
// ============================================================================

// Escreve uma linha do LCD completando com espacos, para apagar sobra do texto anterior.
void lcdLinha(uint8_t linha, const String &texto) {
  String s = texto;
  while (s.length() < 20) s += ' ';
  if (s.length() > 20) s = s.substring(0, 20);
  lcd.setCursor(0, linha);
  lcd.print(s);
}

void lcdLinhaSensor(uint8_t linha, const Sensor &s) {
  if (!s.ok) {
    lcdLinha(linha, String(s.nome) + ": sem sensor");
  } else if (s.tipo != TIPO_SHT40) {
    lcdLinha(linha, String(s.nome) + " " + String(s.temperatura, 1) + "C");
  } else {
    lcdLinha(linha, String(s.nome) + " " + String(s.temperatura, 1) + "C " +
                    String(s.umidadeOuPressao, 0) + "%");
  }
}

// Linha 0: modo. Linhas 1-2: dois sensores em rodizio. Linha 3: estagios e ventoinha.
// IP e sinal WiFi ficam so na pagina web.
void atualizarLCD() {
  if (millis() - ultimoLcdMs < INTERVALO_LCD_MS) return;
  ultimoLcdMs = millis();

  lcdLinha(0, "Secador - " + String(modoManual ? "MANUAL" : "AUTOMATICO"));

  int ultimoLido = (indiceProximoSensor + TOTAL_SENSORES - 1) % TOTAL_SENSORES;
  lcdLinhaSensor(1, sensores[ultimoLido]);
  lcdLinhaSensor(2, sensores[(ultimoLido + 1) % TOTAL_SENSORES]);

  lcdLinha(3, "E1:" + String(estagioAquecendo[0] ? "ON " : "OFF") +
              " E2:" + String(estagioAquecendo[1] ? "ON " : "OFF") +
              " V:" + String(ventoinhaLigada ? "1" : "0") + String(ventoinha2Ligada ? "1" : "0"));
}

// ============================================================================
// Watchdog
// ============================================================================
// Reinicia o ESP sozinho se o loop() travar (WiFi pendurado, I2C travado, sensor sem responder,
// etc.) por mais de WDT_TIMEOUT_MS sem chamar esp_task_wdt_reset() - importante aqui porque um
// travamento com uma resistencia ligada nao teria mais nenhuma checagem de seguranca rodando.
#define WDT_TIMEOUT_MS 30000

#if ESP_ARDUINO_VERSION_MAJOR >= 3
esp_task_wdt_config_t configuracaoWatchdog = {
  .timeout_ms = WDT_TIMEOUT_MS,
  .idle_core_mask = 0,
  .trigger_panic = true
};
#endif

void inicializarWatchdog() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  esp_task_wdt_deinit();
  if (esp_task_wdt_init(&configuracaoWatchdog) != ESP_OK) {
    // Fallback: alguma build especifica do core 3.x pode nao aceitar o struct - tenta a
    // assinatura antiga (timeout em segundos, bool panico).
    esp_task_wdt_init(WDT_TIMEOUT_MS / 1000, true);
  }
#else
  esp_task_wdt_init(WDT_TIMEOUT_MS / 1000, true);
#endif
  esp_task_wdt_add(NULL);
}

// ============================================================================
// Setup e loop
// ============================================================================

void setup() {
  ultimaMudancaEstagioMs[0] = millis();
  ultimaMudancaEstagioMs[1] = millis();

  Serial.begin(115200);
  delay(500);
  Wire.begin(40, 41);  // SDA=IO40, SCL=IO41

  inicializarWatchdog();
  ultimoWifiOkMs = millis();  // da a folga inicial de WIFI_TIMEOUT_SEGURANCA_MS pra conectar

  // Limites salvos na NVS (ou os padroes, se nada foi salvo)
  preferencias.begin("secador", false);
  limiteLigarUmidade = preferencias.getFloat("limLigar", PADRAO_LIMITE_LIGAR_UMIDADE);
  limiteDesligarUmidade = preferencias.getFloat("limDesl", PADRAO_LIMITE_DESLIGAR_UMIDADE);
  temperaturaMaximaManual = preferencias.getFloat("tempMax", PADRAO_TEMPERATURA_MAXIMA_MANUAL);

  // Ciclo incompleto tambem sobrevive a reboot: se o ESP caiu no meio de um corte por
  // temperatura, a prioridade de retomada nao se perde (mas a resistencia em si comeca sempre
  // desligada abaixo, por seguranca - so a prioridade e restaurada).
  for (int i = 0; i < TOTAL_ESTAGIOS; i++) {
    cicloIncompleto[i] = preferencias.getBool(CHAVE_CICLO_INCOMPLETO[i], false);
  }

  // Tudo desligado ao iniciar
  for (int i = 0; i < TOTAL_ESTAGIOS; i++) {
    pinMode(PINOS_RESISTENCIA[i], OUTPUT);
    pinMode(PINOS_LED[i], OUTPUT);
  }
  pinMode(PINO_VENTOINHA, OUTPUT);
  pinMode(PINO_VENTOINHA2, OUTPUT);
  aplicarSaidas();

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcdLinha(0, "Secador Regen - Teste");
  lcdLinha(1, "Conectando WiFi...");

  Serial.println("=== TESTE WIFI / SERVIDOR (4x SHT40 + BMP180) ===");
  Serial.print("Conectando no hotspot: ");
  Serial.println(SSID_HOTSPOT);

  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID_HOTSPOT, SENHA_HOTSPOT);

  unsigned long inicioTentativa = millis();
  const unsigned long TIMEOUT_WIFI_MS = 15000;

  while (WiFi.status() != WL_CONNECTED && (millis() - inicioTentativa) < TIMEOUT_WIFI_MS) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConectado!");
    Serial.print("IP do ESP32: ");
    Serial.println(WiFi.localIP());
    Serial.println("Abra esse IP no navegador do PC pra confirmar.");

    server.on("/", HTTP_GET, handleRaiz);
    server.on("/dados", HTTP_GET, handleDados);
    server.on("/historico", HTTP_GET, handleHistorico);
    server.on("/historico.csv", HTTP_GET, handleExportarCSV);
    server.on("/saida-manual", HTTP_GET, handleSaidaManual);
    server.on("/modo", HTTP_GET, handleModo);
    server.on("/limites", HTTP_GET, handleLimites);
    server.on("/update", HTTP_GET, handleUpdatePagina);
    server.on("/update", HTTP_POST, handleUpdateResultado, handleUpdateUpload);
    server.begin();
    Serial.println("Servidor web iniciado na porta 80");
  } else {
    Serial.println("\nFalha ao conectar - confira SSID/senha e se o hotspot esta ligado");
    lcdLinha(1, "Falha ao conectar");
    lcdLinha(2, "Confira SSID/senha");
  }
}

void loop() {
  // Sempre, independente do WiFi - so cuida de travamento de codigo de verdade (I2C travado,
  // sensor sem responder, etc.).
  esp_task_wdt_reset();

  if (WiFi.status() == WL_CONNECTED) {
    ultimoWifiOkMs = millis();
  } else {
    // Sem supervisao externa ha tempo demais: ninguem consegue ver nem desligar uma resistencia
    // pela pagina, entao corta tudo por seguranca (sem precisar reiniciar o ESP - so tira a
    // saida). Volta a funcionar normal assim que o WiFi reconectar sozinho.
    if (millis() - ultimoWifiOkMs > WIFI_TIMEOUT_SEGURANCA_MS) {
      for (int i = 0; i < TOTAL_ESTAGIOS; i++) definirEstagio(i, false);
      ventoinhaLigada = false;
      ventoinha2Ligada = false;
      aplicarSaidas();
    }
    return;
  }

  server.handleClient();

  if (millis() - ultimaLeituraMs > INTERVALO_LEITURA_MS) {
    ultimaLeituraMs = millis();
    atualizarProximoSensor();
    controlarEstagios();
  }

  if (millis() - ultimoHistoricoMs > HIST_INTERVALO_MS) {
    ultimoHistoricoMs = millis();
    registrarHistoricoSensores();
  }

  atualizarLCD();
}
