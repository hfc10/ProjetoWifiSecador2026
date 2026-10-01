/*
========================================================================================================
TESTE 5 - MODBUS RTU (4x SHT40 + BMP180 + ventoinha)
Objetivo: mapear aos poucos os dados de verdade no Modbus. Layout final: 4 SHT40 (Estagio 1 e 2,
Topo e Base) + 1 BMP180 (Ambiente) + 1 ventoinha. A "Caixa" NAO tem sensor proprio - usa o sensor
onboard da placa (LM35), entao nao entra nesta lista.

Conforme o esquematico "Secador de Ar V1.1": o MAX485 (RS-485) esta ligado nos pinos GPIO38 (RX2)
e GPIO37 (TX2) do ESP32-S3 - uma UART SEPARADA da UART0 (USB), sem conflito entre os dois.
A direcao do MAX485 e automatica por hardware - o firmware nao precisa controlar pino DE/RE.

Como nenhum sensor esta fisicamente instalado ainda (exceto o que voce for plugando aos poucos pra
testar), cada um aparece como "nao conectado" ate responder no canal certo do mux - e por isso cada
sensor tem, alem do valor, um bit de status (Discrete Input) avisando se a leitura e valida ou nao.
Sem isso, um registrador em 0 seria ambiguo (0 graus de verdade, ou sensor ausente?).

Mapa de registradores:
  Holding Register 0        -> valor fixo 1234 (registrador de saude/teste, ja validado)

  Input Registers (Function 0x04) - valores x10 (ex: 234 = 23.4):
    0-1   SHT40 E1-Topo   (temperatura, umidade)
    2-3   SHT40 E1-Base   (temperatura, umidade)
    4-5   SHT40 E2-Topo   (temperatura, umidade)
    6-7   SHT40 E2-Base   (temperatura, umidade)
    8-9   BMP180 Ambiente (temperatura, pressao em hPa)
    10-11 SHT25 (canal 0, TEMPORARIO - so pra ter algo real conectado enquanto testa; no layout
          final esse canal sera do SHT40 E1-Topo)
    12-13 segundos desde que o Estagio 1 / 2 ligou ou desligou (sem x10; satura em 65535 = ~18 h;
          zera no boot)

  Discrete Inputs (Function 0x02) - sensor respondeu na ultima leitura:
    0  SHT40 E1-Topo ok
    1  SHT40 E1-Base ok
    2  SHT40 E2-Topo ok
    3  SHT40 E2-Base ok
    4  BMP180 Ambiente ok
    5  SHT25 (canal 0, TEMPORARIO) ok
    6  ALARME: falha de funcionamento (exigencia da norma ABNT PN 03:014.01-100/1, item 6.3)
    7  ALARME: saturacao/fim de vida da silica-gel (idem, item 6.3)

  Holding Registers:
    0  valor fixo 1234 (registrador de saude/teste, ja validado)
    1  ciclos de regeneracao da silica-gel ja realizados (persistente, sobrevive a reboot)

  Discrete Inputs extras:
    8  Estagio 1 com ciclo incompleto (cortado por temperatura, aguardando retomar com prioridade)
    9  Estagio 2 com ciclo incompleto

  Holding Registers extras (leitura e escrita, x10, salvos na NVS; valor invalido e recusado):
    2  limite de umidade do topo pra LIGAR (%)         padrao 55.0
    3  limite de umidade da base pra DESLIGAR (%)      padrao 8.0  (precisa ser menor que o de ligar)
    4  temperatura maxima do modo manual (C)           padrao 50.0

  Coils (Function 0x01, leitura e escrita) - estado real das saidas:
    0  Ventoinha 1 ligada
    1  Resistencia Estagio 1 ligada
    2  Resistencia Estagio 2 ligada
    3  Ventoinha 2 ligada (TRIAC 4, so manual)
    4  Modo MANUAL (1 = manual, 0 = automatico; sempre inicia em automatico)
  No automatico o firmware decide e sobrescreve os coils 0-3 (escrita do mestre e ignorada); no
  manual os coils 0-3 comandam as saidas. O corte por temperatura vale nos dois modos.

Logica de controle (portada do teste_wifi_servidor.cpp): um estagio aquece por vez; liga quando a
umidade do topo passa do limite, desliga quando a base seca; corta a 85 C e so religa <= 70 C; ciclo
cortado por temperatura tem prioridade de retomada (salvo na NVS); a ventoinha 1 fica ligada com
qualquer estagio aquecendo ou ciclo incompleto; tudo inicia desligado apos reboot; watchdog de 30 s.
Cada ciclo concluido (base seca) incrementa o contador de ciclos da silica (Holding Register 1).

Sobre os dois alarmes (ainda sem logica de aquecimento pronta, entao ficam assim por enquanto):
  - Falha de funcionamento: ativa se NENHUM dos sensores respondeu na ultima leitura de cada um
    (equipamento sem leitura nenhuma = indicio de falha geral, nao so de 1 sensor desconectado).
  - Saturacao da silica: ativa quando o contador de ciclos de regeneracao atinge o limite do
    fabricante (CICLOS_MAXIMOS_SILICA). O contador em si so vai incrementar de verdade quando a
    logica de controle da resistencia/regeneracao existir - por enquanto fica parado em 0,
    persistente em NVS (Preferences), pronto pra receber dados.

PENDENTE (ainda nao portado do projeto antigo):
  - Trava por fluxo do transformador (existia no Secador_Novo2026_completo_260915.cpp: chave de fluxo
    com debounce -> sistemaTravado forca as 2 resistencias e a ventoinha desligadas enquanto o
    transformador "respira"). Ela precisa ter PRECEDENCIA sobre a regra "ventoinha fica ligada
    enquanto houver cicloIncompleto". Decidir se um estagio desligado pela trava ganha a prioridade
    de retomada (cicloIncompleto) como no corte por temperatura. Se a chave virar sensor de pressao
    diferencial, travar so quando o transformador PUXA ar (o sentido de expelir nao precisa travar).
  - O corte por "falta de supervisao" do WiFi (60s sem WiFi desliga tudo) NAO deve virar "mestre
    Modbus sumiu = parar": o controle mora no ESP e deve continuar secando sem o painel. Manter so
    o corte por temperatura e o watchdog; falta do mestre e apenas aviso (assim esta implementado).
  - Canais do multiplexador ja iguais aos do teste_wifi_servidor.cpp (E1-Topo=1, E1-Base=0,
    E2-Topo=4, E2-Base=2, BMP180=6). O SHT25 temporario (canal 0) ficou sem sensor: aparece "nao ok".

Esse arquivo NAO compila junto com os outros testes - use o environment "teste_modbus" do
platformio.ini pra compilar/gravar so este.
========================================================================================================
*/

#include <Arduino.h>
#include <Wire.h>
#include <ModbusRTU.h>
#include <BMP180I2C.h>
#include <Preferences.h>
#include <esp_task_wdt.h>

#define MODBUS_BAUD 9600
#define ID_ESCRAVO 1
#define REG_TESTE 0  // Holding Register 0 - valor fixo 1234, registrador de saude/teste

#define PINO_RX2 38  // RX2 no esquematico - recebe da RS-485 (saida do MAX485, via buffers 74HC14)
#define PINO_TX2 37  // TX2 no esquematico - transmite pra RS-485 (entrada do MAX485, via buffers)

// LED1 (mesmo pino usado como ledResistencia1 no Secador_Novo2026.cpp) - pisca toda vez que chega
// byte na porta do Modbus, pra confirmar visualmente que esta recebendo algo mesmo sem o Monitor Serial.
#define PINO_LED1 15
#define DURACAO_PISCA_MS 150

// --- Enderecos I2C (mesmo esquema dos outros testes) ---
#define ENDERECO_MUX 0x70
#define ENDERECO_SHT40 0x44
#define ENDERECO_BMP180 0x77
#define ENDERECO_SHT2X 0x40  // SHT25 - TEMPORARIO no canal 0 (ainda fisicamente ligado ali; no layout
                              // final esse canal vai ser do SHT40 E1-Topo)
#define INTERVALO_LEITURA_MS 2000

// LED2 (mesmo pino usado como ledResistencia2 no Secador_Novo2026.cpp) - alarme geral: acende se
// falha de funcionamento OU saturacao da silica estiverem ativas.
#define PINO_LED2 16

// --- Alarmes exigidos pela norma (item 6.3 do PN 03:014.01-100/1) ---
#define REG_ALARME_FALHA 6      // Discrete Input 6
#define REG_ALARME_SATURACAO 7  // Discrete Input 7
#define REG_CICLOS_SILICA 1     // Holding Register 1

// Valor de exemplo - trocar pelo numero real de ciclos suportado, conforme ficha tecnica
// do fornecedor da silica-gel, quando disponivel.
#define CICLOS_MAXIMOS_SILICA 5000

HardwareSerial SerialModbus(2);
ModbusRTU mb;
BMP180I2C sensorBMP(ENDERECO_BMP180);
Preferences preferencias;

unsigned long tempoAcendeuLed = 0;
unsigned long tempoUltimaLeitura = 0;

// Pinos de saida (esquematico "Secador de Ar V1.1"), mesmos do teste_wifi_servidor.cpp
#define PINO_RES_ESTAGIO1 14  // RES_1 (TRIAC)
#define PINO_RES_ESTAGIO2 13  // RES_2 (TRIAC)
#define PINO_VENTOINHA 12     // RES_3 (TRIAC, Out3) - ventoinha AC
#define PINO_VENTOINHA2 11    // RES_4 (TRIAC, Out4) - segunda ventoinha AC (so manual)

#define TOTAL_ESTAGIOS 2
const int PINOS_RESISTENCIA[TOTAL_ESTAGIOS] = { PINO_RES_ESTAGIO1, PINO_RES_ESTAGIO2 };
const char* CHAVE_CICLO_INCOMPLETO[TOTAL_ESTAGIOS] = { "cicloIncE1", "cicloIncE2" };

// Coils
#define COIL_VENTOINHA1 0
#define COIL_RESISTENCIA_E1 1
#define COIL_RESISTENCIA_E2 2
#define COIL_VENTOINHA2 3
#define COIL_MODO_MANUAL 4

// Holding Registers de configuracao (x10) e Discrete Inputs de ciclo incompleto
#define REG_LIMITE_LIGAR 2
#define REG_LIMITE_DESLIGAR 3
#define REG_TEMP_MAXIMA_MANUAL 4
#define REG_CICLO_INCOMPLETO_E1 8  // Discrete Input 8 (E1) e 9 (E2)
#define REG_DESDE_MUDANCA_E1 12    // Input Register 12 (E1) e 13 (E2): segundos desde que ligou/desligou

// Limites do controle (mesmos padroes do teste_wifi_servidor.cpp; editaveis por Modbus e salvos na NVS)
#define PADRAO_LIMITE_LIGAR_UMIDADE 55.0f
#define PADRAO_LIMITE_DESLIGAR_UMIDADE 8.0f
#define PADRAO_TEMPERATURA_MAXIMA_MANUAL 50.0f
// Corte automatico fixo, com histerese: desliga acima de DESLIGAR, so religa em RELIGAR ou menos
#define TEMPERATURA_DESLIGAR_AUTO 85.0f
#define TEMPERATURA_RELIGAR_AUTO 70.0f

bool modoManual = false;  // sempre inicia em automatico
float limiteLigarUmidade = PADRAO_LIMITE_LIGAR_UMIDADE;
float limiteDesligarUmidade = PADRAO_LIMITE_DESLIGAR_UMIDADE;
float temperaturaMaximaManual = PADRAO_TEMPERATURA_MAXIMA_MANUAL;

bool estagioAquecendo[TOTAL_ESTAGIOS] = { false, false };
unsigned long ultimaMudancaEstagioMs[TOTAL_ESTAGIOS] = { 0, 0 };  // millis() da ultima vez que ligou/desligou
bool ventoinha1Ligada = false;
bool ventoinha2Ligada = false;  // so o modo manual liga; no automatico fica sempre desligada

// Estagio cortado por temperatura antes da base secar: tem prioridade pra retomar (salvo na NVS,
// sobrevive a reboot; a resistencia em si nunca volta ligada sozinha no boot).
bool cicloIncompleto[TOTAL_ESTAGIOS] = { false, false };

#define WDT_TIMEOUT_MS 30000

// Guarda localmente o ultimo status de cada sensor (mb.Ists nao tem "getter" facil de usar aqui),
// pra calcular o alarme de falha geral sem precisar reler tudo do zero.
bool sensorOkAtual[6] = {false, false, false, false, false, false};
// Ultimo valor lido de cada sensor (temperatura em C, umidade em % / pressao em hPa), pro controle
float leituraTemperatura[6] = {0, 0, 0, 0, 0, 0};
float leituraUmidade[6] = {0, 0, 0, 0, 0, 0};

// Posicao dos sensores dos estagios em sensores[] (usada pela logica de controle)
enum IndiceSensor { E1_TOPO, E1_BASE, E2_TOPO, E2_BASE };

// Ciclos de regeneracao ja realizados - carregado da NVS (sobrevive a reboot/queda de energia).
// So incrementa de verdade quando a logica de controle da resistencia existir (funcao pronta,
// so falta ser chamada no lugar certo quando um ciclo de aquecimento terminar).
uint32_t ciclosRegeneracao = 0;

void incrementarCicloRegeneracao() {
  ciclosRegeneracao++;
  preferencias.putUInt("ciclos", ciclosRegeneracao);
  mb.Hreg(REG_CICLOS_SILICA, (uint16_t)min<uint32_t>(ciclosRegeneracao, 65535));
}

bool calcularSaturacaoSilica() {
  return ciclosRegeneracao >= CICLOS_MAXIMOS_SILICA;
}

enum TipoSensor { TIPO_SHT40, TIPO_BMP180, TIPO_SHT2X };

struct Sensor {
  const char* nome;
  uint8_t canal;
  TipoSensor tipo;
  uint8_t regValor1;  // indice do Input Register (temperatura)
  uint8_t regValor2;  // indice do Input Register (umidade ou pressao)
  uint8_t regStatus;  // indice do Discrete Input (sensor ok)
};

Sensor sensores[] = {
  // Canais iguais aos do teste_wifi_servidor.cpp (fiacao atual)
  {"SHT40 E1-Topo", 1, TIPO_SHT40,  0, 1, 0},
  {"SHT40 E1-Base", 0, TIPO_SHT40,  2, 3, 1},
  {"SHT40 E2-Topo", 4, TIPO_SHT40,  4, 5, 2},
  {"SHT40 E2-Base", 2, TIPO_SHT40,  6, 7, 3},  // canal 3 do mux nao esta disponivel na placa
  {"BMP180 Ambiente", 6, TIPO_BMP180, 8, 9, 4},
  {"SHT25 (canal 0)", 0, TIPO_SHT2X, 10, 11, 5},
};
const int TOTAL_SENSORES = sizeof(sensores) / sizeof(sensores[0]);
int indiceProximoSensor = 0;

// Falha de funcionamento: ativa se NENHUM sensor respondeu na ultima leitura de cada um -
// um unico sensor desconectado ja e sinalizado no proprio Discrete Input dele, isso aqui e
// so pro cenario mais grave (equipamento inteiro sem leitura nenhuma).
bool calcularFalhaGeral() {
  for (int i = 0; i < TOTAL_SENSORES; i++) {
    if (sensorOkAtual[i]) return false;
  }
  return true;
}

bool selecionarCanalMux(uint8_t canal) {
  Wire.beginTransmission(ENDERECO_MUX);
  Wire.write(1 << canal);
  return Wire.endTransmission() == 0;
}

bool dispositivoPresente(uint8_t endereco) {
  Wire.beginTransmission(endereco);
  return Wire.endTransmission() == 0;
}

// Leitura do SHT40 via comando de alta precisao (0xFD) - temperatura e umidade numa unica transacao.
bool lerSHT40(float &temperatura, float &umidade) {
  Wire.beginTransmission(ENDERECO_SHT40);
  Wire.write(0xFD);
  if (Wire.endTransmission() != 0) return false;
  delay(10);
  if (Wire.requestFrom((int)ENDERECO_SHT40, 6) != 6) return false;

  uint16_t rawTemp = (Wire.read() << 8) | Wire.read();
  Wire.read();
  uint16_t rawUmid = (Wire.read() << 8) | Wire.read();
  Wire.read();

  temperatura = -45.0 + 175.0 * ((float)rawTemp / 65535.0);
  umidade = -6.0 + 125.0 * ((float)rawUmid / 65535.0);
  if (umidade < 0) umidade = 0;
  if (umidade > 100) umidade = 100;
  return true;
}

// Leitura do SHT20/SHT25 via comandos "no hold master" (protocolo diferente do SHT40 - comandos
// separados F3/F5 em vez de uma transacao unica).
bool lerSHT2x(float &temperatura, float &umidade) {
  Wire.beginTransmission(ENDERECO_SHT2X);
  Wire.write(0xF3);
  if (Wire.endTransmission() != 0) return false;
  delay(90);
  if (Wire.requestFrom((int)ENDERECO_SHT2X, 3) != 3) return false;
  uint16_t rawTemp = (Wire.read() << 8) | Wire.read();
  Wire.read();
  rawTemp &= 0xFFFC;
  temperatura = -46.85 + 175.72 * ((float)rawTemp / 65536.0);

  Wire.beginTransmission(ENDERECO_SHT2X);
  Wire.write(0xF5);
  if (Wire.endTransmission() != 0) return false;
  delay(30);
  if (Wire.requestFrom((int)ENDERECO_SHT2X, 3) != 3) return false;
  uint16_t rawUmid = (Wire.read() << 8) | Wire.read();
  Wire.read();
  rawUmid &= 0xFFFC;
  umidade = -6 + 125 * ((float)rawUmid / 65536.0);
  return true;
}

// Le um sensor (SHT40, BMP180 ou SHT2x) e atualiza os registradores Modbus dele, alem do status
// local usado pro alarme de falha geral. Se nao responder, so marca "nao ok" - o valor antigo
// fica no registrador (nao zera sozinho).
void lerEAtualizarSensor(Sensor &s) {
  bool ok = false;

  if (selecionarCanalMux(s.canal)) {
    if (s.tipo == TIPO_SHT40) {
      float temperatura, umidade;
      if (dispositivoPresente(ENDERECO_SHT40) && lerSHT40(temperatura, umidade)) {
        mb.Ireg(s.regValor1, (uint16_t)(temperatura * 10));
        mb.Ireg(s.regValor2, (uint16_t)(umidade * 10));
        leituraTemperatura[s.regStatus] = temperatura;
        leituraUmidade[s.regStatus] = umidade;
        ok = true;
      }
    } else if (s.tipo == TIPO_BMP180) {
      if (dispositivoPresente(ENDERECO_BMP180) && sensorBMP.begin()) {
        sensorBMP.resetToDefaults();
        sensorBMP.setSamplingMode(BMP180MI::MODE_UHR);

        unsigned long inicio = millis();
        sensorBMP.measureTemperature();
        do { delay(20); } while (!sensorBMP.hasValue() && millis() - inicio < 800);
        float temperatura = sensorBMP.getTemperature();

        inicio = millis();
        sensorBMP.measurePressure();
        do { delay(20); } while (!sensorBMP.hasValue() && millis() - inicio < 800);
        float pressaoHpa = sensorBMP.getPressure() / 100.0;  // Pa -> hPa (evita estourar o registrador de 16 bits)

        mb.Ireg(s.regValor1, (uint16_t)(temperatura * 10));
        mb.Ireg(s.regValor2, (uint16_t)(pressaoHpa * 10));
        leituraTemperatura[s.regStatus] = temperatura;
        leituraUmidade[s.regStatus] = pressaoHpa;
        ok = true;
      }
    } else {  // TIPO_SHT2X
      float temperatura, umidade;
      if (dispositivoPresente(ENDERECO_SHT2X) && lerSHT2x(temperatura, umidade)) {
        mb.Ireg(s.regValor1, (uint16_t)(temperatura * 10));
        mb.Ireg(s.regValor2, (uint16_t)(umidade * 10));
        leituraTemperatura[s.regStatus] = temperatura;
        leituraUmidade[s.regStatus] = umidade;
        ok = true;
      }
    }
  }

  mb.Ists(s.regStatus, ok);
  sensorOkAtual[s.regStatus] = ok;
}

// Atualiza um sensor por vez, em rodizio (evita travar o loop lendo todos de uma vez)
void atualizarProximoSensor() {
  lerEAtualizarSensor(sensores[indiceProximoSensor]);
  indiceProximoSensor = (indiceProximoSensor + 1) % TOTAL_SENSORES;
}

// ============================================================================
// Controle dos estagios e da ventoinha (portado do teste_wifi_servidor.cpp)
// ============================================================================

// Muda o estado de um estagio e marca quando mudou (Input Registers 12/13). So mexe no relogio
// se o valor realmente mudou - chamar de novo com o mesmo valor nao zera a contagem.
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

void marcarCicloIncompleto(int i, bool valor) {
  cicloIncompleto[i] = valor;
  preferencias.putBool(CHAVE_CICLO_INCOMPLETO[i], valor);
}

// Escreve nos pinos o estado atual das resistencias e das ventoinhas.
void aplicarSaidas() {
  for (int i = 0; i < TOTAL_ESTAGIOS; i++) {
    digitalWrite(PINOS_RESISTENCIA[i], estagioAquecendo[i] ? HIGH : LOW);
  }
  digitalWrite(PINO_VENTOINHA, ventoinha1Ligada ? HIGH : LOW);
  digitalWrite(PINO_VENTOINHA2, ventoinha2Ligada ? HIGH : LOW);
}

// Corte de seguranca por temperatura do topo, em QUALQUER modo: no automatico usa o limite fixo,
// no manual o limite ajustavel. Corte no automatico vira prioridade de retomada (ciclo incompleto);
// no manual e teste de bancada e nao vira.
void aplicarCorteSeguranca() {
  const int topo[TOTAL_ESTAGIOS] = { E1_TOPO, E2_TOPO };
  float limite = modoManual ? temperaturaMaximaManual : TEMPERATURA_DESLIGAR_AUTO;

  for (int i = 0; i < TOTAL_ESTAGIOS; i++) {
    if (!estagioAquecendo[i]) continue;
    if (sensorOkAtual[topo[i]] && leituraTemperatura[topo[i]] > limite) {
      definirEstagio(i, false);
      if (!modoManual) marcarCicloIncompleto(i, true);
    }
  }
}

// Controle automatico, sempre UM estagio por vez: desliga quando a base seca (ciclo concluido);
// liga dando prioridade a ciclo incompleto assim que o topo esfriar ate TEMPERATURA_RELIGAR_AUTO;
// senao escolhe o topo mais umido acima do limite. A ventoinha 1 fica ligada com qualquer estagio
// aquecendo ou ciclo incompleto. So considera sensores com leitura valida.
void controlarAutomatico() {
  const int topo[TOTAL_ESTAGIOS] = { E1_TOPO, E2_TOPO };
  const int base[TOTAL_ESTAGIOS] = { E1_BASE, E2_BASE };
  bool desligadoAgora[TOTAL_ESTAGIOS] = { false, false };

  for (int i = 0; i < TOTAL_ESTAGIOS; i++) {
    if (!estagioAquecendo[i]) continue;
    if (sensorOkAtual[base[i]] && leituraUmidade[base[i]] < limiteDesligarUmidade) {
      definirEstagio(i, false);
      desligadoAgora[i] = true;
      marcarCicloIncompleto(i, false);
      incrementarCicloRegeneracao();
    }
  }

  if (!algumEstagioAquecendo()) {
    int escolhido = -1;

    for (int i = 0; i < TOTAL_ESTAGIOS; i++) {
      if (!cicloIncompleto[i] || desligadoAgora[i]) continue;
      if (sensorOkAtual[base[i]] && leituraUmidade[base[i]] < limiteDesligarUmidade) {
        marcarCicloIncompleto(i, false);  // base ja seca (ex: reboot quase no fim): nada a retomar
        continue;
      }
      if (sensorOkAtual[topo[i]] && leituraTemperatura[topo[i]] <= TEMPERATURA_RELIGAR_AUTO) {
        escolhido = i;
        break;
      }
    }

    if (escolhido < 0) {
      float maiorUmidade = -1;
      for (int i = 0; i < TOTAL_ESTAGIOS; i++) {
        if (cicloIncompleto[i] || desligadoAgora[i]) continue;
        bool quer = sensorOkAtual[topo[i]] && leituraUmidade[topo[i]] > limiteLigarUmidade &&
                    leituraTemperatura[topo[i]] <= TEMPERATURA_RELIGAR_AUTO;
        if (quer && leituraUmidade[topo[i]] > maiorUmidade) {
          maiorUmidade = leituraUmidade[topo[i]];
          escolhido = i;
        }
      }
    }

    if (escolhido >= 0) {
      definirEstagio(escolhido, true);
      // Ciclo em andamento: se o ESP reiniciar no meio, retoma com prioridade (so no automatico)
      if (!cicloIncompleto[escolhido]) marcarCicloIncompleto(escolhido, true);
    }
  }

  ventoinha1Ligada = algumEstagioAquecendo() || algumCicloIncompleto();
}

// Roda a cada leitura de sensor (2 s).
void controlarEstagios() {
  aplicarCorteSeguranca();
  if (!modoManual) controlarAutomatico();
  aplicarSaidas();
}

// Aceita os limites escritos pelo mestre nos Holding Registers (mesmas regras da pagina WiFi);
// se forem invalidos, devolve o valor anterior no registrador.
void sincronizarLimites() {
  float ligar = mb.Hreg(REG_LIMITE_LIGAR) / 10.0f;
  float desligar = mb.Hreg(REG_LIMITE_DESLIGAR) / 10.0f;
  float temp = mb.Hreg(REG_TEMP_MAXIMA_MANUAL) / 10.0f;

  if (fabsf(ligar - limiteLigarUmidade) < 0.05f && fabsf(desligar - limiteDesligarUmidade) < 0.05f &&
      fabsf(temp - temperaturaMaximaManual) < 0.05f) return;

  bool valido = ligar <= 100 && desligar >= 0 && desligar < ligar && temp <= 125;
  if (valido) {
    limiteLigarUmidade = ligar;
    limiteDesligarUmidade = desligar;
    temperaturaMaximaManual = temp;
    preferencias.putFloat("limLigar", ligar);
    preferencias.putFloat("limDesl", desligar);
    preferencias.putFloat("tempMax", temp);
  } else {
    mb.Hreg(REG_LIMITE_LIGAR, (uint16_t)(limiteLigarUmidade * 10 + 0.5f));
    mb.Hreg(REG_LIMITE_DESLIGAR, (uint16_t)(limiteDesligarUmidade * 10 + 0.5f));
    mb.Hreg(REG_TEMP_MAXIMA_MANUAL, (uint16_t)(temperaturaMaximaManual * 10 + 0.5f));
  }
}

// Le o que o mestre escreveu: troca de modo e, no manual, o comando das saidas.
void sincronizarComandos() {
  bool novoManual = mb.Coil(COIL_MODO_MANUAL);
  if (novoManual != modoManual) {
    if (modoManual) {
      // manual -> automatico: parte de tudo desligado
      for (int i = 0; i < TOTAL_ESTAGIOS; i++) definirEstagio(i, false);
      ventoinha1Ligada = false;
      ventoinha2Ligada = false;
    } else {
      // automatico -> manual: descarta ciclos pendentes (quem mexe na bancada assume o controle)
      for (int i = 0; i < TOTAL_ESTAGIOS; i++) marcarCicloIncompleto(i, false);
    }
    modoManual = novoManual;
    aplicarSaidas();
  }

  if (modoManual) {
    definirEstagio(0, mb.Coil(COIL_RESISTENCIA_E1));
    definirEstagio(1, mb.Coil(COIL_RESISTENCIA_E2));
    ventoinha1Ligada = mb.Coil(COIL_VENTOINHA1);
    ventoinha2Ligada = mb.Coil(COIL_VENTOINHA2);
  }
  sincronizarLimites();
}

// Publica o estado real das saidas nos Coils e nos Discrete Inputs de ciclo incompleto.
void publicarEstado() {
  mb.Coil(COIL_VENTOINHA1, ventoinha1Ligada);
  mb.Coil(COIL_RESISTENCIA_E1, estagioAquecendo[0]);
  mb.Coil(COIL_RESISTENCIA_E2, estagioAquecendo[1]);
  mb.Coil(COIL_VENTOINHA2, ventoinha2Ligada);
  mb.Coil(COIL_MODO_MANUAL, modoManual);
  for (int i = 0; i < TOTAL_ESTAGIOS; i++) {
    mb.Ists(REG_CICLO_INCOMPLETO_E1 + i, cicloIncompleto[i]);
    unsigned long seg = (millis() - ultimaMudancaEstagioMs[i]) / 1000;
    mb.Ireg(REG_DESDE_MUDANCA_E1 + i, (uint16_t)min(seg, 65535UL));  // satura em ~18 h
  }
}

// ============================================================================
// Watchdog: reinicia o ESP se o loop() travar (uma resistencia ligada sem nenhuma checagem
// de seguranca rodando seria o pior caso).
// ============================================================================
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
    esp_task_wdt_init(WDT_TIMEOUT_MS / 1000, true);
  }
#else
  esp_task_wdt_init(WDT_TIMEOUT_MS / 1000, true);
#endif
  esp_task_wdt_add(NULL);
}

void setup() {
  // Saidas sempre desligadas no boot, antes de qualquer outra coisa
  for (int i = 0; i < TOTAL_ESTAGIOS; i++) {
    pinMode(PINOS_RESISTENCIA[i], OUTPUT);
  }
  pinMode(PINO_VENTOINHA, OUTPUT);
  pinMode(PINO_VENTOINHA2, OUTPUT);
  aplicarSaidas();

  Serial.begin(115200);
  delay(500);
  Wire.begin(40, 41);

  Serial.println("=== TESTE MODBUS RTU (4x SHT40 + BMP180 + ventoinha) ===");
  Serial.println("UART2 (RS-485): RX2=GPIO38, TX2=GPIO37, 9600 8N1, escravo ID 1");

  pinMode(PINO_LED1, OUTPUT);
  digitalWrite(PINO_LED1, LOW);
  pinMode(PINO_LED2, OUTPUT);
  digitalWrite(PINO_LED2, LOW);

  preferencias.begin("secador", false);
  ciclosRegeneracao = preferencias.getUInt("ciclos", 0);
  Serial.print("Ciclos de regeneracao carregados da NVS: ");
  Serial.println(ciclosRegeneracao);
  limiteLigarUmidade = preferencias.getFloat("limLigar", PADRAO_LIMITE_LIGAR_UMIDADE);
  limiteDesligarUmidade = preferencias.getFloat("limDesl", PADRAO_LIMITE_DESLIGAR_UMIDADE);
  temperaturaMaximaManual = preferencias.getFloat("tempMax", PADRAO_TEMPERATURA_MAXIMA_MANUAL);
  for (int i = 0; i < TOTAL_ESTAGIOS; i++) {
    cicloIncompleto[i] = preferencias.getBool(CHAVE_CICLO_INCOMPLETO[i], false);
  }

  SerialModbus.begin(MODBUS_BAUD, SERIAL_8N1, PINO_RX2, PINO_TX2);
  mb.begin(&SerialModbus);
  mb.slave(ID_ESCRAVO);

  mb.addHreg(REG_TESTE, 1234);
  mb.addHreg(REG_CICLOS_SILICA, ciclosRegeneracao);

  for (int i = 0; i < TOTAL_SENSORES; i++) {
    mb.addIreg(sensores[i].regValor1, 0);
    mb.addIreg(sensores[i].regValor2, 0);
    mb.addIsts(sensores[i].regStatus, false);
  }
  mb.addIsts(REG_ALARME_FALHA, true);       // comeca "true" ate a 1a leitura provar o contrario
  mb.addIsts(REG_ALARME_SATURACAO, calcularSaturacaoSilica());

  mb.addHreg(REG_LIMITE_LIGAR, (uint16_t)(limiteLigarUmidade * 10 + 0.5f));
  mb.addHreg(REG_LIMITE_DESLIGAR, (uint16_t)(limiteDesligarUmidade * 10 + 0.5f));
  mb.addHreg(REG_TEMP_MAXIMA_MANUAL, (uint16_t)(temperaturaMaximaManual * 10 + 0.5f));

  for (int i = 0; i < TOTAL_ESTAGIOS; i++) {
    mb.addIsts(REG_CICLO_INCOMPLETO_E1 + i, cicloIncompleto[i]);
    mb.addIreg(REG_DESDE_MUDANCA_E1 + i, 0);
  }

  mb.addCoil(COIL_VENTOINHA1, false);
  mb.addCoil(COIL_RESISTENCIA_E1, false);
  mb.addCoil(COIL_RESISTENCIA_E2, false);
  mb.addCoil(COIL_VENTOINHA2, false);
  mb.addCoil(COIL_MODO_MANUAL, false);  // sempre inicia em automatico

  inicializarWatchdog();

  Serial.println("Pronto - mapa de registradores ativo, aguardando mestre Modbus");
}

void loop() {
  esp_task_wdt_reset();
  sincronizarComandos();

  if (SerialModbus.available()) {
    digitalWrite(PINO_LED1, HIGH);
    tempoAcendeuLed = millis();
  }
  if (millis() - tempoAcendeuLed > DURACAO_PISCA_MS) {
    digitalWrite(PINO_LED1, LOW);
  }

  if (millis() - tempoUltimaLeitura > INTERVALO_LEITURA_MS) {
    tempoUltimaLeitura = millis();
    atualizarProximoSensor();

    bool falha = calcularFalhaGeral();
    bool saturacao = calcularSaturacaoSilica();
    mb.Ists(REG_ALARME_FALHA, falha);
    mb.Ists(REG_ALARME_SATURACAO, saturacao);
    digitalWrite(PINO_LED2, (falha || saturacao) ? HIGH : LOW);

    controlarEstagios();
  }

  publicarEstado();
  mb.task();
}
