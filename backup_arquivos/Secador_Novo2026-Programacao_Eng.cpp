/* 
========================================================================================================
Programa para SECADOR REGENERATIVO
Data: Outubro de 2024 REVISÃO 01
========================================================================================================
*/

#include <Arduino.h>

// ==============================================================================
// --- Bibliotecas Auxiliares ---
#include <Wire.h>               // Biblioteca para comunicação I2C, usada para comunicação entre o microcontrolador e dispositivos conectados no barramento I2C
#include <BMP180I2C.h>          // Biblioteca para o sensor BMP180, que mede pressão e temperatura ambiente
#include <LiquidCrystal_I2C.h>  // Biblioteca para controlar um display LCD via I2C
#include <AHT20.h> // Biblioteca para o sensor AHT20, que mede umidade e temperatura
#include <esp_task_wdt.h>  // Biblioteca para utilizar o Watchdog, que monitora o sistema e reinicia o ESP32 caso o código trave
#include <nvs_flash.h>  // Biblioteca para usar a memória NVS (Non-Volatile Storage), permitindo salvar dados de forma permanente no ESP32
#include <nvs.h>        // Biblioteca auxiliar para o gerenciamento da NVS
#include "webServerSecador.h"

// ==============================================================================
// --- Pinos de Entrada/Saida ---
// ==============================================================================
// Definindo os pinos de controle para os ventiladores
const int ventilador01 = 10;  // GPIO10 = VENT_1 no esquematico da placa oficial
// GPIO22-25 nao existem na ESP32-S3 e GPIO26/27 sao reservados para a flash interna -
// chaveFluxo nao esta no esquematico oficial (nao documentada/nao conectada ainda),
// usando GPIO18 como placeholder valido ate a fiacao real ser definida.
const int chaveFluxo = 18;


// Definindo os pinos de controle para as resistências (RES_1-4 no esquematico da placa oficial)
const int resistenciaEstagio1 = 14;  // GPIO14 = RES_1
const int resistenciaEstagio2 = 13;  // GPIO13 = RES_2
const int resistenciaEstagio3 = 12;  // GPIO12 = RES_3
const int resistenciaAuxiliar = 11;  // GPIO11 = RES_4

// Definindo os pinos para os botões (BT1-4 no esquematico da placa oficial)
const int botao01 = 4;  // GPIO4 = BT1
const int botao02 = 5;  // GPIO5 = BT2
const int botao03 = 6;  // GPIO6 = BT3
const int botao04 = 7;  // GPIO7 = BT4

// Leds mostrando resistencia ligada (LED1-2 no esquematico da placa oficial)
const int ledResistencia1 = 15;  // GPIO15 = LED1
const int ledResistencia2 = 16;  // GPIO16 = LED2

// Leitura digital dos botões, retornando 0 ou 1
#define BTN1 digitalRead(botao01)  // Macro que simplifica a leitura digital do botão 1
#define BTN2 digitalRead(botao02)  // Macro que simplifica a leitura digital do botão 2
#define BTN3 digitalRead(botao03)  // Macro que simplifica a leitura digital do botão 3
#define BTN4 digitalRead(botao04)  // Macro que simplifica a leitura digital do botão 4


// ==============================================================================
// --- Funcoes de Controle de Hardware (liga/desliga) ---
// ==============================================================================
// Funções para ligar os ventiladores
void ligaVentilador1() {
  digitalWrite(ventilador01, HIGH);
}

// Funções para desligar os ventiladores
void desligaVentilador1() {
  digitalWrite(ventilador01, LOW);
}

// Funções para ligar as resistências
// Liga a resistência do estágio 1
void ligaResistenciaEstagio1() {
  digitalWrite(resistenciaEstagio1, HIGH);
}
// Liga a resistência do estágio 2
void ligaResistenciaEstagio2() {
  digitalWrite(resistenciaEstagio2, HIGH);
}
// Liga a resistência do estágio 3
void ligaResistenciaEstagio3() {
  digitalWrite(resistenciaEstagio3, HIGH);
}

// Liga a resistência auxiliar (proteção contra frio extremo/anticondensação na caixa)
void ligaResistenciaAuxiliar() {
  digitalWrite(resistenciaAuxiliar, HIGH);
}

// Funções para desligar as resistências
// Desliga a resistência do estágio 1
void desligaResistenciaEstagio1() {
  digitalWrite(resistenciaEstagio1, LOW);
}
// Desliga a resistência do estágio 2
void desligaResistenciaEstagio2() {
  digitalWrite(resistenciaEstagio2, LOW);
}
// Desliga a resistência do estágio 3
void desligaResistenciaEstagio3() {
  digitalWrite(resistenciaEstagio3, LOW);
}
// Desliga a resistência auxiliar
void desligaResistenciaAuxiliar() {
  digitalWrite(resistenciaAuxiliar, LOW);
}


// ==============================================================================
// --- Sensores ---
// ==============================================================================

// Instanciando objetos para os sensores
BMP180I2C bmp180Topo(0x77);             // Sensor BMP180 com endereço 0x77
LiquidCrystal_I2C lcd(0x27, 20, 4);  // Display LCD com endereço 0x27, com 20 colunas e 4 linhas

// Sensores AHT20 (7 unidades): topo/base por estágio + caixa de controle
AHT20 sensoresAHT[7] = {
  AHT20(0x38), AHT20(0x38), AHT20(0x38), AHT20(0x38),
  AHT20(0x38), AHT20(0x38), AHT20(0x38)
};
bool sensoresAHTDisponiveis[7] = { false };
bool bmp180TopoDisponivel = false;

// Índices nomeados - deixa claro o que cada posição do array representa
#define sens_cima_est1 0
#define sens_baixo_est1 1
#define sens_cima_est2 2
#define sens_baixo_est2 3
#define sens_cima_est3 4
#define sens_baixo_est3 5
#define sens_caixa 6
#define CANAL_BMP180_TOPO 7

// Facilita loops por estágio 
const int cima_estagio[3] = {sens_cima_est1, sens_cima_est2, sens_cima_est3};
const int baixo_estagio[3] = {sens_baixo_est1, sens_baixo_est2, sens_baixo_est3};

const char* nomesSensoresAHT[7] = {
  "Topo E1", "Base E1", "Topo E2", "Base E2", "Topo E3", "Base E3", "Caixa"
};

// ==============================================================================
// --- Variáveis ---
// ==============================================================================


unsigned long tempoImpressaoLCD, tempoLigarResistencia;  // Variavel para armazenar tempos de execução

float umidadeAHT[7];   // array pra armazenar os valores de umidade dos 7 sensores
float temperaturaAHT[7];   // array pra armazenar os valores de temperatura dos 7 sensores

// Referências nomeadas pra caixa de controle (facilita leitura em outros lugares do código)
float &umidadeCaixa = umidadeAHT[sens_caixa];      
float &temperaturaCaixa = temperaturaAHT[sens_caixa];

// ---- Controle por estágio ----
bool estagioAquecendo[3] = { false, false, false };
bool sistemaTravado = false;  // true = chave de fluxo detectou ar passando, tudo pausado
unsigned long tempoInicioAquecimento[3] = { 0, 0, 0 };

// Limites de controle (compartilhados entre os 3 estágios por enquanto)
#define LIMITE_LIGAR_UMIDADE 80        // umidade no sensor de TOPO que dispara o aquecimento
#define LIMITE_DESLIGAR_UMIDADE 72      // umidade no sensor de BASE que confirma que já secou
#define TEMPERATURA_SEGURANCA_MAX 90.0 // nível de segurança 1: corte por temperatura
#define TEMPO_MAX_AQUECIMENTO_MS (30UL * 60UL * 1000UL)  // nível de segurança 2: 30 min - ajustar depois de testar
#define TEMPERATURA_FRIO_EXTREMO 0.0       // abaixo disso liga a resist da caixa externa protecao(contra o frio)
#define UMIDADE_CAIXA_MAX 80             // acima disso, liga por anti-condesacao na caixa
#define LIMIAR_DIRECAO_PA 5.0 

// Arrays de função pra facilitar iteração pelos 3 estágios
void (*funcLigaResistencia[3])() = { ligaResistenciaEstagio1, ligaResistenciaEstagio2, ligaResistenciaEstagio3 };
void (*funcDesligaResistencia[3])() = { desligaResistenciaEstagio1, desligaResistenciaEstagio2, desligaResistenciaEstagio3 };

// Direcao do fluxo de ar detectada via variacao de pressao do BMP180 (topo) - usada so como informacao/log por enquanto
enum DirecaoFluxo { FLUXO_ESTAVEL, FLUXO_EXPELINDO, FLUXO_PUXANDO };
DirecaoFluxo direcaoFluxo = FLUXO_ESTAVEL;

// Converte o enum DirecaoFluxo em texto legivel para exibicao (Serial/LCD)
const char* direcaoFluxoTexto() {
  switch (direcaoFluxo) {
    case FLUXO_EXPELINDO: return "EXPELINDO";
    case FLUXO_PUXANDO: return "PUXANDO";
    default: return "ESTAVEL";
  }
}

// ---- Detecção de fluxo do transformador (chave mecânica) ----
bool fluxoAtivo = false;              // Estado já filtrado (debounced) - true = ar passando
bool leituraFluxoAnterior = HIGH;     // Última leitura bruta do pino
bool auxiliarAquecendo = false;       // auxiliar para a caixa eletronica(temperatura e pressao)
unsigned long tempoUltimaMudancaFluxo = 0;
const unsigned long DEBOUNCE_FLUXO = 300;  // ms - ajustar depois de testar 

float temperaturaEntrada;   // Armazena valor de leitura de temperatura de entrada (BMP180)
// Armazena a pressao atual e a anterior do BMP180 (topo) - a diferenca entre elas indica a direcao do fluxo de ar
float pressaoTopoAtual = 0, pressaoTopoAnterior = 0;
int ativado = 0;  // Variável para controle de estado geral do sistema (0 = desativado, 1 = ativado).


int menu = 1;  // Variavel que define as informações que vao aparecer no LCD
int menuAnt;   // Variavel para saber quando o menu foi alterado e limpar a pagina


// Configurações do watchdog
#define TEMPO_LIMITE_WATCHDOG 30000        // Define o tempo limite do watchdog como 30 segundos (30000 ms).
#define CONFIG_FREERTOS_NUMBER_OF_CORES 1  // Define o número de núcleos do FreeRTOS para 1.

#if ESP_ARDUINO_VERSION_MAJOR >= 3
esp_task_wdt_config_t configuracaoWatchdog = {
  // Configuração do watchdog timer (WDT) da ESP32.
  .timeout_ms = TEMPO_LIMITE_WATCHDOG,                            // Define o tempo limite do watchdog em milissegundos.
  .idle_core_mask = (1 << CONFIG_FREERTOS_NUMBER_OF_CORES) - 1,  // Máscara de bits para núcleos inativos (núcleo 0, neste caso).
  .trigger_panic = true                                       // Aciona um pânico e reinicializa se o WDT não for resetado a tempo.
};
#endif


// ==============================================================================
// --- Declaracoes Antecipadas (forward declarations) ---
// ==============================================================================
// Funções de controle da Web
void webHandleRoot();
void webHandleUpdate();
void webHandleDados();
void webHandleNotFound();
void webSetupServer();

// Funções de configuração e inicialização
bool configurarTCA9548A(uint8_t canal);  // Retorna false quando o multiplexador não responde
void configurarPinos();
void inicializarNVS();
void inicializarSensores();
void inicializarSensoresAHT();
void inicializarWatchdog();

// Funcoes de logica de controle (aquecimento por estagio e deteccao de fluxo)
void lerChaveFluxo();
void controlarEstagios();
void controlarAuxiliar();

// Funções de leitura e exibição
void lerSensores();
bool lerBMP180Topo();
void exibirLCD();  // Mudando para "exibir" para consistência com a ação

// Funções de armazenamento de dados
void salvarDados(const char* chave, bool valorParaSalvar);
bool lerDados(const char* chave);

// ==============================================================================
// Setup
void setup() {
  Serial.begin(115200);
  Wire.begin(40, 41);  // SDA=IO40, SCL=IO41 - confere com o esquematico e foi confirmado por varredura com reset real. O barramento deve estar pronto antes de qualquer dispositivo I2C.
  Wire.setTimeOut(50);  // Limita a espera do driver I2C (ms) - evita travar o Interrupt Watchdog quando um canal do mux fica flutuando sem sensor conectado.

#if USAR_WIFI
  webSetupServer();
#endif
  lcd.init();       // Inicializa o display LCD (configura o display para operação).
  lcd.backlight();  // Liga a luz de fundo do display LCD.

  inicializarNVS();       // Função para inicializar a NVS (Non-Volatile Storage), utilizada para armazenar dados de forma persistente na ESP32.
  configurarPinos();      // Função para definir os pinos utilizados no sistema (entradas, saídas, etc.).
  inicializarSensores();  // Função para inicializar os sensores conectados (configurações e inicializações necessárias).
  inicializarWatchdog();  // Função para inicializar o watchdog timer, responsável por monitorar se o sistema está travado.

  lerSensores();  // Função que lê os dados de outros sensores adicionais.
  exibirLCD();    // Função para imprimir os dados no display LCD.
}

// ==============================================================================
// Programa Principal
void loop() {  // Função principal que executa continuamente

#if USAR_WIFI
  if (wifiConectado) {
  server.handleClient(); // so chama se o servidor realmente subiu
  }
#endif

  lerSensores();          // Função que lê os dados de outros sensores adicionais.
  lerChaveFluxo();
  exibirLCD();            // Função para imprimir os dados no display LCD.

  sistemaTravado = fluxoAtivo;  // trava geral do sistema segue o estado (debounced) da chave de fluxo

  if (BTN1 == 0)  // botao pressionado = LOW (pull-up) -> troca pra tela do menu 1
    menu = 1;
  if (BTN2 == 0)
    menu = 2;
  if (BTN3 == 0)
    menu = 3;
  if (BTN4 == 0) {  // atalho manual: forca ligar o estagio 1 na mao, sem esperar a umidade
    funcLigaResistencia[0]();
    estagioAquecendo[0] = true;
    tempoInicioAquecimento[0] = millis();  // marca o inicio pra valer o corte de seguranca por tempo maximo
  }

  controlarEstagios();
  controlarAuxiliar();

  esp_task_wdt_reset();  // Reseta o watchdog para evitar que o sistema seja reinicializado caso o loop demore muito.
}  // Fim da função loop

// ==============================================================================
// --- Funcoes de Logica de Controle ---
// ==============================================================================
// Leitura da chave mecânica de fluxo (detecta se o transformador está respirando)
void lerChaveFluxo() {
  bool leituraAtual = digitalRead(chaveFluxo);

  if (leituraAtual != leituraFluxoAnterior) {  // mudou desde a ultima checagem -> reinicia a contagem do debounce
    tempoUltimaMudancaFluxo = millis();
    leituraFluxoAnterior = leituraAtual;
  }

  if ((millis() - tempoUltimaMudancaFluxo) > DEBOUNCE_FLUXO) {  // leitura estavel por tempo suficiente -> confirma o estado
    fluxoAtivo = (leituraAtual == LOW);  // chave fecha (LOW) quando o ar esta passando
  }
}


// Controla o aquecimento dos 3 estágios (liga/desliga por umidade, corta por
// segurança de temperatura/tempo ou trava geral) e atualiza LEDs e ventilador
void controlarEstagios() {
  bool algumAquecendo = false;

  for (int i = 0; i < 3; i++) {
    int indiceTopo = cima_estagio[i];
    int indiceBase = baixo_estagio[i];
    bool forcarDesligar = false;

    if (sistemaTravado) {  // seguranca 0: trava geral (chave de fluxo) sobrepoe tudo
      forcarDesligar = true;
    } else if (temperaturaAHT[indiceTopo] > TEMPERATURA_SEGURANCA_MAX) {  // seguranca 1: temperatura acima do limite
      forcarDesligar = true;
      Serial.print("ALERTA: Estagio "); Serial.print(i + 1);
      Serial.println(" desligado por excesso de temperatura.");
    } else if (estagioAquecendo[i] && (millis() - tempoInicioAquecimento[i] > TEMPO_MAX_AQUECIMENTO_MS)) {  // seguranca 2: tempo maximo de aquecimento estourado
      forcarDesligar = true;
      Serial.print("ALERTA: Estagio "); Serial.print(i + 1);
      Serial.println(" desligado por tempo maximo de aquecimento excedido.");
    }

    if (forcarDesligar) {  // qualquer condicao de seguranca desliga na hora, independente do estado atual
      funcDesligaResistencia[i]();
      estagioAquecendo[i] = false;
    } else if (!estagioAquecendo[i]) {  // esta desligado -> so liga se o topo detectar umidade alta (entrada de ar umido)
      if (umidadeAHT[indiceTopo] > LIMITE_LIGAR_UMIDADE) {
        funcLigaResistencia[i]();
        estagioAquecendo[i] = true;
        tempoInicioAquecimento[i] = millis();
      }
    } else {  // esta ligado -> so desliga quando a base indicar que ja secou o suficiente
      if (umidadeAHT[indiceBase] < LIMITE_DESLIGAR_UMIDADE) {
        funcDesligaResistencia[i]();
        estagioAquecendo[i] = false;
      }
    }

    if (estagioAquecendo[i]) algumAquecendo = true;  // acumula pra decidir o ventilador depois do loop
  }

  // LEDs indicadores (só há 2 pinos de LED hoje - Estagio 3 sem indicador dedicado por enquanto)
  digitalWrite(ledResistencia1, estagioAquecendo[0] ? HIGH : LOW);
  digitalWrite(ledResistencia2, estagioAquecendo[1] ? HIGH : LOW);

  // Ventilador único: liga só quando algum estágio está regenerando
  if (algumAquecendo && !sistemaTravado) {  // nao liga o ventilador se o sistema estiver travado, mesmo com estagio "ligado"
    ligaVentilador1();
  } else {
    desligaVentilador1();
  }
}

// Liga/desliga a resistência auxiliar conforme frio extremo na entrada ou
// excesso de umidade na caixa de controle
void controlarAuxiliar() {
  bool precisaAquecer = (temperaturaEntrada < TEMPERATURA_FRIO_EXTREMO) || (umidadeCaixa > UMIDADE_CAIXA_MAX);  // liga por frio extremo na entrada OU anti-condensacao na caixa

  if (precisaAquecer && !auxiliarAquecendo) {  // so dispara a transicao (evita chamar liga/log toda hora)
    ligaResistenciaAuxiliar();
    auxiliarAquecendo = true;
    Serial.println("Auxiliar (frio extremo/caixa) -> LIGADO");
  } else if (!precisaAquecer && auxiliarAquecendo) {
    desligaResistenciaAuxiliar();
    auxiliarAquecendo = false;
    Serial.println("Auxiliar (frio extremo/caixa) -> DESLIGADO");
  }
}

// Inicializa o Multiplexador com o endereço configurado
bool configurarTCA9548A(uint8_t canal) {
  Wire.beginTransmission(0x70);  // A0= LOW; A1= LOW; A2= LOW
  Wire.write(1 << canal);
  return Wire.endTransmission() == 0;
}

// Verifica rapido se existe um dispositivo respondendo no endereco antes de chamar
// a biblioteca do sensor - evita travar o driver I2C do ESP32 chamando .begin()
// num dispositivo que fisicamente nao esta conectado.
bool i2cDispositivoPresente(uint8_t endereco) {
  Wire.beginTransmission(endereco);
  return Wire.endTransmission() == 0;
}
// ==============================================================================
// Função para definição dos pinos
void configurarPinos() {
  pinMode(resistenciaEstagio1, OUTPUT);  // Define o pino de resistência 1 como saída
  pinMode(resistenciaEstagio2, OUTPUT);  // Define o pino de resistência 2 como saída
  pinMode(resistenciaEstagio3, OUTPUT);  // Define o pino de resistência 3 como saída
  pinMode(resistenciaAuxiliar, OUTPUT);

  pinMode(ventilador01, OUTPUT);  // Define o pino do ventilador 1 como saída
  pinMode(chaveFluxo, INPUT_PULLUP);

  pinMode(botao01, INPUT_PULLUP);  // Define o pino do botão 1 como entrada com resistor pull-up
  pinMode(botao02, INPUT_PULLUP);  // Define o pino do botão 2 como entrada com resistor pull-up
  pinMode(botao03, INPUT_PULLUP);  // Define o pino do botão 3 como entrada com resistor pull-up
  pinMode(botao04, INPUT_PULLUP);  // Define o pino do botão 4 como entrada com resistor pull-up

  pinMode(ledResistencia1, OUTPUT);  // Led auxiliar para analisar quando resistencia 1 ta ligada
  pinMode(ledResistencia2, OUTPUT);  // Led auxiliar para analisar quando resistencia 1 ta ligada

  digitalWrite(resistenciaEstagio1, LOW);
  digitalWrite(resistenciaEstagio2, LOW);
  digitalWrite(resistenciaEstagio3, LOW);
  digitalWrite(resistenciaAuxiliar, LOW);

  digitalWrite(ventilador01, LOW);

  digitalWrite(ledResistencia1, LOW);
  digitalWrite(ledResistencia2, LOW);
}
// ==============================================================================
// Função inicializa NVS
void inicializarNVS() {
  // Inicializa o NVS (Non-Volatile Storage) que permite salvar dados na memória flash
  esp_err_t resultado = nvs_flash_init();  // Inicializa o NVS e retorna o status da operação
  // Verifica se ocorreu algum erro relacionado a páginas ou versões
  if (resultado == ESP_ERR_NVS_NO_FREE_PAGES || resultado == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    nvs_flash_erase();  // Apaga a partição NVS se não houver páginas livres ou se foi encontrada uma versão diferente
    nvs_flash_init();   // Re-inicializa o NVS após a limpeza
  }
}
// ==============================================================================
// Função inicializarSensores
void inicializarSensores() {
  if (!configurarTCA9548A(CANAL_BMP180_TOPO)) {
    Serial.println("TCA9548A (0x70) nao detectado.");
  } else if (!i2cDispositivoPresente(0x77)) {
    Serial.println("BMP180 (topo) nao respondeu no barramento - pulando inicializacao.");
  } else if (!(bmp180TopoDisponivel = bmp180Topo.begin())) {
    Serial.println("BMP180 (topo) nao detectado.");
  }
  if (bmp180TopoDisponivel) {
    bmp180Topo.resetToDefaults();
    bmp180Topo.setSamplingMode(BMP180MI::MODE_UHR);
  }

  inicializarSensoresAHT();
}

// ==============================================================================
// Inicializa os 7 sensores AHT20 (um por canal do multiplexador TCA9548A),
// mostrando no LCD se cada um respondeu ou deu erro
void inicializarSensoresAHT() {
  for (int i = 0; i < 7; i++) {
    if (!configurarTCA9548A(i)) {
      Serial.print("TCA9548A nao respondeu no canal ");
      Serial.println(i);
      continue;
    }
    if (!i2cDispositivoPresente(0x38)) {
      Serial.print("AHT20 nao respondeu no canal ");
      Serial.println(i);
      sensoresAHTDisponiveis[i] = false;
      continue;
    }
    sensoresAHTDisponiveis[i] = sensoresAHT[i].begin();
    if (sensoresAHTDisponiveis[i]) {
      lcd.setCursor(0, 0);
      lcd.print(nomesSensoresAHT[i]);          // mostra todos os sensores no array
      lcd.print(" OK");
    } else {
      lcd.setCursor(0, 0);
      lcd.print("Erro: ");
      lcd.print(nomesSensoresAHT[i]);      // sensores
    }
    delay(300);
  }
}

// Função inicializaWatchDOg
void inicializarWatchdog() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  esp_task_wdt_deinit();
  if (esp_task_wdt_init(&configuracaoWatchdog) != ESP_OK) {
    // Fallback: essa build especifica do core 3.x pode nao aceitar o struct -
    // tenta a assinatura antiga (timeout em segundos, bool panico)
    esp_task_wdt_init(TEMPO_LIMITE_WATCHDOG / 1000, true);
  }
#else
  esp_task_wdt_init(TEMPO_LIMITE_WATCHDOG / 1000, true);
#endif
  esp_task_wdt_add(NULL);
}
// ==============================================================================

  // Le temperatura, pressao e calcula direcao do fluxo do BMP180 (topo).
// Retorna false se o sensor nao respondeu (ex: ainda nao conectado durante testes) -
// isso NAO interrompe a leitura dos outros sensores AHT20, que continua normalmente.
  bool lerBMP180Topo() {
  unsigned long tempoInicio = millis();
  unsigned long tempoLimite = 5000;

  if (!bmp180TopoDisponivel || !configurarTCA9548A(CANAL_BMP180_TOPO)) {
    return false;
  }

  if (!bmp180Topo.measureTemperature()) {
    Serial.println("Erro ao medir temperatura do BMP180 (topo) - sensor pode nao estar conectado ainda");
    return false;
  }
  do { delay(100); } while (!bmp180Topo.hasValue() && (millis() - tempoInicio < tempoLimite));
  if (!bmp180Topo.hasValue()) {
    Serial.println("Timeout ao esperar temperatura do BMP180 (topo)");
    return false;
  }
  temperaturaEntrada = bmp180Topo.getTemperature();

  if (!bmp180Topo.measurePressure()) {
    Serial.println("Erro ao medir pressao do BMP180 (topo)");
    return false;
  }
  do { delay(100); } while (!bmp180Topo.hasValue() && (millis() - tempoInicio < tempoLimite));
  if (!bmp180Topo.hasValue()) {
    Serial.println("Timeout ao esperar pressao do BMP180 (topo)");
    return false;
  }
  pressaoTopoAnterior = pressaoTopoAtual;  // guarda a leitura antiga antes de sobrescrever, pra poder comparar
  pressaoTopoAtual = bmp180Topo.getPressure();

  float deltaPressaoTopo = pressaoTopoAtual - pressaoTopoAnterior;  // variacao de pressao entre as duas ultimas leituras
  if (deltaPressaoTopo > LIMIAR_DIRECAO_PA) {  // pressao subindo -> ar sendo empurrado pra fora no topo
    direcaoFluxo = FLUXO_EXPELINDO;
  } else if (deltaPressaoTopo < -LIMIAR_DIRECAO_PA) {  // pressao caindo -> ar sendo puxado pelo topo
    direcaoFluxo = FLUXO_PUXANDO;
  } else {  // variacao pequena, dentro do limiar -> considera estavel
    direcaoFluxo = FLUXO_ESTAVEL;
  }
  return true;
}
// Função de ler os sensores
void lerSensores() {

  lerBMP180Topo();  // se falhar, so loga erro no Serial - a leitura dos AHT20 abaixo roda de qualquer forma

  // ---- 7 sensores AHT20 via multiplexador (canais 0-6) ----
  for (int i = 0; i < 7; i++) {
    if (sensoresAHTDisponiveis[i] && configurarTCA9548A(i) && sensoresAHT[i].available()) {
      temperaturaAHT[i] = sensoresAHT[i].getTemperature();
      umidadeAHT[i] = sensoresAHT[i].getHumidity();
    }
  }

  // ---- Log serial ----
  Serial.println();
  Serial.println("========== LEITURAS ==========");
  for (int i = 0; i < 7; i++) {
    Serial.print(nomesSensoresAHT[i]);
    Serial.print(" -> T: "); Serial.print(temperaturaAHT[i], 2);
    Serial.print(" C | U: "); Serial.print(umidadeAHT[i], 2);
    Serial.println(" %");
  }

  Serial.print("BMP180 topo -> T: "); Serial.print(temperaturaEntrada, 2);
  Serial.print(" C | P: "); Serial.print(pressaoTopoAtual, 2);
  Serial.print(" Pa | Delta: "); Serial.print(pressaoTopoAtual - pressaoTopoAnterior, 2);
  Serial.print(" | Direcao: "); Serial.println(direcaoFluxoTexto());

  Serial.print("Travado: ");
  Serial.println(sistemaTravado ? "SIM" : "NAO");
  for (int i = 0; i < 3; i++) {
    Serial.print("Estagio "); Serial.print(i + 1);
    Serial.print(": "); Serial.println(estagioAquecendo[i] ? "AQUECENDO" : "OCIOSO");
  }

  Serial.print("Auxiliar (frio/caixa): ");
  Serial.println(auxiliarAquecendo ? "AQUECENDO" : "OCIOSO");
}


// ==============================================================================
// Impressão de dados no LCD

// Layout em 3 menus (troca com os botões B1/B2/B3):
//   Menu 1 - Umidade topo/base de cada estagio + status (*) + caixa de controle
//   Menu 2 - Temperatura ambiente, pressao e direcao do fluxo (sensor do topo) + travado
//   Menu 3 - Status geral: estagios ON/OFF, auxiliar, ventilador
// Os numeros usam largura fixa (dtostrf) para nao deixar "sobra" de digito na tela
// quando um valor menor substitui um valor maior no mesmo lugar.
void exibirLCD() {
  // Atualiza o LCD a cada 1 segundo
  if (((millis() - tempoImpressaoLCD) > 1000) || ativado == 1) {  // so redesenha 1x/segundo (ou se forcado por "ativado")
    tempoImpressaoLCD = millis();  // Atualiza o tempo da última impressão no LCD
    ativado = 0;                   // Reseta o estado de ativação para próximo ciclo

    // Verifica se o menu atual mudou em relação ao anterior
    if (menu != menuAnt)  // troca de tela -> limpa antes de redesenhar pra nao misturar texto da tela antiga
      lcd.clear();  // Limpa o LCD se o menu mudou

    char textoLCD[8];  // buffer auxiliar para formatar numeros com largura fixa

    // ---- Menu 1: umidade topo/base de cada estagio (com status) + caixa ----
    if (menu == 1) {
      menuAnt = menu;

      lcd.setCursor(0, 0);
      lcd.print("E1 T:");
      dtostrf(umidadeAHT[sens_cima_est1], 3, 0, textoLCD);  lcd.print(textoLCD);
      lcd.print("% B:");
      dtostrf(umidadeAHT[sens_baixo_est1], 3, 0, textoLCD); lcd.print(textoLCD);
      lcd.print("% ");
      lcd.print(estagioAquecendo[0] ? '*' : ' ');  // '*' = resistencia ligada nesse estagio

      lcd.setCursor(0, 1);
      lcd.print("E2 T:");
      dtostrf(umidadeAHT[sens_cima_est2], 3, 0, textoLCD);  lcd.print(textoLCD);
      lcd.print("% B:");
      dtostrf(umidadeAHT[sens_baixo_est2], 3, 0, textoLCD); lcd.print(textoLCD);
      lcd.print("% ");
      lcd.print(estagioAquecendo[1] ? '*' : ' ');

      lcd.setCursor(0, 2);
      lcd.print("E3 T:");
      dtostrf(umidadeAHT[sens_cima_est3], 3, 0, textoLCD);  lcd.print(textoLCD);
      lcd.print("% B:");
      dtostrf(umidadeAHT[sens_baixo_est3], 3, 0, textoLCD); lcd.print(textoLCD);
      lcd.print("% ");
      lcd.print(estagioAquecendo[2] ? '*' : ' ');

      lcd.setCursor(0, 3);
      lcd.print("Cx U:");
      dtostrf(umidadeCaixa, 3, 0, textoLCD); lcd.print(textoLCD);
      lcd.print("% T:");
      dtostrf(temperaturaCaixa, 4, 1, textoLCD); lcd.print(textoLCD);
      lcd.print((char)223);  // simbolo de grau nativo do LCD
      lcd.print("C");

    // ---- Menu 2: ambiente, pressao e direcao do fluxo (sensor BMP180 do topo) ----
    } else if (menu == 2) {
      menuAnt = menu;

      lcd.setCursor(0, 0);
      lcd.print("Amb T:");
      dtostrf(temperaturaEntrada, 5, 1, textoLCD); lcd.print(textoLCD);
      lcd.print((char)223);
      lcd.print("C");

      lcd.setCursor(0, 1);
      lcd.print("Pressao:");
      dtostrf(pressaoTopoAtual, 7, 0, textoLCD); lcd.print(textoLCD);
      lcd.print("Pa");

      // Direcao do fluxo, com padding manual pra largura fixa (evita sobra de texto
      // quando troca, por exemplo, de "EXPELINDO" pra "ESTAVEL")
      lcd.setCursor(0, 2);
      lcd.print("Fluxo: ");
      switch (direcaoFluxo) {
        case FLUXO_EXPELINDO: lcd.print("EXPELINDO"); break;
        case FLUXO_PUXANDO:   lcd.print("PUXANDO  "); break;
        default:              lcd.print("ESTAVEL  "); break;
      }

      lcd.setCursor(0, 3);
      lcd.print("Travado: ");
      lcd.print(sistemaTravado ? "SIM" : "NAO");

    // ---- Menu 3: status geral (estagios, auxiliar, ventilador) ----
    } else if (menu == 3) {
      menuAnt = menu;

      lcd.setCursor(0, 0);
      lcd.print("E1:"); lcd.print(estagioAquecendo[0] ? "ON " : "OFF");
      lcd.print(" E2:"); lcd.print(estagioAquecendo[1] ? "ON " : "OFF");
      lcd.print(" E3:"); lcd.print(estagioAquecendo[2] ? "ON " : "OFF");

      lcd.setCursor(0, 1);
      lcd.print("Auxiliar: ");
      lcd.print(auxiliarAquecendo ? "ON " : "OFF");

      // Ventilador nao tem variavel global propria - calculado aqui so pra exibicao,
      // usando a MESMA condicao ja aplicada em controlarEstagios() (nao duplica a logica de controle,
      // so espelha o resultado dela pra mostrar na tela)
      bool ventiladorLigado = (estagioAquecendo[0] || estagioAquecendo[1] || estagioAquecendo[2]) && !sistemaTravado;
      lcd.setCursor(0, 2);
      lcd.print("Ventilador: ");
      lcd.print(ventiladorLigado ? "ON " : "OFF");
    }
  }
}
// ==============================================================================
// Função para salvar os dados na NVS
void salvarDados(const char* chave, bool valorParaSalvar) {
  // Exemplo de uso: saveData("nomeDoArquivoParaSerSalvo", variavelASerSalva)

  nvs_handle_t identificadorNVS;                                          // Handle para a NVS
  esp_err_t resultado = nvs_open("storage", NVS_READWRITE, &identificadorNVS);  // Abre a NVS

  if (resultado == ESP_OK) {                              // Verifica se a NVS foi aberta com sucesso
    nvs_set_u8(identificadorNVS, chave, valorParaSalvar);  // Salva o estado da resistência como um valor unsigned 8-bit
    // Para salvar valores inteiros, descomente a linha abaixo:
    // nvs_set_i32(identificadorNVS, INT_KEY5, int5);
    nvs_commit(identificadorNVS);  // Grava as mudanças na NVS
    nvs_close(identificadorNVS);   // Fecha o handle
  } else {
    Serial.println("Erro ao abrir a NVS.");  // Mensagem de erro se a NVS não puder ser aberta
  }
}
// ==============================================================================
// Função para ler os dados da NVS
bool lerDados(const char* chave) {                                // Exemplo de uso: readData("nomeDoArquivoSalvo")
  nvs_handle_t identificadorNVS;                                         // Handle para a NVS
  esp_err_t resultado = nvs_open("storage", NVS_READONLY, &identificadorNVS);  // Abre a NVS em modo de leitura

  if (resultado == ESP_OK) {
    uint8_t valorLido;  // Variável para armazenar o valor lido

    // Tenta ler o valor armazenado na NVS
    esp_err_t resultadoLeitura = nvs_get_u8(identificadorNVS, chave, &valorLido);
    nvs_close(identificadorNVS);  // Fecha o handle

    if (resultadoLeitura == ESP_OK) {  // Verifica se a leitura foi bem-sucedida
      return valorLido;       // Retorna o valor lido (como booleano)
    } else {
      Serial.println("Erro ao ler os dados.");  // Mensagem de erro se a leitura falhar
    }
  } else {
    Serial.println("Erro ao abrir a NVS.");  // Mensagem de erro se a NVS não puder ser aberta
  }
  return false;  // Retorna false em caso de erro
}
