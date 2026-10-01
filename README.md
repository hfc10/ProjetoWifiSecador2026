# Secador de Ar Regenerativo — Firmware ESP32-S3

Firmware do secador de ar de sílica-gel que protege o respiro de transformadores de potência.
O ESP32-S3 lê os sensores de umidade e temperatura, decide quando regenerar (aquecer) cada estágio
de sílica e aciona resistências e ventoinhas por TRIAC.

O repositório tem dois firmwares principais:

| Firmware | Arquivo | Para que serve |
|---|---|---|
| **WiFi** | `src/teste_wifi_servidor.cpp` | Bancada de testes: página web com leituras, gráficos, modo manual e OTA |
| **Modbus RTU** | `src/teste_modbus.cpp` | Produto final: o ESP vira escravo Modbus via RS-485 e um painel (IHM) no PC lê e comanda |

Os dois rodam a **mesma lógica de controle**. O controle mora no ESP: se a página ou o painel
saírem do ar, o secador continua funcionando sozinho.

---

## Como funciona

O transformador "respira" pelo secador: quando o óleo esquenta e dilata, ele expele ar; quando
esfria e contrai, ele puxa ar de fora. O ar que entra passa pela sílica-gel, que retém a umidade
antes de ela chegar ao óleo. Com o tempo a sílica satura, e o secador a regenera aquecendo-a para
expulsar a água.

São **dois estágios** de sílica, cada um com um sensor no **topo** e outro na **base**. A umidade
entra pelo topo e a regeneração empurra a umidade para baixo, então:

- **Liga** um estágio quando a umidade do **topo** passa do limite de ligar (padrão 55%).
- **Desliga** quando a umidade da **base** cai abaixo do limite de desligar (padrão 8%):
  o estágio inteiro secou e o ciclo está concluído.

### Regras do modo automático

- **Um estágio por vez.** Enquanto um regenera, o outro fica de guarda.
- **Corte por temperatura** do topo: desliga acima de **85 °C** e só religa com **70 °C** ou menos
  (histerese de 15 °C para não ficar ligando e desligando).
- **Ciclo pendente com prioridade.** Todo ciclo iniciado fica marcado como "em andamento" na
  memória (NVS) até a base secar. Se o estágio for cortado por temperatura ou o ESP reiniciar no
  meio (queda de energia), ele retoma esse ciclo com prioridade assim que esfriar, sem depender
  do topo estar úmido de novo. Se a base já estiver seca ao voltar, o ciclo é dado por encerrado.
- **Ventoinha 1** fica ligada enquanto algum estágio aquece ou tem ciclo pendente (ajuda a esfriar
  e mantém o ar circulando durante a espera).
- **Sempre inicia desligado** e em automático após qualquer reboot. Resistência nunca volta
  ligada sozinha no boot; só a marca de ciclo pendente sobrevive.

### Modo manual (bancada)

Liga e desliga resistências e ventoinhas direto pela página ou pelo painel. O corte por
temperatura continua valendo, com limite ajustável (padrão 50 °C). O manual não grava nada:
entrar nele descarta os ciclos pendentes, para o automático não retomar um ciclo antigo depois.

### Segurança

- **Watchdog de 30 s:** se o `loop()` travar, o ESP reinicia sozinho.
- **Só WiFi:** 60 s sem WiFi desliga resistências e ventoinhas (ninguém supervisionando).
  No Modbus é diferente de propósito: se o mestre sumir, o ESP continua secando.
- Sensor sem leitura válida nunca liga nada.

---

## Hardware

Placa **"Secador de Ar V1.1"** com ESP32-S3 (`esp32-s3-devkitc-1` no PlatformIO).

| Função | Pino / endereço |
|---|---|
| Resistência Estágio 1 (TRIAC RES_1) | GPIO14 |
| Resistência Estágio 2 (TRIAC RES_2) | GPIO13 |
| Ventoinha 1 (TRIAC RES_3) | GPIO12 |
| Ventoinha 2 (TRIAC RES_4, só para teste) | GPIO11 |
| LEDs dos estágios | GPIO15, GPIO16 |
| I2C (SDA / SCL) | GPIO40 / GPIO41 |
| RS-485 (MAX485, direção automática) RX2 / TX2 | GPIO38 / GPIO37 |
| Multiplexador I2C TCA9548A | 0x70 |
| SHT40 (umidade e temperatura) | 0x44 |
| BMP180 (pressão e temperatura ambiente) | 0x77 |
| LCD 20x4 I2C | 0x27 |

Os quatro SHT40 têm o mesmo endereço, então cada um fica num canal do multiplexador:

| Sensor | Canal |
|---|---|
| Estágio 1 — Topo | 1 |
| Estágio 1 — Base | 0 |
| Estágio 2 — Topo | 4 |
| Estágio 2 — Base | 2 |
| Ambiente (BMP180) | 6 |

O canal 3 não está disponível na placa.

---

## Como compilar e gravar

**Requisitos:** [VS Code](https://code.visualstudio.com/) com a extensão
[PlatformIO](https://platformio.org/install/ide?install=vscode). As bibliotecas são baixadas ou
já estão na pasta `lib/`.

1. **Credenciais de WiFi.** Copie `include/credenciais_exemplo.h` para `include/credenciais.h` e
   preencha a rede e a senha. Esse arquivo fica fora do Git.
2. **Porta USB.** Ajuste `upload_port` no `platformio.ini` (hoje `COM4`).
3. **Compile e grave** o ambiente desejado:

```
pio run -e teste_wifi_servidor -t upload
pio run -e teste_modbus -t upload
```

Cada teste é um *environment* próprio no `platformio.ini`, compilando só o seu arquivo. Assim os
vários testes convivem em `src/` sem conflito de `setup()`/`loop()`.

| Environment | Arquivo | O que faz |
|---|---|---|
| `teste_resistencias` | `src/Secador_Novo2026.cpp` | Liga cada resistência por botão, para validar fiação |
| `teste_sensores` | `src/teste_sensores.cpp` | Varre os canais do multiplexador e mostra as leituras |
| `teste_ventoinhas` | `src/teste_ventoinhas.cpp` | Liga cada ventoinha por botão |
| `teste_wifi_servidor` | `src/teste_wifi_servidor.cpp` | Firmware WiFi completo (controle + página web) |
| `teste_modbus` | `src/teste_modbus.cpp` | Firmware Modbus completo (controle + escravo RS-485) |

Monitor serial: `pio device monitor` (115200 baud).

---

## Firmware WiFi

Conecta na rede de `credenciais.h` e mostra o IP no Monitor Serial. Abra esse IP no navegador.

A página mostra o esquema do tubo com a saturação de cada estágio, os cartões dos sensores com
gráficos de tendência, o estado de cada estágio ("aquecendo há 4min", "ciclo pendente"), o modo,
as ventoinhas e os limites (editáveis no manual).

| Rota | Função |
|---|---|
| `GET /` | Página principal |
| `GET /dados` | Estado atual em JSON |
| `GET /historico` | Séries dos gráficos em JSON |
| `GET /historico.csv` | Histórico em CSV (planilha) |
| `GET /modo?manual=0\|1` | Troca automático / manual |
| `GET /saida-manual?id=N&estado=0\|1` | Liga/desliga: 0 = Estágio 1, 1 = Estágio 2, 2 = Ventoinha 1, 3 = Ventoinha 2 |
| `GET /limites?ligar=&desligar=&temp=` | Grava os limites (salvos na NVS) |
| `GET /update` | Atualização de firmware por OTA |

**Histórico:** um ponto a cada 2 min, 12 h em memória RAM. Não há cartão SD nem relógio, então
some num reboot e o CSV traz o tempo relativo ("segundos atrás").

**Atualização sem cabo (OTA):** compile com `pio run -e teste_wifi_servidor`, abra `http://<ip>/update`
e envie o arquivo `.pio/build/teste_wifi_servidor/firmware.bin`.

---

## Firmware Modbus RTU

Escravo **ID 1**, **9600 baud, 8N1**, pela UART2 ligada ao MAX485. O LED1 pisca a cada byte
recebido, para confirmar comunicação sem o Monitor Serial.

### Mapa de registradores

**Input Registers (função 0x04)** — valores ×10 (ex.: 234 = 23,4)

| Endereço | Conteúdo |
|---|---|
| 0–1 | Estágio 1 Topo: temperatura, umidade |
| 2–3 | Estágio 1 Base: temperatura, umidade |
| 4–5 | Estágio 2 Topo: temperatura, umidade |
| 6–7 | Estágio 2 Base: temperatura, umidade |
| 8–9 | Ambiente BMP180: temperatura, pressão (hPa) |
| 10–11 | SHT25 no canal 0 (temporário, de teste) |
| 12–13 | Segundos desde que o Estágio 1 / 2 ligou ou desligou (sem ×10; satura em 65535) |

**Discrete Inputs (função 0x02)**

| Endereço | Conteúdo |
|---|---|
| 0–5 | Sensor respondeu na última leitura (mesma ordem acima) |
| 6 | Alarme: falha de funcionamento (nenhum sensor respondendo) |
| 7 | Alarme: sílica no fim da vida útil (limite de ciclos) |
| 8–9 | Estágio 1 / 2 com ciclo pendente |

**Holding Registers (funções 0x03 / 0x06 / 0x10)**

| Endereço | Conteúdo |
|---|---|
| 0 | Valor fixo 1234 (teste de comunicação) |
| 1 | Ciclos de regeneração concluídos (persistente) |
| 2 | Limite de umidade do topo para ligar (×10) |
| 3 | Limite de umidade da base para desligar (×10) |
| 4 | Temperatura máxima do modo manual (×10) |

Os limites 2–4 devem ser gravados juntos, num quadro só (função 0x10). O ESP valida o conjunto e
recusa valores inválidos, devolvendo os anteriores.

**Coils (funções 0x01 / 0x05)**

| Endereço | Conteúdo |
|---|---|
| 0 | Ventoinha 1 |
| 1 | Resistência Estágio 1 |
| 2 | Resistência Estágio 2 |
| 3 | Ventoinha 2 (só manual) |
| 4 | Modo manual (1) / automático (0) |

No automático o ESP decide e sobrescreve os coils 0–3 (escrita do mestre é ignorada). No manual
eles comandam as saídas.

O painel que lê esses registradores (Python + Flask, com gráficos, eventos e modo quiosque para
tela de IHM) fica em [hfc10/Software_modbus](https://github.com/hfc10/Software_modbus).

---

## Estrutura do repositório

```
src/                 firmwares e testes (um environment do PlatformIO para cada)
include/             credenciais.h (fora do Git) e o modelo credenciais_exemplo.h
lib/                 bibliotecas locais: BMP180, LiquidCrystal_I2C, AHT20
modulos_futuros/     código WiFi antigo guardado para reaproveitar (não é compilado)
backup_arquivos/     versões antigas do programa completo (referência, não é compilado)
platformio.ini       placa, environments e dependências
Explicacao_Codigo_Secador.pdf   guia comentado do programa antigo
```

## Bibliotecas

- **Núcleo Arduino-ESP32:** `Wire`, `WiFi`, `WebServer`, `Update`, `Preferences` (NVS) e
  `esp_task_wdt` (watchdog).
- **Locais, em `lib/`:** `BMP180` (BMP180I2C), `LiquidCrystal_I2C` e `AHT20` (só no
  `teste_sensores`).
- **Baixada pelo PlatformIO:** [`emelianov/modbus-esp8266`](https://github.com/emelianov/modbus-esp8266)
  (Modbus RTU, só no `teste_modbus`).
- O SHT40 é lido direto por I2C (comando 0xFD), sem biblioteca.

## Pendências

- **Trava por fluxo do transformador.** O secador não deve regenerar enquanto o transformador
  puxa ar (o ar entraria úmido pela sílica que está soltando água). Existia no programa antigo
  com uma chave de fluxo; ainda não foi portada. Precisa ter precedência sobre a regra da
  ventoinha. Em estudo: chave mecânica ou sensor de pressão diferencial, que também diria o
  sentido do fluxo e permitiria travar só quando o transformador puxa.
- Número real de ciclos suportados pela sílica (hoje um valor de exemplo de 5000), conforme a
  ficha técnica do fornecedor.
