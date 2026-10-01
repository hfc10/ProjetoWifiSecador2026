#include <Arduino.h>
#include "webServerSecador.h"

#if USAR_WIFI
// Função para mostrar a página de upload
void webHandleRoot() {
  server.send(200, "text/html", uploadPage);
}

// Função para realizar a atualização do firmware
void webHandleUpdate() {
  HTTPUpload& upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    Serial.printf("Iniciando atualização: %s\n", upload.filename.c_str());
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {  // Inicia a atualização
      Update.printError(Serial);
    }
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    // Grava os dados recebidos no flash
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
      Update.printError(Serial);
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    // Finaliza a atualização
    if (Update.end(true)) {
      Serial.printf("Atualização completa: %u bytes\n", upload.totalSize);
      server.send(200, "text/plain", "Atualização concluída! Reiniciando...");
      delay(500);
      ESP.restart();
    } else {
      Update.printError(Serial);
      server.send(500, "text/plain", "Falha na atualização");
    }
  }
}

// Configuração do erro 404
void webHandleNotFound() {
  server.send(404, "text/plain", "Página não encontrada");
}

// Retorna os dados atuais do sistema em formato JSON (rota /dados)
// Monta a string manualmente com snprintf (sem lib externa, sem alocação dinâmica -
// evita fragmentação de heap num equipamento que roda ligado por semanas/meses)
void webHandleDados() {
  char json[900];
  int pos = 0;

  pos += snprintf(json + pos, sizeof(json) - pos, "{");

  // Array com os 5 sensores AHT20 (topo/base de cada estagio + caixa)
  pos += snprintf(json + pos, sizeof(json) - pos, "\"sensores\":[");
  for (int i = 0; i < 5; i++) {
    pos += snprintf(json + pos, sizeof(json) - pos,
      "%s{\"nome\":\"%s\",\"umidade\":%.1f,\"temperatura\":%.1f}",
      (i > 0 ? "," : ""), nomesSensoresAHT[i], umidadeAHT[i], temperaturaAHT[i]);
  }
  pos += snprintf(json + pos, sizeof(json) - pos, "],");

  // Sensor do topo (BMP180): temperatura ambiente, pressao e direcao do fluxo
  pos += snprintf(json + pos, sizeof(json) - pos,
    "\"ambiente\":{\"temperatura\":%.1f,\"pressao\":%.1f,\"direcaoFluxo\":\"%s\"},",
    temperaturaEntrada, pressaoTopoAtual, direcaoFluxoTexto());

  // Status de cada estagio (true = aquecendo)
  pos += snprintf(json + pos, sizeof(json) - pos,
    "\"estagios\":[%s,%s],",
    estagioAquecendo[0] ? "true" : "false",
    estagioAquecendo[1] ? "true" : "false");

  // Travado (chave de fluxo) e auxiliar (frio extremo/caixa)
  pos += snprintf(json + pos, sizeof(json) - pos,
    "\"travado\":%s,\"auxiliar\":%s",
    sistemaTravado ? "true" : "false",
    auxiliarAquecendo ? "true" : "false");

  pos += snprintf(json + pos, sizeof(json) - pos, "}");

  server.send(200, "application/json", json);
}

void webSetupServer() {
  Serial.begin(115200);

  WiFi.config(ip, gateway, subnet, primaryDNS, secondaryDNS);
  WiFi.begin(ssid, password);

  unsigned long inicioTentativa = millis();
  const unsigned long TIMEOUT_WIFI_MS = 10000; // 10s - ajuste como preferir

  while (WiFi.status() != WL_CONNECTED &&
(millis() - inicioTentativa) < TIMEOUT_WIFI_MS) {

  delay(500);
  Serial.print(".");
}

if (WiFi.status() == WL_CONNECTED) {
  wifiConectado = true;
  Serial.println("\nConectado ao Wi-Fi");
  Serial.print("Endereco IP: ");
  Serial.println(WiFi.localIP());

  server.on("/", HTTP_GET, webHandleRoot);
  server.on("/dados", HTTP_GET, webHandleDados);
  server.on(
    "/update", HTTP_POST, []() {
     server.send(200, "text/plain",
       (Update.hasError()) ? "Falha na atualizacao" : "Atualizacao bem-sucedida");
    },
    webHandleUpdate);
  server.onNotFound(webHandleNotFound);

  // Inicia o servidor
  server.begin();
  Serial.println("Servidor OTA iniciado");
} else {
  Serial.println("\nWiFi nao conectou - sistema segue funcionando sem rede nesse boot");
  }
}
#endif
