#include "webServerSecador.h"
#include "credenciais.h"  // include/credenciais.h (fora do Git)

// Configurações da rede Wi-Fi
const char* ssid = WIFI_FUTURO_SSID;       // Nome da rede WiFi
const char* password = WIFI_FUTURO_SENHA;  // Senha da rede WiFi

bool wifiConectado = false; // true so se o WiFi conectou dentro do tempo limite

// Configuração do endereço IP estático
IPAddress ip(192, 168, 0, 231);              // Endereço IP do ESP32
IPAddress gateway(192, 168, 0, 1);           // Gateway da rede
IPAddress subnet(255, 255, 255, 0);          // Máscara de sub-rede
IPAddress primaryDNS(201, 159, 154, 3);      // Servidor DNS primário
IPAddress secondaryDNS(201, 159, 155, 134);  // Servidor DNS secundário

// Criação do servidor web na porta 80
WebServer server(80);

// Página HTML para upload
const char* uploadPage =

  "<!DOCTYPE html>"
  "<html lang='pt'>"
  "<head>"
  "  <meta charset='UTF-8'>"
  "  <meta name='viewport' content='width=device-width, initial-scale=1.0'>"
  "  <title>ESP32 OTA Update</title>"
  "  <style>"
  "    body { font-family: Arial, sans-serif; background-color: #f4f4f4; display: flex; align-items: center; justify-content: center; height: 100vh; margin: 0; }"
  "    .container { background-color: #ffffff; padding: 20px; border-radius: 8px; box-shadow: 0 4px 8px rgba(0, 0, 0, 0.2); text-align: center; width: 300px; }"
  "    h1 { color: #333333; }"
  "    p { color: #666666; font-size: 14px; }"
  "    input[type='file'] { padding: 8px; margin: 10px 0; width: 100%; }"
  "    input[type='submit'] { background-color: #007bff; color: #ffffff; padding: 10px; border: none; border-radius: 4px; cursor: pointer; font-size: 16px; width: 100%; }"
  "    input[type='submit']:hover { background-color: #0056b3; }"
  "  </style>"
  "</head>"
  "<body>"
  "  <div class='container'>"
  "    <h1>Atualização de Firmware</h1>"
  "    <p>Escolha o arquivo .bin e clique em 'Atualizar Firmware' para iniciar o processo de atualização OTA.</p>"
  "    <form method='POST' action='/update' enctype='multipart/form-data'>"
  "      <input type='file' name='firmware' accept='.bin'>"
  "      <input type='submit' value='Atualizar Firmware'>"
  "    </form>"
  "  </div>"
  "</body>"
  "</html>";
