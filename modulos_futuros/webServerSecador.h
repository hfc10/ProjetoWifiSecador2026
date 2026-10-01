#pragma once

#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>

#define USAR_WIFI 0

// Configurações da rede Wi-Fi
extern const char* ssid;      // Nome da rede WiFi
extern const char* password;  // Senha da rede WiFi

extern bool wifiConectado;  // true so se o WiFi conectou dentro do tempo limite

// Configuração do endereço IP estático
extern IPAddress ip;           // Endereço IP do ESP32
extern IPAddress gateway;      // Gateway da rede
extern IPAddress subnet;       // Máscara de sub-rede
extern IPAddress primaryDNS;   // Servidor DNS primário
extern IPAddress secondaryDNS; // Servidor DNS secundário

// Servidor web na porta 80
extern WebServer server;

// Página HTML para upload
extern const char* uploadPage;

extern const char* nomesSensoresAHT[5];
extern float umidadeAHT[5];
extern float temperaturaAHT[5];
extern bool estagioAquecendo[2];
extern bool sistemaTravado;
extern bool auxiliarAquecendo;
extern float temperaturaEntrada;
extern float pressaoTopoAtual;

const char* direcaoFluxoTexto();
