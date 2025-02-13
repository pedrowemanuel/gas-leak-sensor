/***
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Complete instructions at https://RandomNerdTutorials.com/esp32-wi-fi-manager-asyncwebserver/
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
***/
#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include "SPIFFS.h"
#include <WebSocketsServer.h>
#include <UniversalTelegramBot.h>
#include <WiFiClientSecure.h>

#define SSID_ESP "GAS-LEAK-SENSOR:200"

#define LED_PIN 2
#define BUZZER_PIN 23
#define A0_PIN A0
#define D0_PIN 25
#define SENSOR_THRESHOLD 1000

// Initialize Telegram BOT
#define BOTtoken "7745094490:AAFtoYY5l-jc7JPoV7PenqM1yQKBMYkqioM" // your Bot Token (Get from Botfather)

// Use @myidbot to find out the chat ID of an individual or a group
// Also note that you need to click "start" on a bot before it can
// message you
// #define CHAT_ID "5353701390"
#define CHAT_ID "5135357770"

WiFiClientSecure client;
UniversalTelegramBot bot(BOTtoken, client);

// Checks for new messages every 1 second.
int botRequestDelay = 1000;
unsigned long lastTimeBotRan;

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

WebSocketsServer webSocket(81);

// Search for parameter in HTTP POST request
const char *PARAM_INPUT_1 = "ssid";
const char *PARAM_INPUT_2 = "pass";
const char *PARAM_INPUT_3 = "ip";
const char *PARAM_INPUT_4 = "gateway";
const char *PARAM_INPUT_5 = "websocketIP";

// Variables to save values from HTML form
String ssid;
String pass;
String ip;
String gateway;
String websocketIP;

// File paths to save input values permanently
const char *ssidPath = "/ssid.txt";
const char *passPath = "/pass.txt";
const char *ipPath = "/ip.txt";
const char *gatewayPath = "/gateway.txt";
const char *websocketIPPath = "/websocketip.txt";

IPAddress localIP;
// IPAddress localIP(192, 168, 1, 201); // hardcoded

// Set your Gateway IP address
IPAddress localGateway;
// IPAddress localGateway(192, 168, 1, 1); //hardcoded
IPAddress subnet(255, 255, 0, 0);

// Timer variables
unsigned long previousMillis = 0;
const long interval = 10000; // interval to wait for Wi-Fi connection (milliseconds)

String alarmState;

// Initialize SPIFFS
void initSPIFFS()
{
  if (!SPIFFS.begin(true))
  {
    Serial.println("An error has occurred while mounting SPIFFS");
    return;
  }
  Serial.println("SPIFFS mounted successfully");
}

// Read File from SPIFFS
String readFile(fs::FS &fs, const char *path)
{
  Serial.printf("Reading file: %s\r\n", path);

  File file = fs.open(path);
  if (!file || file.isDirectory())
  {
    Serial.println("- failed to open file for reading");
    return String();
  }

  String fileContent;
  while (file.available())
  {
    fileContent = file.readStringUntil('\n');
    break;
  }
  return fileContent;
}

// Write file to SPIFFS
void writeFile(fs::FS &fs, const char *path, const char *message)
{
  Serial.printf("Writing file: %s\r\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file)
  {
    Serial.println("- failed to open file for writing");
    return;
  }
  if (file.print(message))
  {
    Serial.println("- file written");
  }
  else
  {
    Serial.println("- write failed");
  }
}

// Initialize WiFi
bool initWiFi()
{
  if (ssid == "" || ip == "" || websocketIP == "")
  {
    Serial.println("Undefined SSID or IP address.");
    return false;
  }

  WiFi.mode(WIFI_STA);
  localIP.fromString(ip.c_str());
  localGateway.fromString(gateway.c_str());

  // if (!WiFi.config(localIP, localGateway, subnet))
  // {
  //   Serial.println("STA Failed to configure");
  //   return false;
  // }
  WiFi.begin(ssid.c_str(), pass.c_str());
  Serial.println("Connecting to WiFi...");

  unsigned long currentMillis = millis();
  previousMillis = currentMillis;

  while (WiFi.status() != WL_CONNECTED)
  {
    currentMillis = millis();
    if (currentMillis - previousMillis >= interval)
    {
      Serial.println("Failed to connect.");
      return false;
    }
  }

  Serial.println("Connected to WiFi!");

  // Printar IP e MAC
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());

  Serial.print("gateway: ");
  Serial.println(WiFi.gatewayIP());

  return true;
}

// Replaces placeholder with LED state value
String processor(const String &var)
{
  if (var == "STATE")
  {
    if (digitalRead(LED_PIN))
    {
      alarmState = "ON";
    }
    else
    {
      alarmState = "OFF";
    }
    return alarmState;
  }
  return String();
}

void alarmON()
{
  digitalWrite(LED_PIN, HIGH);
  digitalWrite(BUZZER_PIN, HIGH);

  delay(5000);
}

void alarmOFF()
{
  digitalWrite(LED_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
}

void reset()
{
  writeFile(SPIFFS, ssidPath, "");
  writeFile(SPIFFS, passPath, "");
  writeFile(SPIFFS, ipPath, "");
  writeFile(SPIFFS, websocketIPPath, "");

  esp_restart();
}

TaskHandle_t ledTaskHandle; // Variável para armazenar a task

// Função da task para acender o LED por 5 segundos
void ledTask(void *parameter)
{

  digitalWrite(LED_PIN, HIGH);          // Acende o LED
  vTaskDelay(500 / portTICK_PERIOD_MS); // Espera
  digitalWrite(LED_PIN, LOW);           // Apaga o LED
  vTaskDelay(500 / portTICK_PERIOD_MS); // Espera
  digitalWrite(LED_PIN, HIGH);          // Apaga o LED

  ledTaskHandle = NULL; // Reseta o handle da task
  vTaskDelete(NULL);    // Deleta a própria task quando terminar
}

void initWebServerHTTP()
{
  // Connect to Wi-Fi network with SSID and password
  Serial.println("Setting AP (Access Point)");
  // NULL sets an open Access Point
  WiFi.softAP(SSID_ESP, NULL);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  // Web Server Root URL
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(SPIFFS, "/wifimanager.html", "text/html");
      Serial.println("wifimanager.html foi acessada!"); });

  server.serveStatic("/", SPIFFS, "/");

  server.on("/", HTTP_POST, [](AsyncWebServerRequest *request)
            {
                int params = request->params();
                for (int i = 0; i < params; i++)
                {
                  const AsyncWebParameter *p = request->getParam(i);
                  if (p->isPost())
                  {
                    // HTTP POST ssid value
                    if (p->name() == PARAM_INPUT_1)
                    {
                      ssid = p->value().c_str();
                      Serial.print("SSID set to: ");
                      Serial.println(ssid);
                      // Write file to save value
                      writeFile(SPIFFS, ssidPath, ssid.c_str());
                    }
                    // HTTP POST pass value
                    if (p->name() == PARAM_INPUT_2)
                    {
                      pass = p->value().c_str();
                      Serial.print("Password set to: ");
                      Serial.println(pass);
                      // Write file to save value
                      writeFile(SPIFFS, passPath, pass.c_str());
                    }
                    // HTTP POST ip value
                    if (p->name() == PARAM_INPUT_3)
                    {
                      ip = p->value().c_str();
                      Serial.print("IP Address set to: ");
                      Serial.println(ip);
                      // Write file to save value
                      writeFile(SPIFFS, ipPath, ip.c_str());
                    }
                    // HTTP POST gateway value
                    if (p->name() == PARAM_INPUT_4)
                    {
                      gateway = p->value().c_str();
                      Serial.print("Gateway set to: ");
                      Serial.println(gateway);
                      // Write file to save value
                      writeFile(SPIFFS, gatewayPath, gateway.c_str());
                    }

                    // HTTP POST websocket ip value
                    if (p->name() == PARAM_INPUT_5)
                    {
                      websocketIP = p->value().c_str();
                      Serial.print("websocket ip set to: ");
                      Serial.println(websocketIP);
                      // Write file to save value
                      writeFile(SPIFFS, websocketIPPath, websocketIP.c_str());
                    }
                    // Serial.printf("POST[%s]: %s\n", p->name().c_str(), p->value().c_str());
                  }
                }
                request->send(200, "text/plain", WiFi.macAddress()); });

  server.on("/restart", HTTP_GET, [](AsyncWebServerRequest *request)
            { esp_restart(); });
  server.begin();
};

void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length)
{
  String event;

  switch (type)
  {
  case WStype_CONNECTED:
    Serial.printf("📡 Cliente conectado! ID: %u\n", num);
    webSocket.sendTXT(num, "Conexão estabelecida com ESP32!");
    break;
  case WStype_TEXT:
    Serial.printf("📩 Recebido do Cliente: %s\n", payload);

    event = String((char *)payload);
    webSocket.sendTXT(num, "🔄 Mensagem recebida: " + event);

    if (event == "on")
    {
      alarmON();
    }

    if (event == "off")
    {
      alarmOFF();
    }

    if (event == "reset")
    {
      reset();
    }

    break;
  case WStype_DISCONNECTED:
    Serial.printf("❌ Cliente desconectado! ID: %u\n", num);
    break;
  }
}

void initWebSocket(String ip)
{
  IPAddress localWebsocketIP;

  if (ip == "")
  {
    Serial.println("Undefined WebsocketIP address.");
    return;
  }

  localWebsocketIP.fromString(ip.c_str());

  IPAddress subnet(255, 255, 255, 0);

  WiFi.softAPConfig(localWebsocketIP, localWebsocketIP, subnet);
  WiFi.softAP(SSID_ESP, "");

  Serial.println("🚀 Wi-Fi AP Criado!");
  Serial.print("WebSocket disponível em: ws://");
  Serial.print(WiFi.softAPIP());
  Serial.println(":81");

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
}

void setup()
{
  // Serial port for debugging purposes
  Serial.begin(115200);

  initSPIFFS();

  // Set GPIO 2 as an OUTPUT
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT); // Define o PINO do buzzer como saída
  pinMode(A0_PIN, INPUT);
  pinMode(D0_PIN, INPUT);

  digitalWrite(LED_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  // Load values saved in SPIFFS
  ssid = readFile(SPIFFS, ssidPath);
  pass = readFile(SPIFFS, passPath);
  ip = readFile(SPIFFS, ipPath);
  gateway = readFile(SPIFFS, gatewayPath);
  websocketIP = readFile(SPIFFS, websocketIPPath);

  Serial.println(ssid);
  Serial.println(pass);
  Serial.println(ip);
  Serial.println(gateway);
  Serial.println(websocketIP);

  if (initWiFi())
  {
    initWebSocket(websocketIP);
    client.setCACert(TELEGRAM_CERTIFICATE_ROOT);
  }
  else
  {
    initWebServerHTTP();
  }
}

int digitalSensorBefore = 1;

void loop()
{
  int analogSensor = analogRead(A0_PIN);
  String analogStringSensor = String(analogSensor);

  int digitalSensor = digitalRead(D0_PIN);
  String digitalStringSensor = String(digitalSensor);

  webSocket.loop();

  if (webSocket.connectedClients() > 0)
  {
    webSocket.broadcastTXT(digitalStringSensor);
  }

  if (digitalSensorBefore != digitalSensor)
  {
    bot.sendMessage(CHAT_ID, "oi yan", "");
  }

  digitalSensorBefore = digitalSensor;

  Serial.print("Pin A0: ");
  Serial.println(analogSensor);

  Serial.print("Pin D0: ");
  Serial.println(digitalSensor);

  // GENRENCIA O ALARME CRITICO (BUZZER + LED CONSTANTE)
  if (!digitalSensor)
  {
    alarmON();
  }
  else
  {
    alarmOFF();

    // GENRENCIA O ALARME PREVENTIVO ( LED PISCANDO)
    if (analogSensor > SENSOR_THRESHOLD)
    {
      if (ledTaskHandle == NULL)
      {
        xTaskCreate(ledTask, "LedTask", 1000, NULL, 1, &ledTaskHandle);
      }
    }
  }

  delay(200);
}