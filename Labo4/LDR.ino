/************************************************************
 * PLANTILLA: ESP32 + LM35 (temperatura) → Adafruit IO
 * 
 * Este código lee el valor analógico del LM35, lo convierte
 * a grados Celsius y lo publica en un feed de Adafruit IO.
 ************************************************************/

#include <WiFi.h>                   // Librería WiFi para ESP32
#include "Adafruit_MQTT.h"          // Librería MQTT
#include "Adafruit_MQTT_Client.h"   // Cliente MQTT para Adafruit IO

/************** CONFIGURACIÓN Wi-Fi **************/
#define WLAN_SSID     "wifi"       // Nombre de tu red WiFi
#define WLAN_PASS     "12345678"   // Contraseña de tu red WiFi

/************** CONFIGURACIÓN Adafruit IO **************/
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883          // Usa 8883 para SSL
#define AIO_USERNAME    "user"  // Tu usuario en Adafruit
#define AIO_KEY         "Key"           // Tu clave AIO

/************** CLIENTE MQTT **************/
WiFiClient client;  
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

/************** FEED DE PUBLICACIÓN **************/
Adafruit_MQTT_Publish pubLuz = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/luz");

/************** PIN DEL LDR **************/
const int PIN_LDR = 33; // ADC1 CH5 (entrada analógica)

/************** CONFIGURACIÓN DEL ADC **************/
void configADC() {
  analogReadResolution(12); // 0..4095
  analogSetPinAttenuation(PIN_LDR, ADC_11db); // Rango ~0..3.3 V
}

/************** FUNCIÓN: Leer luz **************/
float leerLuz() {
  uint16_t raw = analogRead(PIN_LDR);          // Leer valor crudo (0..4095)
  float pct = (raw / 4095.0f) * 100.0f;        // Normalizar a porcentaje
  return pct;                                  // Retornar 0–100 %
}

/************** FUNCIÓN: Conexión a MQTT **************/
void MQTT_connect() {
  if (mqtt.connected()) return;
  Serial.print("Conectando a Adafruit IO... ");
  int8_t ret;
  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) {
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Reintento en 10 s...");
    mqtt.disconnect();
    delay(10000);
    if (--retries == 0) while (1) delay(1);
  }
  Serial.println("¡Conectado!");
}

/************** SETUP **************/
void setup() {
  Serial.begin(115200);
  configADC();

  Serial.print("Conectando a WiFi...");
  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println("\nWiFi conectado");
}

/************** LOOP **************/
void loop() {
  // Mantener conexión MQTT
  MQTT_connect();
  mqtt.processPackets(10000);
  if (!mqtt.ping()) mqtt.disconnect();

  // Leer luz y publicar
  float luzPct = leerLuz();
  if (pubLuz.publish(luzPct)) {
    Serial.printf("Luz enviada: %.1f %%\n", luzPct);
  } else {
    Serial.println("Error al publicar luz");
  }

  delay(2000); // Publicar cada 2 segundos
}