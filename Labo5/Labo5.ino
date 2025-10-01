#include <WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include "DHT.h"


/******** Wi-Fi ********/
#define WLAN_SSID   ""
#define WLAN_PASS   ""

/******** Adafruit IO ********/
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    ""
#define AIO_KEY         ""

/******** MQTT ********/
WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

/******** Feed ********/
Adafruit_MQTT_Publish pubAlarma = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/alarma");
Adafruit_MQTT_Publish pubTemp = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/temperatura");
Adafruit_MQTT_Publish pubHum  = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/humedad");

/******** Sensor FC-51 ********/
const int PIN_FC51 = 34;

#define DHTPIN 33       // Pin donde conectas el DHT11
#define DHTTYPE DHT11  // Tipo de sensor
DHT dht(DHTPIN, DHTTYPE);

void conectarWiFi() {
  Serial.print("Conectando a WiFi...");
  WiFi.begin(WLAN_SSID, WLAN_PASS);
  int intentos = 0;
  while (WiFi.status() != WL_CONNECTED && intentos < 40) {
    delay(500); Serial.print(".");
    intentos++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi conectado");
    Serial.print("IP asignada: "); Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nNo se pudo conectar a WiFi");
  }
}

void MQTT_connect() {
  if (mqtt.connected()) return;
  Serial.print("Conectando a Adafruit IO... ");
  int8_t ret; uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) {
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Reintento en 10 s...");
    mqtt.disconnect(); delay(10000);
    if (--retries == 0) while (1) delay(1);
  }
  Serial.println("¡Conectado!");
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_FC51, INPUT);
  dht.begin();
  conectarWiFi();
}

void loop() {
  MQTT_connect();
  mqtt.processPackets(10000);
  if (!mqtt.ping()) mqtt.disconnect();

  int estado = digitalRead(PIN_FC51);
  bool obstaculo = (estado == LOW);

  if (obstaculo) {
    Serial.println("Objeto detectado");
    pubAlarma.publish("Objeto detectado");
  } else {
    Serial.println("Libre");
    pubAlarma.publish("Libre");
  }

  if (!mqtt.ping()) mqtt.disconnect();

  float t = dht.readTemperature(); // en °C
  float h = dht.readHumidity();    // en %

  if (isnan(t) || isnan(h)) {
    Serial.println("Error leyendo DHT11");
  } else {
    Serial.printf("Temp: %.1f °C | Hum: %.1f %%\n", t, h);

    if (!pubTemp.publish(t)) Serial.println("Error publicando temperatura");
    if (!pubHum.publish(h))  Serial.println("Error publicando humedad");
  }

  delay(2000);
}