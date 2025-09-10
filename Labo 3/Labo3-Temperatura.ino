#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

/* Uncomment the initialize the I2C address , uncomment only one, If you get a totally blank screen try the other*/
#define i2c_Address 0x3c //initialize with the I2C addr 0x3C Typically eBay OLED's

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1   //   QT-PY / XIAO
//Inicializa la configuración de la pantalla
Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET); 

const int pinLM35 = A0;            // pin analógico donde está conectado LM35 Vout
const unsigned long intervalo = 1000; // intervalo de actualización en ms
unsigned long ultimaMillis = 0;

void setup()   {
  Serial.begin(9600);
  
  delay(250); // wait for the OLED to power up
  display.begin(i2c_Address, true); // Address 0x3C default
  display.setContrast (255); // dim display
 
  // Clear the buffer.
  display.clearDisplay();
  // Save to update screen changes
  display.display();
  
  Serial.begin(9600);            // velocidad del Monitor Serial
  while (!Serial) { /* espera en algunas placas */ }
  delay(50);
  display.println("Temperatura:");
}

void loop() {
  display.setTextSize(1); 
  display.setTextColor(SH110X_WHITE); 

  unsigned long ahora = millis();
  if (ahora - ultimaMillis >= intervalo) {
    ultimaMillis = ahora;

    int lectura = analogRead(pinLM35);              // 0..1023
    const float Vref = 5.0;                         // ajustar si tu referencia ADC es distinta
    float voltaje = lectura * (Vref / 1023.0);      // voltaje en V
    float tempC = voltaje * 100.0;                  // LM35: 10 mV/°C -> factor 100

    // Imprime en formato legible
    display.setCursor(20, 0);
    display.println("Temperatura: ");

    display.setCursor(20, 11);
    display.print(tempC);
    display.print(" C");

    display.display();
    display.clearDisplay();

    Serial.print("Temperatura: ");
    Serial.print(tempC, 2);
    Serial.println(" °C");
  }
}