#include <Adafruit_NeoPixel.h>

#define PIN_STRIP_1 4
#define LED_COUNT   100
#define BRIGHTNESS  255

Adafruit_NeoPixel strip1(LED_COUNT, PIN_STRIP_1, NEO_GRB + NEO_KHZ800);

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}

  Serial.println("=== Test chenillard STRIP 1 ===");
  Serial.print("Nombre de LEDs : ");
  Serial.println(LED_COUNT);
  Serial.println("Une LED allumée toutes les 500 ms");

  strip1.begin();
  strip1.setBrightness(BRIGHTNESS);
  strip1.clear();
  strip1.show();

  Serial.println("Initialisation LED OK");
}

void loop() {
  static int idx = 0;

  strip1.clear();
  strip1.setPixelColor(idx, strip1.Color(255, 0, 0)); // rouge
  strip1.show();

  Serial.print("LED allumée : ");
  Serial.println(idx);

  idx++;
  if (idx >= LED_COUNT) {
    idx = 0;
    Serial.println("Retour à la LED 0");
  }

  delay(500);
}
