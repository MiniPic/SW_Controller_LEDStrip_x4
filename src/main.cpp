#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

#define LED_PIN     2
#define LED_COUNT   20
#define BRIGHTNESS  255

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

uint32_t colorWheel(byte pos) {
    if (pos < 85) {
        return strip.Color(pos * 3, 255 - pos * 3, 0);
    } else if (pos < 170) {
        pos -= 85;
        return strip.Color(255 - pos * 3, 0, pos * 3);
    } else {
        pos -= 170;
        return strip.Color(0, pos * 3, 255 - pos * 3);
    }
}

void setup() {
    strip.begin();
    strip.setBrightness(BRIGHTNESS);
    strip.show();
}

void loop() {
    static uint32_t lastEvent = 0;
    static uint16_t rainbowOffset = 0;

    uint32_t now = millis();

    // Phase strobe toutes les 10 secondes pendant 2 secondes
    if ((now - lastEvent) >= 10000 && (now - lastEvent) < 12000) {
        // 10Hz = 100ms par cycle
        static uint32_t lastStrobeToggle = 0;
        static bool strobeState = false;

        if (now - lastStrobeToggle >= 50) {
            lastStrobeToggle = now;
            strobeState = !strobeState;

            uint32_t color = strobeState ? strip.Color(255, 255, 255) : strip.Color(0, 0, 0);

            for (int i = 0; i < LED_COUNT; i++) {
                strip.setPixelColor(i, color);
            }
            strip.show();
        }

        return; // On saute le rainbow pendant le strobe
    }

    // Reset du timer à la fin des 2 secondes de strobe
    if ((now - lastEvent) >= 12000) {
        lastEvent = now;
    }

    // Effet rainbow normal
    for (int i = 0; i < LED_COUNT; i++) {
        uint8_t pos = (i * 256 / LED_COUNT + rainbowOffset) & 255;
        strip.setPixelColor(i, colorWheel(pos));
    }

    strip.show();
    rainbowOffset++;
    delay(25);
}
