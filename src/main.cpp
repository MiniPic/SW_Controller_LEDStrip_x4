#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>
#include <sACN.h>              // Librairie sACN de Stefan Staub
#include <Adafruit_NeoPixel.h>
#include <TeensyThreads.h>
// ======================= CONFIG RESEAU =======================



// ======================= CONFIG sACN / DMX ===================

// Univers sACN (1 et 2)
#define UNIVERSE_1          1   // strips 1 & 2
#define UNIVERSE_2          2   // strips 3 & 4

#define LEDS_PER_STRIP      60
#define CHANNELS_PER_LED    4
#define STRIP_CHANNELS      ((LEDS_PER_STRIP * CHANNELS_PER_LED)+1)  // 241
#define DMX_UNIVERSE_SIZE   512

// Canaux strobe (0-based dans le buffer)
#define STROBE_CH_STRIP_A   (1*STRIP_CHANNELS)-1    //canal 241
#define STROBE_CH_STRIP_B   (2*STRIP_CHANNELS)-1    //canal 482

// ======================= CONFIG LED ==========================

#define PIN_STRIP_1   2
#define PIN_STRIP_2   3
#define PIN_STRIP_3   4
#define PIN_STRIP_4   5

#define GLOBAL_BRIGHTNESS  255

Adafruit_NeoPixel strip1(LEDS_PER_STRIP, PIN_STRIP_1, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip2(LEDS_PER_STRIP, PIN_STRIP_2, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip3(LEDS_PER_STRIP, PIN_STRIP_3, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip4(LEDS_PER_STRIP, PIN_STRIP_4, NEO_GRB + NEO_KHZ800);

// ======================= DEBUG LED ===========================

#define DEBUG1_LED_PIN  22
#define DEBUG2_LED_PIN  21
volatile bool networkReady = false;
volatile bool sACN1_Ready = false;
volatile bool sACN2_Ready = false;

// ======================= DIP-SWITCH ===========================

#define DIPSW1 23
#define DIPSW2 24
#define DIPSW3 25
#define DIPSW4 26

volatile uint32_t UNIVERS_OFFSET = 0;

#define WATCHDOG_TIMOUT 5000 //5s
#define SACN_REBOOT_TIMEOUT_MS  (5UL * 60UL * 1000UL) // 5 minutes
volatile uint32_t lastsACN;


// ======================= sACN / DMX BUFFERS ==================

// sACN : 1 receiver par univers
EthernetUDP sacnUdp1;
EthernetUDP sacnUdp2;
Receiver   recv1(sacnUdp1);
Receiver   recv2(sacnUdp2);

// buffers DMX remplis dans les callbacks sACN
volatile uint8_t dmxUniverse1[DMX_UNIVERSE_SIZE] = {0};
volatile uint8_t dmxUniverse2[DMX_UNIVERSE_SIZE] = {0};

// buffers de travail utilisés par le thread LED (PAS sur le stack)
uint8_t localUniverse1[DMX_UNIVERSE_SIZE] = {0};
uint8_t localUniverse2[DMX_UNIVERSE_SIZE] = {0};

// valeurs strobe extraites des buffers DMX
volatile uint8_t strobeStrip1 = 0;
volatile uint8_t strobeStrip2 = 0;
volatile uint8_t strobeStrip3 = 0;
volatile uint8_t strobeStrip4 = 0;

// ======================= FONCTIONS DIVERSES ===============
uint32_t getUIDWord(int index) {
    volatile uint32_t *UID = (uint32_t *)0x401F4410; // OCOTP_HW_OCOTP_CFG0
    return UID[index];
}

void hard_reset()
{
    delay(100);
    SCB_AIRCR = 0x05FA0004;
}

uint32_t readDipUniverseOffset() {
    uint8_t value = 0;

    // Rappel: LOW = interrupteur ON
    if (digitalRead(DIPSW1) == LOW) value |= (1 << 0);  // bit 0
    if (digitalRead(DIPSW2) == LOW) value |= (1 << 1);  // bit 1
    if (digitalRead(DIPSW3) == LOW) value |= (1 << 2);  // bit 2
    if (digitalRead(DIPSW4) == LOW) value |= (1 << 3);  // bit 3

    return (uint32_t)value*2;
}

// ======================= OUTILS TEMPS / STROBE ===============
//
// 0..10  -> ON permanent
// 11..128 -> fréquence linéaire 0.1 Hz (10s) -> 20 Hz (50ms)
// La fonction ne gère que la phase ON/OFF.
//
bool strobePhase(uint8_t value, uint32_t nowMs, uint32_t &lastToggleMs, bool &state)
{
    state = false;

    // 0 à 10 : pas de strobe, toujours ON
    if (value <= 10) {
        state = true;
        return true;
    }
    else if(value <= 128)
    {
        // 11 à 128 : fréquence linéaire 0.1 Hz -> 20 Hz
        uint32_t period = map(value, 11, 128, 2000, 50); // 2 s -> 50 ms
        if (nowMs - lastToggleMs >= period / 2) {
            lastToggleMs = nowMs;
            state = true;
        }
    }
    else if((value < 255))
    {
        // 129 à 255 : fréquence linéaire 0.1 Hz -> 20 Hz
        uint32_t period = map(value, 129, 254, 2000, 50); // 2 s -> 50 ms
        if (nowMs - lastToggleMs >= period / 2) {
            lastToggleMs = nowMs;
            state = true;
        }
    }
    else    // for 255
        state = true;

    return state;
}

// ======================= MAPPING DMX -> LED ==================
//
// Remplit stripA et stripB à partir du buffer DMX d’un univers
// et met à jour strobeA / strobeB.
//
void applyUniverseToStrips(
    volatile uint8_t *dmxData,
    Adafruit_NeoPixel &stripA,
    Adafruit_NeoPixel &stripB,
    uint8_t &strobeA,
    uint8_t &strobeB
) {
    strobeA = dmxData[STROBE_CH_STRIP_A];
    strobeB = dmxData[STROBE_CH_STRIP_B];

    // Strip A : canaux 1–240 (0–239 index)
    for (int i = 0; i < LEDS_PER_STRIP; i++) {
        int baseIndex = i * CHANNELS_PER_LED;
        uint8_t g   = dmxData[baseIndex + 0];
        uint8_t r   = dmxData[baseIndex + 1];
        uint8_t b   = dmxData[baseIndex + 2];
        uint8_t dim = dmxData[baseIndex + 3];

        uint16_t r16 = (uint16_t)r * dim / 255;
        uint16_t g16 = (uint16_t)g * dim / 255;
        uint16_t b16 = (uint16_t)b * dim / 255;

        stripA.setPixelColor(i, stripA.Color(r16, g16, b16));
    }

    // Strip B : canaux 241–480 (240–479 index)
    for (int i = 0; i < LEDS_PER_STRIP; i++) {
        int baseIndex = STRIP_CHANNELS + i * CHANNELS_PER_LED; // 240 + ...
        if (baseIndex + 3 >= DMX_UNIVERSE_SIZE) break;

        uint8_t g   = dmxData[baseIndex + 0];
        uint8_t r   = dmxData[baseIndex + 1];
        uint8_t b   = dmxData[baseIndex + 2];
        uint8_t dim = dmxData[baseIndex + 3];

        uint16_t r16 = (uint16_t)r * dim / 255;
        uint16_t g16 = (uint16_t)g * dim / 255;
        uint16_t b16 = (uint16_t)b * dim / 255;

        stripB.setPixelColor(i, stripB.Color(r16, g16, b16));
    }
}

// ======================= CALLBACKS sACN ======================

// Univers 1 : réception DMX
void dmxReceived1()
{
    uint8_t buf[DMX_UNIVERSE_SIZE];
    recv1.dmx(buf);   // lit le buffer DMX complet de l’univers

    noInterrupts();
    memcpy((void*)dmxUniverse1, buf, DMX_UNIVERSE_SIZE);
    interrupts();

    sACN1_Ready = true;
    lastsACN = millis();
}

// Univers 2 : réception DMX
void dmxReceived2()
{
    uint8_t buf[DMX_UNIVERSE_SIZE];
    recv2.dmx(buf);

    noInterrupts();
    memcpy((void*)dmxUniverse2, buf, DMX_UNIVERSE_SIZE);
    interrupts();

    sACN2_Ready = true;
    lastsACN = millis();
}

// nouveaux sources
void newSource1()
{
    Serial.print("sACN U1 - new source: ");
    Serial.println(recv1.name());
}

void newSource2()
{
    Serial.print("sACN U2 - new source: ");
    Serial.println(recv2.name());
}

// framerate
void framerate1()
{
    Serial.print("sACN U1 framerate: ");
    Serial.println(recv1.framerate());
}

void framerate2()
{
    Serial.print("sACN U2 framerate: ");
    Serial.println(recv2.framerate());
}

// timeout
void timeout1()
{
    Serial.println("sACN U1 timeout");
    sACN1_Ready = false;
}

void timeout2()
{
    Serial.println("sACN U2 timeout");
    sACN2_Ready = false;
}

// ======================= THREADS =============================

// Thread sACN : update() sur les 2 receivers
void sacnThread()
{
    while (1) {
        recv1.update();
        recv2.update();
        threads.yield();
        if(millis() - lastsACN > SACN_REBOOT_TIMEOUT_MS)
            hard_reset();
    }
}

// Thread LED (mise à jour couleurs + strobe)
void ledThread()
{
    // état du strobe / tempo par strip
    static uint32_t lastToggle1 = 0, lastToggle2 = 0, lastToggle3 = 0, lastToggle4 = 0;
    static bool stState1 = true, stState2 = true, stState3 = true, stState4 = true;

    // mémorisation pour savoir quand on entre en phase ON (pour le random)
    static bool prevOn1 = false, prevOn2 = false, prevOn3 = false, prevOn4 = false;

    while (1) {
        uint32_t now = millis();

        // copie atomique des buffers DMX dans les buffers de travail globaux
        noInterrupts();
        memcpy(localUniverse1, (const void*)dmxUniverse1, DMX_UNIVERSE_SIZE);
        memcpy(localUniverse2, (const void*)dmxUniverse2, DMX_UNIVERSE_SIZE);
        interrupts();

        // DMX -> couleurs (sans strobe)
        applyUniverseToStrips(localUniverse1, strip1, strip2, (uint8_t&)strobeStrip1, (uint8_t&)strobeStrip2);
        applyUniverseToStrips(localUniverse2, strip3, strip4, (uint8_t&)strobeStrip3, (uint8_t&)strobeStrip4);

        // --- Gestion strobe par strip ---

        // phase ON/OFF (tempo commun, mode normal / random)
        bool on1 = strobePhase(strobeStrip1, now, lastToggle1, stState1);
        bool on2 = strobePhase(strobeStrip2, now, lastToggle2, stState2);
        bool on3 = strobePhase(strobeStrip3, now, lastToggle3, stState3);
        bool on4 = strobePhase(strobeStrip4, now, lastToggle4, stState4);

        // Strip 1
        if (strobeStrip1 <= 10) {
            // pas de strobe, on garde les couleurs DMX
        } else if (strobeStrip1 <= 128) {
            // strobe plein : tout ON ou tout OFF
            if (!on1) {
                for (int i = 0; i < LEDS_PER_STRIP; i++)
                    strip1.setPixelColor(i, 0);
            }
        } else { 
            // strobe random : LEDs allumées aléatoirement en phase ON
            if (!on1) {
                for (int i = 0; i < LEDS_PER_STRIP; i++)
                    strip1.setPixelColor(i, 0);
            }
            else
            {
                uint32_t s_rand = random(LEDS_PER_STRIP-1);
                for (int i = 0; i < s_rand; i++)
                    strip1.setPixelColor(i, 0);
                for (int i = s_rand+1; i < LEDS_PER_STRIP; i++)
                    strip1.setPixelColor(i, 0);
            }
        }
        prevOn1 = on1;

        // Strip 2
        if (strobeStrip2 <= 10) {
            // pas de strobe, on garde les couleurs DMX
        } else if (strobeStrip2 <= 128) {
            // strobe plein : tout ON ou tout OFF
            if (!on2) {
                for (int i = 0; i < LEDS_PER_STRIP; i++)
                    strip2.setPixelColor(i, 0);
            }
        } else { 
            // strobe random : LEDs allumées aléatoirement en phase ON
            if (!on2) {
                for (int i = 0; i < LEDS_PER_STRIP; i++)
                    strip2.setPixelColor(i, 0);
            }
            else
            {
                uint32_t s_rand = random(LEDS_PER_STRIP-1);
                for (int i = 0; i < s_rand; i++)
                    strip2.setPixelColor(i, 0);
                for (int i = s_rand+1; i < LEDS_PER_STRIP; i++)
                    strip2.setPixelColor(i, 0);
            }
        }
        prevOn2 = on2;

        // Strip 3
        if (strobeStrip3 <= 10) {
            // pas de strobe, on garde les couleurs DMX
        } else if (strobeStrip3 <= 128) {
            // strobe plein : tout ON ou tout OFF
            if (!on3) {
                for (int i = 0; i < LEDS_PER_STRIP; i++)
                    strip3.setPixelColor(i, 0);
            }
        } else { 
            // strobe random : LEDs allumées aléatoirement en phase ON
            if (!on3) {
                for (int i = 0; i < LEDS_PER_STRIP; i++)
                    strip3.setPixelColor(i, 0);
            }
            else
            {
                uint32_t s_rand = random(LEDS_PER_STRIP-1);
                for (int i = 0; i < s_rand; i++)
                    strip3.setPixelColor(i, 0);
                for (int i = s_rand+1; i < LEDS_PER_STRIP; i++)
                    strip3.setPixelColor(i, 0);
            }
        }
        prevOn3 = on3;

        // Strip 4
        if (strobeStrip4 <= 10) {
            // pas de strobe, on garde les couleurs DMX
        } else if (strobeStrip4 <= 128) {
            // strobe plein : tout ON ou tout OFF
            if (!on4) {
                for (int i = 0; i < LEDS_PER_STRIP; i++)
                    strip4.setPixelColor(i, 0);
            }
        } else { 
            // strobe random : LEDs allumées aléatoirement en phase ON
            if (!on4) {
                for (int i = 0; i < LEDS_PER_STRIP; i++)
                    strip4.setPixelColor(i, 0);
            }
            else
            {
                uint32_t s_rand = random(LEDS_PER_STRIP-1);
                for (int i = 0; i < s_rand; i++)
                    strip4.setPixelColor(i, 0);
                for (int i = s_rand+1; i < LEDS_PER_STRIP; i++)
                    strip4.setPixelColor(i, 0);
            }
        }
        prevOn4 = on4;

        // Envoi aux strips
        strip1.show();
        strip2.show();
        strip3.show();
        strip4.show();
        threads.delay(5);   // maintenant le stack du thread est safe
    }
}

// Thread futur LVGL
void uiThread()
{
    while (1) {
        // lv_timer_handler();
        threads.delay(5);
    }
}

// Thread LED debug réseau
void debug1LedThread()
{
    pinMode(DEBUG1_LED_PIN, OUTPUT);
    digitalWrite(DEBUG1_LED_PIN, LOW);

    while (1) {
        if (networkReady) {
            digitalToggle(DEBUG1_LED_PIN); // clignote
            if(sACN1_Ready)
                threads.delay(125);
            else
                threads.delay(500);
        } else {
            digitalWrite(DEBUG1_LED_PIN, LOW);
            threads.delay(200);
        }
    }
}

// Thread LED debug réseau
void debug2LedThread()
{
    pinMode(DEBUG2_LED_PIN, OUTPUT);
    digitalWrite(DEBUG2_LED_PIN, LOW);

    while (1) {
        if (networkReady) {
            digitalToggle(DEBUG2_LED_PIN); // clignote
            if(sACN2_Ready)
                threads.delay(125);
            else
                threads.delay(500);
        } else {
            digitalWrite(DEBUG2_LED_PIN, LOW);
            threads.delay(200);
        }
    }
}

// ======================= SETUP / LOOP ========================

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}

    // seed random pour le mode random strobe
    randomSeed(analogRead(A0));

    pinMode(DIPSW1,INPUT);
    pinMode(DIPSW2,INPUT);
    pinMode(DIPSW3,INPUT);
    pinMode(DIPSW4,INPUT);

    uint32_t uid = getUIDWord(0);   // Exemple : un entier 32 bits
    byte mac[] = { 0x04, 0xE9, 0xE5, 0x00, 0x00, 0x02 }; // MAC à adapter si besoin
    mac[3] = (uid >> 0)  & 0xFF;
    mac[4] = (uid >> 8)  & 0xFF;
    mac[5] = (uid >> 16) & 0xFF;

    Serial.println("MAC Address : ");
    for (int i = 0; i < sizeof(mac); i++) {
        Serial.print(mac[i], HEX);
    }
    Serial.println();

    // Ethernet en DHCP
    Ethernet.begin(mac,60*5*1000);  //5minutes
    IPAddress ip = Ethernet.localIP();
    Serial.print("IP locale: ");
    Serial.println(ip);

    if (ip != IPAddress(0, 0, 0, 0)) {
        networkReady = true;
        Serial.println("Réseau OK (DHCP).");
    } else {
        networkReady = false;
        Serial.println("DHCP non obtenu.");
        hard_reset();
    }

    // LEDs
    strip1.begin(); strip2.begin(); strip3.begin(); strip4.begin();
    strip1.setBrightness(GLOBAL_BRIGHTNESS);
    strip2.setBrightness(GLOBAL_BRIGHTNESS);
    strip3.setBrightness(GLOBAL_BRIGHTNESS);
    strip4.setBrightness(GLOBAL_BRIGHTNESS);

    strip1.show();
    strip2.show();
    strip3.show();
    strip4.show();

    // Configuration sACN Receiver (lib Stefan Staub)
    recv1.callbackDMX(dmxReceived1);
    recv1.callbackSource(newSource1);
    recv1.callbackFramerate(framerate1);
    recv1.callbackTimeout(timeout1);
    recv1.begin(UNIVERSE_1 + readDipUniverseOffset());    // Univers 1 en multicast

    recv2.callbackDMX(dmxReceived2);
    recv2.callbackSource(newSource2);
    recv2.callbackFramerate(framerate2);
    recv2.callbackTimeout(timeout2);
    recv2.begin(UNIVERSE_2 + readDipUniverseOffset());    // Univers 2 en multicast

    lastsACN = millis();

    Serial.print("Offset Univers: ");
    Serial.println(readDipUniverseOffset());
    Serial.println("sACN receivers démarrés.");

    // Threads
    threads.addThread(sacnThread);
    threads.addThread(ledThread);
    threads.addThread(uiThread);
    threads.addThread(debug1LedThread);
    threads.addThread(debug2LedThread);
}

void loop()
{
    // Tout tourne dans les threads
}
