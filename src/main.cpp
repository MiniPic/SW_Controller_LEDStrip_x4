#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>
#include <sACN.h>              // Stefan Staub
#include <Adafruit_NeoPixel.h>
#include <TeensyThreads.h>

// ======================= CONFIG sACN / DMX ===================

#define DMX_UNIVERSE_SIZE      512
#define CHANNELS_PER_LED       4

#define LEDS_PER_STRIP         400
#define LEDS_PER_UNIVERSE      100
#define PIX_CH_PER_UNIVERSE    (LEDS_PER_UNIVERSE * CHANNELS_PER_LED) // 400
#define STROBE_CH              (PIX_CH_PER_UNIVERSE)                  // index 400 => canal 401

#define UNIVERSE_BASE          1
#define UNIVERSE_COUNT         8

#define SACN_REBOOT_TIMEOUT_MS (5UL * 60UL * 1000UL) // 5 min

// ======================= CONFIG LED ==========================

#define PIN_STRIP_A            4
#define PIN_STRIP_B            5

#define GLOBAL_BRIGHTNESS      255

Adafruit_NeoPixel stripA(LEDS_PER_STRIP, PIN_STRIP_A, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel stripB(LEDS_PER_STRIP, PIN_STRIP_B, NEO_GRB + NEO_KHZ800);

// ======================= DEBUG LED ===========================

#define DEBUG1_LED_PIN         22
#define DEBUG2_LED_PIN         21

volatile bool networkReady = false;

// ======================= DIP-SWITCH ===========================

#define DIPSW1 23
#define DIPSW2 24
#define DIPSW3 25
#define DIPSW4 26

// ======================= WATCHDOG ============================

volatile uint32_t lastsACN = 0;

// ======================= sACN / DMX ==========================

// 8 UDP + 8 receivers (évite les soucis d'init de tableaux d'objets)
EthernetUDP sacnUdp0, sacnUdp1, sacnUdp2, sacnUdp3, sacnUdp4, sacnUdp5, sacnUdp6, sacnUdp7;
Receiver   recv0(sacnUdp0);
Receiver   recv1(sacnUdp1);
Receiver   recv2(sacnUdp2);
Receiver   recv3(sacnUdp3);
Receiver   recv4(sacnUdp4);
Receiver   recv5(sacnUdp5);
Receiver   recv6(sacnUdp6);
Receiver   recv7(sacnUdp7);

// buffers DMX (remplis par callbacks)
volatile uint8_t dmxU[UNIVERSE_COUNT][DMX_UNIVERSE_SIZE] = {{0}};

// buffers de travail (utilisés par le thread LED)
uint8_t localU[UNIVERSE_COUNT][DMX_UNIVERSE_SIZE] = {{0}};

// strobe extrait (1 par univers)
volatile uint8_t strobeU[UNIVERSE_COUNT] = {0};

// état réception
volatile bool sacnReadyU[UNIVERSE_COUNT] = {false};

// ======================= OUTILS DIVERS =======================

uint32_t getUIDWord(int index) {
  volatile uint32_t *UID = (uint32_t *)0x401F4410;
  return UID[index];
}

void hard_reset() {
  delay(100);
  SCB_AIRCR = 0x05FA0004;
}

uint32_t readDipUniverseOffset() {
  uint8_t value = 0;
  // LOW = ON
  if (digitalRead(DIPSW1) == LOW) value |= (1 << 0);
  if (digitalRead(DIPSW2) == LOW) value |= (1 << 1);
  if (digitalRead(DIPSW3) == LOW) value |= (1 << 2);
  if (digitalRead(DIPSW4) == LOW) value |= (1 << 3);
  // tu avais *2 : on conserve
  return (uint32_t)value * 2;
}

// ======================= STROBE ==============================
//
// 0..10   -> ON permanent
// 11..128 -> fréquence linéaire 0.5 Hz (2s) -> 20 Hz (50ms)
// 129..254-> idem (mode "random" côté application)
// 255     -> ON permanent
//
bool strobePhase(uint8_t value, uint32_t nowMs, uint32_t &lastToggleMs, bool &state)
{
  if (value <= 10) { state = true; return true; }

  uint32_t period;
  if (value <= 128) period = map(value, 11, 128, 2000, 50);
  else if (value < 255) period = map(value, 129, 254, 2000, 50);
  else { state = true; return true; }

  if (nowMs - lastToggleMs >= period / 2) {
    lastToggleMs = nowMs;
    state = true;
    return true;  // strob on
  }
  state = false;
  return false;
}

static inline uint8_t strobeMax4(uint8_t a, uint8_t b, uint8_t c, uint8_t d)
{
  uint8_t m = a;
  if (b > m) m = b;
  if (c > m) m = c;
  if (d > m) m = d;
  return m;
}

// ======================= MAPPING DMX -> STRIP (segment 100) ==

static inline void applyUniverseToStripSegment(
  const uint8_t *dmx,
  Adafruit_NeoPixel &strip,
  int ledOffset,
  uint8_t &strobeVal
){
  strobeVal = dmx[STROBE_CH]; // canal 401 (index 400)

  for (int i = 0; i < LEDS_PER_UNIVERSE; i++) {
    int base = i * CHANNELS_PER_LED; // 0..396
    uint8_t g   = dmx[base + 0];
    uint8_t r   = dmx[base + 1];
    uint8_t b   = dmx[base + 2];
    uint8_t dim = dmx[base + 3];

    uint16_t r16 = (uint16_t)r * dim / 255;
    uint16_t g16 = (uint16_t)g * dim / 255;
    uint16_t b16 = (uint16_t)b * dim / 255;

    strip.setPixelColor(ledOffset + i, strip.Color(r16, g16, b16));
  }
}

// ======================= STROBE GLOBAL STRIP =================

static inline void applyGlobalStrobeToStrip(
  Adafruit_NeoPixel &strip,
  uint8_t strobeVal,
  bool onPhase
){
  if (strobeVal <= 10) return;

  if (strobeVal <= 128) {
    if (!onPhase) {
      for (int i = 0; i < LEDS_PER_STRIP; i++) strip.setPixelColor(i, 0);
    }
    return;
  }

  // random global : 1 LED gardée ON en phase ON
  if (!onPhase) {
    for (int i = 0; i < LEDS_PER_STRIP; i++) strip.setPixelColor(i, 0);
  } else {
    int keep = random(LEDS_PER_STRIP);
    for (int i = 0; i < LEDS_PER_STRIP; i++) {
      if (i != keep) strip.setPixelColor(i, 0);
    }
  }
}

// ======================= sACN callbacks ======================

static inline void dmxReceivedCommon(int idx, Receiver &recv)
{
  uint8_t buf[DMX_UNIVERSE_SIZE];
  recv.dmx(buf);

  noInterrupts();
  memcpy((void*)dmxU[idx], buf, DMX_UNIVERSE_SIZE);
  interrupts();

  sacnReadyU[idx] = true;
  lastsACN = millis();
}

static inline void timeoutCommon(int idx)
{
  sacnReadyU[idx] = false;
}

void dmxReceived0(){ dmxReceivedCommon(0, recv0); }
void dmxReceived1(){ dmxReceivedCommon(1, recv1); }
void dmxReceived2(){ dmxReceivedCommon(2, recv2); }
void dmxReceived3(){ dmxReceivedCommon(3, recv3); }
void dmxReceived4(){ dmxReceivedCommon(4, recv4); }
void dmxReceived5(){ dmxReceivedCommon(5, recv5); }
void dmxReceived6(){ dmxReceivedCommon(6, recv6); }
void dmxReceived7(){ dmxReceivedCommon(7, recv7); }

void timeout0(){ timeoutCommon(0); }
void timeout1(){ timeoutCommon(1); }
void timeout2(){ timeoutCommon(2); }
void timeout3(){ timeoutCommon(3); }
void timeout4(){ timeoutCommon(4); }
void timeout5(){ timeoutCommon(5); }
void timeout6(){ timeoutCommon(6); }
void timeout7(){ timeoutCommon(7); }

// ======================= THREADS =============================

void sacnThread()
{
  while (1) {
    recv0.update(); recv1.update(); recv2.update(); recv3.update();
    recv4.update(); recv5.update(); recv6.update(); recv7.update();

    threads.yield();

    if (millis() - lastsACN > SACN_REBOOT_TIMEOUT_MS) hard_reset();
  }
}

void ledThread()
{
  static uint32_t lastToggleA = 0, lastToggleB = 0;
  static bool stStateA = true, stStateB = true;

  while (1) {
    uint32_t now = millis();

    // copie atomique
    noInterrupts();
    memcpy(localU, (const void*)dmxU, sizeof(localU));
    interrupts();

    // --- Couleurs (4 univers => 4 segments) Strip A
    for (int u = 0; u < 4; u++) {
      int ledOffset = u * LEDS_PER_UNIVERSE; // 0,100,200,300
      applyUniverseToStripSegment(localU[u], stripA, ledOffset, (uint8_t&)strobeU[u]);
    }

    // --- Couleurs Strip B
    for (int u = 4; u < 8; u++) {
      int ledOffset = (u - 4) * LEDS_PER_UNIVERSE;
      applyUniverseToStripSegment(localU[u], stripB, ledOffset, (uint8_t&)strobeU[u]);
    }

    // --- Strobe GLOBAL (MAX)
    uint8_t strobeA = strobeMax4(strobeU[0], strobeU[1], strobeU[2], strobeU[3]);
    uint8_t strobeB = strobeMax4(strobeU[4], strobeU[5], strobeU[6], strobeU[7]);

    bool onA = strobePhase(strobeA, now, lastToggleA, stStateA);
    bool onB = strobePhase(strobeB, now, lastToggleB, stStateB);

    applyGlobalStrobeToStrip(stripA, strobeA, onA);
    applyGlobalStrobeToStrip(stripB, strobeB, onB);

    stripA.show();
    stripB.show();

    threads.delay(5);
  }
}

void debug1LedThread()
{
  pinMode(DEBUG1_LED_PIN, OUTPUT);
  digitalWrite(DEBUG1_LED_PIN, LOW);

  while (1) {
    if (!networkReady) {
      digitalWrite(DEBUG1_LED_PIN, LOW);
      threads.delay(200);
      continue;
    }

    // LED1: clignote vite si au moins un univers du stripA est OK
    bool anyA = sacnReadyU[0] || sacnReadyU[1] || sacnReadyU[2] || sacnReadyU[3];
    digitalToggle(DEBUG1_LED_PIN);
    threads.delay(anyA ? 125 : 500);
  }
}

void debug2LedThread()
{
  pinMode(DEBUG2_LED_PIN, OUTPUT);
  digitalWrite(DEBUG2_LED_PIN, LOW);

  while (1) {
    if (!networkReady) {
      digitalWrite(DEBUG2_LED_PIN, LOW);
      threads.delay(200);
      continue;
    }

    // LED2: clignote vite si au moins un univers du stripB est OK
    bool anyB = sacnReadyU[4] || sacnReadyU[5] || sacnReadyU[6] || sacnReadyU[7];
    digitalToggle(DEBUG2_LED_PIN);
    threads.delay(anyB ? 125 : 500);
  }
}

// ======================= SETUP / LOOP ========================

void setup()
{
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}

  randomSeed(analogRead(A0));

  pinMode(DIPSW1, INPUT);
  pinMode(DIPSW2, INPUT);
  pinMode(DIPSW3, INPUT);
  pinMode(DIPSW4, INPUT);

  uint32_t uid = getUIDWord(0);
  byte mac[] = { 0x04, 0xE9, 0xE5, 0x00, 0x00, 0x02 };
  mac[3] = (uid >> 0)  & 0xFF;
  mac[4] = (uid >> 8)  & 0xFF;
  mac[5] = (uid >> 16) & 0xFF;

  Ethernet.begin(mac, 60 * 5 * 1000); // DHCP timeout 5 min
  IPAddress ip = Ethernet.localIP();

  if (ip != IPAddress(0, 0, 0, 0)) {
    networkReady = true;
  } else {
    networkReady = false;
    hard_reset();
  }

  // LEDs
  stripA.begin();
  stripB.begin();
  stripA.setBrightness(GLOBAL_BRIGHTNESS);
  stripB.setBrightness(GLOBAL_BRIGHTNESS);
  stripA.show();
  stripB.show();

  uint32_t off = readDipUniverseOffset();

  // sACN callbacks
  recv0.callbackDMX(dmxReceived0); recv0.callbackTimeout(timeout0);
  recv1.callbackDMX(dmxReceived1); recv1.callbackTimeout(timeout1);
  recv2.callbackDMX(dmxReceived2); recv2.callbackTimeout(timeout2);
  recv3.callbackDMX(dmxReceived3); recv3.callbackTimeout(timeout3);
  recv4.callbackDMX(dmxReceived4); recv4.callbackTimeout(timeout4);
  recv5.callbackDMX(dmxReceived5); recv5.callbackTimeout(timeout5);
  recv6.callbackDMX(dmxReceived6); recv6.callbackTimeout(timeout6);
  recv7.callbackDMX(dmxReceived7); recv7.callbackTimeout(timeout7);

  // begin univers (multicast)
  recv0.begin((UNIVERSE_BASE + 0) + off);
  recv1.begin((UNIVERSE_BASE + 1) + off);
  recv2.begin((UNIVERSE_BASE + 2) + off);
  recv3.begin((UNIVERSE_BASE + 3) + off);
  recv4.begin((UNIVERSE_BASE + 4) + off);
  recv5.begin((UNIVERSE_BASE + 5) + off);
  recv6.begin((UNIVERSE_BASE + 6) + off);
  recv7.begin((UNIVERSE_BASE + 7) + off);

  lastsACN = millis();

  // Threads
  threads.addThread(sacnThread);
  threads.addThread(ledThread);
  threads.addThread(debug1LedThread);
  threads.addThread(debug2LedThread);
}

void loop()
{
  // Tout tourne dans les threads
}
