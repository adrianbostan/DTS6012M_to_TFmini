#include <Arduino.h>
#include <DTS6012M_UART.h>   // Library: DTS6012M_UART

// ----------------------------------------------------
// Enable / disable USB debug (PC Serial)
// ----------------------------------------------------
#define PC_DEBUG      // COMMENT OUT THIS LINE to disable PC debugging

// ----------------- PINS -----------------
// DTS6012M (UART1)
#define DTS_RX_PIN   4   // ESP32-C3 RX1  <-  TX from DTS6012M (pin 3)
#define DTS_TX_PIN   5   // ESP32-C3 TX1  ->  RX to DTS6012M (pin 4, optional)

// TFmini output "fake" (UART0)
#define TFMINI_TX_PIN 21  // ESP32-C3 TX0  ->  FC RX (Pixhawk etc.)

// ----------------- UART INSTANCES -----------------
HardwareSerial SensorSerial(1); // UART1 for DTS6012M
HardwareSerial TfminiSerial(0); // UART0 for fake TFmini output

// ----------------- DTS6012M OBJECT -----------------
DTS6012M_UART dts(SensorSerial);
bool dts_ok = false;   // keep track whether it initialized or not

#ifdef PC_DEBUG
// --- debug timing ---
unsigned long lastFrameMs = 0;
unsigned long lastPrintMs = 0;
uint32_t frameCount = 0;
uint32_t fps = 0;
#endif

// ----------------- SEND TFmini/TFS20-L FRAME -----------------
void sendTfminiFrame(uint16_t dist_mm, uint16_t intensity)
{
  uint16_t dist_cm;

  // 0xFFFF = invalid / out-of-range
  if (dist_mm == 0xFFFF) {
    dist_cm   = 0;
    intensity = 0;
  } else {
    dist_cm = dist_mm / 10;      // mm -> cm
    if (dist_cm > 1200) {        // TFmini standard: 0..1200 cm
      dist_cm = 1200;
    }
  }

  uint8_t frame[9];
  frame[0] = 0x59;
  frame[1] = 0x59;
  frame[2] = dist_cm & 0xFF;
  frame[3] = (dist_cm >> 8) & 0xFF;
  frame[4] = intensity & 0xFF;
  frame[5] = (intensity >> 8) & 0xFF;

  // temperature not available; keep it 0
  frame[6] = 0;
  frame[7] = 0;

  uint16_t sum = 0;
  for (int i = 0; i < 8; i++) {
    sum += frame[i];
  }
  frame[8] = sum & 0xFF;

  TfminiSerial.write(frame, 9);
}

void setup()
{
#ifdef PC_DEBUG
  Serial.begin(115200);
  // wait a bit, but don't block if no PC is connected
  unsigned long start = millis();
  while (!Serial && (millis() - start) < 1000) {
    // wait max 1s for USB to enumerate, then continue anyway
  }
  Serial.println();
  Serial.println("ESP32-C3 DTS6012M -> TFmini bridge BOOT");
#endif

  // UART1 -> DTS6012M
  SensorSerial.begin(921600, SERIAL_8N1, DTS_RX_PIN, DTS_TX_PIN);

  // give the sensor time to start (power, laser, etc.)
  delay(500);

  // try multiple times to initialize DTS6012M
  for (int i = 0; i < 20 && !dts_ok; i++) {   // ~2 seconds total
    dts_ok = dts.begin();
    if (!dts_ok) {
      delay(100);
    }
  }

  if (dts_ok) {
    // disable CRC check to accept all frames
    dts.enableCRC(false);
  }

#ifdef PC_DEBUG
  if (dts_ok) {
    Serial.println("DTS6012M initializat. CRC check dezactivat.");
  } else {
    Serial.println("ATENTIE: DTS6012M NU s-a initializat in setup, vom mai incerca in loop.");
  }
#endif

  // UART0 -> fake TFmini output (TX only)
  TfminiSerial.begin(115200, SERIAL_8N1, -1, TFMINI_TX_PIN);

#ifdef PC_DEBUG
  Serial.println("TFmini output UART ready.");
#endif
}

void loop()
{
  // if begin() failed in setup, retry periodically here
  if (!dts_ok) {
    dts_ok = dts.begin();
#ifdef PC_DEBUG
    if (dts_ok) {
      Serial.println("DTS6012M initializat cu succes din loop().");
      dts.enableCRC(false);
    }
#endif
    delay(100);
    return;  // don't try to read until it's initialized
  }

  bool newData = dts.update();   // process the stream from DTS6012M

  if (newData) {
    uint16_t dist_mm  = dts.getDistance();   // mm
    uint16_t strength = dts.getIntensity();  // intensity

    // send TFmini frame over UART to the FC
    sendTfminiFrame(dist_mm, strength);

#ifdef PC_DEBUG
    // debug timing
    unsigned long now = millis();
    unsigned long dt  = (lastFrameMs == 0) ? 0 : (now - lastFrameMs);
    lastFrameMs = now;

    frameCount++;

    // print only once per second so we don't flood USB
    if (now - lastPrintMs >= 1000) {
      fps = frameCount;
      frameCount = 0;
      lastPrintMs = now;

      Serial.print("FPS=");
      Serial.print(fps);
      Serial.print("  last dt=");
      Serial.print(dt);
      Serial.print(" ms  dist=");

      if (dist_mm == 0xFFFF) {
        Serial.print("OOR");
      } else {
        Serial.print(dist_mm);
        Serial.print(" mm (");
        Serial.print(dist_mm / 10.0f, 1);
        Serial.print(" cm)");
      }
      Serial.print("  I=");
      Serial.println(strength);
    }
#endif
  }

  delay(1);
}