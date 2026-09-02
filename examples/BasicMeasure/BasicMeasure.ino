/*
  BasicMeasure - single-shot ranging with an ERDI 905 nm module.

  Wiring (module -> board):
    VCC  -> 3.3 V or 5 V, per your model's datasheet (family A is 3.3-5 V)
    GND  -> GND
    TX   -> board RX
    RX   -> board TX

  Change ERDI_TARGET_MODEL to your model.  The baud rate comes from the model
  table, so you do not have to remember whether it is 115200 or 460800.

  SPD1200D03 is deliberately excluded from automatic baud selection: its own
  manual states 115200 bps in one place and 15200/9600 bps in another, and adds
  "Do not select a host baud rate until the delivered unit configuration is
  confirmed."  For that model, set BAUD_OVERRIDE below to the rate your unit
  actually shipped with.

  Hardware: https://erdilrf.com/models/
*/

#include <ErdiLRF.h>

/* --- configure me ------------------------------------------------------ */
static const ErdiModel ERDI_TARGET_MODEL = ERDI_LRF3000A1;
static const uint32_t BAUD_OVERRIDE = 0;   /* 0 = use the documented default */
/* ----------------------------------------------------------------------- */

/* Boards with a spare hardware UART (Mega, Due, Teensy, ESP32...) should use
   it.  On an Uno the only hardware UART is also the USB console, so this
   sketch falls back to SoftwareSerial there.

   SoftwareSerial note: it cannot reach the 460800 bps that the family-B
   (LRF-H / LRF-VB) modules use by default.  For those, either use a hardware
   UART or set the module to a lower documented rate first with setBaudB(). */
#if defined(HAVE_HWSERIAL1) || defined(ARDUINO_ARCH_ESP32) || \
    defined(ARDUINO_ARCH_SAM) || defined(ARDUINO_ARCH_SAMD)
  #define LRF_PORT Serial1
#else
  #include <SoftwareSerial.h>
  SoftwareSerial swSerial(10, 11);  /* RX = D10, TX = D11 */
  #define LRF_PORT swSerial
#endif

ErdiLRF lrf(LRF_PORT, ERDI_TARGET_MODEL);

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {
    ;  /* wait briefly for the USB console on native-USB boards */
  }

  uint32_t baud = BAUD_OVERRIDE ? BAUD_OVERRIDE : erdiDefaultBaud(ERDI_TARGET_MODEL);
  char name[16];
  erdiModelName(ERDI_TARGET_MODEL, name, sizeof(name));

  if (baud == 0) {
    Serial.print(F("ERROR: "));
    Serial.print(name);
    Serial.println(F(" has no usable documented default baud rate."));
    Serial.println(F("Its manual contradicts itself (115200 vs 15200/9600)."));
    Serial.println(F("Set BAUD_OVERRIDE to the rate your unit shipped with."));
    while (true) {
      delay(1000);
    }
  }

  LRF_PORT.begin(baud);
  lrf.setTimeout(1000);

  Serial.print(F("ERDI "));
  Serial.print(name);
  Serial.print(F(" @ "));
  Serial.print(baud);
  Serial.println(F(" bps"));
}

void loop() {
  ErdiMeasurement m;
  ErdiStatus st = lrf.measure(&m);

  switch (st) {
    case ERDI_OK:
      if (isnan(m.metres)) {
        /* SPD1200D03: the manual publishes no distance scaling, so the driver
           refuses to invent one and reports the raw count instead. */
        Serial.print(F("raw="));
        Serial.print(m.raw);
        Serial.println(F("  (no documented scaling for this model)"));
      } else {
        Serial.print(m.metres, 3);
        Serial.print(F(" m   (raw="));
        Serial.print(m.raw);
        Serial.println(F(")"));
      }
      break;
    case ERDI_ERR_OUT_OF_RANGE:
      Serial.println(F("target out of range (module reported its marker value)"));
      break;
    case ERDI_ERR_STATUS:
      Serial.print(F("measurement failed, STA=0x"));
      Serial.println(m.status, HEX);
      break;
    case ERDI_ERR_TIMEOUT:
      Serial.println(F("timeout - check wiring, baud rate and power"));
      break;
    default:
      Serial.print(F("error "));
      Serial.println((int)st);
      break;
  }

  delay(500);
}
