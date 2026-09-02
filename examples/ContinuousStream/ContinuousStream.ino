/*
  ContinuousStream - continuous ranging from an ERDI 905 nm module.

  Family A modules are told to stream with 55 AA 89 ...; family B modules
  free-run and only need a start command on the models that document one.

  PROTOCOL HOLD: on LRF1200A1 and LRF3000A1 the manual records a continuous
  *request* function byte of 0x89 and a continuous *reply* function byte of
  0x88, and explicitly declines to authorise a response parser.  This library
  refuses to stream on those two models unless you pass true to
  startContinuous(), so the ambiguity is a decision you make rather than one
  the driver makes silently for you.  Confirm the reply byte with ERDI first:
  https://erdilrf.com/contact/

  Hardware: https://erdilrf.com/models/
*/

#include <ErdiLRF.h>

/* --- configure me ------------------------------------------------------ */
static const ErdiModel ERDI_TARGET_MODEL = ERDI_LRF100H;
static const bool ACKNOWLEDGE_PROTOCOL_HOLD = false;
/* ----------------------------------------------------------------------- */

#if defined(HAVE_HWSERIAL1) || defined(ARDUINO_ARCH_ESP32) || \
    defined(ARDUINO_ARCH_SAM) || defined(ARDUINO_ARCH_SAMD)
  #define LRF_PORT Serial1
#else
  #include <SoftwareSerial.h>
  SoftwareSerial swSerial(10, 11);
  #define LRF_PORT swSerial
#endif

ErdiLRF lrf(LRF_PORT, ERDI_TARGET_MODEL);

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {
    ;
  }

  uint32_t baud = erdiDefaultBaud(ERDI_TARGET_MODEL);
  if (baud == 0) {
    Serial.println(F("this model needs an explicit baud rate - see BasicMeasure"));
    while (true) {
      delay(1000);
    }
  }
  LRF_PORT.begin(baud);
  lrf.setTimeout(500);

  ErdiStatus st = lrf.startContinuous(ACKNOWLEDGE_PROTOCOL_HOLD);
  if (st == ERDI_ERR_PROTOCOL_HOLD) {
    Serial.println(F("This model's manual leaves the continuous reply function"));
    Serial.println(F("byte unresolved (request 0x89, recorded reply 0x88)."));
    Serial.println(F("Set ACKNOWLEDGE_PROTOCOL_HOLD to true to proceed anyway."));
    while (true) {
      delay(1000);
    }
  }

  char name[16];
  erdiModelName(ERDI_TARGET_MODEL, name, sizeof(name));
  Serial.print(F("streaming from ERDI "));
  Serial.println(name);
}

void loop() {
  ErdiMeasurement m;
  ErdiStatus st = lrf.readContinuous(&m);

  if (st == ERDI_OK) {
    Serial.println(m.metres, 3);
  } else if (st == ERDI_ERR_OUT_OF_RANGE) {
    Serial.println(F("out of range"));
  } else if (st == ERDI_ERR_TIMEOUT) {
    Serial.println(F("no frame"));
  }
}
