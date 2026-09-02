/*
  ErdiLRF.h - Arduino driver for ERDI 905 nm laser rangefinder modules.

  Two published protocol families cover the whole catalogue:

    Family A - 8-byte command/response frames, header 55 AA, 115200 bps default.
               55 AA <function> <D1> <D2> <D3> <D4> <checksum>
               The transmit and reply checksums use DIFFERENT byte scopes; the
               manual warns about this explicitly.

    Family B - free-running data frames, header 5C, 460800 bps default.
               5C <dist_lo> <dist_hi> <check>            (4-byte models)
               5C <dist_lo> <dist_mid> <dist_hi> <check> (LRF1500H only)
               Command frames use header 5A.

  UNITS ARE NOT UNIFORM. The VB series reports millimetres and the H series
  reports centimetres; family A reports its ranging-range unit x10. Always use
  the model constant from ErdiModel rather than assuming a family-wide unit -
  guessing is a 10x error.

  Every constant here is transcribed from the model's own ERDI manual.

  Hardware: https://erdilrf.com/models/
  License:  MIT - Copyright (c) 2026 ERDI Technology (Chengdu)
*/

#ifndef ERDI_LRF_H
#define ERDI_LRF_H

#include <Arduino.h>
#include <Stream.h>

/* ---------------------------------------------------------------------- */
/* Models                                                                  */
/* ---------------------------------------------------------------------- */

enum ErdiModel {
  /* Family A - 55 AA, 8-byte frames, 115200 bps, raw counts are value x10 */
  ERDI_LR1000E2 = 0,
  ERDI_LR1000E4,
  ERDI_LR1200E2,
  ERDI_LR1200E4,
  ERDI_LR1500E2,
  ERDI_LR1500E4,
  ERDI_LR2000E2,
  ERDI_LR2000E4,
  ERDI_LRF1200A1,
  ERDI_LRF3000A1,
  ERDI_SPD1200N0,
  ERDI_SPD1200N2,
  ERDI_SPD1200N4,
  ERDI_SPD1200D03,

  /* Family B - 5C data frames, 460800 bps */
  ERDI_LRF50H,
  ERDI_LRF100H,
  ERDI_LRF200H,
  ERDI_LRF300H,
  ERDI_LRF600H,
  ERDI_LRF1500H,
  ERDI_LRF22VB,
  ERDI_LRF50VB,

  ERDI_MODEL_COUNT
};

enum ErdiFamily { ERDI_FAMILY_A = 0, ERDI_FAMILY_B = 1 };

/* Distance unit of one raw count. */
enum ErdiUnit {
  ERDI_UNIT_UNDOCUMENTED = 0, /* SPD1200D03: the manual states no scaling      */
  ERDI_UNIT_DECIMETRE,        /* family A: raw is the value in metres x10      */
  ERDI_UNIT_CENTIMETRE,       /* H series                                      */
  ERDI_UNIT_MILLIMETRE        /* VB series                                     */
};

enum ErdiStatus {
  ERDI_OK = 0,
  ERDI_ERR_TIMEOUT,        /* no frame arrived before the timeout             */
  ERDI_ERR_CHECKSUM,       /* a frame arrived but failed its manual checksum  */
  ERDI_ERR_STATUS,         /* module answered with STA != 1                   */
  ERDI_ERR_UNSUPPORTED,    /* operation not documented for this model         */
  ERDI_ERR_PROTOCOL_HOLD,  /* the manual itself leaves this unresolved        */
  ERDI_ERR_OUT_OF_RANGE    /* module reported its out-of-range marker         */
};

/* One model's wire facts, as published.  The table lives in flash (PROGMEM on
   AVR); erdiGetModelInfo() copies one entry into caller RAM.  The name is an
   inline array rather than a pointer so that the whole table, strings included,
   stays out of RAM -- it is 50% of an Uno's SRAM otherwise. */
struct ErdiModelInfo {
  char name[11];
  uint8_t family;          /* ErdiFamily                                      */
  uint32_t defaultBaud;    /* 0 = the manual gives no usable default          */
  uint8_t frameLen;        /* bytes per data/reply frame                      */
  uint8_t distanceBytes;
  uint8_t unit;            /* ErdiUnit                                        */
  uint32_t outOfRangeRaw;  /* 0 = the manual documents no marker              */
  uint16_t minRangeCm;
  uint32_t maxRangeCm;
  uint8_t hasStartStop;    /* family B: documents 5A 0A start/stop            */
  uint8_t continuousHold;  /* family A: reply function byte unresolved        */
};

/* Look up a model's published facts. Returns false for an unknown index. */
bool erdiGetModelInfo(ErdiModel model, ErdiModelInfo *out);

/* Default baud for a model, or 0 when the manual gives no usable default
   (SPD1200D03 - its own manual states 115200 in one place and 15200/9600 in
   another, so it must be supplied by the integrator). */
uint32_t erdiDefaultBaud(ErdiModel model);

/* Model name as a NUL-terminated string in a caller-supplied buffer. */
const char *erdiModelName(ErdiModel model, char *buffer, size_t size);

/* ---------------------------------------------------------------------- */
/* Measurement result                                                      */
/* ---------------------------------------------------------------------- */

struct ErdiMeasurement {
  uint32_t raw;         /* the distance field exactly as received            */
  float metres;         /* NAN when the manual documents no scaling          */
  uint8_t status;       /* family A STA byte; 1 = success                    */
  bool outOfRange;      /* raw reached the model's documented marker         */
  bool valid;
};

/* ---------------------------------------------------------------------- */
/* Protocol helpers - pure functions, usable without a serial port         */
/* ---------------------------------------------------------------------- */

/* Family A transmit checksum:
   "SUM(Function Code + DATA1 + DATA2 + DATA3 + DATA4), lower 8 bits" */
uint8_t erdiTxChecksumA(uint8_t function, const uint8_t data[4]);

/* Family A reply checksum - note the wider scope:
   "SUM(Frame Header (H) + Frame Header (L) + Function Code + DATA1 + DATA2 +
    DATA3 + DATA4), lower 8 bits" */
uint8_t erdiRxChecksumA(const uint8_t *frameWithoutChecksum);

/* Family B checksum, from the Check_Sum() function printed in the manual:
   "Begin with the second byte and end with the last second byte, find the
    inverse of the sum." */
uint8_t erdiCheckSumB(const uint8_t *payload, size_t length);

/* Build an 8-byte family-A request into out[8]. */
void erdiBuildCommandA(uint8_t function, const uint8_t data[4], uint8_t out[8]);

/* Build a family-B command 5A <cmd> <len> <data..> <check>.
   Returns the number of bytes written, or 0 if the buffer is too small. */
size_t erdiBuildCommandB(uint8_t cmd, const uint8_t *data, uint8_t dataLen,
                         uint8_t *out, size_t outSize);

/* Family A function codes (from the manuals' command tables). */
static const uint8_t ERDI_FN_SINGLE      = 0x88;
static const uint8_t ERDI_FN_CONTINUOUS  = 0x89;
static const uint8_t ERDI_FN_ANGLE       = 0x8A;
static const uint8_t ERDI_FN_STOP        = 0x8E;
static const uint8_t ERDI_FN_SELF_TEST   = 0x80;
static const uint8_t ERDI_FN_LD_ALWAYS_ON = 0x86;

/* Family B command bytes. */
static const uint8_t ERDI_CMD_READ_SERIAL   = 0x0D;
static const uint8_t ERDI_CMD_SET_BAUD      = 0x06;
static const uint8_t ERDI_CMD_SET_FREQUENCY = 0x0B;
static const uint8_t ERDI_CMD_READ_VERSION  = 0x16;
static const uint8_t ERDI_CMD_READ_FREQ     = 0x1B;
static const uint8_t ERDI_CMD_SWITCH_IIC    = 0x1F;
static const uint8_t ERDI_CMD_RANGING       = 0x0A;

/* ---------------------------------------------------------------------- */
/* Driver                                                                  */
/* ---------------------------------------------------------------------- */

class ErdiLRF {
 public:
  /* Works with HardwareSerial, SoftwareSerial or any other Stream.

     Note: family B runs at 460800 bps by default, which SoftwareSerial cannot
     reach on an 8-bit AVR. Either use a hardware UART, or set the module to a
     lower documented rate first (see setBaudB()). */
  explicit ErdiLRF(Stream &stream, ErdiModel model);

  ErdiModel model() const { return _model; }
  const ErdiModelInfo &info() const { return _info; }
  uint32_t defaultBaud() const { return _info.defaultBaud; }

  /* Milliseconds to wait for a frame. Default 1000. */
  void setTimeout(uint32_t ms) { _timeoutMs = ms; }

  /* Single measurement.
     Family A sends 55 AA 88 ... and waits for the reply.
     Family B waits for the next free-running data frame, starting ranging
     first on the models that document a start command. */
  ErdiStatus measure(ErdiMeasurement *out);

  /* Start / stop continuous output.
     Family A: 55 AA 89 ... / 55 AA 8E ...
     Family B: 5A 0A 02 02 00 F1 / 5A 0A 02 00 00 F3, where documented.

     startContinuous() returns ERDI_ERR_PROTOCOL_HOLD on LRF1200A1 and
     LRF3000A1 unless acknowledgeHold is true: their manuals record a request
     function byte of 0x89 and a reply function byte of 0x88 and explicitly
     withhold authorisation for a response parser. */
  ErdiStatus startContinuous(bool acknowledgeHold = false);
  void stopContinuous();

  /* Read one frame of a running continuous stream. */
  ErdiStatus readContinuous(ErdiMeasurement *out);

  /* Angle measurement, family A 0x8A.
     "Effective only for modules equipped with an angle sensor." */
  ErdiStatus measureAngle(ErdiMeasurement *out);

  /* Family A 0x86 LD constant-ON.
     Manual caution: "Do not keep the constant-ON mode enabled for long
     periods." Not documented for LRF1200A1 / LRF3000A1 / SPD1200D03. */
  ErdiStatus setLdAlwaysOn(bool enable);

  /* Family B: request a documented baud rate (9600, 19200, 38400, 115200,
     230400, 256000, 460800). The module replies at the old rate and then
     switches; change your UART afterwards. */
  ErdiStatus setBaudB(uint32_t bps);

  /* Convert a raw count to metres using this model's documented unit.
     Returns NAN when the manual documents no scaling (SPD1200D03). */
  float rawToMetres(uint32_t raw) const;

 private:
  Stream &_stream;
  ErdiModel _model;
  ErdiModelInfo _info;
  uint32_t _timeoutMs;
  bool _streaming;

  bool readFrameA(uint8_t out[8], uint8_t expectedFunction, bool anyFunction);
  bool readFrameB(uint8_t *out);
  void fillMeasurementA(const uint8_t frame[8], ErdiMeasurement *out) const;
  void fillMeasurementB(const uint8_t *frame, ErdiMeasurement *out) const;
  void writeA(uint8_t function, uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4);
};

#endif  /* ERDI_LRF_H */
