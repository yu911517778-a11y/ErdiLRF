/*
  ErdiLRF.cpp - see ErdiLRF.h for the protocol summary and the source policy.

  MIT - Copyright (c) 2026 ERDI Technology (Chengdu)
*/

#include "ErdiLRF.h"

#include <math.h>
#include <string.h>

#if defined(__AVR__)
  #include <avr/pgmspace.h>
  #define ERDI_PROGMEM PROGMEM
  #define ERDI_MEMCPY_P(dst, src, n) memcpy_P((dst), (src), (n))
#else
  #define ERDI_PROGMEM
  #define ERDI_MEMCPY_P(dst, src, n) memcpy((dst), (src), (n))
#endif

/* ---------------------------------------------------------------------- */
/* Model table                                                             */
/*                                                                         */
/* Ranges are in centimetres so the whole table stays in integers.         */
/* outOfRangeRaw == 0 means the manual documents no marker for that model. */
/* defaultBaud == 0 means the manual gives no usable default (SPD1200D03). */
/* ---------------------------------------------------------------------- */

static const ErdiModelInfo kModels[ERDI_MODEL_COUNT] ERDI_PROGMEM = {
  /* name          fam            baud    len dist unit                  oor      minCm  maxCm    ss hold */
  { "LR1000E2",   ERDI_FAMILY_A, 115200UL, 8, 2, ERDI_UNIT_DECIMETRE,        0UL,   400,  100000UL, 0, 0 },
  { "LR1000E4",   ERDI_FAMILY_A, 115200UL, 8, 2, ERDI_UNIT_DECIMETRE,        0UL,   400,  100000UL, 0, 0 },
  { "LR1200E2",   ERDI_FAMILY_A, 115200UL, 8, 2, ERDI_UNIT_DECIMETRE,        0UL,   400,  120000UL, 0, 0 },
  { "LR1200E4",   ERDI_FAMILY_A, 115200UL, 8, 2, ERDI_UNIT_DECIMETRE,        0UL,   400,  120000UL, 0, 0 },
  { "LR1500E2",   ERDI_FAMILY_A, 115200UL, 8, 2, ERDI_UNIT_DECIMETRE,        0UL,   400,  150000UL, 0, 0 },
  { "LR1500E4",   ERDI_FAMILY_A, 115200UL, 8, 2, ERDI_UNIT_DECIMETRE,        0UL,   400,  150000UL, 0, 0 },
  { "LR2000E2",   ERDI_FAMILY_A, 115200UL, 8, 2, ERDI_UNIT_DECIMETRE,        0UL,   400,  200000UL, 0, 0 },
  { "LR2000E4",   ERDI_FAMILY_A, 115200UL, 8, 2, ERDI_UNIT_DECIMETRE,        0UL,   400,  200000UL, 0, 0 },
  { "LRF1200A1",  ERDI_FAMILY_A, 115200UL, 8, 2, ERDI_UNIT_DECIMETRE,        0UL,   500,  120000UL, 0, 1 },
  { "LRF3000A1",  ERDI_FAMILY_A, 115200UL, 8, 2, ERDI_UNIT_DECIMETRE,        0UL,   100,  300000UL, 0, 1 },
  { "SPD1200N0",  ERDI_FAMILY_A, 115200UL, 8, 2, ERDI_UNIT_DECIMETRE,        0UL,    20,  120000UL, 0, 0 },
  { "SPD1200N2",  ERDI_FAMILY_A, 115200UL, 8, 2, ERDI_UNIT_DECIMETRE,        0UL,    20,  120000UL, 0, 0 },
  { "SPD1200N4",  ERDI_FAMILY_A, 115200UL, 8, 2, ERDI_UNIT_DECIMETRE,        0UL,    20,  120000UL, 0, 0 },
  /* SPD1200D03: its manual states 115200 bps in the protocol section and
     15200/9600 bps in the technical table, and adds "Do not select a host baud
     rate until the delivered unit configuration is confirmed."  It also prints
     no distance-scaling statement.  Both gaps are represented, not guessed. */
  { "SPD1200D03", ERDI_FAMILY_A,      0UL, 8, 2, ERDI_UNIT_UNDOCUMENTED,     0UL,    20,  120000UL, 0, 0 },

  { "LRF50H",     ERDI_FAMILY_B, 460800UL, 4, 2, ERDI_UNIT_CENTIMETRE,   65535UL,     5,    5000UL, 1, 0 },
  { "LRF100H",    ERDI_FAMILY_B, 460800UL, 4, 2, ERDI_UNIT_CENTIMETRE,   65535UL,     5,   10000UL, 0, 0 },
  { "LRF200H",    ERDI_FAMILY_B, 460800UL, 4, 2, ERDI_UNIT_CENTIMETRE,   65535UL,     5,   20000UL, 0, 0 },
  { "LRF300H",    ERDI_FAMILY_B, 460800UL, 4, 2, ERDI_UNIT_CENTIMETRE,   65535UL,     5,   30000UL, 0, 0 },
  { "LRF600H",    ERDI_FAMILY_B, 460800UL, 4, 2, ERDI_UNIT_CENTIMETRE,   65535UL,     5,   60000UL, 0, 0 },
  /* LRF1500H is the only 5-byte / 3-byte-distance model in the catalogue. */
  { "LRF1500H",   ERDI_FAMILY_B, 460800UL, 5, 3, ERDI_UNIT_CENTIMETRE, 16777215UL,    5,  150000UL, 0, 0 },
  { "LRF22VB",    ERDI_FAMILY_B, 460800UL, 4, 2, ERDI_UNIT_MILLIMETRE,   20000UL,     5,    2000UL, 1, 0 },
  { "LRF50VB",    ERDI_FAMILY_B, 460800UL, 4, 2, ERDI_UNIT_MILLIMETRE,   50000UL,     5,    5000UL, 1, 0 },
};

/* Family B baud rates: "Any other baud rate is not available". */
static const uint32_t kBaudRatesB[] = {
  9600UL, 19200UL, 38400UL, 115200UL, 230400UL, 256000UL, 460800UL
};
static const uint8_t kBaudRatesBCount =
    (uint8_t)(sizeof(kBaudRatesB) / sizeof(kBaudRatesB[0]));

bool erdiGetModelInfo(ErdiModel model, ErdiModelInfo *out) {
  if (out == NULL || model < 0 || model >= ERDI_MODEL_COUNT) {
    return false;
  }
  ERDI_MEMCPY_P(out, &kModels[model], sizeof(ErdiModelInfo));
  return true;
}

uint32_t erdiDefaultBaud(ErdiModel model) {
  ErdiModelInfo info;
  if (!erdiGetModelInfo(model, &info)) {
    return 0UL;
  }
  return info.defaultBaud;
}

const char *erdiModelName(ErdiModel model, char *buffer, size_t size) {
  if (buffer == NULL || size == 0) {
    return NULL;
  }
  ErdiModelInfo info;
  if (!erdiGetModelInfo(model, &info)) {
    buffer[0] = '\0';
    return buffer;
  }
  strncpy(buffer, info.name, size - 1);
  buffer[size - 1] = '\0';
  return buffer;
}

/* ---------------------------------------------------------------------- */
/* Checksums                                                               */
/* ---------------------------------------------------------------------- */

uint8_t erdiTxChecksumA(uint8_t function, const uint8_t data[4]) {
  uint16_t sum = function;
  for (uint8_t i = 0; i < 4; i++) {
    sum += data[i];
  }
  return (uint8_t)(sum & 0xFF);
}

uint8_t erdiRxChecksumA(const uint8_t *frameWithoutChecksum) {
  uint16_t sum = 0;
  for (uint8_t i = 0; i < 7; i++) {
    sum += frameWithoutChecksum[i];
  }
  return (uint8_t)(sum & 0xFF);
}

uint8_t erdiCheckSumB(const uint8_t *payload, size_t length) {
  uint8_t sum = 0;
  for (size_t i = 0; i < length; i++) {
    sum = (uint8_t)(sum + payload[i]);
  }
  return (uint8_t)(~sum);
}

void erdiBuildCommandA(uint8_t function, const uint8_t data[4], uint8_t out[8]) {
  out[0] = 0x55;
  out[1] = 0xAA;
  out[2] = function;
  for (uint8_t i = 0; i < 4; i++) {
    out[3 + i] = data[i];
  }
  out[7] = erdiTxChecksumA(function, data);
}

size_t erdiBuildCommandB(uint8_t cmd, const uint8_t *data, uint8_t dataLen,
                         uint8_t *out, size_t outSize) {
  size_t total = (size_t)dataLen + 4u; /* 5A + cmd + len + data + check */
  if (out == NULL || outSize < total || (cmd & 0x80) != 0) {
    return 0;
  }
  out[0] = 0x5A;
  out[1] = cmd;
  out[2] = dataLen;
  for (uint8_t i = 0; i < dataLen; i++) {
    out[3 + i] = data[i];
  }
  out[total - 1] = erdiCheckSumB(&out[1], (size_t)dataLen + 2u);
  return total;
}

/* ---------------------------------------------------------------------- */
/* Driver                                                                  */
/* ---------------------------------------------------------------------- */

ErdiLRF::ErdiLRF(Stream &stream, ErdiModel model)
    : _stream(stream), _model(model), _timeoutMs(1000UL), _streaming(false) {
  if (!erdiGetModelInfo(model, &_info)) {
    erdiGetModelInfo(ERDI_LR1000E2, &_info);
  }
}

void ErdiLRF::writeA(uint8_t function, uint8_t d1, uint8_t d2, uint8_t d3,
                     uint8_t d4) {
  uint8_t data[4];
  uint8_t frame[8];
  data[0] = d1;
  data[1] = d2;
  data[2] = d3;
  data[3] = d4;
  erdiBuildCommandA(function, data, frame);
  _stream.write(frame, sizeof(frame));
}

bool ErdiLRF::readFrameA(uint8_t out[8], uint8_t expectedFunction,
                         bool anyFunction) {
  uint8_t window[8];
  uint8_t filled = 0;
  uint32_t deadline = millis() + _timeoutMs;

  while ((int32_t)(millis() - deadline) < 0) {
    if (!_stream.available()) {
      continue;
    }
    int c = _stream.read();
    if (c < 0) {
      continue;
    }
    if (filled < 8) {
      window[filled++] = (uint8_t)c;
    } else {
      for (uint8_t i = 0; i < 7; i++) {
        window[i] = window[i + 1];
      }
      window[7] = (uint8_t)c;
    }
    if (filled < 8) {
      continue;
    }
    if (window[0] != 0x55 || window[1] != 0xAA) {
      continue;
    }
    if (window[7] != erdiRxChecksumA(window)) {
      continue; /* resync one byte later */
    }
    if (!anyFunction && window[2] != expectedFunction) {
      continue;
    }
    memcpy(out, window, 8);
    return true;
  }
  return false;
}

bool ErdiLRF::readFrameB(uint8_t *out) {
  const uint8_t len = _info.frameLen;
  uint8_t window[5];
  uint8_t filled = 0;
  uint32_t deadline = millis() + _timeoutMs;

  while ((int32_t)(millis() - deadline) < 0) {
    if (!_stream.available()) {
      continue;
    }
    int c = _stream.read();
    if (c < 0) {
      continue;
    }
    if (filled < len) {
      window[filled++] = (uint8_t)c;
    } else {
      for (uint8_t i = 0; i < (uint8_t)(len - 1); i++) {
        window[i] = window[i + 1];
      }
      window[len - 1] = (uint8_t)c;
    }
    if (filled < len) {
      continue;
    }
    if (window[0] != 0x5C) {
      continue;
    }
    if (window[len - 1] != erdiCheckSumB(&window[1], (size_t)(len - 2))) {
      continue; /* resync one byte later */
    }
    memcpy(out, window, len);
    return true;
  }
  return false;
}

float ErdiLRF::rawToMetres(uint32_t raw) const {
  switch (_info.unit) {
    case ERDI_UNIT_DECIMETRE:
      return (float)raw / 10.0f;
    case ERDI_UNIT_CENTIMETRE:
      return (float)raw / 100.0f;
    case ERDI_UNIT_MILLIMETRE:
      return (float)raw / 1000.0f;
    default:
      return NAN; /* the manual documents no scaling for this model */
  }
}

void ErdiLRF::fillMeasurementA(const uint8_t frame[8],
                               ErdiMeasurement *out) const {
  /* Reply layout: 55 | AA | fn | STA | FF | DIS_H | DIS_L | SUM */
  out->raw = ((uint32_t)frame[5] << 8) | (uint32_t)frame[6];
  out->metres = rawToMetres(out->raw);
  out->status = frame[3];
  out->outOfRange = false;
  out->valid = true;
}

void ErdiLRF::fillMeasurementB(const uint8_t *frame,
                               ErdiMeasurement *out) const {
  uint32_t raw = 0;
  for (uint8_t i = 0; i < _info.distanceBytes; i++) {
    raw |= (uint32_t)frame[1 + i] << (8 * i); /* little-endian */
  }
  out->raw = raw;
  out->metres = rawToMetres(raw);
  out->status = 1;
  out->outOfRange = (_info.outOfRangeRaw != 0UL && raw >= _info.outOfRangeRaw);
  out->valid = true;
}

ErdiStatus ErdiLRF::measure(ErdiMeasurement *out) {
  if (out == NULL) {
    return ERDI_ERR_UNSUPPORTED;
  }
  memset(out, 0, sizeof(*out));
  out->metres = NAN;

  if (_info.family == ERDI_FAMILY_A) {
    writeA(ERDI_FN_SINGLE, 0xFF, 0xFF, 0xFF, 0xFF);
    uint8_t frame[8];
    if (!readFrameA(frame, ERDI_FN_SINGLE, false)) {
      return ERDI_ERR_TIMEOUT;
    }
    fillMeasurementA(frame, out);
    /* SPD1200D03's manual prints no STA semantics, so it is not judged here. */
    if (_info.unit != ERDI_UNIT_UNDOCUMENTED && out->status != 1) {
      return ERDI_ERR_STATUS;
    }
    return ERDI_OK;
  }

  bool started = false;
  if (_info.hasStartStop && !_streaming) {
    uint8_t cmd[8];
    const uint8_t data[2] = {0x02, 0x00};
    size_t n = erdiBuildCommandB(ERDI_CMD_RANGING, data, 2, cmd, sizeof(cmd));
    _stream.write(cmd, n);
    started = true;
  }

  uint8_t frame[5];
  bool got = readFrameB(frame);

  if (started) {
    uint8_t cmd[8];
    const uint8_t data[2] = {0x00, 0x00};
    size_t n = erdiBuildCommandB(ERDI_CMD_RANGING, data, 2, cmd, sizeof(cmd));
    _stream.write(cmd, n);
  }
  if (!got) {
    return ERDI_ERR_TIMEOUT;
  }
  fillMeasurementB(frame, out);
  return out->outOfRange ? ERDI_ERR_OUT_OF_RANGE : ERDI_OK;
}

ErdiStatus ErdiLRF::startContinuous(bool acknowledgeHold) {
  if (_info.family == ERDI_FAMILY_A) {
    if (_info.continuousHold && !acknowledgeHold) {
      /* The manual records request function 0x89 and reply function 0x88 and
         explicitly withholds authorisation for a response parser. */
      return ERDI_ERR_PROTOCOL_HOLD;
    }
    writeA(ERDI_FN_CONTINUOUS, 0xFF, 0xFF, 0xFF, 0xFF);
    _streaming = true;
    return ERDI_OK;
  }
  if (_info.hasStartStop) {
    uint8_t cmd[8];
    const uint8_t data[2] = {0x02, 0x00};
    size_t n = erdiBuildCommandB(ERDI_CMD_RANGING, data, 2, cmd, sizeof(cmd));
    _stream.write(cmd, n);
  }
  /* Models without a documented start command free-run from power-up. */
  _streaming = true;
  return ERDI_OK;
}

void ErdiLRF::stopContinuous() {
  if (_info.family == ERDI_FAMILY_A) {
    writeA(ERDI_FN_STOP, 0xFF, 0xFF, 0xFF, 0xFF);
  } else if (_info.hasStartStop) {
    uint8_t cmd[8];
    const uint8_t data[2] = {0x00, 0x00};
    size_t n = erdiBuildCommandB(ERDI_CMD_RANGING, data, 2, cmd, sizeof(cmd));
    _stream.write(cmd, n);
  }
  _streaming = false;
}

ErdiStatus ErdiLRF::readContinuous(ErdiMeasurement *out) {
  if (out == NULL) {
    return ERDI_ERR_UNSUPPORTED;
  }
  memset(out, 0, sizeof(*out));
  out->metres = NAN;

  if (_info.family == ERDI_FAMILY_A) {
    uint8_t frame[8];
    /* Accept either function byte: LRF1200A1 / LRF3000A1 record 0x88 in the
       continuous reply while requesting with 0x89. */
    if (!readFrameA(frame, 0, true)) {
      return ERDI_ERR_TIMEOUT;
    }
    if (frame[2] != ERDI_FN_SINGLE && frame[2] != ERDI_FN_CONTINUOUS) {
      return ERDI_ERR_TIMEOUT;
    }
    fillMeasurementA(frame, out);
    if (_info.unit != ERDI_UNIT_UNDOCUMENTED && out->status != 1) {
      return ERDI_ERR_STATUS;
    }
    return ERDI_OK;
  }

  uint8_t frame[5];
  if (!readFrameB(frame)) {
    return ERDI_ERR_TIMEOUT;
  }
  fillMeasurementB(frame, out);
  return out->outOfRange ? ERDI_ERR_OUT_OF_RANGE : ERDI_OK;
}

ErdiStatus ErdiLRF::measureAngle(ErdiMeasurement *out) {
  if (out == NULL) {
    return ERDI_ERR_UNSUPPORTED;
  }
  /* SPD1200D03's transcription has no angle row; family B has no angle at all. */
  if (_info.family != ERDI_FAMILY_A || _model == ERDI_SPD1200D03) {
    return ERDI_ERR_UNSUPPORTED;
  }
  memset(out, 0, sizeof(*out));
  out->metres = NAN;

  writeA(ERDI_FN_ANGLE, 0xFF, 0xFF, 0xFF, 0xFF);
  uint8_t frame[8];
  if (!readFrameA(frame, ERDI_FN_ANGLE, false)) {
    return ERDI_ERR_TIMEOUT;
  }
  fillMeasurementA(frame, out);
  return (out->status == 1) ? ERDI_OK : ERDI_ERR_STATUS;
}

ErdiStatus ErdiLRF::setLdAlwaysOn(bool enable) {
  /* Documented for the LR series and the SPD1200N series only. */
  if (_info.family != ERDI_FAMILY_A || _model == ERDI_SPD1200D03 ||
      _model == ERDI_LRF1200A1 || _model == ERDI_LRF3000A1) {
    return ERDI_ERR_UNSUPPORTED;
  }
  writeA(ERDI_FN_LD_ALWAYS_ON, 0xFF, 0xFF, 0xFF, enable ? 0x01 : 0x00);
  return ERDI_OK;
}

ErdiStatus ErdiLRF::setBaudB(uint32_t bps) {
  if (_info.family != ERDI_FAMILY_B) {
    return ERDI_ERR_UNSUPPORTED;
  }
  bool known = false;
  for (uint8_t i = 0; i < kBaudRatesBCount; i++) {
    if (kBaudRatesB[i] == bps) {
      known = true;
      break;
    }
  }
  if (!known) {
    return ERDI_ERR_UNSUPPORTED; /* "Any other baud rate is not available" */
  }
  uint16_t code = (uint16_t)(bps / 100UL);
  uint8_t data[2];
  data[0] = (uint8_t)(code & 0xFF);
  data[1] = (uint8_t)(code >> 8);
  uint8_t cmd[8];
  size_t n = erdiBuildCommandB(ERDI_CMD_SET_BAUD, data, 2, cmd, sizeof(cmd));
  _stream.write(cmd, n);
  return ERDI_OK;
}
