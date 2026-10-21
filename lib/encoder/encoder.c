#include "encoder.h"

// Lê os dois canais e monta o estado de 2 bits: (A << 1) | B
static uint8_t readAB(const Encoder *enc) {
  return (uint8_t)((digitalRead(enc->pinA) << 1) | digitalRead(enc->pinB));
}

// Regras da ISR: curta, sem Serial, sem delay, sem alocar memória.
static void IRAM_ATTR onChange(void *arg) {
  Encoder *self = (Encoder *)arg;
  uint8_t ab = readAB(self);
  portENTER_CRITICAL_ISR(&self->mux);
  quadrature_update(&self->q, ab);
  portEXIT_CRITICAL_ISR(&self->mux);
}

void encoder_begin(Encoder *enc, uint8_t pinA, uint8_t pinB, bool pullup) {
  enc->pinA = pinA;
  enc->pinB = pinB;
  portMUX_INITIALIZE(&enc->mux);
  pinMode(enc->pinA, pullup ? INPUT_PULLUP : INPUT);
  pinMode(enc->pinB, pullup ? INPUT_PULLUP : INPUT);

  enc->q.errors = 0;
  quadrature_reset(&enc->q, readAB(enc), 0);  // parte do estado real dos pinos

  // A mesma ISR atende os dois pinos; o ponteiro do encoder vai como
  // argumento, assim dá para ter 3 encoders com o mesmo código.
  attachInterruptArg(digitalPinToInterrupt(enc->pinA), onChange, enc, CHANGE);
  attachInterruptArg(digitalPinToInterrupt(enc->pinB), onChange, enc, CHANGE);
}

int32_t encoder_count(Encoder *enc) {
  portENTER_CRITICAL(&enc->mux);
  int32_t c = enc->q.count;
  portEXIT_CRITICAL(&enc->mux);
  return c;
}

uint32_t encoder_errors(Encoder *enc) {
  portENTER_CRITICAL(&enc->mux);
  uint32_t e = enc->q.errors;
  portEXIT_CRITICAL(&enc->mux);
  return e;
}

void encoder_reset(Encoder *enc, int32_t value) {
  uint8_t ab = readAB(enc);
  portENTER_CRITICAL(&enc->mux);
  quadrature_reset(&enc->q, ab, value);
  portEXIT_CRITICAL(&enc->mux);
}
