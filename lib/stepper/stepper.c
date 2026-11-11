#include "stepper.h"

// Liga/desliga cada pino conforme os 4 bits (bit0 = IN1 ... bit3 = IN4).
static void writeCoils(const Stepper *m, uint8_t coils) {
  for (uint8_t i = 0; i < 4; i++) {
    digitalWrite(m->pins[i], (coils >> i) & 0x1);
  }
}

void stepper_begin(Stepper *m, uint8_t in1, uint8_t in2, uint8_t in3, uint8_t in4) {
  m->pins[0] = in1;
  m->pins[1] = in2;
  m->pins[2] = in3;
  m->pins[3] = in4;
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(m->pins[i], OUTPUT);
  }
  stepseq_reset(&m->seq, 0);
  stepper_set_speed(m, 400);  // velocidade segura para começar
  writeCoils(m, 0);           // começa solto
}

void stepper_set_speed(Stepper *m, uint32_t steps_per_s) {
  if (steps_per_s == 0) steps_per_s = 1;
  m->interval_us = 1000000UL / steps_per_s;
}

void stepper_hold(Stepper *m) {
  writeCoils(m, stepseq_coils(m->seq.phase));
  delayMicroseconds(m->interval_us);
}

void stepper_step(Stepper *m, int8_t dir) {
  if (dir == 0) return;
  stepseq_step(&m->seq, dir > 0 ? 1 : -1);
  writeCoils(m, stepseq_coils(m->seq.phase));
  delayMicroseconds(m->interval_us);
}

void stepper_move(Stepper *m, int32_t steps) {
  int8_t dir = (steps > 0) ? 1 : -1;
  int32_t n = (steps > 0) ? steps : -steps;

  // Liga a fase atual antes de andar, para o rotor "encaixar" nela.
  stepper_hold(m);
  for (int32_t i = 0; i < n; i++) stepper_step(m, dir);
}

int32_t stepper_position(const Stepper *m) {
  return m->seq.position;
}

void stepper_reset(Stepper *m, int32_t position) {
  // Mantém a fase: só muda o número que chamamos de "posição".
  m->seq.position = position;
}

void stepper_release(Stepper *m) {
  writeCoils(m, 0);
}

void stepper_test_coils(Stepper *m, uint8_t coils) {
  writeCoils(m, coils & 0xF);
}

uint8_t stepper_pin(const Stepper *m, uint8_t index) {
  return m->pins[index & 0x3];
}
