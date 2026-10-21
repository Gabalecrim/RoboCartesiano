#pragma once
// =============================================================
//  Motor de passo 28BYJ-48 + ULN2003 - camada de hardware (ESP32)
// =============================================================
//
// Liga a sequência de bobinas (stepseq) aos 4 pinos do ULN2003.
//
// ETAPA A (agora): stepper_move() é BLOQUEANTE: dá os passos com
//                  uma espera entre eles e só volta quando termina.
// ETAPA B (depois): movimento sem bloquear (o loop continua rodando)
//                  e rampa de aceleração.
//
// O 28BYJ-48 é lento: acima de ~500 passos/s ele começa a perder passo.
// O motor não sabe nada de milímetros. A conversão fica na camada
// de movimento.

#include <Arduino.h>
#include "stepseq.h"

#ifdef __cplusplus
extern "C" {
#endif

// Os campos são internos: use só as funções stepper_*.
typedef struct Stepper {
  uint8_t pins[4];       // IN1..IN4
  StepSeq seq;
  uint32_t interval_us;  // espera entre um passo e o próximo
} Stepper;

void stepper_begin(Stepper *m, uint8_t in1, uint8_t in2, uint8_t in3, uint8_t in4);
void stepper_set_speed(Stepper *m, uint32_t steps_per_s);  // passos por segundo
void stepper_move(Stepper *m, int32_t steps);              // anda e espera terminar
int32_t stepper_position(const Stepper *m);
void stepper_reset(Stepper *m, int32_t position);          // define a posição atual
void stepper_release(Stepper *m);                          // desliga as bobinas

#ifdef __cplusplus
}
#endif
