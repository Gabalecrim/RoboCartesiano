#pragma once
// =============================================================
//  Sequência de bobinas do 28BYJ-48 - lógica PURA
// =============================================================
//
// Este módulo não sabe nada de ESP32, pinos ou tempo.
// Ele só guarda em que fase o motor está, conta os passos e diz
// quais das 4 bobinas (IN1..IN4 do ULN2003) devem ficar ligadas.
//
// Por ser puro, roda nos testes do PC:  pio test -e native
//
// Bobinas como 4 bits:  bit0 = IN1, bit1 = IN2, bit2 = IN3, bit3 = IN4
//   ex.: IN1 e IN2 ligadas -> 0b0011
//
// Passo completo, 2 bobinas ligadas por vez (mais torque):
//   4 fases (0, 1, 2, 3) que se repetem em ciclo.
//   Sentido positivo = fase aumentando: 0 -> 1 -> 2 -> 3 -> 0
//
// 28BYJ-48: 2048 passos por volta do eixo de saída.

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Bobinas que devem estar ligadas numa fase (0..3).
uint8_t stepseq_coils(uint8_t phase);

typedef struct StepSeq {
  uint8_t phase;     // fase atual (0..3)
  int32_t position;  // passos dados desde o reset
} StepSeq;

// Volta para a fase 0 e define a posição.
void stepseq_reset(StepSeq *s, int32_t position);

// Dá um passo: dir = +1 (positivo), -1 (negativo) ou 0 (nada).
void stepseq_step(StepSeq *s, int8_t dir);

#ifdef __cplusplus
}
#endif
