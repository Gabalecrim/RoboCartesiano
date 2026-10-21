#include "stepseq.h"

uint8_t stepseq_coils(uint8_t phase) {
  static const uint8_t TABLE[4] = {
      0b0011,  // fase 0: IN1 + IN2
      0b0110,  // fase 1   TODO
      0b1100,  // fase 2   TODO
      0b1001,  // fase 3   TODO
  };
  return TABLE[phase & 0x3];
}

void stepseq_reset(StepSeq *s, int32_t position) {
  s->phase = 0;
  s->position = position;
}

void stepseq_step(StepSeq *s, int8_t dir) {
 // 1. Se dir == 0: não faz nada.
  if (dir == 0) {
    return;
  }
 //Adciona o valor de dir a fase atual e aplica o operador AND com 0x3 para garantir que a fase permaneça entre 0 e 3. 
  s->phase = (s->phase + dir) & 0x3;

 // Adiciona o valor de dir à posição atual.
  s->position += dir;

}
