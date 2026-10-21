#pragma once
// =============================================================
//  Encoder - camada de hardware (ESP32)
// =============================================================
//
// Liga os pinos físicos ao decodificador de quadratura.
//
// ETAPA A (agora): interrupção CHANGE nos pinos A e B; a ISR lê
//                  os dois pinos e chama quadrature_update().
// ETAPA B (depois): trocar o "por dentro" pelo periférico PCNT,
//                  mantendo EXATAMENTE esta interface pública.
//                  Quem usa o Encoder não pode perceber a troca.
//
// O encoder não sabe nada de milímetros nem de motor.
// A conversão contagem -> mm fica na camada de movimento.

#include <Arduino.h>
#include <stdbool.h>
#include "quadrature.h"

#ifdef __cplusplus
extern "C" {
#endif

// Os campos são internos: use só as funções encoder_*.
typedef struct Encoder {
  uint8_t pinA;
  uint8_t pinB;
  Quadrature q;
  // Trava para ISR e loop não mexerem em q ao mesmo tempo
  portMUX_TYPE mux;
} Encoder;

// pullup = true ativa o pull-up interno (não existe nos GPIOs 34-39;
// nesses, use o módulo KY-040, que tem os resistores na placa).
void encoder_begin(Encoder *enc, uint8_t pinA, uint8_t pinB, bool pullup);

int32_t encoder_count(Encoder *enc);              // posição atual, em contagens (x4)
uint32_t encoder_errors(Encoder *enc);            // transições inválidas desde o início
void encoder_reset(Encoder *enc, int32_t value);  // define a posição atual (homing)

#ifdef __cplusplus
}
#endif
