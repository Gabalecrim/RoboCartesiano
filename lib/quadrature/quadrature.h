#pragma once
// =============================================================
//  Decodificador de quadratura - lógica PURA
// =============================================================
//
// Este módulo não sabe nada de ESP32, pinos ou interrupções.
// Ele só recebe o estado dos canais A e B e decide se o eixo
// andou +1, -1, nada, ou se houve uma transição impossível.
//
// Por ser puro, roda nos testes do PC:  pio test -e native
//
// Estado de 2 bits:  ab = (A << 1) | B
//
// Sentido positivo (convenção do projeto):
//      00 -> 01 -> 11 -> 10 -> 00      cada seta = +1
// Sentido negativo é a mesma sequência ao contrário (cada seta = -1).
//
// Uma volta do KY-040 = 20 ciclos completos = 80 contagens (leitura x4).

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Valor de retorno para transição impossível (pulou um estado).
#define QUAD_INVALID ((int8_t)2)

// Dado o estado anterior e o novo, devolve +1, -1, 0 ou QUAD_INVALID.
int8_t quadratureDelta(uint8_t prev, uint8_t curr);

typedef struct Quadrature {
  uint8_t prev;     // último estado AB lido
  int32_t count;    // posição acumulada, em contagens
  uint32_t errors;  // quantas transições inválidas já vieram
} Quadrature;

// Em C não há valor padrão nos campos: declare com  Quadrature q = {0};

// Define o estado inicial (lido dos pinos) e a contagem.
void quadrature_reset(Quadrature *q, uint8_t ab, int32_t value);

// Processa uma nova leitura de A e B.
void quadrature_update(Quadrature *q, uint8_t ab);

#ifdef __cplusplus
}
#endif
