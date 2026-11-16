#pragma once
// =============================================================
//  Debounce de um sinal digital - lógica PURA (máquina de estados)
// =============================================================
//
// Este módulo não sabe nada de ESP32, pinos ou tempo. Ele recebe
// uma amostra crua por vez (0 ou 1) e devolve o nível "limpo".
// Quem chama em intervalo fixo é a camada de hardware (encoder.c);
// por isso o tempo de debounce vira um NÚMERO DE AMOSTRAS.
//
// Por ser puro, roda nos testes do PC:  pio test -e native
//
// Regra: o nível limpo só muda quando a amostra crua fica
// diferente dele por 'required' amostras SEGUIDAS. Se no meio
// aparecer de novo o nível antigo, foi repique: a contagem zera.
//
// Máquina de estados (FSM), "raw" = amostra crua:
//
//   ESTAVEL --- raw != level ---> CONFIRMANDO   (count = 1)
//
//   CONFIRMANDO:
//     raw == level ................ repique: volta p/ ESTAVEL, count = 0
//     raw != level ................ count++
//     count chegou em required .... level = raw, volta p/ ESTAVEL
//
// Exemplo com required = 2 e amostragem de 500 us (debounce de 1 ms):
//   amostra: 0 1 2 3 4 5 6 7 8 9 10
//   cru:     0 0 1 0 1 1 1 1 0 1 1
//   limpo:   0 0 0 0 0 1 1 1 1 1 1
//   - amostra 2: repique de 1 amostra, ignorado (3 volta a 0)
//   - amostra 5: 2ª amostra seguida em 1, aceita
//   - amostra 8: repique de 1 amostra, ignorado
//
// Com 'required' amostras de período P:
//   pulsos de até (required - 1) * P são SEMPRE rejeitados;
//   níveis que duram required * P ou mais são SEMPRE aceitos.

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum DebounceState {
  DEB_ESTAVEL = 0,      // amostra crua igual ao nível limpo
  DEB_CONFIRMANDO = 1,  // amostra crua diferente: contando quantas seguidas
} DebounceState;

typedef struct Debounce {
  DebounceState state;
  uint8_t level;     // nível limpo atual (0 ou 1)
  uint8_t count;     // amostras seguidas no nível novo (só em CONFIRMANDO)
  uint8_t required;  // amostras seguidas para aceitar a mudança (>= 1)
} Debounce;

// Em C não há valor padrão nos campos: declare com  Debounce d = {0};

// Começa ESTAVEL no nível dado (lido do pino na partida).
// required = 0 é tratado como 1 (sem filtro: aceita na hora).
void debounce_init(Debounce *d, uint8_t level, uint8_t required);

// Processa uma amostra crua (qualquer valor != 0 conta como 1)
// e devolve o nível limpo DEPOIS dela.
uint8_t debounce_update(Debounce *d, uint8_t raw);

// Nível limpo atual, sem processar amostra.
uint8_t debounce_level(const Debounce *d);

#ifdef __cplusplus
}
#endif
