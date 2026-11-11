#pragma once
// =============================================================
//  Rampa de aceleração (perfil trapezoidal) - lógica PURA
// =============================================================
//
// Gera os passos de UM motor, chamada em tiques de tempo fixo (ex.:
// 20 kHz numa interrupção de timer). A cada tique diz se é hora de
// dar um passo.
//
//   velocidade
//    vmax |     ________
//         |    /        \          acelera 'acel' passos/s² até vmax,
//    v0   |___/          \___      anda em vmax e freia a tempo de
//         +-------------------->   chegar em v0 no último passo.
//                        passos    Movimento curto: vira triângulo.
//
// SÓ INTEIROS: no ESP32 a interrupção não pode usar ponto flutuante.
// Velocidade em Q32 "passos por tique" (2^32 = 1 passo por tique):
//   - a cada tique, acc += v; quando acc estoura (passa de 2^32), é um passo
//   - acelerar = somar dv a v a cada tique
//   - frear quando os passos que faltam <= (v² - v0²) / (2·dv)
//     (comparado sem divisão: faltam·2·dv <= w² - w0², com w = v >> 16)
//
// Por ser puro, roda nos testes do PC:  pio test -e native

#include <stdbool.h>
#include <stdint.h>

#if defined(ESP_PLATFORM)
#include "esp_attr.h"  // IRAM_ATTR: código chamado pela interrupção
#else
#define IRAM_ATTR
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef struct RampaCfg {
  uint32_t tick_hz;  // frequência dos tiques (ex.: 20000)
  uint32_t v0;       // passos/s de partida e de chegada (sem rampa abaixo disso)
  uint32_t vmax;     // passos/s de cruzeiro (limitado a tick_hz / 2)
  uint32_t acel;     // passos/s²; 0 = sem rampa (anda direto em vmax)
} RampaCfg;

typedef struct Rampa {
  uint32_t v;     // velocidade atual, Q32 passos/tique
  uint32_t v0;    // Q32
  uint32_t vmax;  // Q32
  uint32_t dv;    // aceleração, Q32 passos/tique² (0 = sem rampa)
  uint32_t acc;   // acumulador de fase: estourou = passo
  int32_t faltam; // passos que ainda faltam (>= 0)
  int8_t dir;     // +1 ou -1
} Rampa;

// Começa um movimento de 'passos' (sinal = sentido). passos = 0: fica parada.
void rampa_iniciar(Rampa *r, const RampaCfg *cfg, int32_t passos);

// Um tique. Devolve +1/-1 se deu um passo agora, 0 se não.
int8_t IRAM_ATTR rampa_tick(Rampa *r);

bool IRAM_ATTR rampa_ativa(const Rampa *r);
void rampa_parar(Rampa *r);   // freia pela rampa e para (fora da interrupção)
void IRAM_ATTR rampa_abortar(Rampa *r);  // para já, sem rampa

uint32_t rampa_velocidade(const Rampa *r, uint32_t tick_hz);  // passos/s atual

// Quantos tiques um movimento leva (simula a rampa inteira).
// Serve para saber o tempo ideal: tiques / tick_hz.
uint64_t rampa_tiques_total(const RampaCfg *cfg, int32_t passos);

#ifdef __cplusplus
}
#endif
