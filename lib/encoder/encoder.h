#pragma once
// =============================================================
//  Encoder - camada de hardware (ESP32)
// =============================================================
//
// Liga os pinos físicos ao decodificador de quadratura.
//
// ETAPA B (agora): AMOSTRAGEM em intervalo fixo, com debounce.
//   Um timer (esp_timer) chama a amostragem a cada
//   ENCODER_SAMPLE_PERIOD_US. Em cada amostra:
//
//     pino A --> debounce A --+
//                             +--> (A << 1) | B --> quadrature_update()
//     pino B --> debounce B --+                     (validação: salto
//                                                    inválido = erro)
//
//   Por que não interrupção por borda (etapa A): o KY-040 é
//   mecânico e o contato repica; cada repique virava uma
//   interrupção, e repiques nos dois canais juntos viravam
//   transições inválidas (contagens perdidas).
//   Por que não PCNT: o filtro dele vai até ~12,8 us (1023 ciclos
//   de 80 MHz), curto demais para repique de contato (~1 ms).
//
// Interface pública igual à da etapa A: quem usa o Encoder não
// percebe a troca.
//
// O encoder não sabe nada de milímetros nem de motor.
// A conversão contagem -> mm fica na camada de movimento.

#include <Arduino.h>
#include <stdbool.h>
#include "esp_timer.h"
#include "debounce.h"
#include "quadrature.h"

// ---------- Parâmetros de amostragem ----------
// Dá para mudar sem mexer no código, em platformio.ini:
//   build_flags = -DENCODER_DEBOUNCE_US=1500
//
// Só o PERÍODO é configurado; a frequência é consequência dele
// (500 us <-> 2 kHz). Ter os dois como parâmetros deixaria
// alguém mudar um e esquecer o outro.
#ifndef ENCODER_SAMPLE_PERIOD_US
#define ENCODER_SAMPLE_PERIOD_US 500u  // intervalo entre amostras
#endif
#ifndef ENCODER_DEBOUNCE_US
#define ENCODER_DEBOUNCE_US 1000u      // nível precisa durar isso para valer
#endif

#define ENCODER_SAMPLE_RATE_HZ (1000000u / ENCODER_SAMPLE_PERIOD_US)            // 2000
#define ENCODER_DEBOUNCE_SAMPLES (ENCODER_DEBOUNCE_US / ENCODER_SAMPLE_PERIOD_US)  // 2

#ifdef __cplusplus
extern "C" {
#endif

// Medição do custo da amostragem (diagnóstico de desempenho).
typedef struct EncoderStats {
  uint32_t amostras;          // amostras desde o último encoder_stats_reset
  uint64_t soma_us;           // tempo total gasto dentro da amostragem
  uint32_t max_us;            // amostra mais demorada
  uint32_t max_intervalo_us;  // maior espaço entre duas amostras (ideal: o período)
  int8_t core;                // núcleo em que a amostragem roda (0 ou 1)
} EncoderStats;

// Os campos são internos: use só as funções encoder_*.
typedef struct Encoder {
  uint8_t pinA;
  uint8_t pinB;
  Debounce debA;
  Debounce debB;
  Quadrature q;
  esp_timer_handle_t timer;
  EncoderStats stats;
  int64_t ultima_amostra_us;  // para medir o intervalo entre amostras
  // Trava para a amostragem e o loop não mexerem nos dados ao mesmo tempo
  portMUX_TYPE mux;
} Encoder;

// pullup = true ativa o pull-up interno (não existe nos GPIOs 34-39;
// nesses, use o módulo KY-040, que tem os resistores na placa).
// Começa a amostrar na hora.
void encoder_begin(Encoder *enc, uint8_t pinA, uint8_t pinB, bool pullup);

int32_t encoder_count(Encoder *enc);              // posição atual, em contagens (x4)
uint32_t encoder_errors(Encoder *enc);            // transições inválidas desde o início
void encoder_reset(Encoder *enc, int32_t value);  // define a posição atual (homing)

// Diagnóstico de desempenho. Parado, o encoder NÃO conta: o que girar
// nesse tempo se perde (use encoder_reset depois).
void encoder_stop(Encoder *enc);
void encoder_start(Encoder *enc);
void encoder_stats(Encoder *enc, EncoderStats *out);
void encoder_stats_reset(Encoder *enc);

#ifdef __cplusplus
}
#endif
