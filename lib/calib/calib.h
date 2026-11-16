#pragma once
// =============================================================
//  Calibração - lógica PURA
// =============================================================
//
// 1. Estatística da auto calibração: mínimo, máximo e média de uma
//    medida repetida nos ciclos (ex.: em que passo a chave acionou).
// 2. Calibração de distância: o eixo andou 'passos' para fazer
//    'mm_pedido'; a pessoa mediu 'mm_medido' de verdade. Daí saem os
//    passos/mm e as contagens/mm reais.
//
// Por ser puro, roda nos testes do PC:  pio test -e native

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---------- estatística ----------

typedef struct Estat {
  uint32_t n;
  int64_t soma;
  int32_t min;
  int32_t max;
} Estat;

void estat_reset(Estat *e);
void estat_add(Estat *e, int32_t v);
float estat_media(const Estat *e);   // 0 se vazio
int32_t estat_faixa(const Estat *e); // max - min (0 se vazio): quanto variou

// ---------- calibração de distância ----------
//
// Exemplo (TR8x8, 256 passos/mm nominal): pediu 100 mm = 25600 passos,
// mediu 99,2 mm  ->  passos/mm = 25600 / 99,2 = 258,06.
// Os sinais de 'passos' e 'contagens' não importam (usa o módulo):
// a ida pode ser em qualquer sentido.

typedef enum CalibDistErro {
  CAL_DIST_OK = 0,
  CAL_DIST_ENTRADA_INVALIDA,    // passos = 0, mm <= 0 ou valor não numérico
  CAL_DIST_FORA_DA_TOLERANCIA,  // medido longe demais do pedido: confira a medida
} CalibDistErro;

typedef struct CalibDist {
  float passos_por_mm;     // novo valor do motor
  float contagens_por_mm;  // novo valor do encoder
  float erro_pct;          // (medido - pedido) / pedido, em %
  float encoder_pct;       // contagens lidas / esperadas pelos passos, em %
                           // (100% = o encoder viu todo o giro do motor)
} CalibDist;

// 'contagens_por_passo' = resolução nominal do encoder em relação ao
// motor (ex.: 80 / 2048). <= 0 deixa encoder_pct = 0.
// 'out' é preenchido também quando a resposta é FORA_DA_TOLERANCIA,
// para mostrar os números; o chamador só deve APLICAR se for OK.
CalibDistErro calib_distancia(int32_t passos, int32_t contagens, float mm_pedido,
                              float mm_medido, float tolerancia_pct,
                              float contagens_por_passo, CalibDist *out);

const char *calib_dist_erro_nome(CalibDistErro e);

#ifdef __cplusplus
}
#endif
