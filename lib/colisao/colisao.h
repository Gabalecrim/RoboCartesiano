#pragma once
// =============================================================
//  Proteção contra colisão - lógica PURA
// =============================================================
//
// Ideia: se o conjunto bate em algo, o 28BYJ-48 pula passo. O motor
// "conta" passos, mas o carro (e o encoder) quase não andam.
//
// Como vigia (2026-11-15, por JANELAS):
//   O movimento é dividido em janelas de 'janela_cont' contagens esperadas
//   (10 contagens = 1 mm no X). No fim de cada janela compara:
//       avanço = contagens lidas na janela / contagens esperadas   (em %)
//   Avanço abaixo de 'fracao_pct' (ou no sentido contrário) em 'confirmar'
//   janelas seguidas -> COLISÃO.
//   A 1ª janela de cada movimento é ignorada: é onde a folga de inversão
//   (fuso, acoplamento) aparece.
//
// Por que não o erro TOTAL (versão anterior, limite de 6 contagens)?
//   O erro total se ACUMULA: 1% de diferença na calibração vira 19
//   contagens num curso de 190 mm, e o encoder perdendo algumas contagens
//   também soma. Isso dava alarme falso em movimento longo. A janela olha
//   só o último trecho: tolera desvio lento e pega o encoder parado.
//   Perda de passo aos poucos (sem bater) quem pega é o sequenciador, que
//   compara motor x encoder em cada ponto (SEQ_TOL_MM no main.cpp).
//
// A referência é refeita a cada movimento novo (stepper_moves muda) e com
// o motor parado: um zero (homing) entre movimentos não vira colisão.
//
// Por ser puro, roda nos testes do PC:  pio test -e native -f test_colisao

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct ColisaoCfg {
  int32_t janela_cont;  // contagens esperadas por janela (> 0)
  uint8_t fracao_pct;   // avanço mínimo do encoder na janela (% do esperado)
  uint8_t confirmar;    // janelas ruins seguidas para disparar (>= 1)
} ColisaoCfg;

typedef struct ColisaoMonitor {
  ColisaoCfg cfg;
  float cont_por_passo;  // contagens do encoder por passo do motor (> 0)
  bool habilitado;
  bool disparou;         // fica true até colisao_rearmar
  bool referencia;       // tem referência do movimento atual
  uint32_t mov;          // stepper_moves da referência
  int32_t m0, e0;        // motor e encoder no início do movimento
  int32_t mj, ej;        // motor e encoder no início da janela
  uint32_t janelas;      // janelas completas no movimento
  uint8_t seguidas;      // janelas ruins seguidas
  int32_t erro;          // erro total de seguimento (contagens), só diagnóstico
  int32_t erro_max;      // maior |erro| do movimento
  int32_t ultima_pct;    // avanço da última janela (%)
  int32_t pior_pct;      // pior avanço do movimento (%), 100 se nenhuma janela contou
  int32_t esperado_j;    // contagens esperadas na janela que disparou
  int32_t medido_j;      // contagens lidas nela
} ColisaoMonitor;

void colisao_init(ColisaoMonitor *c, const ColisaoCfg *cfg, float cont_por_passo);
// Nova calibração (contagens/mm ÷ passos/mm). Refaz a referência.
void colisao_escala(ColisaoMonitor *c, float cont_por_passo);
// Desligado não dispara (ex.: teste que força perder passo).
void colisao_habilitar(ColisaoMonitor *c, bool on);

// Chamar com frequência (a cada poucos ms) com o estado atual.
// Devolve true UMA vez, na leitura em que detectou a colisão.
bool colisao_atualizar(ColisaoMonitor *c, bool andando, uint32_t movimento, int32_t motor,
                       int32_t encoder);

// Limpa o disparo (alarme reconhecido) e refaz a referência.
void colisao_rearmar(ColisaoMonitor *c);

bool colisao_disparou(const ColisaoMonitor *c);
int32_t colisao_erro(const ColisaoMonitor *c);
int32_t colisao_erro_max(const ColisaoMonitor *c);
int32_t colisao_pior_pct(const ColisaoMonitor *c);

#ifdef __cplusplus
}
#endif
