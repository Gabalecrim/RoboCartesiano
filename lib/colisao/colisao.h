#pragma once
// =============================================================
//  Proteção contra colisão - lógica PURA
// =============================================================
//
// Ideia: durante um movimento, o encoder tem que acompanhar o motor.
//   esperado = passos dados x (contagens por passo)
//   erro     = contagens lidas - esperado      (erro de seguimento)
// Se o conjunto bate em algo, o 28BYJ-48 perde passo: o motor "conta"
// passos, o carro não anda e o erro cresce. Passou do limite em
// 'confirmar' leituras seguidas -> COLISÃO.
//
// A referência (passo e contagem de partida) é refeita a cada movimento
// novo (stepper_moves muda) e com o motor parado. Assim folga de inversão
// e pequenos desvios não se acumulam de um movimento para o outro, e um
// zero (stepper_reset/encoder_reset) entre movimentos não vira colisão.
//
// Na bancada (2026-11-10): erro normal 2 a 3 contagens; perdendo passo, 277.
//
// Por ser puro, roda nos testes do PC:  pio test -e native -f test_colisao

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct ColisaoCfg {
  int32_t limite_cont;  // |erro| acima disso conta como suspeita (contagens)
  uint8_t confirmar;    // leituras seguidas acima do limite para disparar (>= 1)
} ColisaoCfg;

typedef struct ColisaoMonitor {
  ColisaoCfg cfg;
  float cont_por_passo;  // contagens do encoder por passo do motor (> 0)
  bool habilitado;
  bool disparou;         // fica true até colisao_rearmar
  bool referencia;       // tem referência do movimento atual
  uint32_t mov;          // stepper_moves da referência
  int32_t m0, e0;        // motor e encoder na referência
  int32_t erro;          // último erro (contagens)
  int32_t erro_max;      // maior |erro| do movimento atual
  uint8_t seguidas;
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

#ifdef __cplusplus
}
#endif
