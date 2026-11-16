#pragma once
// =============================================================
//  Posição pelo encoder (malha fechada do X) - lógica PURA
// =============================================================
//
// O alvo é dado em CONTAGENS do encoder, não em passos do motor.
// Os passos de cada movimento saem do erro medido no encoder:
//
//     passos = (alvo - encoder) x passos_por_cont
//
//   1. malha_ir(): calcula o 1º movimento a partir do encoder (passos
//      perdidos em movimentos anteriores já entram na conta).
//   2. O motor anda com a rampa normal.
//   3. Parado (e com o encoder assentado), malha_parou() mede o erro:
//        |erro| <= tolerância  -> CHEGOU
//        senão                 -> devolve os passos da correção
//      até 'correcoes_max' correções; depois disso -> FALHOU.
//
// Por que corrigir só parado (e não o tempo todo durante o movimento)?
//   O KY-040 dá 80 contagens/volta = 25,6 passos por contagem, e a leitura
//   tem debounce de ~1 ms. Corrigir em movimento brigaria com a rampa por
//   uma contagem de atraso. Parado, a leitura é estável.
//
// A biblioteca não sabe de motor nem de milímetros: quem chama converte.
// Por ser pura, roda nos testes do PC:  pio test -e native -f test_malha

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct MalhaCfg {
  int32_t tolerancia_cont;  // |erro| <= isso = chegou (>= 0)
  uint8_t correcoes_max;    // correções depois do 1º movimento
} MalhaCfg;

typedef enum {
  MALHA_PARADA,   // nunca foi mandada ou foi cancelada
  MALHA_MOVENDO,  // movimento ou correção em andamento
  MALHA_CHEGOU,   // erro dentro da tolerância
  MALHA_FALHOU,   // esgotou as correções fora da tolerância
} MalhaEstado;

typedef struct Malha {
  MalhaCfg cfg;
  float passos_por_cont;  // > 0
  MalhaEstado estado;
  int32_t alvo;           // contagens
  int32_t erro;           // último erro medido (alvo - encoder)
  uint8_t correcoes;      // correções feitas no movimento atual
} Malha;

void malha_init(Malha *m, const MalhaCfg *cfg, float passos_por_cont);
// Nova calibração (passos/mm ÷ contagens/mm).
void malha_escala(Malha *m, float passos_por_cont);

// Vai para 'alvo' (contagens). Devolve os passos do 1º movimento
// (0 = já está na tolerância: estado CHEGOU, não precisa mover).
int32_t malha_ir(Malha *m, int32_t alvo, int32_t encoder);

// Chamar com o motor PARADO depois de cada movimento da malha.
// Devolve os passos da próxima correção, ou 0 se acabou (CHEGOU/FALHOU).
int32_t malha_parou(Malha *m, int32_t encoder);

// Abandona o movimento (parada, alarme, homing). Estado PARADA.
void malha_cancelar(Malha *m);

bool malha_ativa(const Malha *m);  // MOVENDO
MalhaEstado malha_estado(const Malha *m);
int32_t malha_erro(const Malha *m);
int32_t malha_alvo(const Malha *m);  // alvo do último malha_ir (contagens)
uint8_t malha_correcoes(const Malha *m);
const char *malha_estado_nome(MalhaEstado e);

#ifdef __cplusplus
}
#endif
