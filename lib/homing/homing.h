#pragma once
// =============================================================
//  Homing de um eixo - lógica PURA (máquina de estados)
// =============================================================
//
// Este módulo não sabe nada de ESP32, pinos ou tempo. A cada passo
// a camada de hardware (eixo.c) informa a chave de fim de curso e a
// contagem do encoder, e a FSM responde para onde dar o próximo passo.
//
// Por ser puro, roda nos testes do PC:  pio test -e native
//
// Convenção: +1 = sentido DA CHAVE, -1 = afastando dela. Qual giro do
// motor (f ou b) é o +1 quem decide é a camada de hardware (eixo.c).
//
// Sequência (duas aproximações, para a origem repetir sempre igual):
//
//   BUSCA_RAPIDA  -- chave acionou -->  RECUO
//     anda +1 rápido até a chave acionar
//   RECUO         -- chave soltou + 'recuo' passos -->  BUSCA_LENTA
//     anda -1 até a chave soltar e mais 'recuo' passos
//   BUSCA_LENTA   -- chave acionou -->  OK  (este ponto é a origem)
//     anda +1 devagar até a chave acionar de novo
//
// Se a chave já estiver acionada na partida, começa direto no RECUO.
//
// Erros (estado ERRO, para de andar):
//   CURSO        andou 'curso_max' passos na busca rápida sem achar a chave
//   CHAVE_PRESA  recuou 'recuo_max' passos e a chave não soltou
//   NAO_ACHOU    a busca lenta passou de 'recuo_max + recuo' sem achar a chave
//   TRAVOU       o encoder ficou parado por 'janela_trava' passos seguidos:
//                o motor está mandando passos e o eixo não gira (batente,
//                eixo preso, motor sem força ou encoder desligado)

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum HomingState {
  HOM_PARADO = 0,  // antes do homing_start
  HOM_BUSCA_RAPIDA,
  HOM_RECUO,
  HOM_BUSCA_LENTA,
  HOM_OK,
  HOM_ERRO,
} HomingState;

typedef enum HomingError {
  HOM_ERRO_NENHUM = 0,
  HOM_ERRO_CURSO,
  HOM_ERRO_CHAVE_PRESA,
  HOM_ERRO_NAO_ACHOU,
  HOM_ERRO_TRAVOU,
  HOM_ERRO_CANCELADO,  // pedido de parada (usado pela camada de hardware)
} HomingError;

typedef struct HomingConfig {
  int32_t curso_max;     // passos máximos na busca rápida (> curso do eixo)
  int32_t recuo;         // passos a mais depois que a chave solta
  int32_t recuo_max;     // passos máximos de recuo esperando a chave soltar
  int32_t janela_trava;  // passos seguidos sem o encoder mudar = travou (0 = não vigia)
} HomingConfig;

typedef struct Homing {
  HomingConfig cfg;
  HomingState state;
  HomingError error;
  int32_t passos;      // passos dados no estado atual
  int32_t apos_soltar; // passos de recuo depois que a chave soltou (-1 = ainda presa)
  int32_t enc_ref;     // última contagem do encoder vista mudando
  int32_t sem_mudar;   // passos seguidos sem o encoder mudar
} Homing;

// Em C não há valor padrão nos campos: declare com  Homing h = {0};

// Começa o homing. 'chave' e 'enc' são as leituras atuais.
void homing_start(Homing *h, const HomingConfig *cfg, bool chave, int32_t enc);

// Chamada antes de cada passo, com as leituras atuais.
// Devolve o próximo passo: +1 (para a chave), -1 (afastando) ou 0 (parar).
// Devolve 0 quando terminou: confira homing_state() (OK ou ERRO).
int8_t homing_update(Homing *h, bool chave, int32_t enc);

HomingState homing_state(const Homing *h);
HomingError homing_error(const Homing *h);
bool homing_lento(const Homing *h);  // true na BUSCA_LENTA: usar a velocidade baixa

// Textos para o Monitor Serial.
const char *homing_state_nome(HomingState s);
const char *homing_error_nome(HomingError e);

#ifdef __cplusplus
}
#endif
