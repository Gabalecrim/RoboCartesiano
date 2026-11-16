#pragma once
// =============================================================
//  Homing de um eixo - lógica PURA (máquina de estados por TRECHOS)
// =============================================================
//
// Este módulo não sabe nada de ESP32, pinos ou tempo. Ele planeja o
// homing em TRECHOS ("ande até a chave acionar, no máximo N passos");
// quem executa é a camada de hardware (eixo.c), com o motor sem
// bloqueio: a interrupção do motor lê a chave e para no ponto exato.
// No fim de cada trecho, eixo.c conta como terminou e a FSM devolve
// o próximo trecho.
//
// Por ser puro, roda nos testes do PC:  pio test -e native
//
// Convenção: +1 = sentido DA CHAVE, -1 = afastando dela. Qual giro do
// motor (f ou b) é o +1 quem decide é a camada de hardware (eixo.c).
//
// Sequência (duas aproximações, para a origem repetir sempre igual):
//
//   BUSCA_RAPIDA  +1 rápido até a chave ACIONAR          -> RECUO
//   RECUO         -1 até a chave SOLTAR                  -> RECUO_EXTRA
//   RECUO_EXTRA   -1 mais 'recuo' passos                 -> BUSCA_LENTA
//   BUSCA_LENTA   +1 devagar até a chave ACIONAR         -> OK (origem)
//
// Se a chave já estiver acionada na partida, começa direto no RECUO.
//
// Erros (estado ERRO):
//   CURSO        a busca rápida andou 'curso_max' sem achar a chave
//   CHAVE_PRESA  recuou 'recuo_max' e a chave não soltou
//   NAO_ACHOU    a busca lenta andou 'recuo_max + recuo' sem achar a chave
//   TRAVOU       eixo.c viu o encoder parado por 'janela_trava' passos
//   CANCELADO    pedido de parada (camada de hardware)

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum HomingState {
  HOM_PARADO = 0,  // antes do homing_start
  HOM_BUSCA_RAPIDA,
  HOM_RECUO,
  HOM_RECUO_EXTRA,
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
  int32_t janela_trava;  // passos sem o encoder mudar = travou (0 = não vigia; usado por eixo.c)
} HomingConfig;

// Como um trecho terminou.
typedef enum HomingFim {
  HOM_FIM_SENSOR = 0,  // a chave chegou no estado pedido (o motor parou ali)
  HOM_FIM_LIMITE,      // andou todos os passos do trecho
  HOM_FIM_TRAVOU,      // o encoder parou de mudar
} HomingFim;

// O que a camada de hardware deve fazer agora.
typedef struct HomingTrecho {
  int8_t dir;          // +1 para a chave, -1 afastando; 0 = acabou (veja o estado)
  int32_t passos_max;  // limite de passos do trecho
  bool usa_sensor;     // parar quando a chave ficar == parar_com
  bool parar_com;      // true = parar quando ACIONAR, false = quando SOLTAR
  bool lento;          // true: velocidade lenta (busca lenta)
} HomingTrecho;

typedef struct Homing {
  HomingConfig cfg;
  HomingState state;
  HomingError error;
} Homing;

// Em C não há valor padrão nos campos: declare com  Homing h = {0};

// Começa o homing com a leitura atual da chave e devolve o 1º trecho.
void homing_start(Homing *h, const HomingConfig *cfg, bool chave, HomingTrecho *t);

// O trecho atual terminou por 'fim'. Devolve o próximo (dir = 0: acabou).
void homing_fim_trecho(Homing *h, HomingFim fim, HomingTrecho *t);

// Marca o homing como cancelado (dir = 0 daqui em diante).
void homing_cancelar(Homing *h);

HomingState homing_state(const Homing *h);
HomingError homing_error(const Homing *h);

// Textos para o Monitor Serial.
const char *homing_state_nome(HomingState s);
const char *homing_error_nome(HomingError e);

#ifdef __cplusplus
}
#endif
