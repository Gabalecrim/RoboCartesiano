#pragma once
// =============================================================
//  Eixo - camada de hardware (ESP32): motor + encoder + fim de curso
// =============================================================
//
// Junta as peças de um eixo e roda o homing (FSM em lib/homing) e a
// auto calibração. O motor anda SEM bloquear (interrupção, lib/stepper),
// mas estas funções esperam o procedimento terminar, conferindo o
// cancelamento e o encoder enquanto o motor anda. O homing é feito em
// trechos (lib/homing): a interrupção do motor para na chave, no ponto exato.
//
// Auto calibração (eixo_calibrar):
//   1. homing                      -> define a origem (0) na chave
//   2. anda 'voltas' afastando da chave -> mede as contagens do encoder
//   3. homing de novo              -> mede em que passo/contagem a chave
//                                     acionou, comparado com a origem
//   repete 2 e 3 por 'ciclos' vezes.
//
// Leitura dos resultados de cada ciclo:
//   passos_na_chave  ~0: a origem repete e o motor não perdeu passo.
//                         Diferente de 0: perda de passo ou chave imprecisa.
//   enc_na_chave     ~0: o encoder não perdeu contagem na ida e volta.
//   enc_ida          contagens nas 'voltas' de ida (esperado: 80 por volta,
//                    com o sinal do giro de ida).

#include <Arduino.h>
#include <stdbool.h>
#include "calib.h"
#include "debounce.h"
#include "encoder.h"
#include "homing.h"
#include "stepper.h"

#ifdef __cplusplus
extern "C" {
#endif

// Debounce da chave DURANTE o homing: a interrupção do motor lê a chave
// a cada tique e só aceita depois de EIXO_CHAVE_DEBOUNCE_US seguidos.
#ifndef EIXO_CHAVE_DEBOUNCE_US
#define EIXO_CHAVE_DEBOUNCE_US 10000u  // 2 ms não bastou: a chave repicou ao acionar
#endif
// Debounce da chave FORA do homing (eixo_chave(), chamada pelo loop):
// amostras seguidas para valer.
#define EIXO_CHAVE_DEBOUNCE 3

// Avisos do homing, para a bancada imprimir o que está acontecendo.
typedef enum EixoEventoTipo {
  EIXO_EV_ESTADO = 0,  // a FSM mudou de estado (ou acabou de começar)
  EIXO_EV_CHAVE,       // a chave (já com debounce) foi apertada ou solta
} EixoEventoTipo;

typedef struct EixoEvento {
  EixoEventoTipo tipo;
  HomingState estado;  // estado atual da FSM
  bool chave;          // chave acionada?
  int8_t giro;         // giro do motor neste estado: +1 = f, -1 = b, 0 = parado
  bool inicio;         // true no primeiro aviso do homing
  int32_t motor;       // posição do motor (passos)
  int32_t enc;         // contagem do encoder
} EixoEvento;

typedef struct EixoHomingCfg {
  int8_t sentido;       // giro do motor que vai PARA a chave: +1 = f, -1 = b
  HomingConfig fsm;     // limites da FSM (em passos)
  uint32_t vel_rapida;  // passos/s na busca rápida e no recuo
  uint32_t vel_lenta;   // passos/s na busca lenta (define a precisão)
  // Opcional (pode ser NULL): chamada a cada passo; true = parar agora.
  bool (*cancelar)(void *ctx);
  // Opcional (pode ser NULL): avisa mudança de estado e da chave.
  void (*evento)(const EixoEvento *ev, void *ctx);
  void *ctx;
} EixoHomingCfg;

typedef struct EixoHomeResult {
  HomingError erro;         // HOM_ERRO_NENHUM = deu certo
  HomingState parou_em;     // estado da FSM quando terminou
  int32_t passos_na_chave;  // posição do motor quando a chave acionou (antes de zerar)
  int32_t enc_na_chave;     // contagem do encoder no mesmo instante
  uint32_t passos_dados;    // total de passos do homing
} EixoHomeResult;

typedef struct EixoCalibCiclo {
  uint32_t ciclo;     // 0 = homing inicial; 1.. = ciclos de calibração
  int32_t enc_ida;    // contagens do encoder nas 'voltas' de ida (0 no ciclo 0)
  EixoHomeResult home;
} EixoCalibCiclo;

typedef struct EixoCalibResumo {
  HomingError erro;     // primeiro erro que parou a calibração
  uint32_t ciclos_ok;   // ciclos completos (sem contar o homing inicial)
  Estat passos_na_chave;
  Estat enc_na_chave;
  Estat enc_ida;
} EixoCalibResumo;

typedef struct Eixo {
  Stepper *motor;
  Encoder *enc;
  uint8_t pino_chave;
  bool ativa_em_baixo;  // true: acionada = LOW (pull-up + chave p/ GND)
                        // false: acionada = HIGH (pull-down + chave p/ 3V3)
  Debounce chave;
} Eixo;

void eixo_begin(Eixo *e, Stepper *motor, Encoder *enc, uint8_t pino_chave,
                bool ativa_em_baixo, bool pullup);

bool eixo_chave_crua(const Eixo *e);  // leitura direta do pino (acionada?)
bool eixo_chave(Eixo *e);             // passa uma amostra pelo debounce (acionada?)

// Homing. Se der certo, zera motor e encoder na chave (origem = 0).
// Sempre termina com as bobinas soltas. Muda a velocidade do motor:
// quem chamou deve restaurar a sua depois.
HomingError eixo_home(Eixo *e, const EixoHomingCfg *cfg, EixoHomeResult *out);

// Anda 'passos' (sinal = giro: + f, - b) com o PERFIL ATUAL do motor
// (rampa), esperando terminar e conferindo o cancelamento. Termina com
// as bobinas LIGADAS (segura a posição). false = cancelado ou ocupado.
bool eixo_andar(Eixo *e, const EixoHomingCfg *cfg, int32_t passos);

// Auto calibração (ver topo). 'aviso' (pode ser NULL) é chamada ao fim
// de cada ciclo, para imprimir o andamento.
HomingError eixo_calibrar(Eixo *e, const EixoHomingCfg *cfg, int32_t passos_ida,
                          uint32_t ciclos,
                          void (*aviso)(const EixoCalibCiclo *c, void *ctx), void *ctx,
                          EixoCalibResumo *out);

#ifdef __cplusplus
}
#endif
