#pragma once
// =============================================================
//  Motor de passo 28BYJ-48 + ULN2003 - camada de hardware (ESP32)
// =============================================================
//
// Liga a sequência de bobinas (stepseq) aos 4 pinos do ULN2003.
//
// ETAPA B (agora): SEM BLOQUEIO e COM RAMPA.
//   Uma interrupção de timer de hardware (STEPPER_TICK_HZ, no núcleo
//   em que stepper_begin() rodou) gera os passos de TODOS os motores
//   registrados (até STEPPER_MAX_MOTORES). Cada motor tem uma rampa
//   (lib/rampa) que decide, a cada tique, se é hora do próximo passo.
//
//   stepper_start()  manda andar e VOLTA NA HORA
//   stepper_busy()   ainda andando?
//   stepper_stop()   freia pela rampa;  stepper_abort(): para já
//   stepper_move()   atalho: start + espera terminar (bloqueante)
//
//   Parada por sensor: com stepper_sensor_arm(), a própria interrupção
//   lê o pino a cada tique (com debounce) e para o motor no ponto em
//   que o sensor ficou no estado pedido, guardando essa posição.
//
// As bobinas são escritas direto nos registradores do GPIO (as 4 de
// uma vez). O motor não sabe nada de milímetros.

#include <Arduino.h>
#include "rampa.h"
#include "stepseq.h"

#ifndef STEPPER_TICK_HZ
#define STEPPER_TICK_HZ 20000u  // tiques/s da interrupção dos motores
#endif
#define STEPPER_MAX_MOTORES 3

#ifdef __cplusplus
extern "C" {
#endif

// Perfil de velocidade (ver lib/rampa).
typedef struct StepperPerfil {
  uint32_t v0;    // passos/s de partida e chegada
  uint32_t vmax;  // passos/s de cruzeiro
  uint32_t acel;  // passos/s² (0 = sem rampa: anda direto em vmax)
} StepperPerfil;

// Os campos são internos: use só as funções stepper_*.
typedef struct Stepper {
  uint8_t pins[4];  // IN1..IN4
  // Máscaras dos registradores do GPIO para cada fase (lo = GPIO 0..31, hi = 32..39)
  uint32_t liga_lo[4], desliga_lo[4], liga_hi[4], desliga_hi[4];
  uint32_t todos_lo, todos_hi;

  StepSeq seq;            // fase e posição (atualizadas pela interrupção)
  StepperPerfil perfil;
  Rampa rampa;            // usada pela interrupção
  volatile bool ativo;
  bool soltar_ao_fim;     // desligar as bobinas quando terminar

  // Parada por sensor
  bool sensor_armado;
  uint8_t sensor_pin;
  bool sensor_ativo_alto;  // true: acionado = HIGH
  bool sensor_parar_com;   // parar quando acionado (true) ou solto (false)
  uint16_t sensor_tiques;  // tiques seguidos para valer (debounce)
  uint16_t sensor_cont;
  volatile bool sensor_disparou;
  volatile int32_t sensor_pos;  // posição em que disparou

  uint32_t interval_us;  // só para stepper_step/stepper_hold (bloqueantes)
  portMUX_TYPE mux;      // loop x interrupção
} Stepper;

// Configura os pinos e registra o motor na interrupção (liga o timer
// na primeira chamada; o núcleo de quem chamou atende a interrupção).
void stepper_begin(Stepper *m, uint8_t in1, uint8_t in2, uint8_t in3, uint8_t in4);

// ---------- perfil ----------
void stepper_set_profile(Stepper *m, const StepperPerfil *p);
void stepper_get_profile(const Stepper *m, StepperPerfil *p);
void stepper_set_speed(Stepper *m, uint32_t steps_per_s);  // só a vmax

// ---------- movimento sem bloqueio ----------
// Anda 'steps' (sinal = sentido) com o perfil atual. false se já estava andando.
bool stepper_start(Stepper *m, int32_t steps, bool soltar_ao_fim);
bool stepper_start_to(Stepper *m, int32_t alvo, bool soltar_ao_fim);  // posição absoluta
bool stepper_busy(const Stepper *m);
void stepper_stop(Stepper *m);   // freia pela rampa
void stepper_abort(Stepper *m);  // para já (as bobinas continuam ligadas)
uint32_t stepper_speed_now(Stepper *m);  // passos/s neste instante (0 parado)
void stepper_wait(const Stepper *m);     // espera parar

// Atalho bloqueante: anda e espera terminar (bobinas ficam ligadas).
void stepper_move(Stepper *m, int32_t steps);

// ---------- parada por sensor ----------
// Arma para o PRÓXIMO stepper_start: para quando o pino ficar acionado
// (parar_com = true) ou solto (false) por 'tiques' tiques seguidos.
void stepper_sensor_arm(Stepper *m, uint8_t pin, bool ativo_alto, bool parar_com, uint16_t tiques);
void stepper_sensor_disarm(Stepper *m);
// true se o último movimento parou pelo sensor; 'pos' = onde disparou.
bool stepper_sensor_fired(Stepper *m, int32_t *pos);

// ---------- posição e bobinas ----------
int32_t stepper_position(Stepper *m);
void stepper_reset(Stepper *m, int32_t position);  // define a posição atual (parado)
void stepper_release(Stepper *m);                  // para e desliga as bobinas

// Passo a passo bloqueante (diagnóstico; só com o motor parado).
void stepper_hold(Stepper *m);              // liga a fase atual e espera
void stepper_step(Stepper *m, int8_t dir);  // um passo (+1/-1) e espera

// Diagnóstico de bancada: liga as bobinas pedidas direto nos pinos
// (bit0 = IN1 ... bit3 = IN4), sem passar pela sequência e sem mudar
// a posição. Para o motor antes.
void stepper_test_coils(Stepper *m, uint8_t coils);
uint8_t stepper_pin(const Stepper *m, uint8_t index);  // GPIO de IN1..IN4 (index 0..3)

#ifdef __cplusplus
}
#endif
