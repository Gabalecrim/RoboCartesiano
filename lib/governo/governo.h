#pragma once
// =============================================================
//  Governo da velocidade pelo encoder - lógica PURA
// =============================================================
//
// SETPOINT = velocidade pedida para o movimento (passos/s do motor; com o
// fuso, vira mm/s do carro). A cada amostra (motor, encoder, tempo) o
// governo mede, numa janela de ~'janela_ms':
//
//   velocidade real  = contagens do encoder / tempo   (em passos/s equivalentes)
//   velocidade motor = passos dados / tempo
//   aproveitamento   = real / motor   (100% = o carro acompanha o motor)
//
//   - QUEDA: aproveitamento < 'queda_pct' numa janela -> conta 1 queda e a
//     janela recomeça (janelas sem sobreposição). Com 'quedas_reduzir'
//     quedas SEGUIDAS (uma janela boa zera a conta), o LIMITE de rotação cai
//     para o menor entre 'reduzir_pct' (fator de desaceleração) da rotação
//     do motor e a velocidade real (devagar = mais torque no 28BYJ-48),
//     nunca abaixo de 'vel_min'.
//   - RECUPERAR: a cada 'intervalo_ms' andando sem reduzir, o limite sobe
//     'subida_pct' (fator de aceleração) do que falta até o setpoint (pelo
//     menos 'subida_min'). Chegou no setpoint = recuperou (o limite sai).
//   - SEGURAR: depois de SUBIR, a rotação fica pelo menos 'segurar_ms' sem
//     mudar: quedas e travadas nesse tempo não contam.
//   - TRAVOU (encoder parado, quem detecta é lib/colisao): gov_travou()
//     conta como uma queda; reduz com base na rotação daquele instante.
//
//   rotação
//   setpoint |‾‾‾‾\            ___‾‾‾‾‾‾‾      tentativa a cada 1 s:
//            |     \      ___/                 sobe metade do que falta
//            |      \____/
//            |       ^ queda: real < 75% do motor
//            +--------------------------------> tempo andando
//
// Avalia só com 'min_cont' contagens esperadas na janela (o KY-040 dá 1
// contagem a cada 25,6 passos: janela curta demais seria só ruído); em
// rotação baixa a janela cresce até caber (até GOV_AMOSTRAS amostras).
//
// O limite vale entre movimentos; parado não mede nem sobe.
//
// Por ser puro, roda nos testes do PC:  pio test -e native -f test_governo

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define GOV_AMOSTRAS 40  // a 50 ms: até 2 s de janela

typedef struct GovCfg {
  uint32_t vel_max;       // teto absoluto (passos/s): limite "solto"
  uint32_t vel_min;       // o limite não desce disso
  uint8_t queda_pct;      // aproveitamento abaixo disso = queda
  uint8_t reduzir_pct;    // fator de desaceleração: limite = rotação x pct / 100
  uint32_t janela_ms;     // janela mínima de medição
  int32_t min_cont;       // contagens esperadas mínimas para avaliar
  uint32_t intervalo_ms;  // tenta recuperar a cada
  uint32_t subida_min;    // passos/s mínimos por tentativa
  uint8_t quedas_reduzir; // quedas seguidas para reduzir (>= 1)
  uint8_t subida_pct;     // fator de aceleração: % do que falta subido por tentativa
  uint32_t segurar_ms;    // depois de subir, mantém pelo menos isso
} GovCfg;

typedef struct GovAmostra {
  uint32_t t;
  int32_t motor, enc;
} GovAmostra;

typedef enum {
  GOV_NADA,
  GOV_QUEDA_CONTADA,  // caiu, mas ainda não chegou em 'quedas_reduzir'
  GOV_QUEDA,      // velocidade real caiu: limite baixou
  GOV_SUBIU,      // tentativa de recuperar: limite subiu
  GOV_RECUPEROU,  // chegou no setpoint: sem limite
} GovEvento;

typedef struct Gov {
  GovCfg cfg;
  float cont_por_passo;  // contagens do encoder por passo do motor (> 0)
  uint32_t limite;       // passos/s (vel_max = sem limite)
  uint32_t setpoint;     // passos/s pedidos para o movimento
  GovAmostra a[GOV_AMOSTRAS];
  uint8_t ini, n;        // fila circular
  uint32_t calmo_ms;     // andando sem queda desde a última queda/tentativa
  float vel_real;        // passos/s pelo encoder (janela atual)
  float vel_motor;       // passos/s do motor (janela atual)
  int32_t aproveit_pct;  // última avaliação (100 se nenhuma)
  uint32_t quedas;       // reduções (total)
  uint8_t seguidas;      // quedas seguidas ainda sem reduzir
  uint32_t segurar_ms;   // tempo que falta mantendo a rotação depois de subir
  uint32_t t_ant;        // amostra anterior andando (para o tempo)
  bool tem_ant;
} Gov;

void gov_init(Gov *g, const GovCfg *cfg, float cont_por_passo);
// Troca a configuração sem perder o estado (limite, janela, contagens).
void gov_configurar(Gov *g, const GovCfg *cfg);
void gov_escala(Gov *g, float cont_por_passo);  // nova calibração
void gov_setpoint(Gov *g, uint32_t passos_s);   // velocidade pedida agora

// Uma amostra (chamar a cada ~50 ms). 'andando' = o motor está girando.
GovEvento gov_amostra(Gov *g, uint32_t t_ms, bool andando, int32_t motor, int32_t enc);

// Encoder parado com o motor a 'vel' passos/s (0 = usa o limite/setpoint).
// Conta como queda. true = reduziu agora (false: só contou, ou está segurando).
bool gov_travou(Gov *g, uint32_t vel);
uint8_t gov_seguidas(const Gov *g);       // quedas contadas sem reduzir ainda
uint32_t gov_segurando_ms(const Gov *g);  // tempo que falta segurando (0 = livre)

uint32_t gov_limite(const Gov *g);
bool gov_limitado(const Gov *g);  // limite abaixo do setpoint
uint32_t gov_setpoint_atual(const Gov *g);
float gov_vel_real(const Gov *g);   // passos/s equivalentes
float gov_vel_motor(const Gov *g);  // passos/s
int32_t gov_aproveitamento(const Gov *g);
uint32_t gov_quedas(const Gov *g);

#ifdef __cplusplus
}
#endif
