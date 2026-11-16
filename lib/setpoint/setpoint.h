#pragma once
// =============================================================
//  Setpoints em mm (eixos X, Z e Y) - lógica PURA
// =============================================================
//
// 1. Coordenada de cada eixo em mm (EixoMm):
//
//    X = centro do braço. Na chave do homing o centro está em 30 mm; o
//        carro anda 190 mm a partir da chave -> X de 30 a 220 mm.
//    Z = descida a partir do topo: 0 = topo (posição segura), 80 = embaixo.
//        Malha aberta: a referência (Z = 0) é feita à mão no topo.
//
//    passos do motor = afasta * (pos - home) * passos_por_mm
//    ('afasta' = giro que se afasta da referência: +1 ou -1)
//
// 2. Programa: lista de setpoints (X, Z, velocidade, espera no ponto).
//
// 3. Sequenciador: máquina de estados que percorre o programa. Não
//    mexe no motor: devolve uma AÇÃO (mover o eixo tal para o ponto i,
//    frear) e o chamador executa. Roda nos testes do PC.
//
//    Cada ponto anda em 2 FASES, um eixo por vez, para o braço não
//    arrastar em nada:
//      Z sobe (alvo acima do Z atual): Z primeiro, depois X
//      Z desce ou fica:               X primeiro, depois Z
//    Fase sem deslocamento: o chamador não liga o motor e o sequenciador
//    vê "parado" = fase concluída.
//
//      PARADO --iniciar--> MOVENDO (fase 1 -> fase 2) --chegou--> ESPERANDO --tempo--> próximo
//                            |  ^                                    |
//                         pausar |continuar                       pausar
//                            v  |                                    v
//                         PAUSANDO ---------parou--------------> PAUSADO
//      cancelar (de qualquer estado ativo) -> CANCELADO;  último ponto e ciclo -> FIM

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---------- coordenada em mm ----------

typedef struct EixoMm {
  float home_mm;        // posição (mm) na referência (X: 30, Z: 0)
  float curso_mm;       // quanto anda a partir da referência
  float passos_por_mm;  // calibração atual
  int8_t afasta;        // giro do motor que se afasta da referência: +1 ou -1
} EixoMm;

float eixo_mm_min(const EixoMm *e);  // = home
float eixo_mm_max(const EixoMm *e);  // = home + curso
bool eixo_mm_dentro(const EixoMm *e, float mm);
int32_t eixo_mm_para_passos(const EixoMm *e, float mm);  // posição absoluta do motor
float eixo_passos_para_mm(const EixoMm *e, int32_t passos);

// ---------- programa ----------

#define PROG_MAX_PONTOS 32

typedef struct Setpoint {
  float x_mm;          // centro do braço
  float z_mm;          // descida a partir do topo
  float vel_mm_s;      // 0 = velocidade padrão (a máxima de cada eixo)
  uint32_t espera_ms;  // parado no ponto antes de seguir
  // Eixo Y (2026-11-15): opcional. tem_y = false -> o Y não mexe neste ponto.
  // Anda na mesma fase do X (os dois são horizontais; o Z sobe antes / desce depois).
  bool tem_y;
  float y_mm;
} Setpoint;

typedef struct Programa {
  uint8_t n;
  Setpoint p[PROG_MAX_PONTOS];
} Programa;

// Limites para conferir um setpoint.
typedef struct ProgLimites {
  EixoMm x;
  EixoMm z;
  float vmax_mm_s;  // velocidade mais alta aceita (a do eixo mais rápido)
  EixoMm y;         // eixo Y (só conferido nos pontos com tem_y)
} ProgLimites;

typedef enum ProgErro {
  PROG_OK = 0,
  PROG_CHEIO,
  PROG_X_FORA_DO_CURSO,
  PROG_Z_FORA_DO_CURSO,
  PROG_VEL_INVALIDA,     // negativa ou acima do limite
  PROG_ESPERA_INVALIDA,  // acima de PROG_ESPERA_MAX_MS
  PROG_INDICE_INVALIDO,
  PROG_Y_FORA_DO_CURSO,
} ProgErro;

#define PROG_ESPERA_MAX_MS 600000u  // 10 min

void prog_limpar(Programa *p);
// Confere um setpoint sem guardar.
ProgErro prog_conferir(const ProgLimites *l, const Setpoint *sp);
// Acrescenta no fim (confere antes).
ProgErro prog_adicionar(Programa *p, const ProgLimites *l, Setpoint sp);
// Remove o ponto 'i' (0 = primeiro); os seguintes sobem.
ProgErro prog_remover(Programa *p, uint8_t i);
// Confere o programa inteiro (ex.: lido da flash). Em erro, 'indice' diz qual.
ProgErro prog_validar(const Programa *p, const ProgLimites *l, uint8_t *indice);

const char *prog_erro_nome(ProgErro e);

// ---------- sequenciador ----------

typedef enum SeqEstado {
  SEQ_PARADO = 0,
  SEQ_MOVENDO,
  SEQ_ESPERANDO,
  SEQ_PAUSANDO,  // freando para pausar
  SEQ_PAUSADO,
  SEQ_FIM,
  SEQ_CANCELADO,
} SeqEstado;

typedef enum SeqAcao {
  SEQ_NADA = 0,
  SEQ_MOVER,  // mover o eixo seq_eixo() para a coordenada dele em seq_alvo()
  SEQ_FREAR,  // frear pela rampa (todos os eixos)
} SeqAcao;

typedef enum SeqEixo { SEQ_EIXO_X = 0, SEQ_EIXO_Z = 1 } SeqEixo;

typedef struct Seq {
  const Programa *prog;
  SeqEstado estado;
  uint8_t i;          // ponto atual
  uint8_t fase;       // 0 ou 1
  SeqEixo ordem[2];   // eixo de cada fase
  uint32_t ciclo;     // ciclo atual (0 = primeiro)
  uint32_t ciclos;    // total; 0 = repete sem fim
  uint32_t t0;        // início da espera (ms)
} Seq;

// Começa do ponto 0. 'z_atual' = Z agora (define a ordem do 1º ponto).
// Programa vazio -> FIM e NADA.
SeqAcao seq_iniciar(Seq *s, const Programa *p, uint32_t ciclos, float z_atual);
// Chamar sempre no loop. 'motor_ocupado' = algum eixo ainda anda.
SeqAcao seq_atualizar(Seq *s, bool motor_ocupado, uint32_t agora_ms);
SeqAcao seq_pausar(Seq *s);
// Retoma o ponto atual desde o começo das fases, com a ordem refeita a
// partir de 'z_atual' (os alvos são absolutos, então funciona mesmo se
// os eixos foram movidos durante a pausa).
SeqAcao seq_continuar(Seq *s, float z_atual);
SeqAcao seq_cancelar(Seq *s);

bool seq_ativo(const Seq *s);  // MOVENDO, ESPERANDO ou PAUSANDO (motores sob controle)
SeqEstado seq_estado(const Seq *s);
const Setpoint *seq_alvo(const Seq *s);  // NULL se não houver
SeqEixo seq_eixo(const Seq *s);          // eixo da fase atual
const char *seq_estado_nome(SeqEstado e);

#ifdef __cplusplus
}
#endif
