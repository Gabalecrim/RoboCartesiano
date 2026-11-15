#include "setpoint.h"

#include <stddef.h>

// Folga para comparar float nas bordas do curso (1 µm).
#define EPS_MM 0.001f

static int32_t arredondar(float v) { return (int32_t)(v >= 0.0f ? v + 0.5f : v - 0.5f); }

// ---------- coordenada em mm ----------

float eixo_mm_min(const EixoMm *e) { return e->home_mm; }
float eixo_mm_max(const EixoMm *e) { return e->home_mm + e->curso_mm; }

bool eixo_mm_dentro(const EixoMm *e, float mm) {
  // !(a >= b) também pega NaN
  if (!(mm >= eixo_mm_min(e) - EPS_MM)) return false;
  return mm <= eixo_mm_max(e) + EPS_MM;
}

int32_t eixo_mm_para_passos(const EixoMm *e, float mm) {
  return arredondar((float)e->afasta * (mm - e->home_mm) * e->passos_por_mm);
}

float eixo_passos_para_mm(const EixoMm *e, int32_t passos) {
  return e->home_mm + (float)e->afasta * (float)passos / e->passos_por_mm;
}

// ---------- programa ----------

void prog_limpar(Programa *p) { p->n = 0; }

ProgErro prog_conferir(const ProgLimites *l, const Setpoint *sp) {
  if (!eixo_mm_dentro(&l->x, sp->x_mm)) return PROG_X_FORA_DO_CURSO;
  if (!eixo_mm_dentro(&l->z, sp->z_mm)) return PROG_Z_FORA_DO_CURSO;
  if (!(sp->vel_mm_s >= 0.0f) || sp->vel_mm_s > l->vmax_mm_s + 1e-6f) return PROG_VEL_INVALIDA;
  if (sp->espera_ms > PROG_ESPERA_MAX_MS) return PROG_ESPERA_INVALIDA;
  return PROG_OK;
}

ProgErro prog_adicionar(Programa *p, const ProgLimites *l, Setpoint sp) {
  if (p->n >= PROG_MAX_PONTOS) return PROG_CHEIO;
  ProgErro r = prog_conferir(l, &sp);
  if (r != PROG_OK) return r;
  p->p[p->n++] = sp;
  return PROG_OK;
}

ProgErro prog_remover(Programa *p, uint8_t i) {
  if (i >= p->n) return PROG_INDICE_INVALIDO;
  for (uint8_t k = i; k + 1 < p->n; k++) p->p[k] = p->p[k + 1];
  p->n--;
  return PROG_OK;
}

ProgErro prog_validar(const Programa *p, const ProgLimites *l, uint8_t *indice) {
  if (p->n > PROG_MAX_PONTOS) {
    if (indice) *indice = PROG_MAX_PONTOS;
    return PROG_CHEIO;
  }
  for (uint8_t i = 0; i < p->n; i++) {
    ProgErro r = prog_conferir(l, &p->p[i]);
    if (r != PROG_OK) {
      if (indice) *indice = i;
      return r;
    }
  }
  return PROG_OK;
}

const char *prog_erro_nome(ProgErro e) {
  switch (e) {
    case PROG_OK: return "ok";
    case PROG_CHEIO: return "programa cheio";
    case PROG_X_FORA_DO_CURSO: return "X fora do curso";
    case PROG_Z_FORA_DO_CURSO: return "Z fora do curso";
    case PROG_VEL_INVALIDA: return "velocidade invalida";
    case PROG_ESPERA_INVALIDA: return "espera invalida";
    case PROG_INDICE_INVALIDO: return "ponto inexistente";
  }
  return "?";
}

// ---------- sequenciador ----------

// Começa o ponto 'i': ordem das fases pelo Z de partida.
static SeqAcao irPara(Seq *s, uint8_t i, float z_atual) {
  s->i = i;
  s->fase = 0;
  bool sobe = s->prog->p[i].z_mm < z_atual;  // Z menor = mais perto do topo
  s->ordem[0] = sobe ? SEQ_EIXO_Z : SEQ_EIXO_X;
  s->ordem[1] = sobe ? SEQ_EIXO_X : SEQ_EIXO_Z;
  s->estado = SEQ_MOVENDO;
  return SEQ_MOVER;
}

// Terminou o ponto atual: próximo ponto, próximo ciclo ou fim.
static SeqAcao avancar(Seq *s) {
  float z = s->prog->p[s->i].z_mm;  // onde o Z está agora
  if (s->i + 1 < s->prog->n) return irPara(s, (uint8_t)(s->i + 1), z);
  s->ciclo++;
  if (s->ciclos != 0 && s->ciclo >= s->ciclos) {
    s->estado = SEQ_FIM;
    return SEQ_NADA;
  }
  return irPara(s, 0, z);
}

SeqAcao seq_iniciar(Seq *s, const Programa *p, uint32_t ciclos, float z_atual) {
  s->prog = p;
  s->i = 0;
  s->fase = 0;
  s->ordem[0] = SEQ_EIXO_X;
  s->ordem[1] = SEQ_EIXO_Z;
  s->ciclo = 0;
  s->ciclos = ciclos;
  s->t0 = 0;
  if (p == NULL || p->n == 0) {
    s->estado = SEQ_FIM;
    return SEQ_NADA;
  }
  return irPara(s, 0, z_atual);
}

SeqAcao seq_atualizar(Seq *s, bool motor_ocupado, uint32_t agora_ms) {
  switch (s->estado) {
    case SEQ_MOVENDO:
      if (motor_ocupado) return SEQ_NADA;
      if (s->fase == 0) {
        s->fase = 1;  // primeiro eixo chegou: agora o outro
        return SEQ_MOVER;
      }
      if (s->prog->p[s->i].espera_ms == 0) return avancar(s);
      s->estado = SEQ_ESPERANDO;
      s->t0 = agora_ms;
      return SEQ_NADA;
    case SEQ_ESPERANDO:
      // Subtração em uint32_t: funciona na virada do millis().
      if (agora_ms - s->t0 < s->prog->p[s->i].espera_ms) return SEQ_NADA;
      return avancar(s);
    case SEQ_PAUSANDO:
      if (!motor_ocupado) s->estado = SEQ_PAUSADO;
      return SEQ_NADA;
    default:
      return SEQ_NADA;
  }
}

SeqAcao seq_pausar(Seq *s) {
  switch (s->estado) {
    case SEQ_MOVENDO:
      s->estado = SEQ_PAUSANDO;
      return SEQ_FREAR;
    case SEQ_ESPERANDO:
      // Já está no ponto: ao continuar, refaz a espera inteira.
      s->estado = SEQ_PAUSADO;
      return SEQ_NADA;
    default:
      return SEQ_NADA;
  }
}

SeqAcao seq_continuar(Seq *s, float z_atual) {
  if (s->estado != SEQ_PAUSADO) return SEQ_NADA;
  return irPara(s, s->i, z_atual);
}

SeqAcao seq_cancelar(Seq *s) {
  switch (s->estado) {
    case SEQ_MOVENDO:
    case SEQ_PAUSANDO:
      s->estado = SEQ_CANCELADO;
      return SEQ_FREAR;
    case SEQ_ESPERANDO:
    case SEQ_PAUSADO:
      s->estado = SEQ_CANCELADO;
      return SEQ_NADA;
    default:
      return SEQ_NADA;
  }
}

bool seq_ativo(const Seq *s) {
  return s->estado == SEQ_MOVENDO || s->estado == SEQ_ESPERANDO || s->estado == SEQ_PAUSANDO;
}

SeqEstado seq_estado(const Seq *s) { return s->estado; }

const Setpoint *seq_alvo(const Seq *s) {
  if (s->prog == NULL || s->i >= s->prog->n) return NULL;
  return &s->prog->p[s->i];
}

SeqEixo seq_eixo(const Seq *s) { return s->ordem[s->fase & 1]; }

const char *seq_estado_nome(SeqEstado e) {
  switch (e) {
    case SEQ_PARADO: return "parado";
    case SEQ_MOVENDO: return "movendo";
    case SEQ_ESPERANDO: return "esperando";
    case SEQ_PAUSANDO: return "pausando";
    case SEQ_PAUSADO: return "pausado";
    case SEQ_FIM: return "fim";
    case SEQ_CANCELADO: return "cancelado";
  }
  return "?";
}
