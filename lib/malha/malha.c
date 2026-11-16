#include "malha.h"

static int32_t absi(int32_t v) { return v < 0 ? -v : v; }

static int32_t arred(float v) { return (int32_t)(v >= 0.0f ? v + 0.5f : v - 0.5f); }

// Passos para andar 'erro' contagens; nunca 0 se erro != 0 (anda ao menos 1).
static int32_t passos_do_erro(const Malha *m, int32_t erro) {
  int32_t p = arred((float)erro * m->passos_por_cont);
  if (p == 0 && erro != 0) p = erro > 0 ? 1 : -1;
  return p;
}

void malha_init(Malha *m, const MalhaCfg *cfg, float passos_por_cont) {
  m->cfg = *cfg;
  if (m->cfg.tolerancia_cont < 0) m->cfg.tolerancia_cont = 0;
  m->passos_por_cont = passos_por_cont > 0.0f ? passos_por_cont : 1.0f;
  m->estado = MALHA_PARADA;
  m->alvo = 0;
  m->erro = 0;
  m->correcoes = 0;
}

void malha_escala(Malha *m, float passos_por_cont) {
  if (passos_por_cont > 0.0f) m->passos_por_cont = passos_por_cont;
}

int32_t malha_ir(Malha *m, int32_t alvo, int32_t encoder) {
  m->alvo = alvo;
  m->correcoes = 0;
  m->erro = alvo - encoder;
  if (absi(m->erro) <= m->cfg.tolerancia_cont) {
    m->estado = MALHA_CHEGOU;
    return 0;
  }
  m->estado = MALHA_MOVENDO;
  return passos_do_erro(m, m->erro);
}

int32_t malha_parou(Malha *m, int32_t encoder) {
  if (m->estado != MALHA_MOVENDO) return 0;
  m->erro = m->alvo - encoder;
  if (absi(m->erro) <= m->cfg.tolerancia_cont) {
    m->estado = MALHA_CHEGOU;
    return 0;
  }
  if (m->correcoes >= m->cfg.correcoes_max) {
    m->estado = MALHA_FALHOU;
    return 0;
  }
  m->correcoes++;
  return passos_do_erro(m, m->erro);
}

void malha_cancelar(Malha *m) { m->estado = MALHA_PARADA; }

bool malha_ativa(const Malha *m) { return m->estado == MALHA_MOVENDO; }
MalhaEstado malha_estado(const Malha *m) { return m->estado; }
int32_t malha_erro(const Malha *m) { return m->erro; }
int32_t malha_alvo(const Malha *m) { return m->alvo; }
uint8_t malha_correcoes(const Malha *m) { return m->correcoes; }

const char *malha_estado_nome(MalhaEstado e) {
  switch (e) {
    case MALHA_PARADA: return "parada";
    case MALHA_MOVENDO: return "movendo";
    case MALHA_CHEGOU: return "chegou";
    case MALHA_FALHOU: return "falhou";
  }
  return "?";
}
