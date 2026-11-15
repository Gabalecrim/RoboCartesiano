#include "colisao.h"

static int32_t arredondar(float v) { return (int32_t)(v >= 0.0f ? v + 0.5f : v - 0.5f); }
static int32_t modulo(int32_t v) { return v < 0 ? -v : v; }

void colisao_init(ColisaoMonitor *c, const ColisaoCfg *cfg, float cont_por_passo) {
  c->cfg = *cfg;
  if (c->cfg.confirmar == 0) c->cfg.confirmar = 1;
  c->cont_por_passo = cont_por_passo;
  c->habilitado = true;
  c->disparou = false;
  c->referencia = false;
  c->mov = 0;
  c->m0 = c->e0 = 0;
  c->erro = c->erro_max = 0;
  c->seguidas = 0;
}

void colisao_escala(ColisaoMonitor *c, float cont_por_passo) {
  c->cont_por_passo = cont_por_passo;
  c->referencia = false;
}

void colisao_habilitar(ColisaoMonitor *c, bool on) {
  c->habilitado = on;
  c->referencia = false;
  c->seguidas = 0;
}

static void referenciar(ColisaoMonitor *c, uint32_t movimento, int32_t motor, int32_t encoder) {
  c->referencia = true;
  c->mov = movimento;
  c->m0 = motor;
  c->e0 = encoder;
  c->erro = 0;
  c->erro_max = 0;
  c->seguidas = 0;
}

bool colisao_atualizar(ColisaoMonitor *c, bool andando, uint32_t movimento, int32_t motor,
                       int32_t encoder) {
  if (!c->habilitado || c->disparou) return false;
  if (!andando) {
    // Parado: referência nova para o próximo movimento. Mantém o erro_max
    // do último movimento para consulta.
    c->referencia = false;
    c->seguidas = 0;
    return false;
  }
  if (!c->referencia || movimento != c->mov) {
    referenciar(c, movimento, motor, encoder);
    return false;
  }

  int32_t esperado = arredondar((float)(motor - c->m0) * c->cont_por_passo);
  c->erro = (encoder - c->e0) - esperado;
  int32_t a = modulo(c->erro);
  if (a > c->erro_max) c->erro_max = a;

  if (a <= c->cfg.limite_cont) {
    c->seguidas = 0;
    return false;
  }
  if (++c->seguidas < c->cfg.confirmar) return false;
  c->disparou = true;
  return true;
}

void colisao_rearmar(ColisaoMonitor *c) {
  c->disparou = false;
  c->referencia = false;
  c->seguidas = 0;
}

bool colisao_disparou(const ColisaoMonitor *c) { return c->disparou; }
int32_t colisao_erro(const ColisaoMonitor *c) { return c->erro; }
int32_t colisao_erro_max(const ColisaoMonitor *c) { return c->erro_max; }
