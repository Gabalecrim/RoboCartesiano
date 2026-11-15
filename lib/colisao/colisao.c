#include "colisao.h"

static int32_t arredondar(float v) { return (int32_t)(v >= 0.0f ? v + 0.5f : v - 0.5f); }
static int32_t modulo(int32_t v) { return v < 0 ? -v : v; }

void colisao_init(ColisaoMonitor *c, const ColisaoCfg *cfg, float cont_por_passo) {
  c->cfg = *cfg;
  if (c->cfg.confirmar == 0) c->cfg.confirmar = 1;
  if (c->cfg.janela_cont <= 0) c->cfg.janela_cont = 1;
  c->cont_por_passo = cont_por_passo;
  c->habilitado = true;
  c->disparou = false;
  c->referencia = false;
  c->mov = 0;
  c->m0 = c->e0 = c->mj = c->ej = 0;
  c->janelas = 0;
  c->seguidas = 0;
  c->erro = c->erro_max = 0;
  c->ultima_pct = c->pior_pct = 100;
  c->esperado_j = c->medido_j = 0;
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
  c->m0 = c->mj = motor;
  c->e0 = c->ej = encoder;
  c->janelas = 0;
  c->seguidas = 0;
  c->erro = c->erro_max = 0;
  c->ultima_pct = c->pior_pct = 100;
}

bool colisao_atualizar(ColisaoMonitor *c, bool andando, uint32_t movimento, int32_t motor,
                       int32_t encoder) {
  if (!c->habilitado || c->disparou) return false;
  if (!andando) {
    // Parado: referência nova para o próximo movimento. Mantém os números
    // do último movimento para consulta.
    c->referencia = false;
    c->seguidas = 0;
    return false;
  }
  if (!c->referencia || movimento != c->mov) {
    referenciar(c, movimento, motor, encoder);
    return false;
  }

  // Erro total: só para mostrar (não dispara).
  c->erro = (encoder - c->e0) - arredondar((float)(motor - c->m0) * c->cont_por_passo);
  if (modulo(c->erro) > c->erro_max) c->erro_max = modulo(c->erro);

  // Janela completa? (esperado em contagens, com sinal)
  float esperado = (float)(motor - c->mj) * c->cont_por_passo;
  float abs_esp = esperado < 0.0f ? -esperado : esperado;
  if (abs_esp < (float)c->cfg.janela_cont) return false;

  int32_t medido = encoder - c->ej;
  // Avanço no sentido certo, em % (negativo = andou ao contrário).
  int32_t pct = arredondar(100.0f * (float)medido / esperado);
  c->mj = motor;
  c->ej = encoder;
  c->janelas++;
  if (c->janelas == 1) return false;  // 1ª janela: folga de inversão

  c->ultima_pct = pct;
  if (pct < c->pior_pct) c->pior_pct = pct;
  if (pct >= (int32_t)c->cfg.fracao_pct) {
    c->seguidas = 0;
    return false;
  }
  c->esperado_j = arredondar(esperado);
  c->medido_j = medido;
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
int32_t colisao_pior_pct(const ColisaoMonitor *c) { return c->pior_pct; }
