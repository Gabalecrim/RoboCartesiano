#include "rampa.h"

// passos/s -> Q32 passos/tique
static uint32_t paraQ32(uint32_t passos_s, uint32_t tick_hz) {
  return (uint32_t)(((uint64_t)passos_s << 32) / tick_hz);
}

void rampa_iniciar(Rampa *r, const RampaCfg *cfg, int32_t passos) {
  uint32_t hz = cfg->tick_hz ? cfg->tick_hz : 1;
  uint32_t vmax = cfg->vmax;
  if (vmax > hz / 2) vmax = hz / 2;  // no máximo 1 passo a cada 2 tiques
  if (vmax == 0) vmax = 1;
  uint32_t v0 = cfg->v0;
  if (v0 > vmax) v0 = vmax;
  if (v0 == 0) v0 = 1;

  r->vmax = paraQ32(vmax, hz);
  r->v0 = paraQ32(v0, hz);
  // passos/s² -> Q32 passos/tique²; pelo menos 1 para não "travar" a rampa
  if (cfg->acel == 0) {
    r->dv = 0;
  } else {
    uint64_t dv = ((uint64_t)cfg->acel << 32) / hz / hz;
    r->dv = dv ? (uint32_t)dv : 1;
  }
  r->v = r->dv ? r->v0 : r->vmax;
  r->acc = 0;
  r->dir = passos >= 0 ? 1 : -1;
  r->faltam = passos >= 0 ? passos : -passos;
}

int8_t IRAM_ATTR rampa_tick(Rampa *r) {
  if (r->faltam <= 0) return 0;

  if (r->dv) {
    uint64_t w = r->v >> 16;
    uint64_t w0 = r->v0 >> 16;
    uint64_t precisa = w > w0 ? w * w - w0 * w0 : 0;      // (v² - v0²) em Q32
    uint64_t tem = (uint64_t)r->faltam * 2u * r->dv;      // faltam · 2 · dv
    if (tem <= precisa) {
      r->v = (r->v > r->v0 + r->dv) ? r->v - r->dv : r->v0;  // freia
    } else if (r->v < r->vmax) {
      r->v = (r->vmax - r->v > r->dv) ? r->v + r->dv : r->vmax;  // acelera
    }
  }

  uint32_t antes = r->acc;
  r->acc += r->v;
  if (r->acc < antes) {  // estourou: passo
    r->faltam--;
    return r->dir;
  }
  return 0;
}

bool IRAM_ATTR rampa_ativa(const Rampa *r) { return r->faltam > 0; }

void rampa_parar(Rampa *r) {
  if (r->faltam <= 0) return;
  if (r->dv == 0) {
    r->faltam = 0;
    return;
  }
  uint64_t w = r->v >> 16;
  uint64_t w0 = r->v0 >> 16;
  uint64_t precisa = w > w0 ? w * w - w0 * w0 : 0;
  uint64_t frear = precisa / (2u * (uint64_t)r->dv) + 1;  // passos para chegar em v0
  if ((uint64_t)r->faltam > frear) r->faltam = (int32_t)frear;
}

void IRAM_ATTR rampa_abortar(Rampa *r) { r->faltam = 0; }

uint32_t rampa_velocidade(const Rampa *r, uint32_t tick_hz) {
  if (r->faltam <= 0) return 0;
  return (uint32_t)(((uint64_t)r->v * tick_hz) >> 32);
}

uint64_t rampa_tiques_total(const RampaCfg *cfg, int32_t passos) {
  Rampa r;
  rampa_iniciar(&r, cfg, passos);
  uint64_t n = 0;
  while (rampa_ativa(&r)) {
    rampa_tick(&r);
    n++;
  }
  return n;
}
