#include "calib.h"

#include <math.h>

// ---------- estatística ----------

void estat_reset(Estat *e) {
  e->n = 0;
  e->soma = 0;
  e->min = 0;
  e->max = 0;
}

void estat_add(Estat *e, int32_t v) {
  if (e->n == 0 || v < e->min) e->min = v;
  if (e->n == 0 || v > e->max) e->max = v;
  e->soma += v;
  e->n++;
}

float estat_media(const Estat *e) {
  return e->n ? (float)e->soma / (float)e->n : 0.0f;
}

int32_t estat_faixa(const Estat *e) {
  return e->n ? e->max - e->min : 0;
}

// ---------- calibração de distância ----------

static float modulo(int32_t v) { return v < 0 ? -(float)v : (float)v; }

CalibDistErro calib_distancia(int32_t passos, int32_t contagens, float mm_pedido,
                              float mm_medido, float tolerancia_pct,
                              float contagens_por_passo, CalibDist *out) {
  out->passos_por_mm = 0.0f;
  out->contagens_por_mm = 0.0f;
  out->erro_pct = 0.0f;
  out->encoder_pct = 0.0f;

  if (passos == 0 || !isfinite(mm_pedido) || !isfinite(mm_medido) || mm_pedido <= 0.0f ||
      mm_medido <= 0.0f) {
    return CAL_DIST_ENTRADA_INVALIDA;
  }

  float p = modulo(passos);
  float c = modulo(contagens);
  out->passos_por_mm = p / mm_medido;
  out->contagens_por_mm = c / mm_medido;
  out->erro_pct = (mm_medido - mm_pedido) / mm_pedido * 100.0f;
  if (contagens_por_passo > 0.0f) out->encoder_pct = c / (p * contagens_por_passo) * 100.0f;

  if (fabsf(out->erro_pct) > tolerancia_pct) return CAL_DIST_FORA_DA_TOLERANCIA;
  return CAL_DIST_OK;
}

const char *calib_dist_erro_nome(CalibDistErro e) {
  switch (e) {
    case CAL_DIST_OK: return "ok";
    case CAL_DIST_ENTRADA_INVALIDA: return "medida invalida";
    case CAL_DIST_FORA_DA_TOLERANCIA: return "medida longe demais do pedido (confira)";
  }
  return "?";
}
