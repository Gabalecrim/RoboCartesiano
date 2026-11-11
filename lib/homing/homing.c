#include "homing.h"

// Muda de estado e zera o que é contado por estado.
static void entrar(Homing *h, HomingState s, int32_t enc) {
  h->state = s;
  h->passos = 0;
  h->apos_soltar = -1;
  h->enc_ref = enc;
  h->sem_mudar = 0;
}

static int8_t falhar(Homing *h, HomingError e) {
  h->state = HOM_ERRO;
  h->error = e;
  return 0;
}

// Vigia de travamento: true se o encoder ficou parado tempo demais.
// Chamada uma vez por passo dado (antes de decidir o próximo).
static bool travou(Homing *h, int32_t enc) {
  if (h->cfg.janela_trava <= 0) return false;
  if (enc != h->enc_ref) {
    h->enc_ref = enc;
    h->sem_mudar = 0;
    return false;
  }
  return ++h->sem_mudar >= h->cfg.janela_trava;
}

void homing_start(Homing *h, const HomingConfig *cfg, bool chave, int32_t enc) {
  h->cfg = *cfg;
  h->error = HOM_ERRO_NENHUM;
  entrar(h, chave ? HOM_RECUO : HOM_BUSCA_RAPIDA, enc);
}

int8_t homing_update(Homing *h, bool chave, int32_t enc) {
  switch (h->state) {
    case HOM_BUSCA_RAPIDA:
      if (chave) {
        entrar(h, HOM_RECUO, enc);
        break;  // o RECUO decide o passo abaixo
      }
      if (h->passos > 0 && travou(h, enc)) return falhar(h, HOM_ERRO_TRAVOU);
      if (h->passos >= h->cfg.curso_max) return falhar(h, HOM_ERRO_CURSO);
      h->passos++;
      return +1;

    case HOM_BUSCA_LENTA:
      if (chave) {
        h->state = HOM_OK;  // origem: o ponto exato em que acionou
        return 0;
      }
      if (h->passos > 0 && travou(h, enc)) return falhar(h, HOM_ERRO_TRAVOU);
      if (h->passos >= h->cfg.recuo_max + h->cfg.recuo) return falhar(h, HOM_ERRO_NAO_ACHOU);
      h->passos++;
      return +1;

    case HOM_PARADO:
    case HOM_OK:
    case HOM_ERRO:
    default:
      return 0;

    case HOM_RECUO:
      break;
  }

  // ---- RECUO (também chega aqui logo depois da busca rápida) ----
  if (h->passos > 0 && travou(h, enc)) return falhar(h, HOM_ERRO_TRAVOU);
  if (h->apos_soltar < 0) {
    if (!chave) {
      h->apos_soltar = 0;  // soltou: agora conta o recuo extra
    } else if (h->passos >= h->cfg.recuo_max) {
      return falhar(h, HOM_ERRO_CHAVE_PRESA);
    }
  }
  if (h->apos_soltar >= h->cfg.recuo) {
    entrar(h, HOM_BUSCA_LENTA, enc);
    return homing_update(h, chave, enc);  // primeiro passo da busca lenta
  }
  if (h->apos_soltar >= 0) h->apos_soltar++;
  h->passos++;
  return -1;
}

HomingState homing_state(const Homing *h) { return h->state; }
HomingError homing_error(const Homing *h) { return h->error; }
bool homing_lento(const Homing *h) { return h->state == HOM_BUSCA_LENTA; }

const char *homing_state_nome(HomingState s) {
  switch (s) {
    case HOM_PARADO: return "parado";
    case HOM_BUSCA_RAPIDA: return "busca rapida";
    case HOM_RECUO: return "recuo";
    case HOM_BUSCA_LENTA: return "busca lenta";
    case HOM_OK: return "ok";
    case HOM_ERRO: return "erro";
  }
  return "?";
}

const char *homing_error_nome(HomingError e) {
  switch (e) {
    case HOM_ERRO_NENHUM: return "nenhum";
    case HOM_ERRO_CURSO: return "chave nao encontrada no curso maximo";
    case HOM_ERRO_CHAVE_PRESA: return "chave nao soltou no recuo";
    case HOM_ERRO_NAO_ACHOU: return "chave nao encontrada na busca lenta";
    case HOM_ERRO_TRAVOU: return "eixo travado (encoder parado)";
    case HOM_ERRO_CANCELADO: return "cancelado";
  }
  return "?";
}
