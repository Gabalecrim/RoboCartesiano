#include "homing.h"

static void parado(HomingTrecho *t) {
  t->dir = 0;
  t->passos_max = 0;
  t->usa_sensor = false;
  t->parar_com = false;
  t->lento = false;
}

// Entra num estado e preenche o trecho correspondente.
static void entrar(Homing *h, HomingState s, HomingTrecho *t) {
  h->state = s;
  parado(t);
  switch (s) {
    case HOM_BUSCA_RAPIDA:
      t->dir = +1;
      t->passos_max = h->cfg.curso_max;
      t->usa_sensor = true;
      t->parar_com = true;  // até ACIONAR
      break;
    case HOM_RECUO:
      t->dir = -1;
      t->passos_max = h->cfg.recuo_max;
      t->usa_sensor = true;
      t->parar_com = false;  // até SOLTAR
      break;
    case HOM_RECUO_EXTRA:
      t->dir = -1;
      t->passos_max = h->cfg.recuo;
      break;
    case HOM_BUSCA_LENTA:
      t->dir = +1;
      t->passos_max = h->cfg.recuo_max + h->cfg.recuo;
      t->usa_sensor = true;
      t->parar_com = true;
      t->lento = true;
      break;
    default:
      break;  // PARADO, OK, ERRO: não anda
  }
}

static void falhar(Homing *h, HomingError e, HomingTrecho *t) {
  h->error = e;
  entrar(h, HOM_ERRO, t);
}

void homing_start(Homing *h, const HomingConfig *cfg, bool chave, HomingTrecho *t) {
  h->cfg = *cfg;
  h->error = HOM_ERRO_NENHUM;
  entrar(h, chave ? HOM_RECUO : HOM_BUSCA_RAPIDA, t);
}

void homing_fim_trecho(Homing *h, HomingFim fim, HomingTrecho *t) {
  bool andando = h->state == HOM_BUSCA_RAPIDA || h->state == HOM_RECUO ||
                 h->state == HOM_RECUO_EXTRA || h->state == HOM_BUSCA_LENTA;
  if (!andando) {
    entrar(h, h->state, t);  // OK, ERRO ou PARADO: continua parado
    return;
  }
  if (fim == HOM_FIM_TRAVOU) {
    falhar(h, HOM_ERRO_TRAVOU, t);
    return;
  }

  switch (h->state) {
    case HOM_BUSCA_RAPIDA:
      if (fim == HOM_FIM_SENSOR) entrar(h, HOM_RECUO, t);
      else falhar(h, HOM_ERRO_CURSO, t);
      break;
    case HOM_RECUO:
      if (fim != HOM_FIM_SENSOR) falhar(h, HOM_ERRO_CHAVE_PRESA, t);
      else if (h->cfg.recuo > 0) entrar(h, HOM_RECUO_EXTRA, t);
      else entrar(h, HOM_BUSCA_LENTA, t);
      break;
    case HOM_RECUO_EXTRA:
      entrar(h, HOM_BUSCA_LENTA, t);  // trecho sem sensor: terminou = andou tudo
      break;
    case HOM_BUSCA_LENTA:
      if (fim == HOM_FIM_SENSOR) entrar(h, HOM_OK, t);
      else falhar(h, HOM_ERRO_NAO_ACHOU, t);
      break;
    default:
      break;
  }
}

void homing_cancelar(Homing *h) {
  if (h->state == HOM_OK || h->state == HOM_ERRO) return;
  h->state = HOM_ERRO;
  h->error = HOM_ERRO_CANCELADO;
}

HomingState homing_state(const Homing *h) { return h->state; }
HomingError homing_error(const Homing *h) { return h->error; }

const char *homing_state_nome(HomingState s) {
  switch (s) {
    case HOM_PARADO: return "parado";
    case HOM_BUSCA_RAPIDA: return "busca rapida";
    case HOM_RECUO: return "recuo";
    case HOM_RECUO_EXTRA: return "recuo extra";
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
