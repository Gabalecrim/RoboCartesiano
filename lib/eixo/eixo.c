#include "eixo.h"

// Giro do motor que vai para a chave: +1 (f) ou -1 (b).
static int8_t paraChave(const EixoHomingCfg *cfg) {
  return cfg->sentido < 0 ? -1 : 1;
}

// Giro do motor em cada estado da FSM (+1 = f, -1 = b, 0 = parado).
static int8_t giroDoEstado(const EixoHomingCfg *cfg, HomingState s) {
  switch (s) {
    case HOM_BUSCA_RAPIDA:
    case HOM_BUSCA_LENTA: return paraChave(cfg);
    case HOM_RECUO: return (int8_t)-paraChave(cfg);
    default: return 0;
  }
}

static void avisar(Eixo *e, const EixoHomingCfg *cfg, EixoEventoTipo tipo,
                   HomingState estado, bool chave, bool inicio) {
  if (!cfg->evento) return;
  EixoEvento ev;
  ev.tipo = tipo;
  ev.estado = estado;
  ev.chave = chave;
  ev.giro = giroDoEstado(cfg, estado);
  ev.inicio = inicio;
  ev.motor = stepper_position(e->motor);
  ev.enc = encoder_count(e->enc);
  cfg->evento(&ev, cfg->ctx);
}

static bool cancelado(const EixoHomingCfg *cfg) {
  return cfg->cancelar && cfg->cancelar(cfg->ctx);
}

void eixo_begin(Eixo *e, Stepper *motor, Encoder *enc, uint8_t pino_chave,
                bool ativa_em_baixo, bool pullup) {
  e->motor = motor;
  e->enc = enc;
  e->pino_chave = pino_chave;
  e->ativa_em_baixo = ativa_em_baixo;
  pinMode(pino_chave, pullup ? INPUT_PULLUP : INPUT);
  debounce_init(&e->chave, eixo_chave_crua(e), EIXO_CHAVE_DEBOUNCE);
}

bool eixo_chave_crua(const Eixo *e) {
  int nivel = digitalRead(e->pino_chave);
  return e->ativa_em_baixo ? (nivel == LOW) : (nivel == HIGH);
}

bool eixo_chave(Eixo *e) {
  return debounce_update(&e->chave, eixo_chave_crua(e)) != 0;
}

HomingError eixo_home(Eixo *e, const EixoHomingCfg *cfg, EixoHomeResult *out) {
  Homing h = {0};
  out->erro = HOM_ERRO_NENHUM;
  out->passos_dados = 0;
  out->passos_na_chave = 0;
  out->enc_na_chave = 0;

  // Parte da leitura atual, já aceita (sem esperar o debounce).
  debounce_init(&e->chave, eixo_chave_crua(e), EIXO_CHAVE_DEBOUNCE);
  bool chave = debounce_level(&e->chave);
  homing_start(&h, &cfg->fsm, chave, encoder_count(e->enc));
  HomingState estado = homing_state(&h);
  avisar(e, cfg, EIXO_EV_ESTADO, estado, chave, true);

  uint32_t vel_atual = 0;
  stepper_set_speed(e->motor, cfg->vel_rapida);
  stepper_hold(e->motor);

  for (;;) {
    if (cancelado(cfg)) {
      out->erro = HOM_ERRO_CANCELADO;
      break;
    }
    bool agora = eixo_chave(e);
    if (agora != chave) {
      chave = agora;
      avisar(e, cfg, EIXO_EV_CHAVE, homing_state(&h), chave, false);
    }
    int8_t dir = homing_update(&h, chave, encoder_count(e->enc));
    if (homing_state(&h) != estado) {
      estado = homing_state(&h);
      avisar(e, cfg, EIXO_EV_ESTADO, estado, chave, false);
    }
    if (dir == 0) {
      out->erro = homing_error(&h);
      break;
    }
    uint32_t vel = homing_lento(&h) ? cfg->vel_lenta : cfg->vel_rapida;
    if (vel != vel_atual) {
      stepper_set_speed(e->motor, vel);
      vel_atual = vel;
    }
    stepper_step(e->motor, (int8_t)(dir * paraChave(cfg)));  // FSM -> giro real
    out->passos_dados++;
  }

  out->parou_em = homing_state(&h);
  if (out->erro == HOM_ERRO_NENHUM) {
    // Onde a chave acionou, medido na origem ANTERIOR. Depois zera.
    out->passos_na_chave = stepper_position(e->motor);
    out->enc_na_chave = encoder_count(e->enc);
    stepper_reset(e->motor, 0);
    encoder_reset(e->enc, 0);
  }
  stepper_release(e->motor);  // parado não precisa de corrente
  return out->erro;
}

bool eixo_andar(Eixo *e, const EixoHomingCfg *cfg, int32_t passos) {
  int8_t dir = passos > 0 ? 1 : -1;
  int32_t n = passos > 0 ? passos : -passos;
  stepper_set_speed(e->motor, cfg->vel_rapida);
  stepper_hold(e->motor);
  for (int32_t i = 0; i < n; i++) {
    if (cancelado(cfg)) return false;
    stepper_step(e->motor, dir);
  }
  return true;
}

HomingError eixo_calibrar(Eixo *e, const EixoHomingCfg *cfg, int32_t passos_ida,
                          uint32_t ciclos,
                          void (*aviso)(const EixoCalibCiclo *c, void *ctx), void *ctx,
                          EixoCalibResumo *out) {
  out->erro = HOM_ERRO_NENHUM;
  out->ciclos_ok = 0;
  estat_reset(&out->passos_na_chave);
  estat_reset(&out->enc_na_chave);
  estat_reset(&out->enc_ida);

  // Ciclo 0: homing inicial, só define a origem.
  EixoCalibCiclo c = {0};
  out->erro = eixo_home(e, cfg, &c.home);
  if (aviso) aviso(&c, ctx);
  if (out->erro != HOM_ERRO_NENHUM) return out->erro;

  // A ida é sempre afastando da chave.
  int32_t n = passos_ida >= 0 ? passos_ida : -passos_ida;
  int32_t ida = -paraChave(cfg) * n;

  for (uint32_t i = 1; i <= ciclos; i++) {
    c.ciclo = i;
    int32_t enc0 = encoder_count(e->enc);
    bool ok = eixo_andar(e, cfg, ida);
    stepper_release(e->motor);
    c.enc_ida = encoder_count(e->enc) - enc0;
    if (!ok) {
      out->erro = HOM_ERRO_CANCELADO;
      break;
    }

    out->erro = eixo_home(e, cfg, &c.home);
    if (aviso) aviso(&c, ctx);
    if (out->erro != HOM_ERRO_NENHUM) break;

    estat_add(&out->passos_na_chave, c.home.passos_na_chave);
    estat_add(&out->enc_na_chave, c.home.enc_na_chave);
    estat_add(&out->enc_ida, c.enc_ida);
    out->ciclos_ok++;
  }
  return out->erro;
}
