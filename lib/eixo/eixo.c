#include "eixo.h"

// Tiques da interrupção dos motores para a chave valer (debounce).
#define CHAVE_TIQUES ((uint16_t)((uint64_t)EIXO_CHAVE_DEBOUNCE_US * STEPPER_TICK_HZ / 1000000u))

// Giro do motor que vai para a chave: +1 (f) ou -1 (b).
static int8_t paraChave(const EixoHomingCfg *cfg) {
  return cfg->sentido < 0 ? -1 : 1;
}

// Giro do motor em cada estado da FSM (+1 = f, -1 = b, 0 = parado).
static int8_t giroDoEstado(const EixoHomingCfg *cfg, HomingState s) {
  switch (s) {
    case HOM_BUSCA_RAPIDA:
    case HOM_BUSCA_LENTA: return paraChave(cfg);
    case HOM_RECUO:
    case HOM_RECUO_EXTRA: return (int8_t)-paraChave(cfg);
    default: return 0;
  }
}

static void avisar(Eixo *e, const EixoHomingCfg *cfg, EixoEventoTipo tipo, HomingState estado,
                   bool chave, bool inicio, int32_t motor) {
  if (!cfg->evento) return;
  EixoEvento ev;
  ev.tipo = tipo;
  ev.estado = estado;
  ev.chave = chave;
  ev.giro = giroDoEstado(cfg, estado);
  ev.inicio = inicio;
  ev.motor = motor;
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

// Espera o motor terminar o movimento já iniciado, vigiando o cancelamento
// e (se janela > 0) o encoder: 'janela' passos sem ele mudar = travou.
typedef enum { ESPERA_FIM, ESPERA_TRAVOU, ESPERA_CANCELADO } Espera;

static Espera esperar(Eixo *e, const EixoHomingCfg *cfg, int32_t janela) {
  int32_t pos_ck = stepper_position(e->motor);
  int32_t enc_ck = encoder_count(e->enc);
  while (stepper_busy(e->motor)) {
    if (cancelado(cfg)) {
      stepper_abort(e->motor);
      return ESPERA_CANCELADO;
    }
    if (janela > 0) {
      int32_t pos = stepper_position(e->motor);
      int32_t andou = pos > pos_ck ? pos - pos_ck : pos_ck - pos;
      if (andou >= janela) {
        int32_t enc = encoder_count(e->enc);
        if (enc == enc_ck) {
          stepper_abort(e->motor);
          return ESPERA_TRAVOU;
        }
        pos_ck = pos;
        enc_ck = enc;
      }
    }
    delay(1);
  }
  return ESPERA_FIM;
}

HomingError eixo_home(Eixo *e, const EixoHomingCfg *cfg, EixoHomeResult *out) {
  Homing h = {0};
  HomingTrecho t;
  out->erro = HOM_ERRO_NENHUM;
  out->passos_dados = 0;
  out->passos_na_chave = 0;
  out->enc_na_chave = 0;

  stepper_abort(e->motor);
  StepperPerfil antes;
  stepper_get_profile(e->motor, &antes);

  // Parte da leitura atual, já aceita (sem esperar o debounce).
  debounce_init(&e->chave, eixo_chave_crua(e), EIXO_CHAVE_DEBOUNCE);
  bool chave = debounce_level(&e->chave);
  homing_start(&h, &cfg->fsm, chave, &t);
  avisar(e, cfg, EIXO_EV_ESTADO, homing_state(&h), chave, true, stepper_position(e->motor));

  int32_t na_chave = 0;
  while (t.dir != 0) {
    // Homing sem rampa: velocidade constante, para a origem repetir.
    uint32_t v = t.lento ? cfg->vel_lenta : cfg->vel_rapida;
    StepperPerfil p = {v, v, 0};
    stepper_set_profile(e->motor, &p);
    if (t.usa_sensor) {
      stepper_sensor_arm(e->motor, e->pino_chave, !e->ativa_em_baixo, t.parar_com, CHAVE_TIQUES);
    } else {
      stepper_sensor_disarm(e->motor);
    }

    int32_t pos0 = stepper_position(e->motor);
    stepper_start(e->motor, (int32_t)(t.dir * paraChave(cfg)) * t.passos_max, false);
    Espera r = esperar(e, cfg, cfg->fsm.janela_trava);
    int32_t pos1 = stepper_position(e->motor);
    out->passos_dados += (uint32_t)(pos1 > pos0 ? pos1 - pos0 : pos0 - pos1);

    if (r == ESPERA_CANCELADO) {
      homing_cancelar(&h);
      break;
    }
    HomingFim fim = HOM_FIM_LIMITE;
    int32_t pos_sensor;
    if (r == ESPERA_TRAVOU) {
      fim = HOM_FIM_TRAVOU;
    } else if (stepper_sensor_fired(e->motor, &pos_sensor)) {
      fim = HOM_FIM_SENSOR;
      chave = t.parar_com;
      na_chave = pos_sensor;
      avisar(e, cfg, EIXO_EV_CHAVE, homing_state(&h), chave, false, pos_sensor);
    }

    HomingState antes_estado = homing_state(&h);
    homing_fim_trecho(&h, fim, &t);
    if (homing_state(&h) != antes_estado) {
      avisar(e, cfg, EIXO_EV_ESTADO, homing_state(&h), chave, false, stepper_position(e->motor));
    }
  }

  stepper_sensor_disarm(e->motor);
  stepper_set_profile(e->motor, &antes);
  out->parou_em = homing_state(&h);
  out->erro = homing_error(&h);
  if (out->erro == HOM_ERRO_NENHUM) {
    // Onde a chave acionou, medido na origem ANTERIOR. A origem nova (0)
    // é esse ponto; a interrupção parou o motor nele.
    out->passos_na_chave = na_chave;
    out->enc_na_chave = encoder_count(e->enc);
    stepper_reset(e->motor, stepper_position(e->motor) - na_chave);
    encoder_reset(e->enc, 0);
  }
  stepper_release(e->motor);  // parado não precisa de corrente
  return out->erro;
}

bool eixo_andar(Eixo *e, const EixoHomingCfg *cfg, int32_t passos) {
  stepper_sensor_disarm(e->motor);
  if (!stepper_start(e->motor, passos, false)) return false;
  return esperar(e, cfg, 0) == ESPERA_FIM;
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
    delay(5);  // última transição passa pelo debounce do encoder
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
