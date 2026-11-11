#include "stepper.h"

#include "soc/gpio_struct.h"

static Stepper *g_motores[STEPPER_MAX_MOTORES];
static uint8_t g_n = 0;
static hw_timer_t *g_timer = NULL;

// ---------- GPIO direto (as 4 bobinas de uma vez) ----------

static inline void IRAM_ATTR escreverFase(const Stepper *m, uint8_t fase) {
  fase &= 0x3;
  if (m->desliga_lo[fase]) GPIO.out_w1tc = m->desliga_lo[fase];
  if (m->desliga_hi[fase]) GPIO.out1_w1tc.val = m->desliga_hi[fase];
  if (m->liga_lo[fase]) GPIO.out_w1ts = m->liga_lo[fase];
  if (m->liga_hi[fase]) GPIO.out1_w1ts.val = m->liga_hi[fase];
}

static inline void IRAM_ATTR desligarTudo(const Stepper *m) {
  if (m->todos_lo) GPIO.out_w1tc = m->todos_lo;
  if (m->todos_hi) GPIO.out1_w1tc.val = m->todos_hi;
}

static inline bool IRAM_ATTR lerPino(uint8_t pin) {
  return pin < 32 ? ((GPIO.in >> pin) & 0x1) : ((GPIO.in1.data >> (pin - 32)) & 0x1);
}

// ---------- interrupção dos motores ----------

static void IRAM_ATTR onTick(void) {
  for (uint8_t i = 0; i < g_n; i++) {
    Stepper *m = g_motores[i];
    portENTER_CRITICAL_ISR(&m->mux);
    if (m->ativo) {
      if (m->sensor_armado) {
        bool acionado = lerPino(m->sensor_pin) == m->sensor_ativo_alto;
        if (acionado == m->sensor_parar_com) {
          if (++m->sensor_cont >= m->sensor_tiques) {
            m->sensor_disparou = true;
            m->sensor_pos = m->seq.position;
            m->sensor_armado = false;
            rampa_abortar(&m->rampa);
          }
        } else {
          m->sensor_cont = 0;
        }
      }
      int8_t d = rampa_tick(&m->rampa);
      if (d) {
        m->seq.phase = (uint8_t)((m->seq.phase + d) & 0x3);
        m->seq.position += d;
        escreverFase(m, m->seq.phase);
      }
      if (!rampa_ativa(&m->rampa)) {
        m->ativo = false;
        m->sensor_armado = false;
        if (m->soltar_ao_fim) desligarTudo(m);
      }
    }
    portEXIT_CRITICAL_ISR(&m->mux);
  }
}

static void ligarTimer(void) {
  if (g_timer) return;
  g_timer = timerBegin(0, 80, true);  // 80 MHz / 80 = 1 MHz
  timerAttachInterrupt(g_timer, &onTick, false);
  timerAlarmWrite(g_timer, 1000000u / STEPPER_TICK_HZ, true);
  timerAlarmEnable(g_timer);
}

// ---------- configuração ----------

static void calcularMascaras(Stepper *m) {
  m->todos_lo = 0;
  m->todos_hi = 0;
  for (uint8_t i = 0; i < 4; i++) {
    uint8_t p = m->pins[i];
    if (p < 32) m->todos_lo |= 1u << p;
    else m->todos_hi |= 1u << (p - 32);
  }
  for (uint8_t f = 0; f < 4; f++) {
    uint8_t coils = stepseq_coils(f);
    m->liga_lo[f] = m->liga_hi[f] = 0;
    for (uint8_t i = 0; i < 4; i++) {
      if (!((coils >> i) & 0x1)) continue;
      uint8_t p = m->pins[i];
      if (p < 32) m->liga_lo[f] |= 1u << p;
      else m->liga_hi[f] |= 1u << (p - 32);
    }
    m->desliga_lo[f] = m->todos_lo & ~m->liga_lo[f];
    m->desliga_hi[f] = m->todos_hi & ~m->liga_hi[f];
  }
}

void stepper_begin(Stepper *m, uint8_t in1, uint8_t in2, uint8_t in3, uint8_t in4) {
  m->pins[0] = in1;
  m->pins[1] = in2;
  m->pins[2] = in3;
  m->pins[3] = in4;
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(m->pins[i], OUTPUT);
    digitalWrite(m->pins[i], LOW);
  }
  portMUX_INITIALIZE(&m->mux);
  calcularMascaras(m);
  stepseq_reset(&m->seq, 0);
  m->ativo = false;
  m->soltar_ao_fim = false;
  m->sensor_armado = false;
  m->sensor_disparou = false;
  m->sensor_pos = 0;
  m->rampa.faltam = 0;
  StepperPerfil p = {400, 400, 0};  // seguro: sem rampa, 400 passos/s
  stepper_set_profile(m, &p);

  bool registrado = false;
  for (uint8_t i = 0; i < g_n; i++) registrado |= (g_motores[i] == m);
  if (!registrado && g_n < STEPPER_MAX_MOTORES) {
    portENTER_CRITICAL(&m->mux);
    g_motores[g_n] = m;
    g_n++;
    portEXIT_CRITICAL(&m->mux);
  }
  ligarTimer();
}

void stepper_set_profile(Stepper *m, const StepperPerfil *p) {
  m->perfil = *p;
  if (m->perfil.vmax == 0) m->perfil.vmax = 1;
  if (m->perfil.v0 == 0) m->perfil.v0 = 1;
  m->interval_us = 1000000UL / m->perfil.vmax;
}

void stepper_get_profile(const Stepper *m, StepperPerfil *p) { *p = m->perfil; }

void stepper_set_speed(Stepper *m, uint32_t steps_per_s) {
  StepperPerfil p = m->perfil;
  p.vmax = steps_per_s;
  stepper_set_profile(m, &p);
}

// ---------- movimento ----------

bool stepper_start(Stepper *m, int32_t steps, bool soltar_ao_fim) {
  RampaCfg cfg = {STEPPER_TICK_HZ, m->perfil.v0, m->perfil.vmax, m->perfil.acel};
  Rampa r;
  rampa_iniciar(&r, &cfg, steps);  // contas com divisão: fora da seção crítica

  portENTER_CRITICAL(&m->mux);
  if (m->ativo) {
    portEXIT_CRITICAL(&m->mux);
    return false;
  }
  escreverFase(m, m->seq.phase);  // encaixa o rotor na fase atual
  m->rampa = r;
  m->soltar_ao_fim = soltar_ao_fim;
  m->sensor_disparou = false;
  m->sensor_cont = 0;
  m->ativo = rampa_ativa(&r);
  if (!m->ativo) m->sensor_armado = false;
  portEXIT_CRITICAL(&m->mux);
  return true;
}

bool stepper_start_to(Stepper *m, int32_t alvo, bool soltar_ao_fim) {
  return stepper_start(m, alvo - stepper_position(m), soltar_ao_fim);
}

bool stepper_busy(const Stepper *m) { return m->ativo; }

void stepper_stop(Stepper *m) {
  portENTER_CRITICAL(&m->mux);
  rampa_parar(&m->rampa);
  portEXIT_CRITICAL(&m->mux);
}

void stepper_abort(Stepper *m) {
  portENTER_CRITICAL(&m->mux);
  rampa_abortar(&m->rampa);
  m->ativo = false;
  m->sensor_armado = false;
  portEXIT_CRITICAL(&m->mux);
}

uint32_t stepper_speed_now(Stepper *m) {
  portENTER_CRITICAL(&m->mux);
  uint32_t v = m->ativo ? rampa_velocidade(&m->rampa, STEPPER_TICK_HZ) : 0;
  portEXIT_CRITICAL(&m->mux);
  return v;
}

void stepper_wait(const Stepper *m) {
  while (m->ativo) delay(1);
}

void stepper_move(Stepper *m, int32_t steps) {
  if (stepper_start(m, steps, false)) stepper_wait(m);
}

// ---------- sensor ----------

void stepper_sensor_arm(Stepper *m, uint8_t pin, bool ativo_alto, bool parar_com, uint16_t tiques) {
  portENTER_CRITICAL(&m->mux);
  m->sensor_pin = pin;
  m->sensor_ativo_alto = ativo_alto;
  m->sensor_parar_com = parar_com;
  m->sensor_tiques = tiques ? tiques : 1;
  m->sensor_cont = 0;
  m->sensor_disparou = false;
  m->sensor_armado = true;
  portEXIT_CRITICAL(&m->mux);
}

void stepper_sensor_disarm(Stepper *m) {
  portENTER_CRITICAL(&m->mux);
  m->sensor_armado = false;
  portEXIT_CRITICAL(&m->mux);
}

bool stepper_sensor_fired(Stepper *m, int32_t *pos) {
  portENTER_CRITICAL(&m->mux);
  bool f = m->sensor_disparou;
  if (pos) *pos = m->sensor_pos;
  portEXIT_CRITICAL(&m->mux);
  return f;
}

// ---------- posição e bobinas ----------

int32_t stepper_position(Stepper *m) {
  portENTER_CRITICAL(&m->mux);
  int32_t p = m->seq.position;
  portEXIT_CRITICAL(&m->mux);
  return p;
}

void stepper_reset(Stepper *m, int32_t position) {
  // Mantém a fase: só muda o número que chamamos de "posição".
  portENTER_CRITICAL(&m->mux);
  m->seq.position = position;
  portEXIT_CRITICAL(&m->mux);
}

void stepper_release(Stepper *m) {
  stepper_abort(m);
  desligarTudo(m);
}

void stepper_hold(Stepper *m) {
  if (m->ativo) return;
  escreverFase(m, m->seq.phase);
  delayMicroseconds(m->interval_us);
}

void stepper_step(Stepper *m, int8_t dir) {
  if (dir == 0 || m->ativo) return;
  portENTER_CRITICAL(&m->mux);
  stepseq_step(&m->seq, dir > 0 ? 1 : -1);
  escreverFase(m, m->seq.phase);
  portEXIT_CRITICAL(&m->mux);
  delayMicroseconds(m->interval_us);
}

void stepper_test_coils(Stepper *m, uint8_t coils) {
  stepper_abort(m);
  for (uint8_t i = 0; i < 4; i++) digitalWrite(m->pins[i], (coils >> i) & 0x1);
}

uint8_t stepper_pin(const Stepper *m, uint8_t index) {
  return m->pins[index & 0x3];
}
