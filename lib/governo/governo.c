#include "governo.h"

static int32_t absi(int32_t v) { return v < 0 ? -v : v; }

static GovAmostra *amostra(Gov *g, uint8_t i) { return &g->a[(g->ini + i) % GOV_AMOSTRAS]; }

static void limpar_janela(Gov *g) {
  g->n = 0;
  g->ini = 0;
}

// Valores fora da faixa viram o padrão (nunca trava a lógica).
static void sanear(GovCfg *c) {
  if (c->vel_min == 0) c->vel_min = 1;
  if (c->vel_max < c->vel_min) c->vel_max = c->vel_min;
  if (c->reduzir_pct == 0 || c->reduzir_pct > 100) c->reduzir_pct = 70;
  if (c->intervalo_ms == 0) c->intervalo_ms = 1000;
  if (c->subida_min == 0) c->subida_min = 1;
  if (c->min_cont < 1) c->min_cont = 1;
  if (c->quedas_reduzir == 0) c->quedas_reduzir = 1;
  if (c->subida_pct == 0 || c->subida_pct > 100) c->subida_pct = 50;
}

void gov_init(Gov *g, const GovCfg *cfg, float cont_por_passo) {
  g->cfg = *cfg;
  sanear(&g->cfg);
  g->cont_por_passo = cont_por_passo > 0.0f ? cont_por_passo : 1.0f;
  g->limite = g->cfg.vel_max;
  g->setpoint = g->cfg.vel_max;
  limpar_janela(g);
  g->calmo_ms = 0;
  g->vel_real = 0.0f;
  g->vel_motor = 0.0f;
  g->aproveit_pct = 100;
  g->quedas = 0;
  g->seguidas = 0;
  g->segurar_ms = 0;
  g->tem_ant = false;
}

void gov_configurar(Gov *g, const GovCfg *cfg) {
  g->cfg = *cfg;
  sanear(&g->cfg);
  if (g->limite > g->cfg.vel_max) g->limite = g->cfg.vel_max;
  if (g->seguidas >= g->cfg.quedas_reduzir) g->seguidas = 0;
  if (g->segurar_ms > g->cfg.segurar_ms) g->segurar_ms = g->cfg.segurar_ms;
}

void gov_escala(Gov *g, float cont_por_passo) {
  if (cont_por_passo > 0.0f) g->cont_por_passo = cont_por_passo;
  limpar_janela(g);
}

void gov_setpoint(Gov *g, uint32_t passos_s) {
  if (passos_s > g->cfg.vel_max) passos_s = g->cfg.vel_max;
  if (passos_s < g->cfg.vel_min) passos_s = g->cfg.vel_min;
  g->setpoint = passos_s;
}

// Rotação que vale agora (o limite só importa abaixo do setpoint).
static uint32_t atual(const Gov *g) { return g->limite < g->setpoint ? g->limite : g->setpoint; }

// Baixa o limite para 'novo' (não sobe por aqui; não desce de vel_min).
static void baixar(Gov *g, uint32_t novo) {
  if (novo < g->cfg.vel_min) novo = g->cfg.vel_min;
  if (novo > atual(g)) novo = atual(g);
  g->limite = novo;
  g->calmo_ms = 0;
  g->seguidas = 0;
  g->quedas++;
  limpar_janela(g);
}

// Conta uma queda. true = chegou em 'quedas_reduzir' (quem chamou reduz).
static bool contar_queda(Gov *g) {
  g->seguidas++;
  return g->seguidas >= g->cfg.quedas_reduzir;
}

bool gov_travou(Gov *g, uint32_t vel) {
  if (g->segurar_ms > 0) return false;  // acabou de subir: mantém
  if (!contar_queda(g)) return false;
  uint32_t base = atual(g);
  if (vel > 0 && vel < base) base = vel;
  baixar(g, (uint32_t)((uint64_t)base * g->cfg.reduzir_pct / 100u));
  return true;
}

GovEvento gov_amostra(Gov *g, uint32_t t_ms, bool andando, int32_t motor, int32_t enc) {
  if (!andando) {
    limpar_janela(g);
    g->tem_ant = false;  // parado não conta tempo (nem para segurar, nem para subir)
    g->vel_real = 0.0f;
    g->vel_motor = 0.0f;
    return GOV_NADA;
  }

  // Tempo andando desde a amostra anterior.
  uint32_t dt = g->tem_ant ? t_ms - g->t_ant : 0;
  g->t_ant = t_ms;
  g->tem_ant = true;
  g->segurar_ms = g->segurar_ms > dt ? g->segurar_ms - dt : 0;

  // Põe a amostra na fila (cheia: descarta a mais velha).
  if (g->n == GOV_AMOSTRAS) {
    g->ini = (g->ini + 1) % GOV_AMOSTRAS;
    g->n--;
  }
  GovAmostra *nova = amostra(g, g->n);
  nova->t = t_ms;
  nova->motor = motor;
  nova->enc = enc;
  g->n++;

  // Janela: descarta as mais velhas enquanto sobrar janela_ms e min_cont.
  while (g->n > 2) {
    GovAmostra *seg = amostra(g, 1);
    int32_t esp = (int32_t)(absi(motor - seg->motor) * g->cont_por_passo);
    if (t_ms - seg->t < g->cfg.janela_ms || esp < g->cfg.min_cont) break;
    g->ini = (g->ini + 1) % GOV_AMOSTRAS;
    g->n--;
  }

  GovAmostra *velha = amostra(g, 0);
  uint32_t span = t_ms - velha->t;
  if (span > 0) {
    int32_t dm = motor - velha->motor;
    int32_t de = enc - velha->enc;
    if (dm < 0) de = -de;  // sentido do motor = positivo
    float real = de > 0 ? de / g->cont_por_passo : 0.0f;  // passos equivalentes
    g->vel_real = real * 1000.0f / span;
    g->vel_motor = absi(dm) * 1000.0f / span;
    float esperado = absi(dm) * g->cont_por_passo;
    if (span >= g->cfg.janela_ms && esperado >= g->cfg.min_cont) {
      g->aproveit_pct = (int32_t)(100.0f * real / (float)absi(dm) + 0.5f);
      if (g->aproveit_pct < g->cfg.queda_pct) {
        // Janela ruim: a próxima é nova (quedas seguidas = janelas seguidas).
        uint32_t pelo_motor = (uint32_t)(g->vel_motor * g->cfg.reduzir_pct / 100.0f);
        uint32_t pela_real = (uint32_t)g->vel_real;
        limpar_janela(g);
        if (g->segurar_ms > 0) {
          // Acabou de subir: mantém a rotação, mas não sobe de novo escorregando.
          g->calmo_ms = 0;
          return GOV_NADA;
        }
        if (!contar_queda(g)) return GOV_QUEDA_CONTADA;
        baixar(g, pela_real < pelo_motor ? pela_real : pelo_motor);
        return GOV_QUEDA;
      }
      g->seguidas = 0;  // janela boa: zera a conta
    }
  }

  // Recuperação: a cada intervalo sem reduzir, 'subida_pct' do que falta.
  if (g->limite < g->setpoint) {
    g->calmo_ms += dt;
    if (g->calmo_ms >= g->cfg.intervalo_ms && g->segurar_ms == 0) {
      g->calmo_ms = 0;
      uint32_t falta = g->setpoint - g->limite;
      uint32_t passo = (uint32_t)((uint64_t)falta * g->cfg.subida_pct / 100u);
      if (passo < g->cfg.subida_min) passo = g->cfg.subida_min;
      g->limite += passo;
      if (g->limite >= g->setpoint) {
        g->limite = g->cfg.vel_max;
        return GOV_RECUPEROU;
      }
      // Subiu: mantém pelo menos segurar_ms e mede de novo na rotação nova.
      g->segurar_ms = g->cfg.segurar_ms;
      g->seguidas = 0;
      limpar_janela(g);
      return GOV_SUBIU;
    }
  } else {
    g->calmo_ms = 0;
    if (g->limite < g->cfg.vel_max) g->limite = g->cfg.vel_max;  // setpoint baixou: solta
  }
  return GOV_NADA;
}

uint32_t gov_limite(const Gov *g) { return g->limite; }
bool gov_limitado(const Gov *g) { return g->limite < g->setpoint; }
uint32_t gov_setpoint_atual(const Gov *g) { return g->setpoint; }
float gov_vel_real(const Gov *g) { return g->vel_real; }
float gov_vel_motor(const Gov *g) { return g->vel_motor; }
int32_t gov_aproveitamento(const Gov *g) { return g->aproveit_pct; }
uint32_t gov_quedas(const Gov *g) { return g->quedas; }
uint8_t gov_seguidas(const Gov *g) { return g->seguidas; }
uint32_t gov_segurando_ms(const Gov *g) { return g->segurar_ms; }
