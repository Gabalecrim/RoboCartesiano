// Testes da rampa de aceleração (rodam no PC).
//
//   pio test -e native -f test_rampa
//
// Simula os tiques da interrupção (20 kHz) e confere o perfil.

#include <math.h>
#include <stdint.h>
#include <unity.h>
#include "rampa.h"

#define HZ 20000u

static const RampaCfg PADRAO = {.tick_hz = HZ, .v0 = 300, .vmax = 1000, .acel = 1000};

typedef struct {
  long passos;        // soma dos passos (com sinal)
  uint64_t tiques;    // até parar
  uint32_t v_max;     // maior velocidade vista (passos/s)
  uint32_t v_min;     // menor velocidade com o motor ativo (passos/s)
  uint32_t v_ultimo;  // velocidade no último passo
  uint64_t tiques_ate_vmax;     // do início até chegar em vmax
  uint64_t tiques_desde_vmax;   // de quando saiu de vmax até o fim
} Perfil;

static Perfil rodar(const RampaCfg *cfg, int32_t passos) {
  Perfil p = {0};
  p.v_min = UINT32_MAX;
  Rampa r;
  rampa_iniciar(&r, cfg, passos);
  uint32_t vmax = cfg->vmax;
  bool chegou = false;
  uint64_t saiu = 0;
  while (rampa_ativa(&r)) {
    int8_t d = rampa_tick(&r);
    p.tiques++;
    uint32_t v = (uint32_t)(((uint64_t)r.v * HZ) >> 32);
    if (v > p.v_max) p.v_max = v;
    if (v < p.v_min) p.v_min = v;
    if (!chegou && v >= vmax - 1) {
      chegou = true;
      p.tiques_ate_vmax = p.tiques;
    }
    if (chegou && v >= vmax - 1) saiu = p.tiques;
    if (d) {
      p.passos += d;
      p.v_ultimo = v;
    }
  }
  p.tiques_desde_vmax = p.tiques - saiu;
  return p;
}

void setUp(void) {}
void tearDown(void) {}

void test_da_exatamente_os_passos_pedidos(void) {
  TEST_ASSERT_EQUAL_INT32(25600, rodar(&PADRAO, 25600).passos);
  TEST_ASSERT_EQUAL_INT32(-777, rodar(&PADRAO, -777).passos);
  TEST_ASSERT_EQUAL_INT32(1, rodar(&PADRAO, 1).passos);
}

void test_zero_passos_fica_parada(void) {
  Rampa r;
  rampa_iniciar(&r, &PADRAO, 0);
  TEST_ASSERT_FALSE(rampa_ativa(&r));
  TEST_ASSERT_EQUAL_INT8(0, rampa_tick(&r));
}

void test_nunca_passa_de_vmax_nem_cai_abaixo_de_v0(void) {
  Perfil p = rodar(&PADRAO, 25600);
  TEST_ASSERT_LESS_OR_EQUAL_UINT32(1000, p.v_max);
  TEST_ASSERT_GREATER_OR_EQUAL_UINT32(299, p.v_min);
}

void test_tempo_bate_com_o_trapezio(void) {
  // t_acc = (vmax - v0)/a = 0,7 s; s_acc = (vmax² - v0²)/(2a) = 455 passos
  // t = 2·0,7 + (25600 - 910)/1000 = 26,09 s
  double t = rodar(&PADRAO, 25600).tiques / (double)HZ;
  TEST_ASSERT_FLOAT_WITHIN(26.09 * 0.01, 26.09, t);
}

void test_sem_rampa_anda_direto_em_vmax(void) {
  RampaCfg cfg = PADRAO;
  cfg.acel = 0;
  Perfil p = rodar(&cfg, 6000);
  TEST_ASSERT_FLOAT_WITHIN(6.0 * 0.005, 6.0, p.tiques / (double)HZ);  // 6000 / 1000
  TEST_ASSERT_GREATER_OR_EQUAL_UINT32(999, p.v_min);
}

void test_movimento_curto_vira_triangulo(void) {
  // 200 passos não dão para chegar em 1000: pico = sqrt(v0² + a·d) ~ 538
  Perfil p = rodar(&PADRAO, 200);
  TEST_ASSERT_EQUAL_INT32(200, p.passos);
  TEST_ASSERT_LESS_THAN_UINT32(1000, p.v_max);
  TEST_ASSERT_UINT32_WITHIN(30, 538, p.v_max);
}

void test_acelera_e_freia_igual(void) {
  Perfil p = rodar(&PADRAO, 25600);
  double subida = p.tiques_ate_vmax / (double)HZ;   // ~0,7 s
  double descida = p.tiques_desde_vmax / (double)HZ;
  TEST_ASSERT_FLOAT_WITHIN(0.7 * 0.05, 0.7, subida);
  TEST_ASSERT_FLOAT_WITHIN(0.7 * 0.05, subida, descida);
}

void test_chega_devagar_no_ultimo_passo(void) {
  Perfil p = rodar(&PADRAO, 25600);
  TEST_ASSERT_LESS_OR_EQUAL_UINT32(330, p.v_ultimo);  // ~v0
}

void test_parar_freia_pela_rampa(void) {
  Rampa r;
  rampa_iniciar(&r, &PADRAO, 1000000);
  for (uint32_t i = 0; i < 2 * HZ; i++) rampa_tick(&r);  // 2 s: já em vmax
  TEST_ASSERT_UINT32_WITHIN(2, 1000, rampa_velocidade(&r, HZ));
  rampa_parar(&r);
  // Faltam ~ (1000² - 300²) / (2·1000) = 455 passos
  TEST_ASSERT_INT32_WITHIN(5, 456, r.faltam);
  long passos = 0;
  uint32_t v_ultimo = 0;
  while (rampa_ativa(&r)) {
    if (rampa_tick(&r)) {
      passos++;
      v_ultimo = (uint32_t)(((uint64_t)r.v * HZ) >> 32);
    }
  }
  TEST_ASSERT_INT32_WITHIN(5, 456, passos);
  TEST_ASSERT_LESS_OR_EQUAL_UINT32(330, v_ultimo);
}

void test_parar_sem_rampa_para_na_hora(void) {
  RampaCfg cfg = PADRAO;
  cfg.acel = 0;
  Rampa r;
  rampa_iniciar(&r, &cfg, 5000);
  rampa_parar(&r);
  TEST_ASSERT_FALSE(rampa_ativa(&r));
}

void test_abortar_para_na_hora(void) {
  Rampa r;
  rampa_iniciar(&r, &PADRAO, 5000);
  for (int i = 0; i < 1000; i++) rampa_tick(&r);
  rampa_abortar(&r);
  TEST_ASSERT_FALSE(rampa_ativa(&r));
  for (int i = 0; i < 1000; i++) TEST_ASSERT_EQUAL_INT8(0, rampa_tick(&r));
}

void test_vmax_limitado_a_meia_frequencia(void) {
  RampaCfg cfg = {.tick_hz = HZ, .v0 = 50000, .vmax = 50000, .acel = 0};
  Perfil p = rodar(&cfg, 2000);
  TEST_ASSERT_LESS_OR_EQUAL_UINT32(HZ / 2, p.v_max);
  TEST_ASSERT_EQUAL_INT32(2000, p.passos);
}

void test_v0_maior_que_vmax_vira_vmax(void) {
  RampaCfg cfg = {.tick_hz = HZ, .v0 = 900, .vmax = 400, .acel = 1000};
  Perfil p = rodar(&cfg, 4000);
  TEST_ASSERT_LESS_OR_EQUAL_UINT32(400, p.v_max);
  TEST_ASSERT_FLOAT_WITHIN(10.0 * 0.01, 10.0, p.tiques / (double)HZ);
}

void test_tiques_total_igual_a_simulacao(void) {
  TEST_ASSERT_EQUAL_UINT64(rodar(&PADRAO, 3000).tiques, rampa_tiques_total(&PADRAO, 3000));
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_da_exatamente_os_passos_pedidos);
  RUN_TEST(test_zero_passos_fica_parada);
  RUN_TEST(test_nunca_passa_de_vmax_nem_cai_abaixo_de_v0);
  RUN_TEST(test_tempo_bate_com_o_trapezio);
  RUN_TEST(test_sem_rampa_anda_direto_em_vmax);
  RUN_TEST(test_movimento_curto_vira_triangulo);
  RUN_TEST(test_acelera_e_freia_igual);
  RUN_TEST(test_chega_devagar_no_ultimo_passo);
  RUN_TEST(test_parar_freia_pela_rampa);
  RUN_TEST(test_parar_sem_rampa_para_na_hora);
  RUN_TEST(test_abortar_para_na_hora);
  RUN_TEST(test_vmax_limitado_a_meia_frequencia);
  RUN_TEST(test_v0_maior_que_vmax_vira_vmax);
  RUN_TEST(test_tiques_total_igual_a_simulacao);
  return UNITY_END();
}
