// Testes do governo da velocidade pelo encoder (rodam no PC).
//
//   pio test -e native -f test_governo
//
// Eixo X: 80 contagens / 2048 passos -> 25,6 passos por contagem.

#include <unity.h>
#include "governo.h"

static const float K = 80.0f / 2048.0f;
// teto 500, mín. 120, queda < 75%, reduz a 70%, janela 500 ms, 5 contagens,
// tenta recuperar a cada 1 s, subida mínima 25; reduz com 1 queda; sobe 50%
// do que falta; segura 1 s depois de subir.
static const GovCfg CFG = {500, 120, 75, 70, 500, 5, 1000, 25, 1, 50, 1000};
static Gov g;

// Motor e encoder simulados.
static uint32_t t;
static float motor, enc;

void setUp(void) {
  gov_init(&g, &CFG, K);
  gov_setpoint(&g, 500);
  t = 0;
  motor = enc = 0.0f;
}
void tearDown(void) {}

static int32_t arred(float v) { return (int32_t)(v >= 0.0f ? v + 0.5f : v - 0.5f); }

// Anda 'ms' a 'vel' passos/s (ou no limite, se menor), com o carro
// acompanhando 'aproveit' do motor. Amostra a cada 50 ms. Para na 1ª queda
// (na bancada, com menos rotação o torque volta). Devolve o último evento
// diferente de NADA.
static GovEvento andar(uint32_t ms, float vel, float aproveit) {
  GovEvento ultimo = GOV_NADA;
  for (uint32_t i = 0; i < ms; i += 50) {
    float v = vel < gov_limite(&g) ? vel : (float)gov_limite(&g);
    motor += v * 0.05f;
    enc += v * 0.05f * K * aproveit;
    t += 50;
    GovEvento e = gov_amostra(&g, t, true, arred(motor), arred(enc));
    if (e == GOV_QUEDA) return e;
    if (e != GOV_NADA) ultimo = e;
  }
  return ultimo;
}

void test_sem_queda_fica_sem_limite(void) {
  TEST_ASSERT_EQUAL(GOV_NADA, andar(5000, 500, 1.0f));
  TEST_ASSERT_EQUAL_UINT32(500, gov_limite(&g));
  TEST_ASSERT_FALSE(gov_limitado(&g));
  TEST_ASSERT_INT32_WITHIN(12, 100, gov_aproveitamento(&g));
  TEST_ASSERT_FLOAT_WITHIN(40.0f, 500.0f, gov_vel_real(&g));
}

void test_queda_baixa_a_rotacao(void) {
  andar(1000, 500, 1.0f);
  TEST_ASSERT_EQUAL(GOV_QUEDA, andar(1000, 500, 0.5f));  // carro anda metade
  // Detecta assim que o aproveitamento passa abaixo de 75%: a janela ainda
  // mistura 100% e 50% (real ~370), então vale 70% de 500 = 350.
  TEST_ASSERT_UINT32_WITHIN(10, 350, gov_limite(&g));
  TEST_ASSERT_TRUE(gov_limitado(&g));
  TEST_ASSERT_EQUAL_UINT32(1, gov_quedas(&g));
}

void test_recupera_em_tentativas_de_1_segundo(void) {
  andar(1000, 500, 1.0f);
  andar(1000, 500, 0.5f);
  uint32_t l0 = gov_limite(&g);
  TEST_ASSERT_EQUAL(GOV_NADA, andar(900, 500, 1.0f));  // antes de 1 s: igual
  TEST_ASSERT_EQUAL_UINT32(l0, gov_limite(&g));
  TEST_ASSERT_EQUAL(GOV_SUBIU, andar(150, 500, 1.0f));  // 1 s: metade do que falta
  TEST_ASSERT_EQUAL_UINT32(l0 + (500 - l0) / 2, gov_limite(&g));
  TEST_ASSERT_EQUAL(GOV_RECUPEROU, andar(8000, 500, 1.0f));
  TEST_ASSERT_EQUAL_UINT32(500, gov_limite(&g));
  TEST_ASSERT_FALSE(gov_limitado(&g));
}

void test_queda_de_novo_durante_a_recuperacao(void) {
  andar(1000, 500, 1.0f);
  andar(1000, 500, 0.5f);
  andar(1050, 500, 1.0f);  // subiu uma vez
  uint32_t l1 = gov_limite(&g);
  // Logo depois de subir, segura 1 s: a queda só conta depois.
  TEST_ASSERT_EQUAL(GOV_QUEDA, andar(2000, 500, 0.4f));
  TEST_ASSERT_LESS_THAN_UINT32(l1, gov_limite(&g));
}

void test_nao_desce_do_minimo(void) {
  andar(1000, 500, 1.0f);
  for (int i = 0; i < 10; i++) andar(1500, 500, 0.05f);
  TEST_ASSERT_EQUAL_UINT32(120, gov_limite(&g));
}

void test_parado_nao_mede_nem_sobe(void) {
  andar(1000, 500, 1.0f);
  andar(1000, 500, 0.5f);
  uint32_t l0 = gov_limite(&g);
  for (int i = 0; i < 100; i++) gov_amostra(&g, t += 50, false, arred(motor), arred(enc));
  TEST_ASSERT_EQUAL_UINT32(l0, gov_limite(&g));
  TEST_ASSERT_EQUAL_FLOAT(0.0f, gov_vel_real(&g));
}

void test_devagar_espera_contagens_suficientes(void) {
  // 100 passos/s = 3,9 contagens/s: precisa de > 1 s para 5 contagens.
  gov_setpoint(&g, 120);
  TEST_ASSERT_EQUAL(GOV_NADA, andar(1000, 100, 0.0f));  // ainda sem contagens bastantes
}

void test_travou_baixa_da_rotacao_do_instante(void) {
  TEST_ASSERT_TRUE(gov_travou(&g, 400));
  TEST_ASSERT_EQUAL_UINT32(280, gov_limite(&g));  // 70% de 400
  TEST_ASSERT_TRUE(gov_travou(&g, 0));
  TEST_ASSERT_EQUAL_UINT32(196, gov_limite(&g));  // 70% do limite (280)
}

void test_setpoint_abaixo_do_limite_solta(void) {
  andar(1000, 500, 1.0f);
  andar(1000, 500, 0.5f);  // limite ~250
  gov_setpoint(&g, 200);   // movimento novo mais lento
  andar(100, 200, 1.0f);
  TEST_ASSERT_FALSE(gov_limitado(&g));
  TEST_ASSERT_EQUAL_UINT32(500, gov_limite(&g));
}

void test_sentido_negativo(void) {
  for (uint32_t i = 0; i < 2000; i += 50) {
    motor -= 25.0f;
    enc -= 25.0f * K * 0.5f;
    t += 50;
    if (gov_amostra(&g, t, true, arred(motor), arred(enc)) == GOV_QUEDA) break;
  }
  TEST_ASSERT_TRUE(gov_limitado(&g));
}

// Configuração com N quedas e fatores diferentes.
static void configurar(uint8_t quedas, uint8_t subida_pct, uint8_t reduzir_pct) {
  GovCfg c = CFG;
  c.quedas_reduzir = quedas;
  c.subida_pct = subida_pct;
  c.reduzir_pct = reduzir_pct;
  gov_configurar(&g, &c);
}

// Anda até o próximo evento diferente de NADA (ou 'ms'), sem parar na queda contada.
static GovEvento ate_evento(uint32_t ms, float aproveit) {
  for (uint32_t i = 0; i < ms; i += 50) {
    float v = 500.0f < gov_limite(&g) ? 500.0f : (float)gov_limite(&g);
    motor += v * 0.05f;
    enc += v * 0.05f * K * aproveit;
    t += 50;
    GovEvento e = gov_amostra(&g, t, true, arred(motor), arred(enc));
    if (e != GOV_NADA) return e;
  }
  return GOV_NADA;
}

void test_precisa_de_n_quedas_seguidas(void) {
  configurar(3, 50, 70);
  andar(1000, 500, 1.0f);
  TEST_ASSERT_EQUAL(GOV_QUEDA_CONTADA, ate_evento(2000, 0.5f));
  TEST_ASSERT_EQUAL_UINT8(1, gov_seguidas(&g));
  TEST_ASSERT_EQUAL(GOV_QUEDA_CONTADA, ate_evento(2000, 0.5f));
  TEST_ASSERT_EQUAL_UINT32(500, gov_limite(&g));  // ainda não reduziu
  TEST_ASSERT_EQUAL(GOV_QUEDA, ate_evento(2000, 0.5f));
  TEST_ASSERT_TRUE(gov_limitado(&g));
  TEST_ASSERT_EQUAL_UINT8(0, gov_seguidas(&g));
}

void test_janela_boa_zera_a_contagem(void) {
  configurar(2, 50, 70);
  andar(1000, 500, 1.0f);
  TEST_ASSERT_EQUAL(GOV_QUEDA_CONTADA, ate_evento(2000, 0.5f));
  andar(1000, 500, 1.0f);  // janelas boas
  TEST_ASSERT_EQUAL_UINT8(0, gov_seguidas(&g));
  TEST_ASSERT_EQUAL(GOV_QUEDA_CONTADA, ate_evento(2000, 0.5f));  // recomeçou do 1
}

void test_segura_1_s_depois_de_subir(void) {
  andar(1000, 500, 1.0f);
  andar(1000, 500, 0.5f);                            // reduziu
  TEST_ASSERT_EQUAL(GOV_SUBIU, ate_evento(2000, 1.0f));
  uint32_t l = gov_limite(&g);
  TEST_ASSERT_EQUAL_UINT32(1000, gov_segurando_ms(&g));
  TEST_ASSERT_EQUAL(GOV_NADA, ate_evento(950, 0.2f));  // escorregando, mas segura
  TEST_ASSERT_EQUAL_UINT32(l, gov_limite(&g));
  TEST_ASSERT_FALSE(gov_travou(&g, 400));              // travada também não conta
  TEST_ASSERT_EQUAL_UINT32(l, gov_limite(&g));
  TEST_ASSERT_EQUAL(GOV_QUEDA, ate_evento(2000, 0.2f));  // depois de 1 s, reduz
}

void test_fator_de_aceleracao(void) {
  configurar(1, 25, 70);
  andar(1000, 500, 1.0f);
  andar(1000, 500, 0.5f);
  uint32_t l0 = gov_limite(&g);
  TEST_ASSERT_EQUAL(GOV_SUBIU, ate_evento(2000, 1.0f));
  TEST_ASSERT_EQUAL_UINT32(l0 + (500 - l0) / 4, gov_limite(&g));  // 25% do que falta
}

void test_fator_de_desaceleracao(void) {
  configurar(1, 50, 40);
  TEST_ASSERT_TRUE(gov_travou(&g, 400));
  TEST_ASSERT_EQUAL_UINT32(160, gov_limite(&g));  // 40% de 400
}

void test_configurar_mantem_o_estado(void) {
  andar(1000, 500, 1.0f);
  andar(1000, 500, 0.5f);
  uint32_t l = gov_limite(&g);
  configurar(2, 30, 60);
  TEST_ASSERT_EQUAL_UINT32(l, gov_limite(&g));
  TEST_ASSERT_EQUAL_UINT8(2, g.cfg.quedas_reduzir);
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_sem_queda_fica_sem_limite);
  RUN_TEST(test_queda_baixa_a_rotacao);
  RUN_TEST(test_recupera_em_tentativas_de_1_segundo);
  RUN_TEST(test_queda_de_novo_durante_a_recuperacao);
  RUN_TEST(test_nao_desce_do_minimo);
  RUN_TEST(test_parado_nao_mede_nem_sobe);
  RUN_TEST(test_devagar_espera_contagens_suficientes);
  RUN_TEST(test_travou_baixa_da_rotacao_do_instante);
  RUN_TEST(test_setpoint_abaixo_do_limite_solta);
  RUN_TEST(test_sentido_negativo);
  RUN_TEST(test_precisa_de_n_quedas_seguidas);
  RUN_TEST(test_janela_boa_zera_a_contagem);
  RUN_TEST(test_segura_1_s_depois_de_subir);
  RUN_TEST(test_fator_de_aceleracao);
  RUN_TEST(test_fator_de_desaceleracao);
  RUN_TEST(test_configurar_mantem_o_estado);
  return UNITY_END();
}
