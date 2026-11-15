// Testes da proteção contra colisão (rodam no PC).
//
//   pio test -e native -f test_colisao
//
// Eixo X: 2048 passos e 80 contagens por volta -> 80/2048 contagens por
// passo (25,6 passos por contagem).

#include <unity.h>
#include "colisao.h"

static const float K = 80.0f / 2048.0f;
static const ColisaoCfg CFG = {6, 2};  // limite 6 contagens, 2 leituras seguidas

static ColisaoMonitor c;

void setUp(void) { colisao_init(&c, &CFG, K); }
void tearDown(void) {}

// Encoder que acompanha o motor perfeitamente.
static int32_t enc(int32_t passos) { return (int32_t)(passos * K + (passos >= 0 ? 0.5f : -0.5f)); }

void test_movimento_normal_nao_dispara(void) {
  for (int32_t p = 0; p <= 25600; p += 64) {
    TEST_ASSERT_FALSE(colisao_atualizar(&c, true, 1, p, enc(p)));
  }
  TEST_ASSERT_FALSE(colisao_disparou(&c));
  TEST_ASSERT_LESS_OR_EQUAL_INT32(1, colisao_erro_max(&c));
}

void test_sentido_negativo_normal_nao_dispara(void) {
  for (int32_t p = 0; p >= -25600; p -= 64) {
    TEST_ASSERT_FALSE(colisao_atualizar(&c, true, 1, p, enc(p)));
  }
  TEST_ASSERT_FALSE(colisao_disparou(&c));
}

void test_atraso_pequeno_e_tolerado(void) {
  // encoder 5 contagens atrás o tempo todo (folga): abaixo do limite
  for (int32_t p = 0; p <= 25600; p += 64) {
    int32_t e = enc(p) - 5;
    if (e < 0) e = 0;
    TEST_ASSERT_FALSE(colisao_atualizar(&c, true, 1, p, e));
  }
}

void test_colisao_encoder_para_e_dispara(void) {
  int32_t p = 0;
  for (; p <= 5120; p += 64) colisao_atualizar(&c, true, 1, p, enc(p));
  int32_t parado = enc(p);  // bateu: o carro para, o motor continua contando
  bool disparou = false;
  int32_t passos_ate = 0;
  for (int32_t q = p; q < p + 2048 && !disparou; q += 32) {
    disparou = colisao_atualizar(&c, true, 1, q, parado);
    passos_ate = q - p;
  }
  TEST_ASSERT_TRUE(disparou);
  TEST_ASSERT_TRUE(colisao_disparou(&c));
  // limite 6 contagens = ~154 passos + 1 leitura de confirmação
  TEST_ASSERT_LESS_OR_EQUAL_INT32(256, passos_ate);
}

void test_pico_isolado_nao_dispara(void) {
  colisao_atualizar(&c, true, 1, 0, 0);
  colisao_atualizar(&c, true, 1, 2560, enc(2560));
  TEST_ASSERT_FALSE(colisao_atualizar(&c, true, 1, 2560, enc(2560) + 20));  // 1 leitura ruim
  TEST_ASSERT_FALSE(colisao_atualizar(&c, true, 1, 2600, enc(2600)));       // voltou ao normal
  TEST_ASSERT_FALSE(colisao_atualizar(&c, true, 1, 2600, enc(2600) + 20));
  TEST_ASSERT_FALSE(colisao_disparou(&c));
}

void test_dispara_uma_vez_e_trava_ate_rearmar(void) {
  colisao_atualizar(&c, true, 1, 0, 0);
  TEST_ASSERT_FALSE(colisao_atualizar(&c, true, 1, 1000, 0));
  TEST_ASSERT_TRUE(colisao_atualizar(&c, true, 1, 1100, 0));
  TEST_ASSERT_FALSE(colisao_atualizar(&c, true, 1, 1200, 0));  // já disparou: não repete
  TEST_ASSERT_TRUE(colisao_disparou(&c));
  TEST_ASSERT_FALSE(colisao_atualizar(&c, false, 1, 1200, 0));
  TEST_ASSERT_TRUE(colisao_disparou(&c));  // parado não limpa
  colisao_rearmar(&c);
  TEST_ASSERT_FALSE(colisao_disparou(&c));
}

void test_movimento_novo_refaz_a_referencia(void) {
  // Entre dois movimentos alguém zerou o motor e o encoder (homing) sem
  // o monitor ver o motor parado: o movimento novo não pode virar colisão.
  colisao_atualizar(&c, true, 1, 0, 0);
  colisao_atualizar(&c, true, 1, 5000, enc(5000));
  TEST_ASSERT_FALSE(colisao_atualizar(&c, true, 2, 0, 0));     // movimento 2, depois do zero
  TEST_ASSERT_FALSE(colisao_atualizar(&c, true, 2, -64, enc(-64)));
  TEST_ASSERT_FALSE(colisao_atualizar(&c, true, 2, -2560, enc(-2560)));
  TEST_ASSERT_FALSE(colisao_disparou(&c));
}

void test_parado_refaz_a_referencia(void) {
  colisao_atualizar(&c, true, 1, 0, 0);
  colisao_atualizar(&c, true, 1, 2048, 76);  // terminou 4 contagens atrás (folga)
  colisao_atualizar(&c, false, 1, 2048, 76);
  // mesmo movimento (contador igual), mas a referência foi refeita parado
  colisao_atualizar(&c, true, 1, 2048, 76);
  TEST_ASSERT_FALSE(colisao_atualizar(&c, true, 1, 4096, 156));
  TEST_ASSERT_FALSE(colisao_atualizar(&c, true, 1, 4096, 156));
  TEST_ASSERT_EQUAL_INT32(0, colisao_erro(&c));
}

void test_primeira_leitura_so_referencia(void) {
  // Mesmo com números "absurdos", a primeira leitura do movimento é a referência.
  TEST_ASSERT_FALSE(colisao_atualizar(&c, true, 7, 100000, -300));
  TEST_ASSERT_FALSE(colisao_atualizar(&c, true, 7, 100064, -298));
  TEST_ASSERT_FALSE(colisao_disparou(&c));
}

void test_desabilitado_nao_dispara(void) {
  colisao_habilitar(&c, false);
  colisao_atualizar(&c, true, 1, 0, 0);
  for (int i = 0; i < 10; i++) TEST_ASSERT_FALSE(colisao_atualizar(&c, true, 1, 5000, 0));
  TEST_ASSERT_FALSE(colisao_disparou(&c));
  colisao_habilitar(&c, true);
  colisao_atualizar(&c, true, 1, 5000, 0);  // religou: referência nova
  TEST_ASSERT_FALSE(colisao_atualizar(&c, true, 1, 5064, enc(64)));
}

void test_escala_da_calibracao(void) {
  // Calibrado: 258 passos/mm e 10,1 contagens/mm
  colisao_escala(&c, 10.1f / 258.0f);
  colisao_atualizar(&c, true, 1, 0, 0);
  TEST_ASSERT_FALSE(colisao_atualizar(&c, true, 1, 25800, 1010));
  TEST_ASSERT_FALSE(colisao_atualizar(&c, true, 1, 25800, 1010));
  TEST_ASSERT_EQUAL_INT32(0, colisao_erro(&c));
}

void test_confirmar_zero_vira_um(void) {
  ColisaoCfg cfg = {6, 0};
  colisao_init(&c, &cfg, K);
  colisao_atualizar(&c, true, 1, 0, 0);
  TEST_ASSERT_TRUE(colisao_atualizar(&c, true, 1, 1000, 0));
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_movimento_normal_nao_dispara);
  RUN_TEST(test_sentido_negativo_normal_nao_dispara);
  RUN_TEST(test_atraso_pequeno_e_tolerado);
  RUN_TEST(test_colisao_encoder_para_e_dispara);
  RUN_TEST(test_pico_isolado_nao_dispara);
  RUN_TEST(test_dispara_uma_vez_e_trava_ate_rearmar);
  RUN_TEST(test_movimento_novo_refaz_a_referencia);
  RUN_TEST(test_parado_refaz_a_referencia);
  RUN_TEST(test_primeira_leitura_so_referencia);
  RUN_TEST(test_desabilitado_nao_dispara);
  RUN_TEST(test_escala_da_calibracao);
  RUN_TEST(test_confirmar_zero_vira_um);
  return UNITY_END();
}
