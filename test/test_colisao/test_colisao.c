// Testes da proteção contra colisão (rodam no PC).
//
//   pio test -e native -f test_colisao
//
// Eixo X: 2048 passos e 80 contagens por volta -> 80/2048 contagens por
// passo (25,6 passos por contagem). Janela de 10 contagens = 256 passos = 1 mm.

#include <unity.h>
#include "colisao.h"

static const float K = 80.0f / 2048.0f;
static const ColisaoCfg CFG = {10, 40, 2};  // janela 10 cont, avanço mínimo 40%, 2 janelas

static ColisaoMonitor c;

void setUp(void) { colisao_init(&c, &CFG, K); }
void tearDown(void) {}

static int32_t arred(float v) { return (int32_t)(v >= 0.0f ? v + 0.5f : v - 0.5f); }

// Encoder que acompanha o motor com um ganho (1.0 = perfeito).
static int32_t enc(int32_t passos, float ganho) { return arred(passos * K * ganho); }

// Anda de 'de' até 'ate' lendo a cada 'passo' passos com encoder de ganho 'g'.
// Devolve true se disparou.
static bool andar(uint32_t mov, int32_t de, int32_t ate, int32_t passo, float g) {
  int32_t dir = ate >= de ? 1 : -1;
  for (int32_t p = de; dir > 0 ? p <= ate : p >= ate; p += dir * passo) {
    if (colisao_atualizar(&c, true, mov, p, enc(p, g))) return true;
  }
  return false;
}

void test_movimento_normal_nao_dispara(void) {
  TEST_ASSERT_FALSE(andar(1, 0, 48640, 64, 1.0f));  // 190 mm
  TEST_ASSERT_FALSE(colisao_disparou(&c));
  TEST_ASSERT_GREATER_OR_EQUAL_INT32(90, colisao_pior_pct(&c));
}

void test_sentido_negativo_normal_nao_dispara(void) {
  TEST_ASSERT_FALSE(andar(1, 0, -48640, 64, 1.0f));
  TEST_ASSERT_GREATER_OR_EQUAL_INT32(90, colisao_pior_pct(&c));
}

void test_calibracao_3_pct_diferente_nao_dispara(void) {
  // O caso do alarme falso: no curso todo o erro TOTAL passa de 50
  // contagens, mas cada janela anda ~97%.
  TEST_ASSERT_FALSE(andar(1, 0, 48640, 64, 0.97f));
  TEST_ASSERT_GREATER_THAN_INT32(6, colisao_erro_max(&c));
}

void test_encoder_perdendo_15_pct_nao_dispara(void) {
  TEST_ASSERT_FALSE(andar(1, 0, 48640, 64, 0.85f));
}

void test_folga_na_partida_nao_dispara(void) {
  // Os primeiros 0,8 mm (205 passos) o encoder não anda (folga de inversão).
  colisao_atualizar(&c, true, 1, 0, 0);
  bool disparou = false;
  for (int32_t p = 64; p <= 25600 && !disparou; p += 64) {
    int32_t e = p < 205 ? 0 : enc(p - 205, 1.0f);
    disparou = colisao_atualizar(&c, true, 1, p, e);
  }
  TEST_ASSERT_FALSE(disparou);
}

void test_colisao_encoder_para_e_dispara(void) {
  TEST_ASSERT_FALSE(andar(1, 0, 5120, 64, 1.0f));
  int32_t parado = enc(5120, 1.0f);  // bateu: o carro para, o motor continua contando
  bool disparou = false;
  int32_t passos_ate = 0;
  for (int32_t q = 5120; q < 5120 + 4096 && !disparou; q += 32) {
    disparou = colisao_atualizar(&c, true, 1, q, parado);
    passos_ate = q - 5120;
  }
  TEST_ASSERT_TRUE(disparou);
  // 2 janelas de 256 passos (+ a janela em andamento) = no máximo ~3 mm
  TEST_ASSERT_LESS_OR_EQUAL_INT32(3 * 256, passos_ate);
  TEST_ASSERT_LESS_THAN_INT32(40, colisao_pior_pct(&c));
}

void test_colisao_na_partida_dispara(void) {
  // Já encostado: o encoder não anda nunca. A 1ª janela é ignorada (folga),
  // as 2 seguintes disparam.
  colisao_atualizar(&c, true, 1, 0, 0);
  bool disparou = false;
  int32_t p = 0;
  while (p < 4096 && !disparou) {
    p += 32;
    disparou = colisao_atualizar(&c, true, 1, p, 0);
  }
  TEST_ASSERT_TRUE(disparou);
  TEST_ASSERT_LESS_OR_EQUAL_INT32(3 * 256 + 32, p);
}

void test_encoder_ao_contrario_dispara(void) {
  // Encoder invertido (fios A/B trocados): avanço negativo.
  TEST_ASSERT_TRUE(andar(1, 0, 25600, 64, -1.0f));
  TEST_ASSERT_LESS_THAN_INT32(0, colisao_pior_pct(&c));
}

void test_uma_janela_ruim_nao_dispara(void) {
  TEST_ASSERT_FALSE(andar(1, 0, 2560, 64, 1.0f));
  // Uma janela (256 passos) com o encoder parado, depois volta a acompanhar.
  int32_t base = enc(2560, 1.0f);
  TEST_ASSERT_FALSE(colisao_atualizar(&c, true, 1, 2560 + 256, base));
  int32_t atraso = 10;  // ficou 10 contagens para trás e segue normal
  for (int32_t p = 2560 + 256 + 64; p <= 12800; p += 64) {
    TEST_ASSERT_FALSE(colisao_atualizar(&c, true, 1, p, enc(p, 1.0f) - atraso));
  }
  TEST_ASSERT_FALSE(colisao_disparou(&c));
}

void test_dispara_uma_vez_e_trava_ate_rearmar(void) {
  colisao_atualizar(&c, true, 1, 0, 0);
  TEST_ASSERT_FALSE(colisao_atualizar(&c, true, 1, 256, 10));  // 1ª janela
  TEST_ASSERT_FALSE(colisao_atualizar(&c, true, 1, 512, 10));  // ruim 1
  TEST_ASSERT_TRUE(colisao_atualizar(&c, true, 1, 768, 10));   // ruim 2: dispara
  TEST_ASSERT_FALSE(colisao_atualizar(&c, true, 1, 1024, 10)); // já disparou: não repete
  TEST_ASSERT_TRUE(colisao_disparou(&c));
  TEST_ASSERT_FALSE(colisao_atualizar(&c, false, 1, 1024, 10));
  TEST_ASSERT_TRUE(colisao_disparou(&c));  // parado não limpa
  colisao_rearmar(&c);
  TEST_ASSERT_FALSE(colisao_disparou(&c));
}

void test_movimento_novo_refaz_a_referencia(void) {
  // Entre dois movimentos alguém zerou o motor e o encoder (homing) sem
  // o monitor ver o motor parado: o movimento novo não pode virar colisão.
  TEST_ASSERT_FALSE(andar(1, 0, 5000, 64, 1.0f));
  TEST_ASSERT_FALSE(andar(2, 0, -5120, 64, 1.0f));
  TEST_ASSERT_FALSE(colisao_disparou(&c));
}

void test_parado_refaz_a_referencia(void) {
  TEST_ASSERT_FALSE(andar(1, 0, 2048, 64, 1.0f));
  colisao_atualizar(&c, false, 1, 2048, 70);  // parou; alguém mexeu no encoder
  // mesmo contador de movimento, mas a referência foi refeita parado
  colisao_atualizar(&c, true, 1, 2048, 70);
  for (int32_t p = 2048 + 64; p <= 6144; p += 64) {
    TEST_ASSERT_FALSE(colisao_atualizar(&c, true, 1, p, 70 + enc(p - 2048, 1.0f)));
  }
}

void test_desabilitado_nao_dispara(void) {
  colisao_habilitar(&c, false);
  colisao_atualizar(&c, true, 1, 0, 0);
  for (int32_t p = 256; p <= 5120; p += 256) TEST_ASSERT_FALSE(colisao_atualizar(&c, true, 1, p, 0));
  TEST_ASSERT_FALSE(colisao_disparou(&c));
}

void test_escala_da_calibracao(void) {
  // Calibrado: 258 passos/mm e 10,1 contagens/mm -> o avanço fica ~100%.
  colisao_escala(&c, 10.1f / 258.0f);
  colisao_atualizar(&c, true, 1, 0, 0);
  for (int32_t p = 64; p <= 25800; p += 64) {
    TEST_ASSERT_FALSE(colisao_atualizar(&c, true, 1, p, arred(p * 10.1f / 258.0f)));
  }
  TEST_ASSERT_GREATER_OR_EQUAL_INT32(85, colisao_pior_pct(&c));
}

void test_confirmar_zero_vira_um(void) {
  ColisaoCfg cfg = {10, 40, 0};
  colisao_init(&c, &cfg, K);
  colisao_atualizar(&c, true, 1, 0, 0);
  TEST_ASSERT_FALSE(colisao_atualizar(&c, true, 1, 256, 10));  // 1ª janela ignorada
  TEST_ASSERT_TRUE(colisao_atualizar(&c, true, 1, 512, 10));
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_movimento_normal_nao_dispara);
  RUN_TEST(test_sentido_negativo_normal_nao_dispara);
  RUN_TEST(test_calibracao_3_pct_diferente_nao_dispara);
  RUN_TEST(test_encoder_perdendo_15_pct_nao_dispara);
  RUN_TEST(test_folga_na_partida_nao_dispara);
  RUN_TEST(test_colisao_encoder_para_e_dispara);
  RUN_TEST(test_colisao_na_partida_dispara);
  RUN_TEST(test_encoder_ao_contrario_dispara);
  RUN_TEST(test_uma_janela_ruim_nao_dispara);
  RUN_TEST(test_dispara_uma_vez_e_trava_ate_rearmar);
  RUN_TEST(test_movimento_novo_refaz_a_referencia);
  RUN_TEST(test_parado_refaz_a_referencia);
  RUN_TEST(test_desabilitado_nao_dispara);
  RUN_TEST(test_escala_da_calibracao);
  RUN_TEST(test_confirmar_zero_vira_um);
  return UNITY_END();
}
