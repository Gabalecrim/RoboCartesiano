// Testes da posição pelo encoder (rodam no PC).
//
//   pio test -e native -f test_malha
//
// Eixo X: 2048 passos e 80 contagens por volta -> 25,6 passos por contagem.

#include <unity.h>
#include "malha.h"

static const float PPC = 2048.0f / 80.0f;  // 25,6
static const MalhaCfg CFG = {1, 4};        // tolerância 1 contagem, 4 correções

static Malha m;

void setUp(void) { malha_init(&m, &CFG, PPC); }
void tearDown(void) {}

void test_ja_no_alvo_nao_move(void) {
  TEST_ASSERT_EQUAL_INT32(0, malha_ir(&m, 100, 101));
  TEST_ASSERT_EQUAL(MALHA_CHEGOU, malha_estado(&m));
  TEST_ASSERT_FALSE(malha_ativa(&m));
}

void test_primeiro_movimento_vem_do_encoder(void) {
  // 100 contagens = 2560 passos, nos dois sentidos.
  TEST_ASSERT_EQUAL_INT32(2560, malha_ir(&m, 100, 0));
  TEST_ASSERT_TRUE(malha_ativa(&m));
  TEST_ASSERT_EQUAL_INT32(-2560, malha_ir(&m, 0, 100));
}

void test_chegou_sem_correcao(void) {
  malha_ir(&m, 100, 0);
  TEST_ASSERT_EQUAL_INT32(0, malha_parou(&m, 100));
  TEST_ASSERT_EQUAL(MALHA_CHEGOU, malha_estado(&m));
  TEST_ASSERT_EQUAL_UINT8(0, malha_correcoes(&m));
}

void test_perdeu_passo_corrige_o_que_falta(void) {
  malha_ir(&m, 100, 0);
  // Andou só 90 contagens: faltam 10 = 256 passos.
  TEST_ASSERT_EQUAL_INT32(256, malha_parou(&m, 90));
  TEST_ASSERT_TRUE(malha_ativa(&m));
  TEST_ASSERT_EQUAL_INT32(0, malha_parou(&m, 100));
  TEST_ASSERT_EQUAL(MALHA_CHEGOU, malha_estado(&m));
  TEST_ASSERT_EQUAL_UINT8(1, malha_correcoes(&m));
}

void test_passou_do_alvo_volta(void) {
  malha_ir(&m, 100, 0);
  TEST_ASSERT_EQUAL_INT32(-77, malha_parou(&m, 103));  // -3 x 25,6 = -76,8
}

void test_tolerancia_aceita_uma_contagem(void) {
  malha_ir(&m, 100, 0);
  TEST_ASSERT_EQUAL_INT32(0, malha_parou(&m, 99));
  TEST_ASSERT_EQUAL(MALHA_CHEGOU, malha_estado(&m));
  TEST_ASSERT_EQUAL_INT32(1, malha_erro(&m));
}

void test_esgota_as_correcoes_e_falha(void) {
  malha_ir(&m, 100, 0);
  for (int i = 0; i < 4; i++) TEST_ASSERT_NOT_EQUAL(0, malha_parou(&m, 50));  // travado
  TEST_ASSERT_EQUAL_INT32(0, malha_parou(&m, 50));
  TEST_ASSERT_EQUAL(MALHA_FALHOU, malha_estado(&m));
  TEST_ASSERT_EQUAL_INT32(50, malha_erro(&m));
  TEST_ASSERT_FALSE(malha_ativa(&m));
}

void test_cancelar_para_de_corrigir(void) {
  malha_ir(&m, 100, 0);
  malha_cancelar(&m);
  TEST_ASSERT_EQUAL(MALHA_PARADA, malha_estado(&m));
  TEST_ASSERT_EQUAL_INT32(0, malha_parou(&m, 50));
}

void test_nova_escala(void) {
  malha_escala(&m, 25.0f);
  TEST_ASSERT_EQUAL_INT32(250, malha_ir(&m, 10, 0));
  malha_escala(&m, 0.0f);  // inválida: mantém a anterior
  TEST_ASSERT_EQUAL_INT32(250, malha_ir(&m, 10, 0));
}

void test_erro_pequeno_com_tolerancia_zero_anda_pelo_menos_um_passo(void) {
  MalhaCfg c = {0, 4};
  malha_init(&m, &c, 0.4f);  // 1 contagem = 0,4 passo -> arredonda para 0
  TEST_ASSERT_EQUAL_INT32(1, malha_ir(&m, 1, 0));
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_ja_no_alvo_nao_move);
  RUN_TEST(test_primeiro_movimento_vem_do_encoder);
  RUN_TEST(test_chegou_sem_correcao);
  RUN_TEST(test_perdeu_passo_corrige_o_que_falta);
  RUN_TEST(test_passou_do_alvo_volta);
  RUN_TEST(test_tolerancia_aceita_uma_contagem);
  RUN_TEST(test_esgota_as_correcoes_e_falha);
  RUN_TEST(test_cancelar_para_de_corrigir);
  RUN_TEST(test_nova_escala);
  RUN_TEST(test_erro_pequeno_com_tolerancia_zero_anda_pelo_menos_um_passo);
  return UNITY_END();
}
