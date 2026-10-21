// Testes do decodificador de quadratura (rodam no PC).
//
//   pio test -e native
//
// Cada teste é um comportamento que o encoder PRECISA ter no robô.

#include <stdbool.h>
#include <unity.h>
#include "quadrature.h"

// Sequência no sentido positivo: 00 -> 01 -> 11 -> 10 -> 00
static const uint8_t FWD[4] = {0b00, 0b01, 0b11, 0b10};

// Gira 'ciclos' ciclos completos (4 contagens cada) num sentido.
static void girar(Quadrature *q, int ciclos, bool positivo) {
  for (int c = 0; c < ciclos; c++) {
    for (int i = 1; i <= 4; i++) {
      int idx = positivo ? (i % 4) : ((4 - i) % 4);
      quadrature_update(q, FWD[idx]);
    }
  }
}

void setUp(void) {}
void tearDown(void) {}

// ---------- tabela ----------

void test_mesmo_estado_nao_conta(void) {
  for (uint8_t s = 0; s < 4; s++) {
    TEST_ASSERT_EQUAL_INT8(0, quadratureDelta(s, s));
  }
}

void test_passos_positivos(void) {
  TEST_ASSERT_EQUAL_INT8(+1, quadratureDelta(0b00, 0b01));
  TEST_ASSERT_EQUAL_INT8(+1, quadratureDelta(0b01, 0b11));
  TEST_ASSERT_EQUAL_INT8(+1, quadratureDelta(0b11, 0b10));
  TEST_ASSERT_EQUAL_INT8(+1, quadratureDelta(0b10, 0b00));
}

void test_passos_negativos(void) {
  TEST_ASSERT_EQUAL_INT8(-1, quadratureDelta(0b01, 0b00));
  TEST_ASSERT_EQUAL_INT8(-1, quadratureDelta(0b11, 0b01));
  TEST_ASSERT_EQUAL_INT8(-1, quadratureDelta(0b10, 0b11));
  TEST_ASSERT_EQUAL_INT8(-1, quadratureDelta(0b00, 0b10));
}

void test_transicoes_invalidas(void) {
  TEST_ASSERT_EQUAL_INT8(QUAD_INVALID, quadratureDelta(0b00, 0b11));
  TEST_ASSERT_EQUAL_INT8(QUAD_INVALID, quadratureDelta(0b11, 0b00));
  TEST_ASSERT_EQUAL_INT8(QUAD_INVALID, quadratureDelta(0b01, 0b10));
  TEST_ASSERT_EQUAL_INT8(QUAD_INVALID, quadratureDelta(0b10, 0b01));
}

// ---------- contagem ----------

void test_uma_volta_positiva_da_80(void) {
  Quadrature q = {0};
  quadrature_reset(&q, 0b00, 0);
  girar(&q, 20, true);  // KY-040: 20 ciclos por volta
  TEST_ASSERT_EQUAL_INT32(80, q.count);
  TEST_ASSERT_EQUAL_UINT32(0, q.errors);
}

void test_ida_e_volta_retorna_a_zero(void) {
  Quadrature q = {0};
  quadrature_reset(&q, 0b00, 0);
  girar(&q, 20, true);
  girar(&q, 20, false);
  TEST_ASSERT_EQUAL_INT32(0, q.count);
}

void test_inversao_no_meio_do_ciclo(void) {
  Quadrature q = {0};
  quadrature_reset(&q, 0b00, 0);
  quadrature_update(&q, 0b01);  // +1
  quadrature_update(&q, 0b11);  // +2
  quadrature_update(&q, 0b01);  // volta: +1
  quadrature_update(&q, 0b00);  // volta: 0
  quadrature_update(&q, 0b10);  // negativo: -1
  TEST_ASSERT_EQUAL_INT32(-1, q.count);
}

void test_bounce_se_anula(void) {
  // Contato mecânico tremendo na borda: 00 -> 01 -> 00 -> 01
  Quadrature q = {0};
  quadrature_reset(&q, 0b00, 0);
  quadrature_update(&q, 0b01);
  quadrature_update(&q, 0b00);
  quadrature_update(&q, 0b01);
  TEST_ASSERT_EQUAL_INT32(1, q.count);
  TEST_ASSERT_EQUAL_UINT32(0, q.errors);
}

void test_invalida_conta_erro_e_nao_mexe_na_posicao(void) {
  Quadrature q = {0};
  quadrature_reset(&q, 0b00, 10);
  quadrature_update(&q, 0b11);  // pulou um estado
  TEST_ASSERT_EQUAL_INT32(10, q.count);
  TEST_ASSERT_EQUAL_UINT32(1, q.errors);
  // depois do erro, volta a contar normalmente a partir de 11
  quadrature_update(&q, 0b10);  // 11 -> 10 = +1
  TEST_ASSERT_EQUAL_INT32(11, q.count);
}

void test_reset_define_posicao(void) {
  Quadrature q = {0};
  quadrature_reset(&q, 0b00, 0);
  girar(&q, 5, true);
  quadrature_reset(&q, 0b00, -40);  // ex.: homing define a origem
  TEST_ASSERT_EQUAL_INT32(-40, q.count);
  quadrature_update(&q, 0b01);
  TEST_ASSERT_EQUAL_INT32(-39, q.count);
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_mesmo_estado_nao_conta);
  RUN_TEST(test_passos_positivos);
  RUN_TEST(test_passos_negativos);
  RUN_TEST(test_transicoes_invalidas);
  RUN_TEST(test_uma_volta_positiva_da_80);
  RUN_TEST(test_ida_e_volta_retorna_a_zero);
  RUN_TEST(test_inversao_no_meio_do_ciclo);
  RUN_TEST(test_bounce_se_anula);
  RUN_TEST(test_invalida_conta_erro_e_nao_mexe_na_posicao);
  RUN_TEST(test_reset_define_posicao);
  return UNITY_END();
}
