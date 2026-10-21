// Testes da sequência de bobinas do 28BYJ-48 (rodam no PC).
//
//   pio test -e native
//
// Cada teste é um comportamento que o motor PRECISA ter no robô.

#include <unity.h>
#include "stepseq.h"

void setUp(void) {}
void tearDown(void) {}

// ---------- tabela ----------

void test_tabela_de_fases(void) {
  TEST_ASSERT_EQUAL_HEX8(0b0011, stepseq_coils(0));  // IN1 + IN2
  TEST_ASSERT_EQUAL_HEX8(0b0110, stepseq_coils(1));  // IN2 + IN3
  TEST_ASSERT_EQUAL_HEX8(0b1100, stepseq_coils(2));  // IN3 + IN4
  TEST_ASSERT_EQUAL_HEX8(0b1001, stepseq_coils(3));  // IN4 + IN1
}

void test_sempre_duas_bobinas_ligadas(void) {
  for (uint8_t f = 0; f < 4; f++) {
    uint8_t c = stepseq_coils(f);
    int ligadas = (c & 1) + ((c >> 1) & 1) + ((c >> 2) & 1) + ((c >> 3) & 1);
    TEST_ASSERT_EQUAL_INT(2, ligadas);
  }
}

// ---------- passo ----------

void test_passo_positivo_avanca_fase(void) {
  StepSeq s;
  stepseq_reset(&s, 0);
  stepseq_step(&s, +1);
  TEST_ASSERT_EQUAL_UINT8(1, s.phase);
  TEST_ASSERT_EQUAL_INT32(1, s.position);
}

void test_passo_negativo_da_a_volta(void) {
  StepSeq s;
  stepseq_reset(&s, 0);
  stepseq_step(&s, -1);  // 0 -> 3
  TEST_ASSERT_EQUAL_UINT8(3, s.phase);
  TEST_ASSERT_EQUAL_INT32(-1, s.position);
}

void test_quatro_passos_voltam_a_fase_inicial(void) {
  StepSeq s;
  stepseq_reset(&s, 0);
  for (int i = 0; i < 4; i++) stepseq_step(&s, +1);
  TEST_ASSERT_EQUAL_UINT8(0, s.phase);
  TEST_ASSERT_EQUAL_INT32(4, s.position);
}

void test_dir_zero_nao_mexe(void) {
  StepSeq s;
  stepseq_reset(&s, 7);
  stepseq_step(&s, 0);
  TEST_ASSERT_EQUAL_UINT8(0, s.phase);
  TEST_ASSERT_EQUAL_INT32(7, s.position);
}

// ---------- volta completa ----------

void test_uma_volta_conta_2048(void) {
  StepSeq s;
  stepseq_reset(&s, 0);
  for (int i = 0; i < 2048; i++) stepseq_step(&s, +1);
  TEST_ASSERT_EQUAL_INT32(2048, s.position);
}

void test_ida_e_volta_retorna_a_zero(void) {
  StepSeq s;
  stepseq_reset(&s, 0);
  for (int i = 0; i < 13; i++) stepseq_step(&s, +1);
  for (int i = 0; i < 13; i++) stepseq_step(&s, -1);
  TEST_ASSERT_EQUAL_INT32(0, s.position);
  TEST_ASSERT_EQUAL_UINT8(0, s.phase);
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_tabela_de_fases);
  RUN_TEST(test_sempre_duas_bobinas_ligadas);
  RUN_TEST(test_passo_positivo_avanca_fase);
  RUN_TEST(test_passo_negativo_da_a_volta);
  RUN_TEST(test_quatro_passos_voltam_a_fase_inicial);
  RUN_TEST(test_dir_zero_nao_mexe);
  RUN_TEST(test_uma_volta_conta_2048);
  RUN_TEST(test_ida_e_volta_retorna_a_zero);
  return UNITY_END();
}
