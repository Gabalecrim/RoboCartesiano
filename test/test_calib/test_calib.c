// Testes da calibração de distância (rodam no PC).
//
//   pio test -e native -f test_calib
//
// Números do eixo X: TR8x8 (8 mm por volta), 28BYJ-48 com 2048 passos
// por volta e KY-040 com 80 contagens por volta:
//   256 passos/mm e 10 contagens/mm nominais.

#include <math.h>
#include <unity.h>
#include "calib.h"

static const float CONT_POR_PASSO = 80.0f / 2048.0f;

void setUp(void) {}
void tearDown(void) {}

void test_medida_igual_ao_pedido_mantem_o_nominal(void) {
  CalibDist r;
  TEST_ASSERT_EQUAL_INT(CAL_DIST_OK,
                        calib_distancia(25600, 1000, 100.0f, 100.0f, 10.0f, CONT_POR_PASSO, &r));
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 256.0f, r.passos_por_mm);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 10.0f, r.contagens_por_mm);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, r.erro_pct);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 100.0f, r.encoder_pct);
}

void test_andou_menos_aumenta_passos_por_mm(void) {
  // Pediu 100, andou 99,2: precisa de MAIS passos para cada mm.
  CalibDist r;
  TEST_ASSERT_EQUAL_INT(CAL_DIST_OK,
                        calib_distancia(25600, 1000, 100.0f, 99.2f, 10.0f, CONT_POR_PASSO, &r));
  TEST_ASSERT_FLOAT_WITHIN(1e-2f, 258.06f, r.passos_por_mm);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, -0.8f, r.erro_pct);
}

void test_andou_mais_diminui_passos_por_mm(void) {
  CalibDist r;
  calib_distancia(25600, 1000, 100.0f, 101.0f, 10.0f, CONT_POR_PASSO, &r);
  TEST_ASSERT_FLOAT_WITHIN(1e-2f, 253.47f, r.passos_por_mm);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 1.0f, r.erro_pct);
}

void test_aplicar_o_novo_valor_acerta_a_distancia(void) {
  // Com o passos/mm calibrado, os mesmos 99,2 mm saem com os mesmos passos.
  CalibDist r;
  calib_distancia(25600, 1000, 100.0f, 99.2f, 10.0f, CONT_POR_PASSO, &r);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 25600.0f, 99.2f * r.passos_por_mm);
}

void test_sentido_negativo_da_o_mesmo_resultado(void) {
  CalibDist a, b;
  calib_distancia(25600, 1000, 100.0f, 99.5f, 10.0f, CONT_POR_PASSO, &a);
  calib_distancia(-25600, -1000, 100.0f, 99.5f, 10.0f, CONT_POR_PASSO, &b);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, a.passos_por_mm, b.passos_por_mm);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, a.contagens_por_mm, b.contagens_por_mm);
}

void test_encoder_que_perde_contagens_aparece_no_percentual(void) {
  // 12% a menos de contagens (como o KY-040 sem debounce).
  CalibDist r;
  calib_distancia(25600, 880, 100.0f, 100.0f, 10.0f, CONT_POR_PASSO, &r);
  TEST_ASSERT_FLOAT_WITHIN(1e-2f, 88.0f, r.encoder_pct);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 8.8f, r.contagens_por_mm);
}

void test_fora_da_tolerancia_avisa_mas_mostra_os_numeros(void) {
  // Digitou 10 em vez de 100: não pode virar 2560 passos/mm.
  CalibDist r;
  TEST_ASSERT_EQUAL_INT(CAL_DIST_FORA_DA_TOLERANCIA,
                        calib_distancia(25600, 1000, 100.0f, 10.0f, 10.0f, CONT_POR_PASSO, &r));
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, -90.0f, r.erro_pct);
}

void test_no_limite_da_tolerancia_aceita(void) {
  CalibDist r;
  TEST_ASSERT_EQUAL_INT(CAL_DIST_OK,
                        calib_distancia(25600, 1000, 100.0f, 90.0f, 10.0f, CONT_POR_PASSO, &r));
}

void test_entradas_invalidas(void) {
  CalibDist r;
  TEST_ASSERT_EQUAL_INT(CAL_DIST_ENTRADA_INVALIDA,
                        calib_distancia(0, 0, 100.0f, 100.0f, 10.0f, CONT_POR_PASSO, &r));
  TEST_ASSERT_EQUAL_INT(CAL_DIST_ENTRADA_INVALIDA,
                        calib_distancia(25600, 1000, 100.0f, 0.0f, 10.0f, CONT_POR_PASSO, &r));
  TEST_ASSERT_EQUAL_INT(CAL_DIST_ENTRADA_INVALIDA,
                        calib_distancia(25600, 1000, 100.0f, -5.0f, 10.0f, CONT_POR_PASSO, &r));
  TEST_ASSERT_EQUAL_INT(CAL_DIST_ENTRADA_INVALIDA,
                        calib_distancia(25600, 1000, 0.0f, 100.0f, 10.0f, CONT_POR_PASSO, &r));
  TEST_ASSERT_EQUAL_INT(CAL_DIST_ENTRADA_INVALIDA,
                        calib_distancia(25600, 1000, 100.0f, NAN, 10.0f, CONT_POR_PASSO, &r));
}

void test_sem_resolucao_do_encoder_nao_calcula_percentual(void) {
  CalibDist r;
  calib_distancia(25600, 1000, 100.0f, 100.0f, 10.0f, 0.0f, &r);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, r.encoder_pct);
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_medida_igual_ao_pedido_mantem_o_nominal);
  RUN_TEST(test_andou_menos_aumenta_passos_por_mm);
  RUN_TEST(test_andou_mais_diminui_passos_por_mm);
  RUN_TEST(test_aplicar_o_novo_valor_acerta_a_distancia);
  RUN_TEST(test_sentido_negativo_da_o_mesmo_resultado);
  RUN_TEST(test_encoder_que_perde_contagens_aparece_no_percentual);
  RUN_TEST(test_fora_da_tolerancia_avisa_mas_mostra_os_numeros);
  RUN_TEST(test_no_limite_da_tolerancia_aceita);
  RUN_TEST(test_entradas_invalidas);
  RUN_TEST(test_sem_resolucao_do_encoder_nao_calcula_percentual);
  return UNITY_END();
}
