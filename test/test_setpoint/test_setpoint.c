// Testes dos setpoints em mm, eixos X e Z (rodam no PC).
//
//   pio test -e native -f test_setpoint
//
// X: centro do braço em 30 mm na chave, curso 190 mm (X de 30 a 220),
//    TR8x8 = 256 passos/mm, afastar da chave = giro 'b' (-1).
// Z: 0 no topo, curso 80 mm, fuso de 24 mm/volta = 85,33 passos/mm,
//    descer = giro 'f' (+1).

#include <unity.h>
#include "setpoint.h"

static const EixoMm EX = {30.0f, 190.0f, 256.0f, -1};
static const EixoMm EZ = {0.0f, 80.0f, 2048.0f / 24.0f, +1};
static ProgLimites LIM;

static Setpoint sp(float x, float z, float v, uint32_t espera) {
  Setpoint s = {x, z, v, espera};
  return s;
}

void setUp(void) {
  LIM.x = EX;
  LIM.z = EZ;
  LIM.vmax_mm_s = 7.6f;
}
void tearDown(void) {}

// ---------- coordenada ----------

void test_limites_do_curso_x(void) {
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 30.0f, eixo_mm_min(&EX));
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 220.0f, eixo_mm_max(&EX));
  TEST_ASSERT_TRUE(eixo_mm_dentro(&EX, 30.0f));
  TEST_ASSERT_TRUE(eixo_mm_dentro(&EX, 220.0f));
  TEST_ASSERT_FALSE(eixo_mm_dentro(&EX, 29.9f));
  TEST_ASSERT_FALSE(eixo_mm_dentro(&EX, 220.1f));
}

void test_limites_do_curso_z(void) {
  TEST_ASSERT_TRUE(eixo_mm_dentro(&EZ, 0.0f));
  TEST_ASSERT_TRUE(eixo_mm_dentro(&EZ, 80.0f));
  TEST_ASSERT_FALSE(eixo_mm_dentro(&EZ, -0.1f));
  TEST_ASSERT_FALSE(eixo_mm_dentro(&EZ, 80.1f));
}

void test_nan_fica_fora_do_curso(void) {
  volatile float zero = 0.0f;
  TEST_ASSERT_FALSE(eixo_mm_dentro(&EX, zero / zero));
  TEST_ASSERT_FALSE(eixo_mm_dentro(&EZ, zero / zero));
}

void test_referencia_e_passo_zero(void) {
  TEST_ASSERT_EQUAL_INT32(0, eixo_mm_para_passos(&EX, 30.0f));
  TEST_ASSERT_EQUAL_INT32(0, eixo_mm_para_passos(&EZ, 0.0f));
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 30.0f, eixo_passos_para_mm(&EX, 0));
}

void test_mm_para_passos_x(void) {
  // 100 mm do braço = 70 mm do carro = 17920 passos, giro 'b' (negativo)
  TEST_ASSERT_EQUAL_INT32(-17920, eixo_mm_para_passos(&EX, 100.0f));
  TEST_ASSERT_EQUAL_INT32(-48640, eixo_mm_para_passos(&EX, 220.0f));
}

void test_mm_para_passos_z(void) {
  // 24 mm = 1 volta = 2048 passos; 80 mm = 3,33 voltas = 6827 passos
  TEST_ASSERT_EQUAL_INT32(2048, eixo_mm_para_passos(&EZ, 24.0f));
  TEST_ASSERT_EQUAL_INT32(6827, eixo_mm_para_passos(&EZ, 80.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.02f, 80.0f, eixo_passos_para_mm(&EZ, 6827));
}

void test_ida_e_volta_mm_passos(void) {
  for (float x = 30.0f; x <= 220.0f; x += 7.3f) {
    TEST_ASSERT_FLOAT_WITHIN(1.0f / 256.0f, x, eixo_passos_para_mm(&EX, eixo_mm_para_passos(&EX, x)));
  }
  for (float z = 0.0f; z <= 80.0f; z += 3.1f) {
    TEST_ASSERT_FLOAT_WITHIN(24.0f / 2048.0f, z, eixo_passos_para_mm(&EZ, eixo_mm_para_passos(&EZ, z)));
  }
}

void test_usa_a_calibracao(void) {
  EixoMm c = EX;
  c.passos_por_mm = 258.0f;
  TEST_ASSERT_EQUAL_INT32(-25800, eixo_mm_para_passos(&c, 130.0f));
}

// ---------- programa ----------

void test_adicionar_e_remover(void) {
  Programa p;
  prog_limpar(&p);
  TEST_ASSERT_EQUAL_INT(PROG_OK, prog_adicionar(&p, &LIM, sp(50, 0, 0, 0)));
  TEST_ASSERT_EQUAL_INT(PROG_OK, prog_adicionar(&p, &LIM, sp(100, 40, 1.5f, 500)));
  TEST_ASSERT_EQUAL_INT(PROG_OK, prog_adicionar(&p, &LIM, sp(150, 80, 0, 0)));
  TEST_ASSERT_EQUAL_UINT8(3, p.n);
  TEST_ASSERT_EQUAL_INT(PROG_OK, prog_remover(&p, 0));
  TEST_ASSERT_EQUAL_UINT8(2, p.n);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 100.0f, p.p[0].x_mm);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 40.0f, p.p[0].z_mm);
  TEST_ASSERT_EQUAL_UINT32(500, p.p[0].espera_ms);
  TEST_ASSERT_EQUAL_INT(PROG_INDICE_INVALIDO, prog_remover(&p, 2));
}

void test_recusa_fora_do_curso_velocidade_e_espera(void) {
  Programa p;
  prog_limpar(&p);
  TEST_ASSERT_EQUAL_INT(PROG_X_FORA_DO_CURSO, prog_adicionar(&p, &LIM, sp(10, 0, 0, 0)));
  TEST_ASSERT_EQUAL_INT(PROG_X_FORA_DO_CURSO, prog_adicionar(&p, &LIM, sp(230, 0, 0, 0)));
  TEST_ASSERT_EQUAL_INT(PROG_Z_FORA_DO_CURSO, prog_adicionar(&p, &LIM, sp(50, -1, 0, 0)));
  TEST_ASSERT_EQUAL_INT(PROG_Z_FORA_DO_CURSO, prog_adicionar(&p, &LIM, sp(50, 81, 0, 0)));
  TEST_ASSERT_EQUAL_INT(PROG_VEL_INVALIDA, prog_adicionar(&p, &LIM, sp(50, 0, 8.0f, 0)));
  TEST_ASSERT_EQUAL_INT(PROG_VEL_INVALIDA, prog_adicionar(&p, &LIM, sp(50, 0, -1.0f, 0)));
  TEST_ASSERT_EQUAL_INT(PROG_ESPERA_INVALIDA,
                        prog_adicionar(&p, &LIM, sp(50, 0, 0, PROG_ESPERA_MAX_MS + 1)));
  TEST_ASSERT_EQUAL_UINT8(0, p.n);
}

void test_programa_cheio(void) {
  Programa p;
  prog_limpar(&p);
  for (int i = 0; i < PROG_MAX_PONTOS; i++) {
    TEST_ASSERT_EQUAL_INT(PROG_OK, prog_adicionar(&p, &LIM, sp(40, 10, 0, 0)));
  }
  TEST_ASSERT_EQUAL_INT(PROG_CHEIO, prog_adicionar(&p, &LIM, sp(40, 10, 0, 0)));
}

void test_validar_aponta_o_ponto_ruim(void) {
  Programa p;
  prog_limpar(&p);
  prog_adicionar(&p, &LIM, sp(50, 0, 0, 0));
  prog_adicionar(&p, &LIM, sp(200, 70, 0, 0));
  uint8_t i = 99;
  TEST_ASSERT_EQUAL_INT(PROG_OK, prog_validar(&p, &LIM, &i));
  ProgLimites curto = LIM;
  curto.z.curso_mm = 50.0f;  // ex.: mudou a constante
  TEST_ASSERT_EQUAL_INT(PROG_Z_FORA_DO_CURSO, prog_validar(&p, &curto, &i));
  TEST_ASSERT_EQUAL_UINT8(1, i);
  p.n = 200;  // lixo da flash
  TEST_ASSERT_EQUAL_INT(PROG_CHEIO, prog_validar(&p, &LIM, &i));
}

// ---------- sequenciador ----------

// 0: desce até 40 em X 50; 1: sobe ao topo e vai a X 150 (espera 1 s); 2: X 100 sem mexer Z
static Programa tres(void) {
  Programa p;
  prog_limpar(&p);
  prog_adicionar(&p, &LIM, sp(50, 40, 0, 0));
  prog_adicionar(&p, &LIM, sp(150, 0, 0, 1000));
  prog_adicionar(&p, &LIM, sp(100, 0, 0, 0));
  return p;
}

// Conclui a fase atual (motor parado) e devolve a ação.
static SeqAcao parou(Seq *s, uint32_t t) { return seq_atualizar(s, false, t); }

void test_programa_vazio_termina_na_hora(void) {
  Programa p;
  prog_limpar(&p);
  Seq s;
  TEST_ASSERT_EQUAL_INT(SEQ_NADA, seq_iniciar(&s, &p, 1, 0.0f));
  TEST_ASSERT_EQUAL_INT(SEQ_FIM, seq_estado(&s));
  TEST_ASSERT_NULL(seq_alvo(&s));
}

void test_descer_anda_x_primeiro(void) {
  Programa p = tres();
  Seq s;
  TEST_ASSERT_EQUAL_INT(SEQ_MOVER, seq_iniciar(&s, &p, 1, 0.0f));  // Z no topo, alvo 40: desce
  TEST_ASSERT_EQUAL_INT(SEQ_EIXO_X, seq_eixo(&s));
  TEST_ASSERT_EQUAL_INT(SEQ_NADA, seq_atualizar(&s, true, 0));  // X andando
  TEST_ASSERT_EQUAL_INT(SEQ_MOVER, parou(&s, 10));
  TEST_ASSERT_EQUAL_INT(SEQ_EIXO_Z, seq_eixo(&s));             // depois desce o Z
  TEST_ASSERT_EQUAL_PTR(&p.p[0], seq_alvo(&s));
}

void test_subir_anda_z_primeiro(void) {
  Programa p = tres();
  Seq s;
  seq_iniciar(&s, &p, 1, 0.0f);
  parou(&s, 0);                                     // X do ponto 0
  TEST_ASSERT_EQUAL_INT(SEQ_MOVER, parou(&s, 0));   // Z do ponto 0 chegou: ponto 1
  TEST_ASSERT_EQUAL_PTR(&p.p[1], seq_alvo(&s));
  TEST_ASSERT_EQUAL_INT(SEQ_EIXO_Z, seq_eixo(&s));  // Z em 40, alvo 0: sobe primeiro
  TEST_ASSERT_EQUAL_INT(SEQ_MOVER, parou(&s, 0));
  TEST_ASSERT_EQUAL_INT(SEQ_EIXO_X, seq_eixo(&s));
}

void test_z_parado_anda_x_primeiro(void) {
  Programa p;
  prog_limpar(&p);
  prog_adicionar(&p, &LIM, sp(80, 20, 0, 0));
  Seq s;
  seq_iniciar(&s, &p, 1, 20.0f);
  TEST_ASSERT_EQUAL_INT(SEQ_EIXO_X, seq_eixo(&s));
}

void test_percorre_os_pontos_e_espera(void) {
  Programa p = tres();
  Seq s;
  seq_iniciar(&s, &p, 1, 0.0f);
  parou(&s, 0);
  parou(&s, 0);                                       // ponto 0 feito -> ponto 1
  parou(&s, 100);                                     // fase 1 do ponto 1
  TEST_ASSERT_EQUAL_INT(SEQ_NADA, parou(&s, 2000));   // chegou: espera 1000 ms
  TEST_ASSERT_EQUAL_INT(SEQ_ESPERANDO, seq_estado(&s));
  TEST_ASSERT_EQUAL_INT(SEQ_NADA, parou(&s, 2999));
  TEST_ASSERT_EQUAL_INT(SEQ_MOVER, parou(&s, 3000));
  TEST_ASSERT_EQUAL_PTR(&p.p[2], seq_alvo(&s));
  parou(&s, 3100);
  TEST_ASSERT_EQUAL_INT(SEQ_NADA, parou(&s, 3200));   // último ponto: fim
  TEST_ASSERT_EQUAL_INT(SEQ_FIM, seq_estado(&s));
  TEST_ASSERT_FALSE(seq_ativo(&s));
}

void test_espera_atravessa_a_virada_do_millis(void) {
  Programa p = tres();
  Seq s;
  seq_iniciar(&s, &p, 1, 0.0f);
  parou(&s, 0);
  parou(&s, 0);                 // -> ponto 1
  parou(&s, 0);
  parou(&s, 0xFFFFFF00u);       // chegou: espera começa perto da virada
  TEST_ASSERT_EQUAL_INT(SEQ_NADA, parou(&s, 0x100u));   // 512 ms
  TEST_ASSERT_EQUAL_INT(SEQ_MOVER, parou(&s, 0x2F0u));  // 1008 ms
}

void test_repete_os_ciclos_e_o_z_do_fim_vale_no_proximo(void) {
  Programa p;
  prog_limpar(&p);
  prog_adicionar(&p, &LIM, sp(50, 0, 0, 0));
  prog_adicionar(&p, &LIM, sp(150, 60, 0, 0));
  Seq s;
  seq_iniciar(&s, &p, 2, 0.0f);
  parou(&s, 0);
  parou(&s, 0);  // -> ponto 1
  parou(&s, 0);
  TEST_ASSERT_EQUAL_INT(SEQ_MOVER, parou(&s, 0));  // ciclo 2: volta ao ponto 0
  TEST_ASSERT_EQUAL_UINT32(1, s.ciclo);
  TEST_ASSERT_EQUAL_PTR(&p.p[0], seq_alvo(&s));
  TEST_ASSERT_EQUAL_INT(SEQ_EIXO_Z, seq_eixo(&s));  // Z em 60 -> 0: sobe primeiro
  parou(&s, 0);
  parou(&s, 0);
  parou(&s, 0);
  TEST_ASSERT_EQUAL_INT(SEQ_NADA, parou(&s, 0));
  TEST_ASSERT_EQUAL_INT(SEQ_FIM, seq_estado(&s));
}

void test_ciclos_zero_repete_sem_fim(void) {
  Programa p;
  prog_limpar(&p);
  prog_adicionar(&p, &LIM, sp(50, 0, 0, 0));
  Seq s;
  seq_iniciar(&s, &p, 0, 0.0f);
  for (int i = 0; i < 100; i++) {
    TEST_ASSERT_EQUAL_INT(SEQ_MOVER, parou(&s, 0));  // fase 2
    TEST_ASSERT_EQUAL_INT(SEQ_MOVER, parou(&s, 0));  // próximo ciclo
  }
  TEST_ASSERT_EQUAL_UINT32(100, s.ciclo);
  TEST_ASSERT_TRUE(seq_ativo(&s));
}

void test_pausar_andando_freia_e_continua_no_mesmo_ponto(void) {
  Programa p = tres();
  Seq s;
  seq_iniciar(&s, &p, 1, 0.0f);
  parou(&s, 0);  // fase 2 do ponto 0: Z descendo
  TEST_ASSERT_EQUAL_INT(SEQ_FREAR, seq_pausar(&s));
  TEST_ASSERT_EQUAL_INT(SEQ_PAUSANDO, seq_estado(&s));
  TEST_ASSERT_TRUE(seq_ativo(&s));
  TEST_ASSERT_EQUAL_INT(SEQ_NADA, seq_atualizar(&s, true, 0));  // ainda freando
  TEST_ASSERT_EQUAL_INT(SEQ_NADA, parou(&s, 0));
  TEST_ASSERT_EQUAL_INT(SEQ_PAUSADO, seq_estado(&s));
  TEST_ASSERT_FALSE(seq_ativo(&s));
  TEST_ASSERT_EQUAL_INT(SEQ_NADA, parou(&s, 99999));  // pausado não anda sozinho
  // parou com o Z em 25 (a meio caminho de 40): continua descendo, X primeiro (já está lá)
  TEST_ASSERT_EQUAL_INT(SEQ_MOVER, seq_continuar(&s, 25.0f));
  TEST_ASSERT_EQUAL_PTR(&p.p[0], seq_alvo(&s));
  TEST_ASSERT_EQUAL_INT(SEQ_EIXO_X, seq_eixo(&s));
}

void test_continuar_com_z_abaixo_do_alvo_sobe_primeiro(void) {
  Programa p = tres();
  Seq s;
  seq_iniciar(&s, &p, 1, 0.0f);
  seq_pausar(&s);
  parou(&s, 0);
  // durante a pausa alguém desceu o Z para 70: o alvo (40) está acima
  seq_continuar(&s, 70.0f);
  TEST_ASSERT_EQUAL_INT(SEQ_EIXO_Z, seq_eixo(&s));
}

void test_pausar_esperando_refaz_a_espera(void) {
  Programa p = tres();
  Seq s;
  seq_iniciar(&s, &p, 1, 0.0f);
  parou(&s, 0);
  parou(&s, 0);  // -> ponto 1
  parou(&s, 0);
  parou(&s, 0);  // espera de 1000 ms
  TEST_ASSERT_EQUAL_INT(SEQ_NADA, seq_pausar(&s));
  TEST_ASSERT_EQUAL_INT(SEQ_PAUSADO, seq_estado(&s));
  TEST_ASSERT_EQUAL_INT(SEQ_MOVER, seq_continuar(&s, 0.0f));  // já está lá: fases nulas
  TEST_ASSERT_EQUAL_PTR(&p.p[1], seq_alvo(&s));
  parou(&s, 5000);
  parou(&s, 5000);  // chegou: espera de novo
  TEST_ASSERT_EQUAL_INT(SEQ_ESPERANDO, seq_estado(&s));
  TEST_ASSERT_EQUAL_INT(SEQ_NADA, parou(&s, 5999));
  TEST_ASSERT_EQUAL_INT(SEQ_MOVER, parou(&s, 6000));
}

void test_cancelar(void) {
  Programa p = tres();
  Seq s;
  seq_iniciar(&s, &p, 1, 0.0f);
  TEST_ASSERT_EQUAL_INT(SEQ_FREAR, seq_cancelar(&s));
  TEST_ASSERT_EQUAL_INT(SEQ_CANCELADO, seq_estado(&s));
  TEST_ASSERT_EQUAL_INT(SEQ_NADA, parou(&s, 0));
  TEST_ASSERT_EQUAL_INT(SEQ_NADA, seq_continuar(&s, 0.0f));
  TEST_ASSERT_EQUAL_INT(SEQ_CANCELADO, seq_estado(&s));

  seq_iniciar(&s, &p, 1, 0.0f);
  parou(&s, 0);
  parou(&s, 0);
  parou(&s, 0);
  parou(&s, 0);  // esperando: motores parados, não precisa frear
  TEST_ASSERT_EQUAL_INT(SEQ_NADA, seq_cancelar(&s));
  TEST_ASSERT_EQUAL_INT(SEQ_CANCELADO, seq_estado(&s));
}

void test_pausar_ou_continuar_fora_de_hora_nao_faz_nada(void) {
  Programa p = tres();
  Seq s;
  seq_iniciar(&s, &p, 1, 0.0f);
  TEST_ASSERT_EQUAL_INT(SEQ_NADA, seq_continuar(&s, 0.0f));  // não está pausado
  TEST_ASSERT_EQUAL_INT(SEQ_MOVENDO, seq_estado(&s));
  seq_cancelar(&s);
  TEST_ASSERT_EQUAL_INT(SEQ_NADA, seq_pausar(&s));
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_limites_do_curso_x);
  RUN_TEST(test_limites_do_curso_z);
  RUN_TEST(test_nan_fica_fora_do_curso);
  RUN_TEST(test_referencia_e_passo_zero);
  RUN_TEST(test_mm_para_passos_x);
  RUN_TEST(test_mm_para_passos_z);
  RUN_TEST(test_ida_e_volta_mm_passos);
  RUN_TEST(test_usa_a_calibracao);
  RUN_TEST(test_adicionar_e_remover);
  RUN_TEST(test_recusa_fora_do_curso_velocidade_e_espera);
  RUN_TEST(test_programa_cheio);
  RUN_TEST(test_validar_aponta_o_ponto_ruim);
  RUN_TEST(test_programa_vazio_termina_na_hora);
  RUN_TEST(test_descer_anda_x_primeiro);
  RUN_TEST(test_subir_anda_z_primeiro);
  RUN_TEST(test_z_parado_anda_x_primeiro);
  RUN_TEST(test_percorre_os_pontos_e_espera);
  RUN_TEST(test_espera_atravessa_a_virada_do_millis);
  RUN_TEST(test_repete_os_ciclos_e_o_z_do_fim_vale_no_proximo);
  RUN_TEST(test_ciclos_zero_repete_sem_fim);
  RUN_TEST(test_pausar_andando_freia_e_continua_no_mesmo_ponto);
  RUN_TEST(test_continuar_com_z_abaixo_do_alvo_sobe_primeiro);
  RUN_TEST(test_pausar_esperando_refaz_a_espera);
  RUN_TEST(test_cancelar);
  RUN_TEST(test_pausar_ou_continuar_fora_de_hora_nao_faz_nada);
  return UNITY_END();
}
