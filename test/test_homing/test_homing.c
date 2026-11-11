// Testes do homing e da estatística da calibração (rodam no PC).
//
//   pio test -e native -f test_homing
//
// Um eixo simulado substitui o hardware: posição em passos, chave de
// fim de curso com histerese, encoder de 80 contagens por 2048 passos
// e, se quiser, um batente que impede o eixo de andar.

#include <limits.h>
#include <stdbool.h>
#include <unity.h>
#include "calib.h"
#include "homing.h"

// ---------- eixo simulado ----------

typedef struct {
  int32_t pos;          // posição real, em passos
  int32_t chave_em;     // a chave aciona quando pos >= chave_em
  int32_t histerese;    // solta só quando pos < chave_em - histerese
  bool chave;           // estado atual da chave
  bool chave_quebra;    // depois do 1º acionamento a chave para de funcionar
  int acionamentos;
  int32_t batente;      // pos não passa daqui no sentido + (INT32_MAX = sem batente)
  bool encoder_parado;  // encoder desligado: sempre 0
  int32_t min_pos;      // menor posição alcançada
  bool usou_lento;      // a FSM pediu velocidade lenta em algum passo
  bool lento_fora_da_busca_lenta;
} Sim;

static void sim_init(Sim *s, int32_t pos) {
  s->pos = pos;
  s->chave_em = 0;
  s->histerese = 30;
  s->chave = false;
  s->chave_quebra = false;
  s->acionamentos = 0;
  s->batente = INT32_MAX;
  s->encoder_parado = false;
  s->min_pos = pos;
  s->usou_lento = false;
  s->lento_fora_da_busca_lenta = false;
}

static void sim_atualiza_chave(Sim *s) {
  bool quebrou = s->chave_quebra && s->acionamentos > 0;
  if (!s->chave && !quebrou && s->pos >= s->chave_em) {
    s->chave = true;
    s->acionamentos++;
  } else if (s->chave && s->pos < s->chave_em - s->histerese) {
    s->chave = false;
  }
}

static int32_t sim_enc(const Sim *s) {
  if (s->encoder_parado) return 0;
  int64_t v = (int64_t)s->pos * 80;  // divisão com arredondamento para baixo
  return (int32_t)(v >= 0 ? v / 2048 : -((-v + 2047) / 2048));
}

static void sim_passo(Sim *s, int8_t dir) {
  int32_t nova = s->pos + dir;
  if (dir > 0 && nova > s->batente) nova = s->batente;  // bateu: não anda
  s->pos = nova;
  if (s->pos < s->min_pos) s->min_pos = s->pos;
  sim_atualiza_chave(s);
}

static const HomingConfig CFG = {
    .curso_max = 30 * 2048,
    .recuo = 512,
    .recuo_max = 4 * 2048,
    .janela_trava = 256,
};

// Roda o homing até parar; devolve o estado final.
static HomingState rodar(Homing *h, Sim *s, const HomingConfig *cfg, long *passos) {
  sim_atualiza_chave(s);
  homing_start(h, cfg, s->chave, sim_enc(s));
  long n = 0;
  for (; n < 2000000; n++) {
    int8_t d = homing_update(h, s->chave, sim_enc(s));
    if (d == 0) break;
    bool lento = homing_lento(h);
    s->usou_lento |= lento;
    if (lento && homing_state(h) != HOM_BUSCA_LENTA) s->lento_fora_da_busca_lenta = true;
    sim_passo(s, d);
  }
  if (passos) *passos = n;
  return homing_state(h);
}

void setUp(void) {}
void tearDown(void) {}

// ---------- caminho feliz ----------

void test_acha_a_origem_partindo_de_longe(void) {
  Homing h = {0};
  Sim s;
  sim_init(&s, -5000);
  TEST_ASSERT_EQUAL_INT(HOM_OK, rodar(&h, &s, &CFG, NULL));
  TEST_ASSERT_EQUAL_INT(HOM_ERRO_NENHUM, homing_error(&h));
  TEST_ASSERT_EQUAL_INT32(0, s.pos);  // parou exatamente onde a chave aciona
}

void test_origem_repete_de_qualquer_ponto(void) {
  const int32_t partidas[] = {-20480, -5000, -123, -1};
  for (unsigned i = 0; i < sizeof partidas / sizeof partidas[0]; i++) {
    Homing h = {0};
    Sim s;
    sim_init(&s, partidas[i]);
    TEST_ASSERT_EQUAL_INT(HOM_OK, rodar(&h, &s, &CFG, NULL));
    TEST_ASSERT_EQUAL_INT32(0, s.pos);
  }
}

void test_comeca_com_a_chave_acionada(void) {
  Homing h = {0};
  Sim s;
  sim_init(&s, 10);  // já em cima da chave
  sim_atualiza_chave(&s);
  homing_start(&h, &CFG, s.chave, sim_enc(&s));
  TEST_ASSERT_EQUAL_INT(HOM_RECUO, homing_state(&h));
  TEST_ASSERT_EQUAL_INT(HOM_OK, rodar(&h, &s, &CFG, NULL));
  TEST_ASSERT_EQUAL_INT32(0, s.pos);
}

void test_recuo_passa_da_histerese_e_mais_recuo(void) {
  // Solta em pos = -histerese - 1 e ainda recua 'recuo' passos.
  Homing h = {0};
  Sim s;
  sim_init(&s, 0);
  rodar(&h, &s, &CFG, NULL);
  TEST_ASSERT_EQUAL_INT32(-s.histerese - 1 - CFG.recuo, s.min_pos);
}

void test_velocidade_lenta_so_na_busca_lenta(void) {
  Homing h = {0};
  Sim s;
  sim_init(&s, -5000);
  rodar(&h, &s, &CFG, NULL);
  TEST_ASSERT_TRUE(s.usou_lento);
  TEST_ASSERT_FALSE(s.lento_fora_da_busca_lenta);
}

// ---------- erros ----------

void test_erro_curso_sem_chave(void) {
  Homing h = {0};
  Sim s;
  sim_init(&s, 0);
  s.chave_em = INT32_MAX;  // chave inexistente
  HomingConfig cfg = CFG;
  cfg.curso_max = 1000;
  long passos;
  TEST_ASSERT_EQUAL_INT(HOM_ERRO, rodar(&h, &s, &cfg, &passos));
  TEST_ASSERT_EQUAL_INT(HOM_ERRO_CURSO, homing_error(&h));
  TEST_ASSERT_EQUAL_INT32(1000, passos);
}

void test_erro_travou_no_batente_antes_da_chave(void) {
  Homing h = {0};
  Sim s;
  sim_init(&s, -5000);
  s.batente = -100;  // o eixo bate antes de chegar na chave
  TEST_ASSERT_EQUAL_INT(HOM_ERRO, rodar(&h, &s, &CFG, NULL));
  TEST_ASSERT_EQUAL_INT(HOM_ERRO_TRAVOU, homing_error(&h));
  TEST_ASSERT_EQUAL_INT32(-100, s.pos);
}

void test_travou_para_logo_depois_da_janela(void) {
  // Do batente até o erro: no máximo a janela + os passos de 1 contagem.
  Homing h = {0};
  Sim s;
  sim_init(&s, -2048);
  s.batente = -1000;
  long passos;
  rodar(&h, &s, &CFG, &passos);
  long ate_batente = -1000 - (-2048);
  TEST_ASSERT_LESS_OR_EQUAL(ate_batente + CFG.janela_trava + 26, passos);
}

void test_erro_encoder_parado(void) {
  Homing h = {0};
  Sim s;
  sim_init(&s, -5000);
  s.encoder_parado = true;
  TEST_ASSERT_EQUAL_INT(HOM_ERRO, rodar(&h, &s, &CFG, NULL));
  TEST_ASSERT_EQUAL_INT(HOM_ERRO_TRAVOU, homing_error(&h));
}

void test_janela_zero_desliga_a_vigia(void) {
  Homing h = {0};
  Sim s;
  sim_init(&s, -5000);
  s.encoder_parado = true;
  HomingConfig cfg = CFG;
  cfg.janela_trava = 0;
  TEST_ASSERT_EQUAL_INT(HOM_OK, rodar(&h, &s, &cfg, NULL));
}

void test_erro_chave_presa(void) {
  Homing h = {0};
  Sim s;
  sim_init(&s, 0);
  s.histerese = INT32_MAX / 2;  // nunca solta
  long passos;
  TEST_ASSERT_EQUAL_INT(HOM_ERRO, rodar(&h, &s, &CFG, &passos));
  TEST_ASSERT_EQUAL_INT(HOM_ERRO_CHAVE_PRESA, homing_error(&h));
  TEST_ASSERT_EQUAL_INT32(CFG.recuo_max, passos);
}

void test_erro_nao_achou_na_busca_lenta(void) {
  Homing h = {0};
  Sim s;
  sim_init(&s, -5000);
  s.chave_quebra = true;  // aciona na busca rápida e nunca mais
  TEST_ASSERT_EQUAL_INT(HOM_ERRO, rodar(&h, &s, &CFG, NULL));
  TEST_ASSERT_EQUAL_INT(HOM_ERRO_NAO_ACHOU, homing_error(&h));
}

// ---------- estados finais ----------

void test_sem_start_nao_anda(void) {
  Homing h = {0};
  TEST_ASSERT_EQUAL_INT8(0, homing_update(&h, false, 0));
  TEST_ASSERT_EQUAL_INT(HOM_PARADO, homing_state(&h));
}

void test_depois_de_ok_fica_parado(void) {
  Homing h = {0};
  Sim s;
  sim_init(&s, -300);
  rodar(&h, &s, &CFG, NULL);
  TEST_ASSERT_EQUAL_INT8(0, homing_update(&h, false, 123));
  TEST_ASSERT_EQUAL_INT8(0, homing_update(&h, true, 456));
  TEST_ASSERT_EQUAL_INT(HOM_OK, homing_state(&h));
}

// ---------- estatística da calibração ----------

void test_estat_vazia(void) {
  Estat e;
  estat_reset(&e);
  TEST_ASSERT_EQUAL_UINT32(0, e.n);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, estat_media(&e));
  TEST_ASSERT_EQUAL_INT32(0, estat_faixa(&e));
}

void test_estat_min_max_media(void) {
  Estat e;
  estat_reset(&e);
  estat_add(&e, 3);
  estat_add(&e, -2);
  estat_add(&e, 5);
  TEST_ASSERT_EQUAL_UINT32(3, e.n);
  TEST_ASSERT_EQUAL_INT32(-2, e.min);
  TEST_ASSERT_EQUAL_INT32(5, e.max);
  TEST_ASSERT_EQUAL_INT32(7, estat_faixa(&e));
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 2.0f, estat_media(&e));
}

void test_estat_so_negativos(void) {
  // O primeiro valor define min e max (não pode ficar preso no 0 do reset).
  Estat e;
  estat_reset(&e);
  estat_add(&e, -800);
  estat_add(&e, -790);
  TEST_ASSERT_EQUAL_INT32(-800, e.min);
  TEST_ASSERT_EQUAL_INT32(-790, e.max);
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_acha_a_origem_partindo_de_longe);
  RUN_TEST(test_origem_repete_de_qualquer_ponto);
  RUN_TEST(test_comeca_com_a_chave_acionada);
  RUN_TEST(test_recuo_passa_da_histerese_e_mais_recuo);
  RUN_TEST(test_velocidade_lenta_so_na_busca_lenta);
  RUN_TEST(test_erro_curso_sem_chave);
  RUN_TEST(test_erro_travou_no_batente_antes_da_chave);
  RUN_TEST(test_travou_para_logo_depois_da_janela);
  RUN_TEST(test_erro_encoder_parado);
  RUN_TEST(test_janela_zero_desliga_a_vigia);
  RUN_TEST(test_erro_chave_presa);
  RUN_TEST(test_erro_nao_achou_na_busca_lenta);
  RUN_TEST(test_sem_start_nao_anda);
  RUN_TEST(test_depois_de_ok_fica_parado);
  RUN_TEST(test_estat_vazia);
  RUN_TEST(test_estat_min_max_media);
  RUN_TEST(test_estat_so_negativos);
  return UNITY_END();
}
