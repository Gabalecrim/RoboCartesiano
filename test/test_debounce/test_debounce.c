// Testes do debounce (rodam no PC).
//
//   pio test -e native -f test_debounce
//
// Cada teste é um comportamento que o filtro PRECISA ter no robô.
// No fim há testes da CADEIA inteira (debounce + quadratura), do
// jeito que o encoder.c usa: uma amostra de A e B a cada 500 us.

#include <stdio.h>
#include <unity.h>
#include "debounce.h"
#include "quadrature.h"

// Passa uma sequência de amostras e confere o nível limpo depois de cada uma.
static void conferir(Debounce *d, const uint8_t *cru, const uint8_t *limpo, int n) {
  for (int i = 0; i < n; i++) {
    char msg[32];
    snprintf(msg, sizeof msg, "amostra %d", i);
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(limpo[i], debounce_update(d, cru[i]), msg);
  }
}

void setUp(void) {}
void tearDown(void) {}

// ---------- partida ----------

void test_init_define_nivel_e_estado(void) {
  Debounce d = {0};
  debounce_init(&d, 1, 2);
  TEST_ASSERT_EQUAL_UINT8(1, debounce_level(&d));
  TEST_ASSERT_EQUAL_INT(DEB_ESTAVEL, d.state);
  TEST_ASSERT_EQUAL_UINT8(2, d.required);
}

void test_required_zero_vira_um(void) {
  Debounce d = {0};
  debounce_init(&d, 0, 0);
  TEST_ASSERT_EQUAL_UINT8(1, d.required);
}

// ---------- comportamento ----------

void test_nivel_igual_nao_muda(void) {
  Debounce d = {0};
  debounce_init(&d, 0, 2);
  const uint8_t cru[] = {0, 0, 0, 0};
  const uint8_t limpo[] = {0, 0, 0, 0};
  conferir(&d, cru, limpo, 4);
  TEST_ASSERT_EQUAL_INT(DEB_ESTAVEL, d.state);
}

void test_primeira_amostra_diferente_entra_em_confirmando(void) {
  Debounce d = {0};
  debounce_init(&d, 0, 2);
  TEST_ASSERT_EQUAL_UINT8(0, debounce_update(&d, 1));  // ainda não aceitou
  TEST_ASSERT_EQUAL_INT(DEB_CONFIRMANDO, d.state);
  TEST_ASSERT_EQUAL_UINT8(1, d.count);
}

void test_subida_aceita_na_amostra_required(void) {
  Debounce d = {0};
  debounce_init(&d, 0, 2);
  const uint8_t cru[] = {1, 1, 1};
  const uint8_t limpo[] = {0, 1, 1};
  conferir(&d, cru, limpo, 3);
  TEST_ASSERT_EQUAL_INT(DEB_ESTAVEL, d.state);
}

void test_descida_aceita_na_amostra_required(void) {
  Debounce d = {0};
  debounce_init(&d, 1, 2);
  const uint8_t cru[] = {0, 0, 0};
  const uint8_t limpo[] = {1, 0, 0};
  conferir(&d, cru, limpo, 3);
}

void test_repique_de_uma_amostra_e_ignorado(void) {
  Debounce d = {0};
  debounce_init(&d, 0, 2);
  const uint8_t cru[] = {1, 0, 0};
  const uint8_t limpo[] = {0, 0, 0};
  conferir(&d, cru, limpo, 3);
  TEST_ASSERT_EQUAL_INT(DEB_ESTAVEL, d.state);
}

void test_repique_zera_a_contagem(void) {
  // required = 3: 1 1 (quase) 0 (repique) 1 1 -> ainda não; 1 -> agora
  Debounce d = {0};
  debounce_init(&d, 0, 3);
  const uint8_t cru[] = {1, 1, 0, 1, 1, 1};
  const uint8_t limpo[] = {0, 0, 0, 0, 0, 1};
  conferir(&d, cru, limpo, 6);
}

void test_required_maior_espera_mais(void) {
  Debounce d = {0};
  debounce_init(&d, 0, 4);
  const uint8_t cru[] = {1, 1, 1, 1};
  const uint8_t limpo[] = {0, 0, 0, 1};
  conferir(&d, cru, limpo, 4);
}

void test_required_um_nao_filtra(void) {
  Debounce d = {0};
  debounce_init(&d, 0, 1);
  const uint8_t cru[] = {1, 0, 1, 1, 0};
  const uint8_t limpo[] = {1, 0, 1, 1, 0};
  conferir(&d, cru, limpo, 5);
}

void test_contato_tremendo_na_borda(void) {
  // Exemplo do debounce.h: repique, depois o nível firma.
  Debounce d = {0};
  debounce_init(&d, 0, 2);
  const uint8_t cru[] = {0, 0, 1, 0, 1, 1, 1, 1, 0, 1, 1};
  const uint8_t limpo[] = {0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1};
  conferir(&d, cru, limpo, 11);
}

void test_qualquer_valor_diferente_de_zero_e_um(void) {
  // digitalRead devolve 0/1, mas a função não pode depender disso.
  Debounce d = {0};
  debounce_init(&d, 0, 2);
  debounce_update(&d, 5);
  TEST_ASSERT_EQUAL_UINT8(1, debounce_update(&d, 200));
}

// ---------- cadeia: debounce + quadratura ----------
// Simula o que o encoder.c faz a cada amostra.

typedef struct {
  Debounce a, b;
  Quadrature q;
} Cadeia;

static void cadeia_init(Cadeia *c, uint8_t ab, uint8_t required) {
  debounce_init(&c->a, (ab >> 1) & 1, required);
  debounce_init(&c->b, ab & 1, required);
  Quadrature zero = {0};
  c->q = zero;
  quadrature_reset(&c->q, ab, 0);
}

static void cadeia_amostra(Cadeia *c, uint8_t ab_cru) {
  uint8_t a = debounce_update(&c->a, (ab_cru >> 1) & 1);
  uint8_t b = debounce_update(&c->b, ab_cru & 1);
  quadrature_update(&c->q, (uint8_t)((a << 1) | b));
}

void test_cadeia_ciclo_limpo_conta_4(void) {
  // 00 -> 01 -> 11 -> 10 -> 00, cada estado por 4 amostras (2 ms)
  Cadeia c;
  cadeia_init(&c, 0b00, 2);
  const uint8_t seq[] = {0b01, 0b11, 0b10, 0b00};
  for (int s = 0; s < 4; s++)
    for (int k = 0; k < 4; k++) cadeia_amostra(&c, seq[s]);
  TEST_ASSERT_EQUAL_INT32(4, c.q.count);
  TEST_ASSERT_EQUAL_UINT32(0, c.q.errors);
}

void test_cadeia_repique_nos_dois_canais_nao_gera_erro(void) {
  // Sem filtro, o salto 00 -> 11 seria uma transição inválida.
  // Com required = 2, o repique de 1 amostra some antes da quadratura.
  Cadeia c;
  cadeia_init(&c, 0b00, 2);
  cadeia_amostra(&c, 0b11);  // ruído nos dois canais
  cadeia_amostra(&c, 0b00);
  cadeia_amostra(&c, 0b00);
  TEST_ASSERT_EQUAL_INT32(0, c.q.count);
  TEST_ASSERT_EQUAL_UINT32(0, c.q.errors);
}

void test_cadeia_bordas_juntas_demais_ainda_sao_invalidas(void) {
  // Validação: se A e B firmam na MESMA amostra, a quadratura não
  // sabe o sentido. O debounce não inventa a ordem: conta como erro.
  Cadeia c;
  cadeia_init(&c, 0b00, 2);
  cadeia_amostra(&c, 0b11);
  cadeia_amostra(&c, 0b11);
  TEST_ASSERT_EQUAL_INT32(0, c.q.count);
  TEST_ASSERT_EQUAL_UINT32(1, c.q.errors);
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_init_define_nivel_e_estado);
  RUN_TEST(test_required_zero_vira_um);
  RUN_TEST(test_nivel_igual_nao_muda);
  RUN_TEST(test_primeira_amostra_diferente_entra_em_confirmando);
  RUN_TEST(test_subida_aceita_na_amostra_required);
  RUN_TEST(test_descida_aceita_na_amostra_required);
  RUN_TEST(test_repique_de_uma_amostra_e_ignorado);
  RUN_TEST(test_repique_zera_a_contagem);
  RUN_TEST(test_required_maior_espera_mais);
  RUN_TEST(test_required_um_nao_filtra);
  RUN_TEST(test_contato_tremendo_na_borda);
  RUN_TEST(test_qualquer_valor_diferente_de_zero_e_um);
  RUN_TEST(test_cadeia_ciclo_limpo_conta_4);
  RUN_TEST(test_cadeia_repique_nos_dois_canais_nao_gera_erro);
  RUN_TEST(test_cadeia_bordas_juntas_demais_ainda_sao_invalidas);
  return UNITY_END();
}
