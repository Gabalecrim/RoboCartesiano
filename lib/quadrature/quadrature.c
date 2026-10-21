#include "quadrature.h"

// =============================================================
//  EXERCÍCIO 1: tabela de transição
// =============================================================
//
// O índice da tabela junta os dois estados num número de 4 bits:
//
//      indice = (prev << 2) | curr        -> vai de 0 a 15
//
// Exemplo: prev = 01, curr = 11  ->  indice = 0111 = 7  -> +1
//
// Para cada uma das 16 combinações, decidam:
//   - mesmo estado (prev == curr) ............. 0
//   - um passo no sentido positivo ............ +1
//   - um passo no sentido negativo ............ -1
//   - os dois bits mudaram juntos ............. QUAD_INVALID
//
// Dica: desenhem as 4 posições num círculo (00, 01, 11, 10) e
// vejam qual seta vai para cada lado.
//
// Os testes em test/test_quadrature verificam a tabela.
// Enquanto ela estiver zerada, eles falham. Esse é o ponto de partida.

int8_t quadratureDelta(uint8_t prev, uint8_t curr) {
  static const int8_t TABLE[16] = {
      //  curr: 00  01  10  11
      0, -1, 1, 0,  // prev = 00   TODO
      1, 0, 0, -1,  // prev = 01   TODO
      -1, 0, 0, 1,  // prev = 10   TODO
      0, 1, -1, 0,  // prev = 11   TODO
  };
  return TABLE[((prev & 0x3) << 2) | (curr & 0x3)];
}

void quadrature_reset(Quadrature *q, uint8_t ab, int32_t value) {
  q->prev = ab & 0x3;
  q->count = value;
}

// =============================================================
//  EXERCÍCIO 2: atualizar a contagem
// =============================================================
//
// 1. Consultar quadratureDelta(prev, ab).
// 2. Se for QUAD_INVALID: incrementar errors e NÃO mexer em count.
//    Senão: somar o resultado em count.
// 3. Guardar ab como novo prev (nos dois casos, para o decodificador
//    se "ressincronizar" depois de um erro).

void quadrature_update(Quadrature *q, uint8_t ab) {
  ab &= 0x3;
  int8_t delta = quadratureDelta(q->prev, ab);
  if (delta == QUAD_INVALID) {
    q->errors++;
  } else {
    q->count += delta;
  }
  q->prev = ab;
}
