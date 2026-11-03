// =============================================================
//  Bancada de teste: motor X + encoder X
// =============================================================
//
// Comandos pelo Monitor Serial (115200):
//   f  -> uma volta para frente   (+2048 passos)
//   b  -> uma volta para trás     (-2048 passos)
//   l  -> solta as bobinas (motor esfria, perde a retenção)
//   r  -> zera motor e encoder
//
// Testador de bobinas (conferir os LEDs A..D da ULN2003):
//   1..4 -> liga SÓ aquela entrada (1 = IN1 ... 4 = IN4)
//   p    -> próxima fase da sequência (2 bobinas), sentido de 'f'
//   o    -> fase anterior, sentido de 'b'
//   l    -> apaga tudo
//   Cada comando imprime quais INs, GPIOs e LEDs devem estar acesos.
//
// Monitor do encoder:
//   e    -> liga/desliga: imprime A e B crus a cada mudança.
//           Gire devagar com a mão. Um clique deve passar por
//           4 estados, um bit mudando por vez: 11 01 00 10 11
//           (ou o contrário, girando para o outro lado).
//
// Checkpoint motor:
//   - 'f' dá uma volta e o encoder mostra ~+80
//   - 'b' volta e os dois voltam para ~0
//   - se o encoder contar negativo com 'f', o sentido está invertido

#include <Arduino.h>
#include "pins.h"
#include "encoder.h"
#include "stepper.h"

static const int32_t PASSOS_POR_VOLTA = 2048;  // 28BYJ-48, passo completo

Encoder encX;
Stepper motX;

static void imprimir() {
  Serial.printf("motor: %6ld   encoder: %5ld   erros: %lu\n",
                (long)stepper_position(&motX), (long)encoder_count(&encX),
                (unsigned long)encoder_errors(&encX));
}

// Fase do testador, separada da fase do motor. Começa em 3 para o
// primeiro 'p' mostrar a fase 0.
static uint8_t faseTeste = 3;

// Liga as bobinas pedidas e diz o que deve aparecer na placa.
static void testarBobinas(uint8_t coils) {
  stepper_test_coils(&motX, coils);
  Serial.print("ligado:");
  for (uint8_t i = 0; i < 4; i++) {
    if ((coils >> i) & 0x1) {
      Serial.printf("  IN%u (GPIO %u, LED %c)", i + 1, stepper_pin(&motX, i), 'A' + i);
    }
  }
  Serial.println();
}

static void testarFase(int8_t dir) {
  faseTeste = (faseTeste + dir) & 0x3;
  Serial.printf("fase %u -> ", faseTeste);
  testarBobinas(stepseq_coils(faseTeste));
}

// Lê os pinos por polling, por fora do driver: só serve para girar
// devagar com a mão e ver se os dois canais chegam limpos.
static bool monitorEncoder = false;
static uint8_t abAnterior = 0xFF;

static void monitorarEncoder() {
  uint8_t a = digitalRead(ENC_X_A);
  uint8_t b = digitalRead(ENC_X_B);
  uint8_t ab = (a << 1) | b;
  if (ab == abAnterior) return;
  abAnterior = ab;
  Serial.printf("A=%u B=%u   contagem: %ld   erros: %lu\n", a, b,
                (long)encoder_count(&encX), (unsigned long)encoder_errors(&encX));
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println("== Teste do motor X ==");
  Serial.println("f = volta p/ frente, b = volta p/ tras, l = solta, r = zera");
  Serial.println("teste de bobinas: 1..4 = so uma entrada, p/o = proxima/anterior fase");
  Serial.println("e = liga/desliga monitor do encoder (A e B crus)");

  encoder_begin(&encX, ENC_X_A, ENC_X_B, false);
  stepper_begin(&motX, MOT_X_IN1, MOT_X_IN2, MOT_X_IN3, MOT_X_IN4);
  imprimir();
}

void loop() {
  if (monitorEncoder) monitorarEncoder();
  if (!Serial.available()) return;

  char c = Serial.read();
  switch (c) {
    case 'f': stepper_move(&motX, PASSOS_POR_VOLTA); break;   // ~5 s parado aqui
    case 'b': stepper_move(&motX, -PASSOS_POR_VOLTA); break;
    case 'l': stepper_release(&motX); Serial.println("-- solto --"); break;
    case 'r':
      stepper_reset(&motX, 0);
      encoder_reset(&encX, 0);
      Serial.println("-- zerado --");
      break;
    case '1': case '2': case '3': case '4':
      testarBobinas(1 << (c - '1'));
      return;
    case 'p': testarFase(+1); return;
    case 'o': testarFase(-1); return;
    case 'e':
      monitorEncoder = !monitorEncoder;
      abAnterior = 0xFF;  // força imprimir o estado atual
      Serial.println(monitorEncoder ? "-- monitor do encoder ligado --"
                                    : "-- monitor do encoder desligado --");
      return;
    default: return;  // ignora Enter e outras teclas
  }
  imprimir();
}
