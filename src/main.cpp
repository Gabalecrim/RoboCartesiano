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

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println("== Teste do motor X ==");
  Serial.println("f = volta p/ frente, b = volta p/ tras, l = solta, r = zera");

  encoder_begin(&encX, ENC_X_A, ENC_X_B, false);
  stepper_begin(&motX, MOT_X_IN1, MOT_X_IN2, MOT_X_IN3, MOT_X_IN4);
  imprimir();
}

void loop() {
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
    default: return;  // ignora Enter e outras teclas
  }
  imprimir();
}
