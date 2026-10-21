#pragma once
// =============================================================
//  Mapa de pinos do robô. TODA pinagem do projeto mora aqui.
// =============================================================
//
// Regras usadas na escolha:
//  - Encoders nos GPIOs 34 e 35: são "só entrada", então não roubam
//    pinos de saída, que vão faltar (3 motores x 4 bobinas = 12 saídas).
//    Eles NÃO têm pull-up interno: só funcionam com o módulo KY-040,
//    que já tem os resistores na placa.
//  - Evitados: strapping (0, 2, 5, 12, 15), flash (6-11) e 36/39,
//    que geram interrupções falsas quando o Wi-Fi está ligado
//    (errata do ESP32), e o Wi-Fi vai ser usado pelo Modbus.
//  - Encoder alimentado em 3,3 V. O ESP32 não tolera 5 V nas entradas.

#include <stdint.h>

// ---------- Encoders (A = CLK, B = DT no KY-040) ----------
constexpr uint8_t ENC_X_A = 18;
constexpr uint8_t ENC_X_B = 19;

constexpr uint8_t ENC_Y_A = 32;
constexpr uint8_t ENC_Y_B = 33;
constexpr uint8_t ENC_Z_A = 25;
constexpr uint8_t ENC_Z_B = 26;

// ---------- Motores (ULN2003 IN1..IN4) ----------
// Sem strapping/boot: não dão pulso no motor quando a placa liga.
constexpr uint8_t MOT_X_IN1 = 16;
constexpr uint8_t MOT_X_IN2 = 17;
constexpr uint8_t MOT_X_IN3 = 21;
constexpr uint8_t MOT_X_IN4 = 22;
// TODO: Y e Z. Atenção: com os encoders Y/Z nos pinos atuais, sobram
// poucas saídas livres para 8 bobinas + garra (rever na etapa 3 eixos).

// ---------- Garra (solenoide) ----------
// TODO: definir na etapa da garra
