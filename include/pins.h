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
constexpr uint8_t ENC_X_A = 34;
constexpr uint8_t ENC_X_B = 35;

constexpr uint8_t ENC_Y_A = 33;
constexpr uint8_t ENC_Y_B = 36;
// Z em malha aberta (2026-11-14): sem encoder. Os GPIOs 25 e 26, antes
// reservados para ele, foram para o motor Z.

// ---------- Motores (ULN2003 IN1..IN4) ----------
// Sem strapping/boot: não dão pulso no motor quando a placa liga.
constexpr uint8_t MOT_X_IN1 = 18;
constexpr uint8_t MOT_X_IN2 = 19;
constexpr uint8_t MOT_X_IN3 = 21;
constexpr uint8_t MOT_X_IN4 = 22;
constexpr uint8_t MOT_Z_IN1 = 14;
constexpr uint8_t MOT_Z_IN2 = 25;
constexpr uint8_t MOT_Z_IN3 = 26;
constexpr uint8_t MOT_Z_IN4 = 27;
// O 14 (IN1 do Z) dá pulsos no boot: no máximo um tranco no Z ao ligar.
// Y (2026-11-15): as 4 saídas livres que sobravam. 16/17 são livres no
// ESP32-WROOM (DevKit); no WROVER eles vão para a PSRAM.
constexpr uint8_t MOT_Y_IN1 = 13;
constexpr uint8_t MOT_Y_IN2 = 16;
constexpr uint8_t MOT_Y_IN3 = 17;
constexpr uint8_t MOT_Y_IN4 = 23;
// TODO: Y. Saídas livres que sobram: 13, 16, 17, 23 (faltam 5: 4 bobinas do
// Y + garra). Livre só como entrada: 39.

// ---------- Fim de curso (homing) ----------
// As duas chaves: NA para 3V3 com pull-down EXTERNO de 10 kΩ (acionada = HIGH):
//   GND --[10 kΩ]--+-- GPIO
//                  +-- chave NA -- 3V3
// São lidas por amostragem com debounce (na interrupção do motor), não por
// interrupção de pino: isso rejeita os pulsos falsos da errata do 36/39.
//
// X: fim do curso do lado 'f' (lado em homingX.sentido, main.cpp).
constexpr uint8_t HOME_X = 32;
// Z: EMBAIXO, no fundo do curso (Z = 40 mm). O GPIO 4 é entrada E saída: gasta uma das
// saídas do Y/garra (o 39, só entrada, está livre). É de strapping, mas com o
// pull-down a placa liga normal.
constexpr uint8_t HOME_Z = 4;
// Y (2026-11-15): GPIO 39, só entrada e SEM pull interno -> o pull-down
// externo de 10 kΩ é obrigatório (mesma ligação das outras chaves). A errata
// do 36/39 é de interrupção falsa: aqui a chave é lida por amostragem.
constexpr uint8_t HOME_Y = 39;

// ---------- Alarme ----------
// LED azul da placa (DevKit). GPIO 2 é de strapping, mas só importa no
// boot; depois vale como saída. Um buzzer ativo (com transistor) pode ir
// no mesmo pino.
constexpr uint8_t ALARME_LED = 2;

// ---------- Garra (solenoide) ----------
// TODO: definir na etapa da garra
