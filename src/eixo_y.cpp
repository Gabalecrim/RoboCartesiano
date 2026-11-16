// Eixo Y da bancada: ver eixo_y.h. Tudo o que o X tem, com comandos "Y...".
//
// Movimento (posição SÓ pelo encoder do Y):
//   Y         -> mostra posição, alvo, velocidade, calibração e ajustes
//   YH        -> homing do Y (chave HOME_Y): Y = Y_HOME_MM na chave
//   Y120      -> vai para Y = 120 mm (precisa de YH ou YR)
//   Y+10 / Y-1 -> anda / volta em mm pelo encoder (vale sem homing)
//   YR        -> "aqui é Y = Y_HOME_MM" (referência manual, sem a chave)
//   YZ        -> zera motor e encoder aqui (perde a referência)
//   YQ        -> freia;  YL -> para na hora e solta as bobinas
//   YF1 / YB0.5 -> voltas do motor (diagnóstico: sentido e bobinas)
// Velocidade:
//   YV1.5     -> setpoint em mm/s;  YA1000 aceleração;  YN300 partida (passos/s)
//   YG        -> governo: YGQ2 quedas, YGA50 fator de aceleração %, YGD70 fator
//                de desaceleração %, YGS1000 mantém (ms), YG0 padrões
//   YC        -> mostra a detecção;  YC0 / YC1 desliga / liga (governo, encoder
//                sem sinal e travamento do homing)
// Homing:
//   YHV500 rápida, YHL150 lenta, YHT256 travamento (0 desliga), YHS+1 / YHS-1
//   sentido, YHD diagnóstico (anda 2 mm e confere encoder e chave), YS chave
// Calibração:
//   YD100     -> calibração de distância (homing, afasta 2 mm, espera 5 s, anda)
//   YM99.6    -> medida real;  YK mostra;  YK0 volta ao nominal
//   YCAL3     -> auto calibração do homing (3 ciclos de ida e volta)
// Testes:
//   YI1..YI4  -> liga só aquela entrada do ULN2003;  YIP / YIO próxima / anterior
//                fase;  YI0 fim;  YI mostra os pinos
//   YE        -> monitor do encoder (A e B crus);  YTV1 tempo com encoder ligado x parado
//   YED / YED2 -> diagnóstico do encoder: 1 (ou 2) volta(s) para cada lado, A e B lidos
//                direto: canal morto, eixo patinando (vai-e-volta) ou ruído
//   YT        -> teste do Y (pontos e repetição da chave);  YW / YW800 varredura de
//                velocidade;  YP30 curva da rampa (CSV)
//
// Telemetria (sempre: andando 10/s, parado 4/s):
//   @Y <ms> <Y mm pelo encoder> <encoder cont> <vy passos/s> <vy mm/s> <rotação permitida>
//      <ref 0/1> <real passos/s> <setpoint passos/s> <alvo mm> <alvo cont> <m/c/f/->
//      <ok/SEMSINAL>          ("-" = sem alvo)

#include "eixo_y.h"

#include <Preferences.h>
#include "calib.h"
#include "eixo.h"
#include "encoder.h"
#include "governo.h"
#include "malha.h"
#include "pins.h"
#include "rampa.h"
#include "stepper.h"
#include "stepseq.h"

// ---------- mecânica do Y (A CONFERIR na bancada: igual ao X até medir) ----------
static const int32_t Y_PASSOS_VOLTA = 2048;       // 28BYJ-48, passo completo
static const float Y_FUSO_MM = 8.0f;              // TR8x8: 8 mm por volta
static const float Y_PASSOS_MM_NOM = Y_PASSOS_VOLTA / Y_FUSO_MM;  // 256
static const int32_t Y_CONT_VOLTA = 80;           // KY-040: 20 ciclos x 4
static const float Y_CONT_MM_NOM = Y_CONT_VOLTA / Y_FUSO_MM;      // 10
static const float CURSO_Y_MM = 190.0f;           // curso a partir da chave
static const float Y_HOME_MM = 0.0f;              // Y na chave do homing

// ---------- velocidades (mesmos limites do X: 500 passos/s, 1000 passos/s²) ----------
static const uint32_t Y_VEL_LIMITE = 500;
static const uint32_t Y_ACEL_LIMITE = 1000;
static const uint32_t Y_PARTIDA = 300;

// ---------- malha pelo encoder (igual ao X) ----------
static const int32_t Y_MALHA_TOL_CONT = 1;      // 0,1 mm
static const uint8_t Y_MALHA_CORRECOES = 4;
static const uint32_t Y_MALHA_ASSENTAR_MS = 30;

// ---------- governo (igual ao X; os ajustáveis ficam salvos em "govY") ----------
static const uint32_t Y_GOV_AMOSTRA_MS = 50;
static const uint32_t Y_GOV_JANELA_MS = 500;
static const int32_t Y_GOV_MIN_CONT = 5;
static const uint8_t Y_GOV_QUEDA_PCT = 75;
static const uint32_t Y_GOV_VEL_MIN = 120;
static const uint32_t Y_GOV_INTERVALO_MS = 1000;
static const uint32_t Y_GOV_SUBIDA_MIN = 25;
static const uint8_t Y_GOV_QUEDAS_PADRAO = 2;
static const uint8_t Y_GOV_SUBIDA_PCT_PADRAO = 50;
static const uint8_t Y_GOV_REDUZIR_PCT_PADRAO = 70;
static const uint32_t Y_GOV_SEGURAR_MS_MIN = 1000;

static const float Y_SEM_SINAL_MM = 5.0f;     // motor andou isso sem o encoder mudar
static const uint32_t Y_TELEM_MS = 100;       // andando
static const uint32_t Y_TELEM_PARADO_MS = 250;

// ---------- testes e calibração (iguais aos do X) ----------
static const float Y_TESTE_PONTOS[] = {40.0f, 150.0f, 95.0f, 20.0f};  // mm (YT)
static const int32_t Y_TESTE_ORIGEM_TOL_CONT = 2;
static const float Y_DIST_MM_PADRAO = 100.0f;
static const float Y_DIST_PRE_MM = 2.0f;
static const uint32_t Y_DIST_ESPERA_MS = 5000;
static const float Y_DIST_TOLERANCIA_PCT = 10.0f;
static const int32_t Y_CAL_VOLTAS = 10;
static const uint32_t Y_CAL_CICLOS_MAX = 50;
static const float Y_HD_MM = 2.0f;
static const float Y_VAI_A_MM = 40.0f;  // vaivém da varredura e da curva
static const float Y_VAI_B_MM = 70.0f;
static const uint32_t Y_VARRE_VELS[] = {500, 550, 600, 650, 700, 750, 800};
static const int32_t Y_VARRE_TOL_CONT = 5;
static const float Y_VARRE_TOL_PCT = 2.0f;

// ---------- estado ----------
static Stepper motY;
static Encoder encY;
static Eixo eixoY;
static EixoHomingCfg homY;
static Malha malhaY;
static Gov govY;
static Preferences prefsY;

static float yPassosMm = Y_PASSOS_MM_NOM;  // calibração (YD/YM), salva em "calibY"
static float yContMm = Y_CONT_MM_NOM;
static int8_t ySentido = +1;               // giro que vai para a chave: +1 = f (YHS)
static int32_t yTravaPassos = 0;           // janela de travamento do homing (YHT; 0 = desligado)
static bool yDeteccao = true;              // YC0 / YC1

static bool yRef = false;             // posição em mm vale (YH ou YR)
static uint32_t yVelocidade = Y_VEL_LIMITE;
static bool yAndando = false;         // para avisar quando um movimento termina
static uint32_t yParadoDesde = 0;
static bool yMonitor = false;
static uint8_t yAbAnterior = 0xFF;
static bool yChaveAnterior = false;
static uint32_t yChaveUltima = 0;
static uint8_t yFaseTeste = 3;        // testador de bobinas (YIP mostra a fase 0 primeiro)

static uint8_t yGovQuedas = Y_GOV_QUEDAS_PADRAO;
static uint8_t yGovSubidaPct = Y_GOV_SUBIDA_PCT_PADRAO;
static uint8_t yGovReduzirPct = Y_GOV_REDUZIR_PCT_PADRAO;
static uint32_t yGovSegurarMs = Y_GOV_SEGURAR_MS_MIN;
static uint32_t yGovUltimoMs = 0;
static uint32_t yGovAplicado = 0;
static bool yGovSuspenso = false;     // varredura YW: sem limite

static bool ySemSinal = false;
static int32_t yEncUltima = 0;
static int32_t yMotorRef = 0;
static uint32_t yPassosSemMudar = 0;

static uint32_t yTelemUltimo = 0;

// Calibração de distância pendente (YD -> YM).
static bool yMedidaPendente = false;
static int32_t yPendPassos = 0, yPendContagens = 0;
static float yPendMm = 0.0f;

// ---------- conversões (posição do Y = só o encoder) ----------
static float yContParaMm(int32_t c) { return Y_HOME_MM + (float)(-ySentido * c) / yContMm; }
static int32_t yMmParaCont(float mm) { return (int32_t)lroundf(-ySentido * (mm - Y_HOME_MM) * yContMm); }
static float yMm() { return yContParaMm(encoder_count(&encY)); }
static bool yDentro(float mm) { return mm >= Y_HOME_MM - 0.001f && mm <= Y_HOME_MM + CURSO_Y_MM + 0.001f; }
static float yErroMm(int32_t cont) { return -ySentido * cont / yContMm; }
static int32_t absi(int32_t v) { return v < 0 ? -v : v; }
static char giro(int8_t g) { return g > 0 ? 'f' : (g < 0 ? 'b' : '-'); }

static float lerNumero(String s) {
  s.trim();
  s.replace(',', '.');
  return s.toFloat();
}

bool yOcupado() { return stepper_busy(&motY) || malha_ativa(&malhaY); }

// ---------- calibração salva ----------
static void yCalibCarregar() {
  prefsY.begin("calibY", true);
  float p = prefsY.getFloat("ppmm", Y_PASSOS_MM_NOM);
  float c = prefsY.getFloat("cpmm", Y_CONT_MM_NOM);
  prefsY.end();
  yPassosMm = p > 0.0f ? p : Y_PASSOS_MM_NOM;
  yContMm = c > 0.0f ? c : Y_CONT_MM_NOM;
}

static void yCalibSalvar() {
  prefsY.begin("calibY", false);
  prefsY.putFloat("ppmm", yPassosMm);
  prefsY.putFloat("cpmm", yContMm);
  prefsY.end();
  malha_escala(&malhaY, yPassosMm / yContMm);
  gov_escala(&govY, yContMm / yPassosMm);
}

static void yCalibMostrar() {
  Serial.printf("calibracao Y: %.3f passos/mm (nominal %.1f), %.3f contagens/mm (nominal %.1f)\n",
                yPassosMm, Y_PASSOS_MM_NOM, yContMm, Y_CONT_MM_NOM);
}

// ---------- governo ----------
static GovCfg yGovCfg() {
  GovCfg c = {Y_VEL_LIMITE,    Y_GOV_VEL_MIN,  Y_GOV_QUEDA_PCT,    yGovReduzirPct,
              Y_GOV_JANELA_MS, Y_GOV_MIN_CONT, Y_GOV_INTERVALO_MS, Y_GOV_SUBIDA_MIN,
              yGovQuedas,      yGovSubidaPct,  yGovSegurarMs};
  return c;
}

static void yGovCarregar() {
  prefsY.begin("govY", true);
  yGovQuedas = prefsY.getUChar("quedas", Y_GOV_QUEDAS_PADRAO);
  yGovSubidaPct = prefsY.getUChar("subida", Y_GOV_SUBIDA_PCT_PADRAO);
  yGovReduzirPct = prefsY.getUChar("reduzir", Y_GOV_REDUZIR_PCT_PADRAO);
  yGovSegurarMs = prefsY.getULong("segurar", Y_GOV_SEGURAR_MS_MIN);
  prefsY.end();
  if (yGovQuedas < 1 || yGovQuedas > 20) yGovQuedas = Y_GOV_QUEDAS_PADRAO;
  if (yGovSubidaPct < 5 || yGovSubidaPct > 100) yGovSubidaPct = Y_GOV_SUBIDA_PCT_PADRAO;
  if (yGovReduzirPct < 10 || yGovReduzirPct > 95) yGovReduzirPct = Y_GOV_REDUZIR_PCT_PADRAO;
  if (yGovSegurarMs < Y_GOV_SEGURAR_MS_MIN || yGovSegurarMs > 30000) yGovSegurarMs = Y_GOV_SEGURAR_MS_MIN;
}

static void yGovSalvar() {
  prefsY.begin("govY", false);
  prefsY.putUChar("quedas", yGovQuedas);
  prefsY.putUChar("subida", yGovSubidaPct);
  prefsY.putUChar("reduzir", yGovReduzirPct);
  prefsY.putULong("segurar", yGovSegurarMs);
  prefsY.end();
}

static void yGovMostrar() {
  Serial.printf("governo Y: quedas para reduzir %u, fator de aceleracao %u%%, fator de desaceleracao"
                " %u%%, mantem %lu ms depois de subir\n",
                yGovQuedas, yGovSubidaPct, yGovReduzirPct, (unsigned long)yGovSegurarMs);
}

static void yGovAplicar() {
  if (yGovSuspenso) return;
  uint32_t l = gov_limite(&govY);
  if (l == yGovAplicado) return;
  stepper_set_limit(&motY, l);
  yGovAplicado = l;
}

// Encoder do Y sem sinal: o motor andou Y_SEM_SINAL_MM e o encoder não mudou.
static void yVigiarSinal() {
  int32_t e = encoder_count(&encY);
  int32_t m = stepper_position(&motY);
  if (e != yEncUltima) {
    yEncUltima = e;
    yMotorRef = m;
    yPassosSemMudar = 0;
    if (ySemSinal) {
      ySemSinal = false;
      Serial.println("[encoder Y] voltou a contar: faca o homing (YH) para a posicao valer de novo");
    }
    return;
  }
  yPassosSemMudar += (uint32_t)absi(m - yMotorRef);
  yMotorRef = m;
  if (!yDeteccao || ySemSinal || yPassosSemMudar < (uint32_t)(Y_SEM_SINAL_MM * yPassosMm)) return;
  ySemSinal = true;
  malha_cancelar(&malhaY);
  yRef = false;
  GovCfg c = yGovCfg();
  gov_init(&govY, &c, yContMm / yPassosMm);
  yGovAplicar();
  Serial.printf("[encoder Y] SEM SINAL: o motor andou %.1f mm e o encoder nao mudou nenhuma contagem"
                " (fica em %ld). Posicao do Y perdida. Confira o cabo do encoder (GPIO %u/%u), o 3V3"
                " e o acoplamento; teste: YE + girar o fuso a mao\n",
                yPassosSemMudar / yPassosMm, (long)e, ENC_Y_A, ENC_Y_B);
}

// A cada Y_GOV_AMOSTRA_MS: mede motor x encoder, baixa ou recupera a rotação.
static void yGovernar() {
  uint32_t agora = millis();
  if (agora - yGovUltimoMs < Y_GOV_AMOSTRA_MS) return;
  yGovUltimoMs = agora;
  yVigiarSinal();
  StepperPerfil p;
  stepper_get_profile(&motY, &p);
  gov_setpoint(&govY, p.vmax);
  uint32_t antes = gov_limite(&govY);
  GovEvento ev = gov_amostra(&govY, agora,
                             stepper_busy(&motY) && !ySemSinal && yDeteccao && !yGovSuspenso,
                             stepper_position(&motY), encoder_count(&encY));
  yGovAplicar();
  float real = gov_vel_real(&govY) / yPassosMm;
  float sp = gov_setpoint_atual(&govY) / yPassosMm;
  if (ev == GOV_QUEDA_CONTADA) {
    Serial.printf("[vel Y] queda %u/%u: encoder %.2f mm/s, motor %.2f mm/s (%ld%%): ainda nao reduz\n",
                  gov_seguidas(&govY), yGovQuedas, real, gov_vel_motor(&govY) / yPassosMm,
                  (long)gov_aproveitamento(&govY));
  } else if (ev == GOV_QUEDA) {
    Serial.printf("[vel Y] CAIU (%u queda(s) seguida(s)): encoder %.2f mm/s, motor %.2f mm/s"
                  " (%ld%%, setpoint %.2f mm/s): rotacao %lu -> %lu passos/s (mais torque)\n",
                  yGovQuedas, real, gov_vel_motor(&govY) / yPassosMm, (long)gov_aproveitamento(&govY),
                  sp, (unsigned long)(antes < p.vmax ? antes : p.vmax), (unsigned long)gov_limite(&govY));
  } else if (ev == GOV_SUBIU) {
    Serial.printf("[vel Y] tentando recuperar: rotacao %lu passos/s (%.2f mm/s) de %.2f mm/s;"
                  " mantem pelo menos %lu ms\n",
                  (unsigned long)gov_limite(&govY), gov_limite(&govY) / yPassosMm, sp,
                  (unsigned long)yGovSegurarMs);
  } else if (ev == GOV_RECUPEROU) {
    Serial.printf("[vel Y] setpoint recuperado: %.2f mm/s (%lu passos/s)\n", sp,
                  (unsigned long)gov_setpoint_atual(&govY));
  }
}

// ---------- telemetria "@Y" ----------
static void yTelemetria() {
  uint32_t agora = millis();
  bool andando = yOcupado();
  if (agora - yTelemUltimo < (andando ? Y_TELEM_MS : Y_TELEM_PARADO_MS)) return;
  yTelemUltimo = agora;
  uint32_t vy = stepper_speed_now(&motY);
  uint32_t lim = gov_limite(&govY);
  uint32_t sp = gov_setpoint_atual(&govY);
  Serial.printf("@Y %lu %.2f %ld %lu %.2f %lu %d %lu %lu", (unsigned long)agora, yMm(),
                (long)encoder_count(&encY), (unsigned long)vy, vy / yPassosMm,
                (unsigned long)(lim < sp ? lim : sp), yRef ? 1 : 0,
                (unsigned long)lroundf(gov_vel_real(&govY)), (unsigned long)sp);
  MalhaEstado me = malha_estado(&malhaY);
  if (me == MALHA_PARADA) {
    Serial.print(" - - -");
  } else {
    int32_t a = malha_alvo(&malhaY);
    Serial.printf(" %.2f %ld %c", yContParaMm(a), (long)a,
                  me == MALHA_MOVENDO ? 'm' : (me == MALHA_CHEGOU ? 'c' : 'f'));
  }
  Serial.println(ySemSinal ? " SEMSINAL" : " ok");
}

// ---------- esperas bloqueantes ----------
// Cancela com qualquer tecla que não seja fim de linha; mantém governo e telemetria.
static bool yCancelar(void *) {
  yGovernar();
  yTelemetria();
  while (Serial.available()) {
    int c = Serial.read();
    if (c != '\r' && c != '\n') return true;
  }
  return false;
}

static bool yEsperarMs(uint32_t ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    if (yCancelar(nullptr)) return false;
    delay(10);
  }
  return true;
}

// ---------- movimento pelo encoder ----------
static bool yIrCont(int32_t alvo) {
  if (ySemSinal) {
    Serial.println("[encoder Y] SEM SINAL: movimento pelo encoder nao feito (a posicao lida e falsa)."
                   " Confira cabo, 3V3 e acoplamento; teste: YE + girar o fuso a mao");
    return false;
  }
  int32_t passos = malha_ir(&malhaY, alvo, encoder_count(&encY));
  if (passos == 0) return false;
  stepper_start(&motY, passos, true);  // solta as bobinas ao chegar
  yParadoDesde = millis();
  return true;
}

static void yAtualizarMalha() {
  if (!malha_ativa(&malhaY)) return;
  if (ySemSinal) {
    malha_cancelar(&malhaY);
    return;
  }
  if (stepper_busy(&motY)) {
    yParadoDesde = millis();
    return;
  }
  if (millis() - yParadoDesde < Y_MALHA_ASSENTAR_MS) return;
  int32_t passos = malha_parou(&malhaY, encoder_count(&encY));
  if (passos != 0) {
    Serial.printf("[malha Y] correcao %u: erro %+ld cont (%+.2f mm) -> %+ld passos\n",
                  malha_correcoes(&malhaY), (long)malha_erro(&malhaY), yErroMm(malha_erro(&malhaY)),
                  (long)passos);
    stepper_start(&motY, passos, true);
    yParadoDesde = millis();
  } else if (malha_estado(&malhaY) == MALHA_FALHOU) {
    Serial.printf("[malha Y] aviso: ficou a %+ld cont (%+.2f mm) do alvo depois de %u correcoes"
                  " (so aviso: nada parou)\n",
                  (long)malha_erro(&malhaY), yErroMm(malha_erro(&malhaY)), malha_correcoes(&malhaY));
  }
}

static bool yEsperarMalha() {
  while (yOcupado()) {
    if (yCancelar(nullptr)) {
      malha_cancelar(&malhaY);
      stepper_release(&motY);
      return false;
    }
    yAtualizarMalha();
    delay(1);
  }
  return true;
}

// ---------- estado na serial ----------
static void yImprimir() {
  // Sem "X:" nem "Z:" nesta linha (o painel lê esses do X e do Z).
  Serial.printf("encoder Y: %ld cont (erros %lu)%s", (long)encoder_count(&encY),
                (unsigned long)encoder_errors(&encY), ySemSinal ? " SEM SINAL" : "");
  if (yRef) {
    Serial.printf("   Y: %.2f mm", yMm());
  } else {
    Serial.print("   Y: sem homing");
  }
  if (malha_estado(&malhaY) != MALHA_PARADA) {
    int32_t a = malha_alvo(&malhaY);
    const char *est = malha_estado_nome(malha_estado(&malhaY));
    if (yRef) {
      Serial.printf("   alvo Y: %.2f mm (%s)", yContParaMm(a), est);
    } else {
      Serial.printf("   alvo Y: %ld cont (%s)", (long)a, est);
    }
  }
  Serial.println();
}

static void yMostrarRampa() {
  StepperPerfil p;
  stepper_get_profile(&motY, &p);
  Serial.printf("rampa Y: partida %lu passos/s, maxima %lu passos/s (%.2f mm/s), aceleracao %lu passos/s2\n",
                (unsigned long)p.v0, (unsigned long)p.vmax, p.vmax / yPassosMm, (unsigned long)p.acel);
}

static void yMostrarHoming() {
  Serial.printf("homing Y: sentido %+d ('%c' vai para a chave), rapida %lu passos/s, lenta %lu passos/s,"
                " travamento %ld passos (%s)\n",
                ySentido, giro(ySentido), (unsigned long)homY.vel_rapida, (unsigned long)homY.vel_lenta,
                (long)yTravaPassos, homY.fsm.janela_trava > 0 ? "ligado" : "DESLIGADO");
}

void yMostrarChave() {
  Serial.printf("chave Y: %s (GPIO %u = %s)\n", eixo_chave_crua(&eixoY) ? "ACIONADA" : "solta", HOME_Y,
                digitalRead(HOME_Y) ? "HIGH" : "LOW");
}

static void yMostrar() {
  yImprimir();
  StepperPerfil p;
  stepper_get_profile(&motY, &p);
  uint32_t lim = gov_limite(&govY);
  Serial.printf("eixo Y: curso %.0f a %.0f mm (chave em %.0f); setpoint %.2f mm/s (%lu passos/s),"
                " rotacao agora %lu passos/s; deteccao %s\n",
                Y_HOME_MM, Y_HOME_MM + CURSO_Y_MM, Y_HOME_MM, p.vmax / yPassosMm,
                (unsigned long)p.vmax, (unsigned long)(lim < p.vmax ? lim : p.vmax),
                yDeteccao ? "ligada" : "DESLIGADA");
  Serial.printf("   motor Y: GPIO %u %u %u %u;  encoder Y: GPIO %u/%u\n", MOT_Y_IN1, MOT_Y_IN2,
                MOT_Y_IN3, MOT_Y_IN4, ENC_Y_A, ENC_Y_B);
  yMostrarChave();
  yMostrarRampa();
  yMostrarHoming();
  yCalibMostrar();
  yGovMostrar();
}

// ---------- homing ----------
static void yAvisoHoming(const EixoEvento *ev, void *) {
  if (ev->tipo == EIXO_EV_ESTADO) {
    Serial.printf("[homing Y] estado: %s, girando '%c'  (motor %ld, encoder %ld)\n",
                  homing_state_nome(ev->estado), giro(ev->giro), (long)ev->motor, (long)ev->enc);
  } else {
    Serial.printf("[homing Y] SENSOR %s  (motor %ld, encoder %ld)\n", ev->chave ? "APERTADO" : "solto",
                  (long)ev->motor, (long)ev->enc);
  }
}

static void yAplicarTrava() { homY.fsm.janela_trava = yDeteccao ? yTravaPassos : 0; }

static void yImprimirHome(const EixoHomeResult &r) {
  if (r.erro == HOM_ERRO_NENHUM) {
    Serial.printf("homing Y ok em %lu passos: Y = %.2f mm na chave\n", (unsigned long)r.passos_dados,
                  Y_HOME_MM);
    return;
  }
  Serial.printf("homing Y FALHOU (%s) na fase '%s', depois de %lu passos\n", homing_error_nome(r.erro),
                homing_state_nome(r.parou_em), (unsigned long)r.passos_dados);
  if (r.erro == HOM_ERRO_CURSO) {
    Serial.printf("   Y andou o curso todo sem a chave acionar: confira a chave (GPIO %u, pull-down"
                  " de 10k) e o sentido (YHD testa; trocar: YHS%+d)\n", HOME_Y, -ySentido);
  } else if (r.erro == HOM_ERRO_CHAVE_PRESA) {
    Serial.println("   a chave Y nao soltou: chave NF no lugar de NA, curto com o 3V3 ou chave travada (YS)");
  }
}

static EixoHomeResult yUltimoHome = {};

static bool yHoming() {
  malha_cancelar(&malhaY);
  Serial.println("-- homing do Y (Enter com qualquer tecla cancela) --");
  bool tinhaRef = yRef;
  eixo_home(&eixoY, &homY, &yUltimoHome);
  stepper_set_speed(&motY, yVelocidade);  // o homing muda a velocidade
  const EixoHomeResult &r = yUltimoHome;
  yRef = (r.erro == HOM_ERRO_NENHUM);
  yChaveAnterior = eixo_chave_crua(&eixoY);
  yImprimirHome(r);
  if (yRef && tinhaRef) {
    Serial.printf("   chave acionou a %+ld contagens (%+.2f mm) da origem anterior\n",
                  (long)r.enc_na_chave, r.enc_na_chave / yContMm);
  }
  yImprimir();
  return yRef;
}

// HD do Y: anda 2 mm para a chave (ou afastando, se ela já está apertada).
static void yDiagnosticoHoming() {
  bool apertada = eixo_chave_crua(&eixoY);
  Serial.println("-- diagnostico do homing Y (Enter com qualquer tecla cancela) --");
  Serial.printf("1) chave Y: %s (GPIO %u = %s). Apertando com a mao tem que virar ACIONADA (HIGH)\n",
                apertada ? "ACIONADA" : "solta", HOME_Y, digitalRead(HOME_Y) ? "HIGH" : "LOW");
  int8_t dir = apertada ? (int8_t)-ySentido : ySentido;
  int32_t passos = (int32_t)lroundf(Y_HD_MM * yPassosMm);
  Serial.printf("2) Y andando %.0f mm %s (giro '%c'): OLHE o carro do Y\n", Y_HD_MM,
                apertada ? "AFASTANDO da chave (ela ja esta apertada)" : "PARA a chave", giro(dir));
  int32_t enc0 = encoder_count(&encY);
  malha_cancelar(&malhaY);
  if (!apertada) {
    stepper_sensor_arm(&motY, HOME_Y, !eixoY.ativa_em_baixo, true,
                       (uint16_t)((uint64_t)EIXO_CHAVE_DEBOUNCE_US * STEPPER_TICK_HZ / 1000000u));
  }
  stepper_start(&motY, dir * passos, true);
  bool cancelou = false;
  while (stepper_busy(&motY)) {
    if (yCancelar(nullptr)) {
      stepper_release(&motY);
      cancelou = true;
      break;
    }
    delay(1);
  }
  stepper_sensor_disarm(&motY);
  delay(5);
  int32_t pos;
  bool acionou = stepper_sensor_fired(&motY, &pos);
  stepper_release(&motY);
  yRef = false;  // andou sem a origem: refazer o YH
  if (cancelou) {
    Serial.println("-- diagnostico do Y cancelado --");
    return;
  }
  int32_t enc = encoder_count(&encY) - enc0;
  float esp = dir * passos * yContMm / yPassosMm;
  Serial.printf("   encoder Y: %+ld contagens (esperado ~%+.0f): ", (long)enc, esp);
  if (enc > -2 && enc < 2) {
    Serial.println("PARADO -> encoder solto/desligado, ou o motor nao girou (teste as bobinas: YI)");
  } else if ((enc > 0) != (esp > 0)) {
    Serial.println("AO CONTRARIO -> troque os fios A e B do encoder Y (ou ENC_Y_A/ENC_Y_B no pins.h)");
  } else if (fabsf((float)enc) < 0.6f * fabsf(esp)) {
    Serial.println("bem menos que o esperado -> motor perdendo passo ou encoder patinando");
  } else {
    Serial.println("ok");
  }
  if (acionou) Serial.println("   a chave Y ACIONOU no caminho: o sentido para a chave esta certo");
  Serial.printf("3) o carro do Y foi %s? Se foi para o lado contrario, o sentido esta trocado: envie YHS%+d\n",
                apertada ? "para LONGE da chave" : "na direcao da chave", -ySentido);
}

// YH + V/L/T/S/D: ajustes do homing (valem até reiniciar).
static void yComandoHoming(String arg) {
  char sub = arg[0];
  String num = arg.substring(1);
  num.trim();
  long v = num.toInt();
  switch (sub) {
    case 'V':
    case 'L':
      if (v <= 0 || v > (long)Y_VEL_LIMITE) {
        Serial.printf("velocidade invalida: '%s' (use 1 a %lu passos/s)\n", num.c_str(),
                      (unsigned long)Y_VEL_LIMITE);
        return;
      }
      if (sub == 'V') homY.vel_rapida = (uint32_t)v;
      else homY.vel_lenta = (uint32_t)v;
      break;
    case 'T':
      if (num.length() == 0 || v < 0 || v > 8 * Y_PASSOS_VOLTA) {
        Serial.printf("janela invalida: '%s' (0 = desliga, ate %ld passos)\n", num.c_str(),
                      (long)(8 * Y_PASSOS_VOLTA));
        return;
      }
      yTravaPassos = (int32_t)v;
      yAplicarTrava();
      break;
    case 'S':
      if (v != 1 && v != -1) {
        Serial.printf("sentido invalido: '%s' (YHS+1 = chave no lado 'f', YHS-1 = lado 'b')\n", num.c_str());
        return;
      }
      if (v != ySentido) {
        ySentido = (int8_t)v;
        homY.sentido = ySentido;
        yRef = false;
        malha_cancelar(&malhaY);
        Serial.println("-- sentido do homing Y trocado: faca o homing (YH) de novo --");
      }
      break;
    case 'D':
      yDiagnosticoHoming();
      yImprimir();
      return;
    default:
      Serial.printf("comando invalido: 'YH%s' (YH, YHV, YHL, YHT, YHS, YHD)\n", arg.c_str());
      return;
  }
  yMostrarHoming();
}

// ---------- calibração de distância (YD / YM / YK) ----------
static void yCalibrarDistancia(String arg) {
  float mm = arg.length() ? lerNumero(arg) : Y_DIST_MM_PADRAO;
  if (!(mm > 0.0f) || Y_DIST_PRE_MM + mm > CURSO_Y_MM) {
    Serial.printf("distancia invalida: 'YD%s' (use 0 < mm <= %.0f)\n", arg.c_str(), CURSO_Y_MM - Y_DIST_PRE_MM);
    return;
  }
  yMedidaPendente = false;
  Serial.printf("-- calibracao de distancia do Y: %.1f mm (Enter com qualquer tecla cancela) --\n", mm);
  malha_cancelar(&malhaY);
  eixo_home(&eixoY, &homY, &yUltimoHome);
  stepper_set_speed(&motY, yVelocidade);
  yImprimirHome(yUltimoHome);
  yRef = (yUltimoHome.erro == HOM_ERRO_NENHUM);
  if (!yRef) return;

  const int8_t afasta = (int8_t)-ySentido;
  bool ok = eixo_andar(&eixoY, &homY, afasta * (int32_t)lroundf(Y_DIST_PRE_MM * yPassosMm));
  if (ok) {
    Serial.printf(">> MARQUE a posicao do carro do Y agora. Andando %.1f mm em %lu s...\n", mm,
                  (unsigned long)(Y_DIST_ESPERA_MS / 1000));
    ok = yEsperarMs(Y_DIST_ESPERA_MS);
  }
  int32_t passos = (int32_t)lroundf(mm * yPassosMm);
  int32_t enc0 = encoder_count(&encY);
  if (ok) ok = eixo_andar(&eixoY, &homY, afasta * passos);
  delay(5);
  int32_t contagens = encoder_count(&encY) - enc0;
  stepper_release(&motY);
  if (!ok) {
    Serial.println("-- calibracao de distancia do Y cancelada --");
    yImprimir();
    return;
  }
  yMedidaPendente = true;
  yPendPassos = passos;
  yPendContagens = contagens;
  yPendMm = mm;
  Serial.printf("Y andou %ld passos; encoder: %ld contagens (%.2f mm pelo encoder atual)\n", (long)passos,
                (long)contagens, fabsf(contagens / yContMm));
  Serial.println(">> MECA o deslocamento real do Y desde a marca e envie: YM<mm>  (ex.: YM99.6)");
  yImprimir();
}

static void yReceberMedida(String arg) {
  if (!yMedidaPendente) {
    Serial.println("nada para calibrar no Y: use YD (ex.: YD100) primeiro");
    return;
  }
  float medido = lerNumero(arg);
  CalibDist r;
  CalibDistErro e = calib_distancia(yPendPassos, yPendContagens, yPendMm, medido, Y_DIST_TOLERANCIA_PCT,
                                    (float)Y_CONT_VOLTA / Y_PASSOS_VOLTA, &r);
  if (e == CAL_DIST_ENTRADA_INVALIDA) {
    Serial.printf("%s: '%s'. Envie YM<mm>, ex.: YM99.6\n", calib_dist_erro_nome(e), arg.c_str());
    return;
  }
  Serial.printf("Y: pedido %.2f mm, medido %.2f mm -> erro %+.2f%%\n", yPendMm, medido, r.erro_pct);
  Serial.printf("encoder Y viu %.1f%% do giro do motor\n", r.encoder_pct);
  if (r.encoder_pct < 95.0f || r.encoder_pct > 105.0f) {
    Serial.println("   atencao: encoder Y longe de 100% (perdendo ou sobrando contagens)");
  }
  if (e == CAL_DIST_FORA_DA_TOLERANCIA) {
    Serial.printf("NAO aplicado: %s (limite +-%.0f%%). Envie YM<mm> de novo.\n", calib_dist_erro_nome(e),
                  Y_DIST_TOLERANCIA_PCT);
    return;
  }
  float pAnt = yPassosMm, cAnt = yContMm;
  yPassosMm = r.passos_por_mm;
  yContMm = r.contagens_por_mm;
  yCalibSalvar();
  yMedidaPendente = false;
  Serial.printf("Y passos/mm:    %.3f -> %.3f\n", pAnt, yPassosMm);
  Serial.printf("Y contagens/mm: %.3f -> %.3f. Salvo\n", cAnt, yContMm);
  yImprimir();
}

// ---------- auto calibração do homing (YCAL3) ----------
static void yAvisoCiclo(const EixoCalibCiclo *c, void *) {
  if (c->ciclo == 0) {
    Serial.print("ciclo Y 0 (origem): ");
    yImprimirHome(c->home);
    return;
  }
  Serial.printf("ciclo Y %lu: ida %ld contagens (esperado %ld) | ", (unsigned long)c->ciclo,
                (long)c->enc_ida, (long)(-ySentido * Y_CAL_VOLTAS * Y_CONT_VOLTA));
  if (c->home.erro == HOM_ERRO_NENHUM) {
    Serial.printf("chave em %+ld passos, encoder %+ld\n", (long)c->home.passos_na_chave,
                  (long)c->home.enc_na_chave);
  } else {
    yImprimirHome(c->home);
  }
}

static void yAutoCalibrar(String arg) {
  long ciclos = arg.length() ? arg.toInt() : 1;
  if (ciclos <= 0 || ciclos > (long)Y_CAL_CICLOS_MAX) {
    Serial.printf("ciclos invalidos: '%s' (use 1..%lu)\n", arg.c_str(), (unsigned long)Y_CAL_CICLOS_MAX);
    return;
  }
  Serial.printf("-- auto calibracao do Y: %ld ciclo(s) de %ld voltas (Enter com qualquer tecla cancela) --\n",
                ciclos, (long)Y_CAL_VOLTAS);
  malha_cancelar(&malhaY);
  EixoCalibResumo r;
  eixo_calibrar(&eixoY, &homY, Y_CAL_VOLTAS * Y_PASSOS_VOLTA, (uint32_t)ciclos, yAvisoCiclo, nullptr, &r);
  stepper_set_speed(&motY, yVelocidade);
  yRef = (r.erro == HOM_ERRO_NENHUM);
  Serial.printf("== resumo Y: %lu de %ld ciclo(s) ok%s ==\n", (unsigned long)r.ciclos_ok, ciclos,
                r.erro == HOM_ERRO_NENHUM ? "" : " (parou por erro)");
  if (r.ciclos_ok == 0) return;
  Serial.printf("chave Y (passos):  min %+ld  max %+ld  media %+.1f  -> variacao %ld passos\n",
                (long)r.passos_na_chave.min, (long)r.passos_na_chave.max, estat_media(&r.passos_na_chave),
                (long)estat_faixa(&r.passos_na_chave));
  Serial.printf("encoder Y na chave: min %+ld  max %+ld  media %+.1f contagens\n", (long)r.enc_na_chave.min,
                (long)r.enc_na_chave.max, estat_media(&r.enc_na_chave));
  Serial.printf("encoder Y por volta: %.2f contagens (esperado %ld)\n",
                estat_media(&r.enc_ida) * -ySentido / Y_CAL_VOLTAS, (long)Y_CONT_VOLTA);
}

// ---------- testador de bobinas (YI) ----------
static void yTestarBobinas(uint8_t coils) {
  malha_cancelar(&malhaY);
  stepper_test_coils(&motY, coils);
  yAndando = false;
  Serial.print("ligado Y:");
  if (coils == 0) Serial.print("  nada");
  for (uint8_t i = 0; i < 4; i++) {
    if ((coils >> i) & 0x1) Serial.printf("  IN%u (GPIO %u, LED %c)", i + 1, stepper_pin(&motY, i), 'A' + i);
  }
  Serial.println();
}

static void yComandoBobinas(String arg) {
  if (arg.length() == 0) {
    Serial.printf("testador Y: IN1..IN4 = GPIO %u %u %u %u\n", stepper_pin(&motY, 0), stepper_pin(&motY, 1),
                  stepper_pin(&motY, 2), stepper_pin(&motY, 3));
    return;
  }
  char c = arg[0];
  if (c >= '1' && c <= '4') {
    yTestarBobinas((uint8_t)(1 << (c - '1')));
  } else if (c == 'P' || c == 'O') {
    yFaseTeste = (uint8_t)((yFaseTeste + (c == 'P' ? 1 : -1)) & 0x3);
    Serial.printf("fase Y %u -> ", yFaseTeste);
    yTestarBobinas(stepseq_coils(yFaseTeste));
  } else if (c == '0') {
    stepper_release(&motY);
    yFaseTeste = 3;
    Serial.println("-- bobinas do Y: fim do teste (Y solto) --");
  } else {
    Serial.printf("comando invalido: 'YI%s' (YI, YI1..YI4, YIP, YIO, YI0)\n", arg.c_str());
  }
}

// ---------- tempo com o encoder ligado x parado (YTV) ----------
static uint32_t yCronometrar(int32_t passos) {
  uint32_t t0 = micros();
  stepper_move(&motY, passos);
  return micros() - t0;
}

static void yRelatar(const char *nome, int32_t passos, uint32_t us, uint32_t ideal_us) {
  Serial.printf("%-16s %8.3f s  (ideal %.3f s)  -> %7.1f passos/s  %+6.2f%%\n", nome, us / 1e6f,
                ideal_us / 1e6f, passos / (us / 1e6f), (float)((int64_t)us - ideal_us) * 100.0f / ideal_us);
}

// ---------- diagnóstico do encoder (YED) ----------
// Gira o motor devagar 'voltas' para cada lado e lê A e B DIRETO nos pinos
// (a cada ~200 us, por fora do driver), contando por canal: mudanças de A,
// mudanças de B, transições válidas para frente / para trás e transições
// impossíveis (A e B mudando juntos). Diz se o problema é canal morto,
// eixo patinando (vai-e-volta) ou ruído.
struct YDiagSentido {
  uint32_t mudA, mudB, frente, tras, invalidas;
  int32_t cont_driver;  // o que o driver do encoder contou no mesmo trecho
};

// Transição de quadratura: +1, -1, 0 (sem mudança) ou 2 (impossível).
static int8_t yQuad(uint8_t de, uint8_t para) {
  static const int8_t t[16] = {0, -1, +1, 2, +1, 0, 2, -1, -1, 2, 0, +1, 2, +1, -1, 0};
  return t[(de << 2) | para];
}

static bool yDiagGirar(int32_t passos, YDiagSentido *r) {
  *r = YDiagSentido{};
  int32_t c0 = encoder_count(&encY);
  uint8_t ab = (uint8_t)((digitalRead(ENC_Y_A) << 1) | digitalRead(ENC_Y_B));
  stepper_start(&motY, passos, false);
  uint32_t ultimoServico = millis();
  while (stepper_busy(&motY)) {
    uint8_t novo = (uint8_t)((digitalRead(ENC_Y_A) << 1) | digitalRead(ENC_Y_B));
    if (novo != ab) {
      if ((novo ^ ab) & 0x2) r->mudA++;
      if ((novo ^ ab) & 0x1) r->mudB++;
      int8_t d = yQuad(ab, novo);
      if (d == 1) r->frente++;
      else if (d == -1) r->tras++;
      else if (d == 2) r->invalidas++;
      ab = novo;
    }
    if (millis() - ultimoServico >= 20) {  // cancelamento sem atrapalhar a leitura
      ultimoServico = millis();
      while (Serial.available()) {
        int c = Serial.read();
        if (c != '\r' && c != '\n') {
          stepper_release(&motY);
          return false;
        }
      }
    }
    delayMicroseconds(200);
  }
  delay(5);
  r->cont_driver = encoder_count(&encY) - c0;
  return true;
}

static void yDiagRelatar(const char *nome, int32_t passos, const YDiagSentido &r) {
  int32_t esperado = (int32_t)lroundf(passos * yContMm / yPassosMm);
  int32_t liquido = (int32_t)r.frente - (int32_t)r.tras;
  Serial.printf("[diag encoder Y] %s: A mudou %lu, B mudou %lu | validas +%lu / -%lu (liquido %+ld),"
                " impossiveis %lu | driver contou %+ld (esperado ~%+ld)\n",
                nome, (unsigned long)r.mudA, (unsigned long)r.mudB, (unsigned long)r.frente,
                (unsigned long)r.tras, (long)liquido, (unsigned long)r.invalidas, (long)r.cont_driver,
                (long)esperado);
}

// Conclusão de um sentido (para a linha final).
static const char *yDiagConclusao(int32_t passos, const YDiagSentido &r) {
  float esperado = fabsf(passos * yContMm / yPassosMm);  // mudanças de estado esperadas
  float canalEsp = esperado / 2.0f;                       // cada canal muda metade das vezes
  int32_t liquido = (int32_t)r.frente - (int32_t)r.tras;
  bool aMorto = r.mudA < canalEsp * 0.1f, bMorto = r.mudB < canalEsp * 0.1f;
  if (aMorto && bMorto) return "NENHUM canal mudou: encoder parado (acoplamento solto/patinando ou sem alimentacao)";
  if (aMorto) return "canal A (CLK, GPIO 33) NAO muda: fio do A solto ou ruim";
  if (bMorto) return "canal B (DT, GPIO 36) NAO muda: fio do B solto ou ruim";
  uint32_t total = r.frente + r.tras;
  if (total > 0 && (float)abs(liquido) < 0.3f * total) {
    return "A e B mudam mas VAI-E-VOLTA (o eixo do encoder treme sem girar): acoplamento patinando ou encoder raspando";
  }
  if (r.invalidas > total / 5 + 2) return "muitas transicoes impossiveis: ruido no cabo (afaste dos fios do motor) ou sinal fraco";
  if ((float)abs(liquido) < 0.6f * esperado) return "conta menos que o esperado: acoplamento escorregando em parte do giro";
  return "ok";
}

static void yDiagEncoder(String arg) {
  float voltas = arg.length() ? lerNumero(arg) : 1.0f;
  if (!(voltas > 0.0f) || voltas > 5.0f) {
    Serial.printf("voltas invalidas: 'YED%s' (0 a 5)\n", arg.c_str());
    return;
  }
  malha_cancelar(&malhaY);
  int32_t passos = (int32_t)lroundf(voltas * Y_PASSOS_VOLTA);
  StepperPerfil orig;
  stepper_get_profile(&motY, &orig);
  StepperPerfil lento = {150, 150, 0};  // devagar e sem rampa: torque alto, leitura fácil
  yGovSuspenso = true;
  stepper_set_limit(&motY, 0);
  stepper_set_profile(&motY, &lento);
  Serial.printf("-- diagnostico do encoder Y: %.1f volta(s) para cada lado a 150 passos/s"
                " (OLHE o fuso; qualquer tecla cancela) --\n", voltas);
  YDiagSentido f, b;
  bool ok = yDiagGirar(+passos, &f);
  if (ok) {
    delay(200);
    ok = yDiagGirar(-passos, &b);
  }
  stepper_release(&motY);
  stepper_set_profile(&motY, &orig);
  yGovSuspenso = false;
  yGovAplicado = 0;
  yGovAplicar();
  yRef = false;  // andou fora da malha: refazer o YH
  if (!ok) {
    Serial.println("-- diagnostico do encoder Y cancelado --");
    return;
  }
  yDiagRelatar("giro f (rumo a chave)", +passos, f);
  yDiagRelatar("giro b (afastando)   ", -passos, b);
  const char *cf = yDiagConclusao(passos, f);
  const char *cb = yDiagConclusao(passos, b);
  Serial.printf("== diagnostico do encoder Y: f -> %s | b -> %s ==\n", cf, cb);
}

static void yMedirTempo(String arg) {
  float voltas = arg.length() ? lerNumero(arg) : 1.0f;
  if (!(voltas > 0.0f) || voltas > 10.0f) {
    Serial.printf("voltas invalidas: '%s' (use 0 < voltas <= 10)\n", arg.c_str());
    return;
  }
  malha_cancelar(&malhaY);
  int32_t passos = (int32_t)lroundf(voltas * Y_PASSOS_VOLTA);
  StepperPerfil p;
  stepper_get_profile(&motY, &p);
  RampaCfg rc = {STEPPER_TICK_HZ, p.v0, p.vmax, p.acel};
  uint32_t ideal = (uint32_t)(rampa_tiques_total(&rc, passos) * 1000000ULL / STEPPER_TICK_HZ);
  Serial.printf("-- medicao Y: %.2f volta(s) = %ld passos --\n", voltas, (long)passos);
  yMostrarRampa();
  encoder_stats_reset(&encY);
  int32_t enc0 = encoder_count(&encY);
  uint32_t us_on = yCronometrar(passos);
  delay(5);
  int32_t cont_on = encoder_count(&encY) - enc0;
  EncoderStats st;
  encoder_stats(&encY, &st);
  encoder_stop(&encY);
  uint32_t us_off = yCronometrar(-passos);
  encoder_start(&encY);
  stepper_release(&motY);
  if (yRef) {
    yRef = false;  // o encoder ficou desligado na volta: a posição dele não vale
    Serial.println("-- o encoder Y ficou desligado na volta: posicao do Y perdida, faca o homing (YH) --");
  }
  yRelatar("encoder Y LIGADO", passos, us_on, ideal);
  yRelatar("encoder Y PARADO", passos, us_off, ideal);
  Serial.printf("diferenca ligado - parado: %+ld us (%+.3f%%)\n", (long)us_on - (long)us_off,
                ((float)us_on - (float)us_off) * 100.0f / us_off);
  Serial.printf("encoder Y no trecho ligado: %ld contagens (esperado ~%ld)\n", (long)cont_on,
                (long)lroundf(passos * (float)Y_CONT_VOLTA / Y_PASSOS_VOLTA));
  if (st.amostras > 0) {
    Serial.printf("amostragem Y: nucleo %d, %lu amostras (%.0f/s), media %.2f us, maxima %lu us,"
                  " maior intervalo %lu us\n",
                  st.core, (unsigned long)st.amostras, st.amostras / (us_on / 1e6f),
                  (float)st.soma_us / st.amostras, (unsigned long)st.max_us,
                  (unsigned long)st.max_intervalo_us);
  }
  yImprimir();
}

// ---------- varredura (YW) e curva (YP): erro de seguimento motor x encoder ----------
struct YAmostra {
  uint32_t t_ms;
  uint32_t v;
  int32_t motor;
  int32_t enc;
};
static const uint16_t Y_CURVA_MAX = 600;
static YAmostra yCurva[Y_CURVA_MAX];
static uint16_t yCurvaN = 0;

struct YMov {
  bool ok;
  uint32_t us, ideal_us;
  int32_t passos, esperado, lido, erro_max, erro_fim;
};

// Alvo do motor (passos) para levar o Y a 'mm', a partir de onde o ENCODER diz que ele está.
static int32_t yAlvoPassos(float mm) {
  return stepper_position(&motY) + (int32_t)lroundf(-ySentido * (mm - yMm()) * yPassosMm);
}

static YMov yMedirMovimento(int32_t alvo, bool gravar, uint32_t periodo_ms) {
  YMov r = {};
  int32_t m0 = stepper_position(&motY);
  int32_t e0 = encoder_count(&encY);
  r.passos = alvo - m0;
  StepperPerfil p;
  stepper_get_profile(&motY, &p);
  RampaCfg rc = {STEPPER_TICK_HZ, p.v0, p.vmax, p.acel};
  r.ideal_us = (uint32_t)(rampa_tiques_total(&rc, r.passos) * 1000000ULL / STEPPER_TICK_HZ);
  const float k = yContMm / yPassosMm;
  if (gravar) yCurvaN = 0;
  r.ok = true;
  uint32_t t0 = micros(), prox = 0;
  stepper_start_to(&motY, alvo, false);
  while (stepper_busy(&motY)) {
    if (yCancelar(nullptr)) {
      stepper_abort(&motY);
      r.ok = false;
      break;
    }
    int32_t m = stepper_position(&motY) - m0;
    int32_t e = encoder_count(&encY) - e0;
    int32_t erro = e - (int32_t)lroundf(m * k);
    if (absi(erro) > r.erro_max) r.erro_max = absi(erro);
    uint32_t t_ms = (micros() - t0) / 1000;
    if (gravar && t_ms >= prox && yCurvaN < Y_CURVA_MAX) {
      yCurva[yCurvaN++] = YAmostra{t_ms, stepper_speed_now(&motY), m, e};
      prox = t_ms + periodo_ms;
    }
    delay(1);
  }
  r.us = micros() - t0;
  delay(5);
  int32_t m = stepper_position(&motY) - m0;
  r.lido = encoder_count(&encY) - e0;
  r.esperado = (int32_t)lroundf(m * k);
  r.erro_fim = r.lido - r.esperado;
  if (absi(r.erro_fim) > r.erro_max) r.erro_max = absi(r.erro_fim);
  if (gravar && yCurvaN < Y_CURVA_MAX) yCurva[yCurvaN++] = YAmostra{r.us / 1000, 0, m, r.lido};
  return r;
}

static int32_t yTolerancia(const YMov &r) {
  return Y_VARRE_TOL_CONT + (int32_t)(absi(r.esperado) * Y_VARRE_TOL_PCT / 100.0f);
}
static bool yPassou(const YMov &r) { return r.ok && r.erro_max <= yTolerancia(r); }

static bool yPrepararTeste(float fim_mm) {
  if (!yDentro(fim_mm)) {
    Serial.printf("o teste vai ate Y = %.0f mm, alem do curso (%.0f a %.0f mm)\n", fim_mm, Y_HOME_MM,
                  Y_HOME_MM + CURSO_Y_MM);
    return false;
  }
  if (!yRef && !yHoming()) return false;
  yIrCont(yMmParaCont(Y_VAI_A_MM));  // começo do teste pelo ENCODER
  return yEsperarMalha();
}

static void yVarrer(String arg) {
  uint32_t ate = Y_VARRE_VELS[sizeof Y_VARRE_VELS / sizeof Y_VARRE_VELS[0] - 1];
  if (arg.length()) {
    long v = arg.toInt();
    if (v <= 0) {
      Serial.printf("valor invalido: 'YW%s'\n", arg.c_str());
      return;
    }
    ate = (uint32_t)v;
  }
  // A varredura procura de propósito onde o motor perde passo: governo e detecção desligados.
  yGovSuspenso = true;
  stepper_set_limit(&motY, 0);
  Serial.println("   governo e deteccao do Y DESLIGADOS durante a varredura");
  StepperPerfil orig;
  stepper_get_profile(&motY, &orig);
  Serial.printf("-- varredura de velocidade do Y ate %lu passos/s: vaivem %.0f <-> %.0f mm --\n",
                (unsigned long)ate, Y_VAI_A_MM, Y_VAI_B_MM);
  bool cancelou = false, falhou = false;
  uint32_t limite = 0;
  if (!yPrepararTeste(Y_VAI_B_MM)) {
    cancelou = true;
  } else {
    Serial.println(" vmax | ida: real (ideal) s | volta: real (ideal) s | erro max (tol) |");
    for (uint32_t v : Y_VARRE_VELS) {
      if (v > ate) break;
      StepperPerfil p = orig;
      p.vmax = v;
      stepper_set_profile(&motY, &p);
      YMov ida = yMedirMovimento(yAlvoPassos(Y_VAI_B_MM), false, 0);
      YMov volta = ida.ok ? yMedirMovimento(yAlvoPassos(Y_VAI_A_MM), false, 0) : ida;
      if (!ida.ok || !volta.ok) {
        cancelou = true;
        break;
      }
      bool ok = yPassou(ida) && yPassou(volta);
      int32_t emax = ida.erro_max > volta.erro_max ? ida.erro_max : volta.erro_max;
      Serial.printf(" %4lu |   %6.2f (%6.2f)    |   %6.2f (%6.2f)      |   %3ld (%3ld)    | %s\n",
                    (unsigned long)v, ida.us / 1e6f, ida.ideal_us / 1e6f, volta.us / 1e6f,
                    volta.ideal_us / 1e6f, (long)emax, (long)yTolerancia(ida), ok ? "ok" : "PERDEU PASSO");
      if (!ok) {
        falhou = true;
        break;
      }
      limite = v;
    }
  }
  stepper_set_profile(&motY, &orig);
  stepper_release(&motY);
  yGovSuspenso = false;
  yGovAplicado = 0;
  yGovAplicar();
  Serial.println("   governo e deteccao do Y religados");
  if (cancelou) {
    Serial.println("-- varredura do Y cancelada --");
    return;
  }
  Serial.println("conferencia: homing do Y");
  yHoming();
  Serial.println("== resultado Y ==");
  if (limite == 0) {
    Serial.println("Y perdeu passo ja na primeira velocidade: baixe a aceleracao (YA) ou a partida (YN)");
  } else {
    Serial.printf("maior vmax do Y sem perder passo: %lu passos/s (%.1f mm/s)%s\n", (unsigned long)limite,
                  limite / yPassosMm, falhou ? "" : "  (nao chegou a falhar: limite real pode ser maior)");
  }
}

static void yGravarCurva(String arg) {
  float mm = arg.length() ? lerNumero(arg) : (Y_VAI_B_MM - Y_VAI_A_MM);
  if (!(mm > 0.0f)) {
    Serial.printf("distancia invalida: 'YP%s'\n", arg.c_str());
    return;
  }
  Serial.printf("-- curva do Y: %.1f mm a partir de %.0f mm (Enter com qualquer tecla cancela) --\n", mm,
                Y_VAI_A_MM);
  yMostrarRampa();
  if (!yPrepararTeste(Y_VAI_A_MM + mm)) {
    stepper_release(&motY);
    return;
  }
  StepperPerfil p;
  stepper_get_profile(&motY, &p);
  RampaCfg rc = {STEPPER_TICK_HZ, p.v0, p.vmax, p.acel};
  int32_t passos = yAlvoPassos(Y_VAI_A_MM + mm) - stepper_position(&motY);
  uint32_t ideal_ms = (uint32_t)(rampa_tiques_total(&rc, passos) * 1000ULL / STEPPER_TICK_HZ);
  uint32_t periodo = ideal_ms / (Y_CURVA_MAX - 2) + 1;
  if (periodo < 10) periodo = 10;
  YMov r = yMedirMovimento(yAlvoPassos(Y_VAI_A_MM + mm), true, periodo);
  yMedirMovimento(yAlvoPassos(Y_VAI_A_MM), false, 0);
  stepper_release(&motY);
  if (!r.ok) {
    Serial.println("-- curva do Y cancelada --");
    return;
  }
  Serial.printf("Y: tempo %.3f s (ideal %.3f s), encoder %ld de %ld contagens, erro max %ld (tol %ld): %s\n",
                r.us / 1e6f, r.ideal_us / 1e6f, (long)r.lido, (long)r.esperado, (long)r.erro_max,
                (long)yTolerancia(r), yPassou(r) ? "ok" : "PERDEU PASSO");
  Serial.printf("%u amostras a cada %lu ms. Copie para uma planilha (separador ;)\n", yCurvaN,
                (unsigned long)periodo);
  Serial.println("--- CSV Y ---");
  Serial.println("t_ms;v_motor;v_encoder;motor_passos;encoder_cont;erro_cont");
  const float k = yContMm / yPassosMm;
  const uint16_t janela = (uint16_t)((100 + periodo - 1) / periodo);
  for (uint16_t i = 0; i < yCurvaN; i++) {
    const YAmostra &a = yCurva[i];
    Serial.printf("%lu;%lu;", (unsigned long)a.t_ms, (unsigned long)a.v);
    if (i >= janela) {
      const YAmostra &b = yCurva[i - janela];
      float dt = (a.t_ms - b.t_ms) / 1000.0f;
      Serial.printf("%.0f", dt > 0 ? fabsf((a.enc - b.enc) / k / dt) : 0.0f);
    }
    Serial.printf(";%ld;%ld;%ld\n", (long)a.motor, (long)a.enc, (long)(a.enc - (int32_t)lroundf(a.motor * k)));
  }
  Serial.println("--- fim do CSV Y ---");
}

// ---------- teste do Y (YT; também pelo E do main) ----------
// 0 = OK, 1 = FALHOU, 2 = CANCELADO.
static int yTeste() {
  Serial.println("== teste do Y: pontos pelo encoder e repeticao da chave ==");
  if (!yRef && !yHoming()) {
    Serial.println("== teste do Y: FALHOU (homing) ==");
    return 1;
  }
  stepper_set_speed(&motY, yVelocidade);
  bool ok = true;
  const size_t n = sizeof Y_TESTE_PONTOS / sizeof Y_TESTE_PONTOS[0];
  for (size_t i = 0; i < n; i++) {
    float alvo = Y_TESTE_PONTOS[i];
    if (!yDentro(alvo)) continue;
    uint32_t t0 = millis();
    yIrCont(yMmParaCont(alvo));
    if (!yEsperarMalha()) {
      Serial.println("== teste do Y: CANCELADO ==");
      return 2;
    }
    bool chegou = malha_estado(&malhaY) == MALHA_CHEGOU;
    if (!chegou) ok = false;
    Serial.printf("[teste Y] %u/%u alvo %.2f mm: encoder %.2f mm (%ld cont), erro %+.2f mm, "
                  "%u correcao(oes), %.1f s -> %s\n",
                  (unsigned)(i + 1), (unsigned)n, alvo, yMm(), (long)encoder_count(&encY),
                  yErroMm(malha_erro(&malhaY)), malha_correcoes(&malhaY), (millis() - t0) / 1000.0f,
                  chegou ? "ok" : "NAO CHEGOU");
  }
  Serial.println("[teste Y] refazendo o homing: a chave tem que acionar na origem");
  if (!yHoming()) {
    Serial.println("== teste do Y: FALHOU (homing de volta) ==");
    return 1;
  }
  int32_t dc = yUltimoHome.enc_na_chave;
  bool origemOk = dc >= -Y_TESTE_ORIGEM_TOL_CONT && dc <= Y_TESTE_ORIGEM_TOL_CONT;
  Serial.printf("[teste Y] chave a %+ld cont (%+.2f mm) da origem pelo encoder (limite +-%ld) -> %s\n",
                (long)dc, dc / yContMm, (long)Y_TESTE_ORIGEM_TOL_CONT, origemOk ? "ok" : "FORA");
  if (!origemOk) ok = false;
  Serial.printf("== teste do Y: %s ==\n", ok ? "OK" : "FALHOU");
  return ok ? 0 : 1;
}

// Para o homing geral (HA) e o teste geral (E) do main.cpp.
bool yHomingGeral() {
  if (yOcupado()) {
    Serial.println("Y andando: YQ para");
    return false;
  }
  stepper_set_speed(&motY, yVelocidade);
  return yHoming();
}

int yTesteGeral() {
  if (yOcupado()) {
    Serial.println("Y andando: YQ para");
    return 1;
  }
  return yTeste();
}

float yPosicaoMm() { return yMm(); }

Stepper *yMotor() {
  malha_cancelar(&malhaY);  // o testador mexe nas bobinas: a malha não corrige junto
  yAndando = false;
  return &motY;
}

bool yReferenciado() { return yRef; }

bool yIrMm(float mm, float vel_mm_s) {
  if (!yDentro(mm)) return false;
  uint32_t v = yVelocidade;
  if (vel_mm_s > 0.0f) {
    v = (uint32_t)lroundf(vel_mm_s * yPassosMm);
    if (v > Y_VEL_LIMITE) v = Y_VEL_LIMITE;
    if (v < 1) v = 1;
  }
  stepper_set_speed(&motY, v);
  return yIrCont(yMmParaCont(mm));
}

void yRestaurarVelocidade() { stepper_set_speed(&motY, yVelocidade); }

EixoMm yEixoMm() {
  // afasta = giro que se afasta da chave (o contrário do que vai para ela).
  EixoMm e = {Y_HOME_MM, CURSO_Y_MM, yPassosMm, (int8_t)-ySentido};
  return e;
}

// ---------- governo e detecção: comandos YG / YC ----------
static void yComandoGoverno(String arg) {
  if (arg.length()) {
    char sub = arg[0];
    String num = arg.substring(1);
    num.trim();
    long v = num.toInt();
    bool numOk = num.length() > 0;
    switch (sub) {
      case 'Q':
        if (!numOk || v < 1 || v > 20) {
          Serial.printf("quedas invalidas: '%s' (1 a 20)\n", num.c_str());
          return;
        }
        yGovQuedas = (uint8_t)v;
        break;
      case 'A':
        if (!numOk || v < 5 || v > 100) {
          Serial.printf("fator de aceleracao invalido: '%s' (5 a 100 %%)\n", num.c_str());
          return;
        }
        yGovSubidaPct = (uint8_t)v;
        break;
      case 'D':
        if (!numOk || v < 10 || v > 95) {
          Serial.printf("fator de desaceleracao invalido: '%s' (10 a 95 %%)\n", num.c_str());
          return;
        }
        yGovReduzirPct = (uint8_t)v;
        break;
      case 'S':
        if (!numOk || v < (long)Y_GOV_SEGURAR_MS_MIN || v > 30000) {
          Serial.printf("tempo invalido: '%s' (%lu a 30000 ms)\n", num.c_str(),
                        (unsigned long)Y_GOV_SEGURAR_MS_MIN);
          return;
        }
        yGovSegurarMs = (uint32_t)v;
        break;
      case '0':
        yGovQuedas = Y_GOV_QUEDAS_PADRAO;
        yGovSubidaPct = Y_GOV_SUBIDA_PCT_PADRAO;
        yGovReduzirPct = Y_GOV_REDUZIR_PCT_PADRAO;
        yGovSegurarMs = Y_GOV_SEGURAR_MS_MIN;
        break;
      default:
        Serial.printf("comando invalido: 'YG%s' (YG, YGQ2, YGA50, YGD70, YGS1000, YG0)\n", arg.c_str());
        return;
    }
    GovCfg c = yGovCfg();
    gov_configurar(&govY, &c);
    yGovSalvar();
  }
  yGovMostrar();
}

static void yComandoDeteccao(String arg) {
  if (arg == "0" || arg == "1") {
    yDeteccao = (arg == "1");
    yAplicarTrava();
    if (!yDeteccao) {
      GovCfg c = yGovCfg();
      gov_init(&govY, &c, yContMm / yPassosMm);  // solta a rotação reduzida
      yGovAplicar();
      ySemSinal = false;
      Serial.println("-- deteccao do Y DESLIGADA: sem governo, sem 'sem sinal' e sem travamento do homing (YC1 liga) --");
    }
  } else if (arg.length()) {
    Serial.printf("comando invalido: 'YC%s' (YC, YC0, YC1)\n", arg.c_str());
    return;
  }
  Serial.printf("deteccao Y: %s; rotacao permitida %lu passos/s; %lu reducao(oes) desde o boot%s\n",
                yDeteccao ? "ligada" : "DESLIGADA", (unsigned long)gov_limite(&govY),
                (unsigned long)gov_quedas(&govY), ySemSinal ? "; ENCODER SEM SINAL" : "");
}

// ---------- comandos ----------
void yParar() {
  malha_cancelar(&malhaY);
  stepper_stop(&motY);
}

void ySoltar() {
  malha_cancelar(&malhaY);
  stepper_release(&motY);
  yAndando = false;
}

void yComando(String resto) {
  resto.trim();
  resto.toUpperCase();
  if (resto.length() == 0) {
    yMostrar();
    return;
  }
  char sub = resto[0];
  String arg = resto.substring(1);
  arg.trim();

  // ---- não mexem no motor ----
  if (sub == 'G') {
    yComandoGoverno(arg);
    return;
  }
  if (sub == 'C' && !arg.startsWith("AL")) {
    yComandoDeteccao(arg);
    return;
  }
  if (sub == 'E' && arg.length() == 0) {
    yMonitor = !yMonitor;
    yAbAnterior = 0xFF;
    Serial.println(yMonitor ? "-- monitor do encoder Y ligado --" : "-- monitor do encoder Y desligado --");
    return;
  }
  if (sub == 'S' && arg.length() == 0) {
    yMostrarChave();
    return;
  }
  if (sub == 'K') {
    if (arg == "0") {
      yPassosMm = Y_PASSOS_MM_NOM;
      yContMm = Y_CONT_MM_NOM;
      yCalibSalvar();
      Serial.println("-- calibracao do Y voltou ao nominal --");
    }
    yCalibMostrar();
    return;
  }
  if (sub == 'M') {
    yReceberMedida(arg);
    return;
  }
  if (sub == 'Q') {
    yParar();
    Serial.println("-- Y freando --");
    return;
  }
  if (sub == 'L') {
    ySoltar();
    Serial.println("-- Y parado e solto --");
    return;
  }
  if (sub == 'V' || sub == 'A' || sub == 'N') {
    StepperPerfil p;
    stepper_get_profile(&motY, &p);
    if (arg.length()) {
      float v = lerNumero(arg);
      if (sub == 'V') {
        float max = Y_VEL_LIMITE / yPassosMm;
        if (!(v > 0.0f)) {
          Serial.printf("setpoint invalido: 'YV%s' (use 0 < mm/s <= %.2f)\n", arg.c_str(), max);
          return;
        }
        if (v > max) v = max;
        yVelocidade = (uint32_t)lroundf(v * yPassosMm);
        if (yVelocidade < 1) yVelocidade = 1;
        p.vmax = yVelocidade;
      } else if (sub == 'A') {
        if (v < 0) v = 0;
        if (v > Y_ACEL_LIMITE) {
          Serial.printf("acima do limite (%lu passos/s2): ficou no limite\n", (unsigned long)Y_ACEL_LIMITE);
          v = Y_ACEL_LIMITE;
        }
        p.acel = (uint32_t)v;
      } else {
        if (!(v >= 1.0f)) {
          Serial.printf("partida invalida: 'YN%s' (1 a %lu passos/s)\n", arg.c_str(), (unsigned long)Y_VEL_LIMITE);
          return;
        }
        if (v > Y_VEL_LIMITE) v = Y_VEL_LIMITE;
        p.v0 = (uint32_t)v;
      }
      stepper_set_profile(&motY, &p);
    }
    if (sub == 'V') {
      Serial.printf("setpoint Y: %.2f mm/s (%lu passos/s); encoder %.2f mm/s\n", p.vmax / yPassosMm,
                    (unsigned long)p.vmax, gov_vel_real(&govY) / yPassosMm);
    }
    yMostrarRampa();
    return;
  }
  if (sub == 'I') {
    if (stepper_busy(&motY)) {
      Serial.println("Y andando: espere parar (YQ)");
      return;
    }
    yComandoBobinas(arg);
    return;
  }

  // ---- daqui para baixo mexe no motor Y ----
  if (yOcupado()) {
    Serial.printf("Y andando (%lu passos/s): YQ para, YL solta\n", (unsigned long)stepper_speed_now(&motY));
    return;
  }
  stepper_set_speed(&motY, yVelocidade);
  if (sub == 'H') {
    if (arg.length()) {
      yComandoHoming(arg);
    } else {
      yHoming();
    }
    return;
  }
  if (sub == 'C') {  // YCAL3
    yAutoCalibrar(arg.substring(2));
    return;
  }
  if (sub == 'D') {
    yCalibrarDistancia(arg);
    return;
  }
  if (sub == 'E' && arg.startsWith("D")) {  // YED: diagnóstico do encoder
    yDiagEncoder(arg.substring(1));
    return;
  }
  if (sub == 'T') {
    if (arg.startsWith("V")) {
      yMedirTempo(arg.substring(1));
    } else {
      yTeste();
    }
    return;
  }
  if (sub == 'W') {
    yVarrer(arg);
    return;
  }
  if (sub == 'P') {
    yGravarCurva(arg);
    return;
  }
  if (sub == 'R') {
    encoder_reset(&encY, yMmParaCont(Y_HOME_MM));
    stepper_reset(&motY, 0);
    yRef = true;
    malha_cancelar(&malhaY);
    Serial.printf("-- Y referenciado a mao: aqui e Y = %.2f mm --\n", Y_HOME_MM);
    yImprimir();
    return;
  }
  if (sub == 'Z') {
    encoder_reset(&encY, 0);
    stepper_reset(&motY, 0);
    yRef = false;
    malha_cancelar(&malhaY);
    Serial.println("-- Y zerado (motor e encoder): referencia perdida, faca o YH --");
    yImprimir();
    return;
  }
  if (sub == 'F' || sub == 'B') {
    float voltas = arg.length() ? lerNumero(arg) : 1.0f;
    if (!(voltas > 0.0f) || voltas > 30.0f) {
      Serial.printf("voltas invalidas: '%s' (0 a 30)\n", arg.c_str());
      return;
    }
    malha_cancelar(&malhaY);
    stepper_start(&motY, (sub == 'F' ? 1 : -1) * (int32_t)lroundf(voltas * Y_PASSOS_VOLTA), true);
    yAndando = true;
    Serial.printf("Y girando %.2f volta(s) em '%c' (YQ para)\n", voltas, sub == 'F' ? 'f' : 'b');
    return;
  }
  if (sub == '+' || sub == '-') {
    float d = lerNumero(arg);
    if (!(d > 0.0f) || d > CURSO_Y_MM) {
      Serial.printf("deslocamento invalido: 'Y%s' (use Y+mm ou Y-mm, ate %.0f)\n", resto.c_str(), CURSO_Y_MM);
      return;
    }
    if (sub == '-') d = -d;
    if (yRef && !yDentro(yMm() + d)) {
      Serial.printf("fora do curso: Y iria para %.2f mm (%.0f a %.0f)\n", yMm() + d, Y_HOME_MM,
                    Y_HOME_MM + CURSO_Y_MM);
      return;
    }
    int32_t alvo = encoder_count(&encY) + (int32_t)lroundf(-ySentido * d * yContMm);
    if (yIrCont(alvo)) {
      yAndando = true;
      Serial.printf("Y %s %.2f mm pelo encoder (alvo %ld cont; YQ para)%s\n", d > 0 ? "anda" : "volta",
                    fabsf(d), (long)alvo, yRef ? "" : " (sem homing: sem limite de curso!)");
    } else {
      yImprimir();
    }
    return;
  }
  if (isDigit(sub) || sub == '.') {
    if (!yRef) {
      Serial.println("Y sem referencia: faca o homing (YH) ou YR; Y+mm / Y-mm valem sem homing");
      return;
    }
    float mm = lerNumero(resto);
    if (!yDentro(mm)) {
      Serial.printf("posicao invalida: 'Y%s' (use Y de %.0f a %.0f mm)\n", resto.c_str(), Y_HOME_MM,
                    Y_HOME_MM + CURSO_Y_MM);
      return;
    }
    if (yIrCont(yMmParaCont(mm))) {
      yAndando = true;
      Serial.printf("indo para Y = %.2f mm pelo encoder (YQ para)\n", mm);
    } else {
      yImprimir();
    }
    return;
  }
  Serial.printf("comando Y invalido: 'Y%s' (envie Y? ou veja a ajuda no boot)\n", resto.c_str());
}

void yAjuda() {
  Serial.printf("Y (encoder, %.0f a %.0f mm): YH homing, Y120 vai, Y+10 / Y-1 anda/volta pelo encoder,"
                " YR = Y %.0f aqui, YZ zera,\n", Y_HOME_MM, Y_HOME_MM + CURSO_Y_MM, Y_HOME_MM);
  Serial.println("           YV1.5 setpoint mm/s, YA1000 aceleracao, YN300 partida, YQ freia, YL solta,"
                 " YF1/YB1 voltas,");
  Serial.println("           YHV500/YHL150/YHT256/YHS+1 ajustes do homing, YHD diagnostico, YS chave,");
  Serial.println("           YD100 / YM99.6 calibra distancia, YK / YK0 calibracao, YCAL3 auto calibracao,");
  Serial.println("           YI1..YI4 / YIP / YIO / YI0 bobinas, YE monitor do encoder, YED diagnostico do encoder, YTV1 tempo,");
  Serial.println("           YT teste, YW varredura, YP30 curva (CSV), YG governo, YC / YC0 / YC1 deteccao");
}

// ---------- setup / loop ----------
void yIniciar() {
  stepper_begin(&motY, MOT_Y_IN1, MOT_Y_IN2, MOT_Y_IN3, MOT_Y_IN4);
  StepperPerfil p = {Y_PARTIDA, yVelocidade, Y_ACEL_LIMITE};
  stepper_set_profile(&motY, &p);
  encoder_begin(&encY, ENC_Y_A, ENC_Y_B, false);  // KY-040 já tem os resistores
  eixo_begin(&eixoY, &motY, &encY, HOME_Y, false, false);
  pinMode(HOME_Y, INPUT);  // GPIO 39: só entrada, sem pull interno (pull-down externo)
  yChaveAnterior = eixo_chave_crua(&eixoY);
  yCalibCarregar();

  homY.sentido = ySentido;
  homY.fsm.curso_max = 30 * Y_PASSOS_VOLTA;
  homY.fsm.recuo = Y_PASSOS_VOLTA / 4;
  homY.fsm.recuo_max = 4 * Y_PASSOS_VOLTA;
  homY.vel_rapida = 500;
  homY.vel_lenta = 150;
  homY.segurar = false;  // TR8x8 trava sozinho: solta (não aquece)
  homY.cancelar = yCancelar;
  homY.evento = yAvisoHoming;
  homY.ctx = nullptr;
  yAplicarTrava();       // 0 = não para por encoder (termina pela chave ou pelo curso)

  MalhaCfg mc = {Y_MALHA_TOL_CONT, Y_MALHA_CORRECOES};
  malha_init(&malhaY, &mc, yPassosMm / yContMm);
  yGovCarregar();
  GovCfg gc = yGovCfg();
  gov_init(&govY, &gc, yContMm / yPassosMm);
  stepper_set_limit(&motY, gov_limite(&govY));
  yGovAplicado = gov_limite(&govY);
  yEncUltima = encoder_count(&encY);

  Serial.printf("eixo Y: motor GPIO %u %u %u %u, encoder GPIO %u/%u, chave GPIO %u (%s);"
                " curso %.0f a %.0f mm pelo encoder (YH homing)\n",
                MOT_Y_IN1, MOT_Y_IN2, MOT_Y_IN3, MOT_Y_IN4, ENC_Y_A, ENC_Y_B, HOME_Y,
                yChaveAnterior ? "ACIONADA" : "solta", Y_HOME_MM, Y_HOME_MM + CURSO_Y_MM);
  yCalibMostrar();
  yMostrarRampa();
  yMostrarHoming();
  yGovMostrar();
  yImprimir();
}

void yLoop() {
  if (yMonitor) {
    uint8_t ab = (uint8_t)((digitalRead(ENC_Y_A) << 1) | digitalRead(ENC_Y_B));
    if (ab != yAbAnterior) {
      yAbAnterior = ab;
      Serial.printf("[encoder Y] A=%u B=%u   contagem: %ld   erros: %lu\n", (ab >> 1) & 1, ab & 1,
                    (long)encoder_count(&encY), (unsigned long)encoder_errors(&encY));
    }
  }
  uint32_t agora = millis();
  if (agora - yChaveUltima >= 5) {
    yChaveUltima = agora;
    bool c = eixo_chave(&eixoY);
    if (c != yChaveAnterior) {
      yChaveAnterior = c;
      Serial.println(c ? "[sensor] homing Y APERTADO" : "[sensor] homing Y solto");
    }
  }
  yAtualizarMalha();
  yGovernar();
  yTelemetria();
  if (yAndando && !yOcupado()) {
    yAndando = false;
    Serial.print("-- Y parou -- ");
    yImprimir();
  }
}
