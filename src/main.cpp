// =============================================================
//  Bancada de teste: motor X + encoder X
// =============================================================
//
// Comandos pelo Monitor Serial (115200), sempre terminados com Enter:
//   f  -> uma volta para frente   (+2048 passos)
//   b  -> uma volta para trás     (-2048 passos)
//   f3, b0.5, f 2.25 ... -> número de voltas depois da letra
//         (aceita fração; sem número = 1 volta; máximo VOLTAS_MAX)
//   v300 -> velocidade em passos/s (começa em VELOCIDADE_MAX); 'v' só mostra
//         f e b soltam as bobinas sozinhos ao terminar (motor não aquece parado)
//   l  -> solta as bobinas (útil depois do testador de bobinas)
//   r  -> zera motor e encoder
//
// Homing e auto calibração (chave de fim de curso no GPIO 36, lado 'f'):
//   h    -> homing: busca rápida em 'f', recua, busca lenta; zera na chave
//   c, c3 -> auto calibração: homing, depois N ciclos de
//           (CAL_VOLTAS no sentido b + homing). Sem número = 1 ciclo.
//   O lado da chave fica em homingX.sentido (configurarHoming).
//   s    -> mostra a chave (leitura direta do pino, para conferir a fiação)
//   Qualquer tecla + Enter durante h/c cancela.
//   O homing imprime cada mudança de estado (com o giro: f ou b) e
//   cada vez que o sensor é apertado/solto. Fora do homing, apertar ou
//   soltar o sensor também aparece na serial.
//   Com um fio no lugar da chave: encoste no 3V3 (apertou), TIRE para o
//   motor recuar até 'solto', e encoste de novo na busca lenta.
//
// Calibração de distância (fuso TR8x8: 8 mm/volta, 256 passos/mm nominal):
//   d, d100 -> homing, afasta 2 mm (tira a folga), espera 5 s para marcar
//              a posição do carro e anda 100 mm (ou o valor pedido)
//   m99.6  -> informa a distância REAL medida desde a marca (aceita vírgula);
//              calcula e SALVA os novos passos/mm e contagens/mm
//   k      -> mostra a calibração; k0 volta ao nominal
//   x50    -> vai para 50 mm da chave (precisa de homing); 'x' só mostra
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
#include <Preferences.h>
#include "pins.h"
#include "eixo.h"
#include "encoder.h"
#include "stepper.h"

static const int32_t PASSOS_POR_VOLTA = 2048;  // 28BYJ-48, passo completo
// Acima do limite típico do 28BYJ-48 sem rampa (~500): pode perder
// passo ou só vibrar. Se travar na bancada, baixe com 'v' (ex.: v500).
static const uint32_t VELOCIDADE_MAX = 600;  // passos/s (~17,6 RPM)
// stepper_move() bloqueia: cada volta trava o loop por ~3,4 s.
static const float VOLTAS_MAX = 20.0f;

// Encoder KY-040: 20 ciclos x 4 = 80 contagens por volta.
static const int32_t CONTAGENS_POR_VOLTA = 80;
// Auto calibração: quanto recua entre um homing e outro.
static const int32_t CAL_VOLTAS = 10;
static const uint32_t CAL_CICLOS_MAX = 50;

// Fuso TR8x8: avanço de 8 mm por volta do motor.
static const float PASSO_FUSO_MM = 8.0f;
static const float PASSOS_MM_NOMINAL = PASSOS_POR_VOLTA / PASSO_FUSO_MM;        // 256
static const float CONTAGENS_MM_NOMINAL = CONTAGENS_POR_VOLTA / PASSO_FUSO_MM;  // 10

// Calibração de distância.
static const float DIST_MM_PADRAO = 100.0f;
static const float DIST_PRE_MM = 2.0f;           // afasta antes de marcar: tira a folga
static const uint32_t DIST_ESPERA_MS = 5000;     // tempo para marcar a posição do carro
static const float DIST_TOLERANCIA_PCT = 10.0f;  // medida mais longe que isso: recusa
// Curso útil do eixo X a partir da chave. AJUSTAR ao eixo montado.
static const float CURSO_X_MM = 200.0f;

Encoder encX;
Stepper motX;
Eixo eixoX;
static EixoHomingCfg homingX;
// Último estado do sensor já avisado na serial (fora do homing).
static bool chaveAnterior = false;

// Depois de h/c: o homing reiniciou o debounce da chave; sincroniza para
// o aviso fora do homing não repetir o que o homing já mostrou.
static void sincronizarChave() { chaveAnterior = debounce_level(&eixoX.chave) != 0; }

// ---------- calibração em mm (salva na flash, vale depois do reset) ----------
static Preferences prefs;
static float passosPorMm = PASSOS_MM_NOMINAL;
static float contagensPorMm = CONTAGENS_MM_NOMINAL;
static bool referenciado = false;  // homing feito: a posição em mm vale

// Medida esperando a resposta 'm' depois do 'd'.
static bool medidaPendente = false;
static int32_t pendPassos = 0;
static int32_t pendContagens = 0;
static float pendMm = 0.0f;

// mm a partir da chave, positivo afastando dela.
static float posicaoMm() {
  return (float)(-homingX.sentido * stepper_position(&motX)) / passosPorMm;
}
static float encoderMm() {
  return (float)(-homingX.sentido * encoder_count(&encX)) / contagensPorMm;
}

// Cancela h/c: qualquer tecla que não seja o fim da linha do próprio comando.
static bool cancelarPelaSerial(void *) {
  while (Serial.available()) {
    int c = Serial.read();
    if (c != '\r' && c != '\n') return true;
  }
  return false;
}

static const char *giroNome(int8_t g) { return g > 0 ? "f" : (g < 0 ? "b" : "-"); }

// Impressão dos avisos do homing (chamada de dentro do eixo_home).
static void avisoHoming(const EixoEvento *ev, void *) {
  if (ev->tipo == EIXO_EV_CHAVE) {
    Serial.printf("[homing] SENSOR %s  (motor %ld, encoder %ld)\n",
                  ev->chave ? "APERTADO" : "solto", (long)ev->motor, (long)ev->enc);
    return;
  }
  if (ev->inicio && ev->chave) {
    Serial.println("[homing] sensor JA APERTADO na partida: recua primeiro");
  }
  Serial.printf("[homing] estado: %s, girando '%s'  (motor %ld, encoder %ld)\n",
                homing_state_nome(ev->estado), giroNome(ev->giro), (long)ev->motor,
                (long)ev->enc);
  if (ev->estado == HOM_RECUO) Serial.println("[homing]   -> esperando o sensor soltar");
  if (ev->estado == HOM_BUSCA_LENTA) Serial.println("[homing]   -> aperte o sensor de novo");
}

static void configurarHoming() {
  homingX.sentido = +1;  // chave no lado 'f' (giro do comando f). Lado 'b': -1
  homingX.fsm.curso_max = 30 * PASSOS_POR_VOLTA;  // maior que o curso do eixo
  homingX.fsm.recuo = PASSOS_POR_VOLTA / 4;       // 1/4 de volta além de soltar
  homingX.fsm.recuo_max = 4 * PASSOS_POR_VOLTA;   // a chave tem que soltar antes disso
  homingX.fsm.janela_trava = 256;                 // ~10 contagens esperadas sem mudar = travou
  homingX.vel_rapida = 500;                       // passos/s
  homingX.vel_lenta = 150;                        // passos/s: define a precisão da origem
  homingX.cancelar = cancelarPelaSerial;
  homingX.evento = avisoHoming;
  homingX.ctx = nullptr;
}

static void imprimir() {
  Serial.printf("motor: %6ld   encoder: %5ld   erros: %lu", (long)stepper_position(&motX),
                (long)encoder_count(&encX), (unsigned long)encoder_errors(&encX));
  if (referenciado) Serial.printf("   X: %.2f mm (encoder %.2f mm)", posicaoMm(), encoderMm());
  Serial.println();
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

// Lê o resto da linha depois de 'f'/'b' como número de voltas.
// Linha vazia = 1 volta. Devolve 0 se o número for inválido.
static int32_t lerPassos(String resto) {
  resto.trim();
  if (resto.length() == 0) return PASSOS_POR_VOLTA;
  float voltas = resto.toFloat();  // texto inválido vira 0
  if (voltas <= 0.0f || voltas > VOLTAS_MAX) {
    Serial.printf("voltas invalidas: '%s' (use 0 < voltas <= %.0f)\n",
                  resto.c_str(), VOLTAS_MAX);
    return 0;
  }
  int32_t passos = lroundf(voltas * PASSOS_POR_VOLTA);
  Serial.printf("%.2f volta(s) = %ld passos\n", voltas, (long)passos);
  return passos;
}

// 'v' + passos/s, ex.: v300. Sem número mostra a velocidade atual.
static uint32_t velocidade = VELOCIDADE_MAX;

static void ajustarVelocidade(String resto) {
  resto.trim();
  if (resto.length() > 0) {
    long v = resto.toInt();
    if (v <= 0) {
      Serial.printf("velocidade invalida: '%s'\n", resto.c_str());
      return;
    }
    velocidade = (uint32_t)v;
    stepper_set_speed(&motX, velocidade);
  }
  Serial.printf("velocidade: %lu passos/s (%.1f RPM)\n", (unsigned long)velocidade,
                velocidade * 60.0f / PASSOS_POR_VOLTA);
}

static void imprimirHome(const EixoHomeResult &r) {
  if (r.erro == HOM_ERRO_NENHUM) {
    Serial.printf("homing ok em %lu passos\n", (unsigned long)r.passos_dados);
  } else {
    Serial.printf("homing FALHOU (%s) no estado '%s', depois de %lu passos\n",
                  homing_error_nome(r.erro), homing_state_nome(r.parou_em),
                  (unsigned long)r.passos_dados);
  }
}

static void fazerHoming() {
  Serial.println("-- homing (Enter com qualquer tecla cancela) --");
  EixoHomeResult r;
  eixo_home(&eixoX, &homingX, &r);
  stepper_set_speed(&motX, velocidade);  // o homing usa as velocidades dele
  sincronizarChave();
  referenciado = (r.erro == HOM_ERRO_NENHUM);
  imprimirHome(r);
  if (r.erro == HOM_ERRO_NENHUM) {
    Serial.printf("chave acionou em %ld passos / %ld contagens da origem anterior\n",
                  (long)r.passos_na_chave, (long)r.enc_na_chave);
  }
}

static float graus(float passos) { return passos * 360.0f / PASSOS_POR_VOLTA; }

// Ida = afastando da chave. 'f' conta positivo no encoder.
static int32_t enc_ida_esperado() {
  return -homingX.sentido * CAL_VOLTAS * CONTAGENS_POR_VOLTA;
}

static void avisoCiclo(const EixoCalibCiclo *c, void *) {
  if (c->ciclo == 0) {
    Serial.print("ciclo 0 (origem): ");
    imprimirHome(c->home);
    return;
  }
  Serial.printf("ciclo %lu: ida %ld contagens (esperado %ld) | ", (unsigned long)c->ciclo,
                (long)c->enc_ida, (long)enc_ida_esperado());
  if (c->home.erro == HOM_ERRO_NENHUM) {
    Serial.printf("chave em %+ld passos, encoder %+ld\n", (long)c->home.passos_na_chave,
                  (long)c->home.enc_na_chave);
  } else {
    imprimirHome(c->home);
  }
}

static void calibrar(String resto) {
  resto.trim();
  long ciclos = resto.length() ? resto.toInt() : 1;
  if (ciclos <= 0 || ciclos > (long)CAL_CICLOS_MAX) {
    Serial.printf("ciclos invalidos: '%s' (use 1..%lu)\n", resto.c_str(),
                  (unsigned long)CAL_CICLOS_MAX);
    return;
  }
  Serial.printf("-- auto calibracao: %ld ciclo(s) de %ld voltas (Enter com qualquer tecla cancela) --\n",
                ciclos, (long)CAL_VOLTAS);

  EixoCalibResumo r;
  eixo_calibrar(&eixoX, &homingX, CAL_VOLTAS * PASSOS_POR_VOLTA, (uint32_t)ciclos,
                avisoCiclo, nullptr, &r);
  stepper_set_speed(&motX, velocidade);
  sincronizarChave();
  referenciado = (r.erro == HOM_ERRO_NENHUM);

  Serial.printf("== resumo: %lu de %ld ciclo(s) ok%s ==\n", (unsigned long)r.ciclos_ok, ciclos,
                r.erro == HOM_ERRO_NENHUM ? "" : " (parou por erro)");
  if (r.ciclos_ok == 0) return;
  const Estat &p = r.passos_na_chave;
  const Estat &e = r.enc_na_chave;
  const Estat &i = r.enc_ida;
  Serial.printf("chave (passos):    min %+ld  max %+ld  media %+.1f  -> variacao %ld passos (%.2f graus)\n",
                (long)p.min, (long)p.max, estat_media(&p), (long)estat_faixa(&p),
                graus((float)estat_faixa(&p)));
  Serial.printf("encoder na chave:  min %+ld  max %+ld  media %+.1f contagens\n",
                (long)e.min, (long)e.max, estat_media(&e));
  Serial.printf("encoder por volta: %.2f contagens (esperado %ld)\n",
                estat_media(&i) * -homingX.sentido / CAL_VOLTAS, (long)CONTAGENS_POR_VOLTA);
}

static void mostrarChave() {
  // Leitura direta do pino: serve para conferir a fiação apertando a chave.
  Serial.printf("chave: %s (GPIO %u = %s)\n", eixo_chave_crua(&eixoX) ? "ACIONADA" : "solta",
                HOME_X, digitalRead(HOME_X) ? "HIGH" : "LOW");
}

// ---------- calibração de distância ----------

static void carregarCalibracao() {
  prefs.begin("calib", false);
  float p = prefs.getFloat("x_ppmm", PASSOS_MM_NOMINAL);
  float c = prefs.getFloat("x_cpmm", CONTAGENS_MM_NOMINAL);
  prefs.end();
  passosPorMm = (p > 0.0f) ? p : PASSOS_MM_NOMINAL;
  contagensPorMm = (c > 0.0f) ? c : CONTAGENS_MM_NOMINAL;
}

static void salvarCalibracao() {
  prefs.begin("calib", false);
  prefs.putFloat("x_ppmm", passosPorMm);
  prefs.putFloat("x_cpmm", contagensPorMm);
  prefs.end();
}

static void mostrarCalibracao() {
  Serial.printf("calibracao X: %.3f passos/mm (nominal %.1f), %.3f contagens/mm (nominal %.1f)\n",
                passosPorMm, PASSOS_MM_NOMINAL, contagensPorMm, CONTAGENS_MM_NOMINAL);
}

// Aceita "99,6" e "99.6".
static float lerMm(String s) {
  s.replace(',', '.');
  s.trim();
  return s.toFloat();
}

// Espera 'ms' segurando a posição; false se cancelarem pela serial.
static bool esperar(uint32_t ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    if (cancelarPelaSerial(nullptr)) return false;
    delay(10);
  }
  return true;
}

static void calibrarDistancia(String resto) {
  resto.trim();
  float mm = resto.length() ? lerMm(resto) : DIST_MM_PADRAO;
  if (!(mm > 0.0f) || DIST_PRE_MM + mm > CURSO_X_MM) {
    Serial.printf("distancia invalida: '%s' (use 0 < mm <= %.0f)\n", resto.c_str(),
                  CURSO_X_MM - DIST_PRE_MM);
    return;
  }
  medidaPendente = false;
  Serial.printf("-- calibracao de distancia: %.1f mm (Enter com qualquer tecla cancela) --\n", mm);

  EixoHomeResult h;
  eixo_home(&eixoX, &homingX, &h);
  sincronizarChave();
  imprimirHome(h);
  referenciado = (h.erro == HOM_ERRO_NENHUM);
  if (!referenciado) {
    stepper_set_speed(&motX, velocidade);
    return;
  }

  const int8_t afasta = (int8_t)-homingX.sentido;
  bool ok = eixo_andar(&eixoX, &homingX, afasta * (int32_t)lroundf(DIST_PRE_MM * passosPorMm));
  if (ok) {
    Serial.printf(">> MARQUE a posicao do carro agora. Andando %.1f mm em %lu s...\n", mm,
                  (unsigned long)(DIST_ESPERA_MS / 1000));
    ok = esperar(DIST_ESPERA_MS);
  }
  int32_t passos = (int32_t)lroundf(mm * passosPorMm);
  int32_t enc0 = encoder_count(&encX);
  if (ok) ok = eixo_andar(&eixoX, &homingX, afasta * passos);
  delay(5);  // deixa a última transição passar pelo debounce do encoder
  int32_t contagens = encoder_count(&encX) - enc0;
  stepper_release(&motX);
  stepper_set_speed(&motX, velocidade);
  if (!ok) {
    Serial.println("-- calibracao de distancia cancelada --");
    imprimir();
    return;
  }

  medidaPendente = true;
  pendPassos = passos;
  pendContagens = contagens;
  pendMm = mm;
  Serial.printf("andou %ld passos; encoder: %ld contagens (%.2f mm pelo encoder atual)\n",
                (long)passos, (long)contagens, fabsf(contagens / contagensPorMm));
  Serial.println(">> MECA o deslocamento real desde a marca e envie: m<mm>  (ex.: m99.6)");
  imprimir();
}

static void receberMedida(const String &resto) {
  if (!medidaPendente) {
    Serial.println("nada para calibrar: use d (ex.: d100) primeiro");
    return;
  }
  float medido = lerMm(resto);
  CalibDist r;
  CalibDistErro e = calib_distancia(pendPassos, pendContagens, pendMm, medido,
                                    DIST_TOLERANCIA_PCT,
                                    (float)CONTAGENS_POR_VOLTA / PASSOS_POR_VOLTA, &r);
  if (e == CAL_DIST_ENTRADA_INVALIDA) {
    Serial.printf("%s: '%s'. Envie m<mm>, ex.: m99.6\n", calib_dist_erro_nome(e), resto.c_str());
    return;
  }
  Serial.printf("pedido %.2f mm, medido %.2f mm -> erro %+.2f%%\n", pendMm, medido, r.erro_pct);
  Serial.printf("encoder viu %.1f%% do giro do motor\n", r.encoder_pct);
  if (r.encoder_pct < 95.0f || r.encoder_pct > 105.0f) {
    Serial.println("   atencao: encoder longe de 100% (perdendo ou sobrando contagens)");
  }
  if (e == CAL_DIST_FORA_DA_TOLERANCIA) {
    Serial.printf("NAO aplicado: %s (limite +-%.0f%%). Envie m<mm> de novo.\n",
                  calib_dist_erro_nome(e), DIST_TOLERANCIA_PCT);
    return;
  }

  float pAnt = passosPorMm;
  float cAnt = contagensPorMm;
  passosPorMm = r.passos_por_mm;
  contagensPorMm = r.contagens_por_mm;
  salvarCalibracao();
  medidaPendente = false;
  Serial.printf("passos/mm:    %.3f -> %.3f\n", pAnt, passosPorMm);
  Serial.printf("contagens/mm: %.3f -> %.3f\n", cAnt, contagensPorMm);
  Serial.println("salvo na memoria (vale depois do reset). Confira com x<mm>, ex.: x50");
  imprimir();
}

static void comandoCalibracao(String resto) {
  resto.trim();
  if (resto == "0") {
    passosPorMm = PASSOS_MM_NOMINAL;
    contagensPorMm = CONTAGENS_MM_NOMINAL;
    salvarCalibracao();
    Serial.println("-- calibracao voltou ao nominal (TR8x8) --");
  }
  mostrarCalibracao();
}

static void irPara(String resto) {
  if (!referenciado) {
    Serial.println("sem referencia: faca o homing primeiro (h ou d)");
    return;
  }
  resto.trim();
  if (resto.length() > 0) {
    float mm = lerMm(resto);
    if (!(mm >= 0.0f) || mm > CURSO_X_MM) {
      Serial.printf("posicao invalida: '%s' (use 0..%.0f mm)\n", resto.c_str(), CURSO_X_MM);
      return;
    }
    int32_t alvo = (int32_t)lroundf(-homingX.sentido * mm * passosPorMm);
    int32_t delta = alvo - stepper_position(&motX);
    if (delta != 0) {
      stepper_move(&motX, delta);
      stepper_release(&motX);
    }
  }
  imprimir();
}

static void mover(int32_t sentido, const String &resto) {
  int32_t passos = lerPassos(resto);
  if (passos == 0) return;
  stepper_move(&motX, sentido * passos);
  stepper_release(&motX);  // parado não precisa de corrente: evita aquecer
  imprimir();
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println("== Teste do motor X ==");
  Serial.println("f = volta p/ frente, b = volta p/ tras, l = solta, r = zera");
  Serial.println("f3 / b0.5 = numero de voltas (sem numero = 1), v300 = passos/s");
  Serial.println("teste de bobinas: 1..4 = so uma entrada, p/o = proxima/anterior fase");
  Serial.println("e = liga/desliga monitor do encoder (A e B crus)");
  Serial.println("h = homing, c3 = auto calibracao (3 ciclos), s = mostra a chave");
  Serial.println("d100 = calibra distancia, m99.6 = medida real, k = calibracao (k0 = nominal)");
  Serial.println("x50 = vai para 50 mm da chave (depois do homing)");
  Serial.printf("encoder: amostra a cada %u us (%u Hz), debounce %u us = %u amostras\n",
                ENCODER_SAMPLE_PERIOD_US, ENCODER_SAMPLE_RATE_HZ, ENCODER_DEBOUNCE_US,
                ENCODER_DEBOUNCE_SAMPLES);

  encoder_begin(&encX, ENC_X_A, ENC_X_B, false);
  stepper_begin(&motX, MOT_X_IN1, MOT_X_IN2, MOT_X_IN3, MOT_X_IN4);
  // Chave ativa em HIGH: chave NA para 3V3 e pull-down de 10 kΩ externo
  // para GND (o GPIO 36 não tem pull interno). false = ativa em HIGH.
  eixo_begin(&eixoX, &motX, &encX, HOME_X, false, false);
  configurarHoming();
  carregarCalibracao();
  mostrarCalibracao();
  chaveAnterior = eixo_chave_crua(&eixoX);
  Serial.printf("sensor do homing: %s\n", chaveAnterior ? "APERTADO" : "solto");
  stepper_set_speed(&motX, velocidade);
  ajustarVelocidade("");  // vazio: só mostra
  imprimir();
}

// Executa uma linha completa: a letra é o comando, o resto é o número.
static void executar(const String &linha) {
  char c = linha[0];
  String resto = linha.substring(1);
  switch (c) {
    case 'f': mover(+1, resto); return;   // ~3,4 s parado por volta
    case 'b': mover(-1, resto); return;
    case 'v': ajustarVelocidade(resto); return;
    case 'h': fazerHoming(); break;
    case 'c': calibrar(resto); break;
    case 's': mostrarChave(); return;
    case 'd': calibrarDistancia(resto); return;
    case 'm': receberMedida(resto); return;
    case 'k': comandoCalibracao(resto); return;
    case 'x': irPara(resto); return;
    case 'l': stepper_release(&motX); Serial.println("-- solto --"); break;
    case 'r':
      stepper_reset(&motX, 0);
      encoder_reset(&encX, 0);
      referenciado = false;  // zero manual: a origem da chave deixou de valer
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
    default: return;  // ignora outras teclas
  }
  imprimir();
}

// Junta os caracteres até o Enter. O monitor do PlatformIO manda cada
// tecla na hora; sem isso o '3' de "v300" virava o testador de bobinas.
static String linha;

// Fora do homing: avisa quando o sensor muda (amostra a cada 5 ms,
// debounce de 3 amostras = 15 ms).
static uint32_t ultimaAmostraChave = 0;

static void vigiarChave() {
  uint32_t agora = millis();
  if (agora - ultimaAmostraChave < 5) return;
  ultimaAmostraChave = agora;
  bool c = eixo_chave(&eixoX);
  if (c == chaveAnterior) return;
  chaveAnterior = c;
  Serial.println(c ? "[sensor] homing APERTADO" : "[sensor] homing solto");
}

void loop() {
  if (monitorEncoder) monitorarEncoder();
  vigiarChave();
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r' || c == '\n') {   // aceita LF, CR e CRLF
      linha.trim();
      if (linha.length() > 0) executar(linha);
      linha = "";
    } else if (linha.length() < 32) {
      linha += c;
    }
  }
}
