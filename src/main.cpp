// =============================================================
//  Bancada de teste: motor X + encoder X
// =============================================================
//
// Comandos pelo Monitor Serial (115200), sempre terminados com Enter:
// O motor anda SEM BLOQUEAR (interrupção de timer, lib/stepper) e COM
// RAMPA (lib/rampa): f, b e x voltam na hora; a serial continua
// respondendo e avisa quando o movimento termina.
//
//   f  -> uma volta para frente   (+2048 passos)
//   b  -> uma volta para trás     (-2048 passos)
//   f3, b0.5, f 2.25 ... -> número de voltas depois da letra
//         (aceita fração; sem número = 1 volta; máximo VOLTAS_MAX)
//         f e b soltam as bobinas sozinhos ao terminar (motor não aquece parado)
//   q  -> para freando pela rampa;  l -> para na hora e solta as bobinas
//   v800 -> velocidade máxima (passos/s);  a1500 -> aceleração (passos/s²,
//           a0 = sem rampa);  n300 -> velocidade de partida;  'v' mostra tudo
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
// Testes automáticos da rampa (precisam de homing; fazem se faltar):
//   w, w1400 -> varredura: vaivém entre TESTE_A_MM e TESTE_B_MM subindo a
//               vmax (500, 550 ... 800 ou até o número), para na primeira
//               que perder passo (encoder fica para trás) e confere com homing
//   g, g50   -> curva: grava um movimento de 30 mm (ou o valor) a cada 10 ms
//               e imprime CSV: velocidade comandada x medida pelo encoder
//
// Medição de velocidade (o encoder atrapalha o motor?):
//   t, t2  -> N voltas em f com o encoder LIGADO e N voltas em b com o
//             timer do encoder PARADO; compara os tempos com o ideal e
//             mostra o custo da amostragem do encoder. Usa a rampa atual (v, a, n).
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
// Perfil inicial da rampa (ajuste na bancada com v, a, n).
// Medido com 'w' (2026-11-10, a1500): 650 passos/s ok, 700 perde passo, e a
// aceleração (1000 a 2500) não mudou o limite. Trabalho = 80% de 650.
static const uint32_t VEL_MAX_PADRAO = 520;   // passos/s (~15,2 RPM, ~2 mm/s)
static const uint32_t ACEL_PADRAO = 1500;     // passos/s²
static const uint32_t VEL_PARTIDA_PADRAO = 300;  // passos/s
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

// Perfil da rampa usado por f, b, x, d e c (o homing usa o dele).
static uint32_t velocidade = VEL_MAX_PADRAO;  // vmax, passos/s

static void mostrarPerfil() {
  StepperPerfil p;
  stepper_get_profile(&motX, &p);
  Serial.printf("rampa: partida %lu passos/s, maxima %lu passos/s (%.1f RPM), aceleracao %lu passos/s2%s\n",
                (unsigned long)p.v0, (unsigned long)p.vmax, p.vmax * 60.0f / PASSOS_POR_VOLTA,
                (unsigned long)p.acel, p.acel ? "" : " (sem rampa)");
  if (p.acel) {
    float t = (p.vmax > p.v0) ? (float)(p.vmax - p.v0) / p.acel : 0.0f;
    float s = (p.vmax > p.v0) ? ((float)p.vmax * p.vmax - (float)p.v0 * p.v0) / (2.0f * p.acel) : 0.0f;
    Serial.printf("       acelera em %.2f s / %.0f passos (%.2f volta)\n", t, s, s / PASSOS_POR_VOLTA);
  }
}

// Lê um inteiro > 0 (ou >= 0 se 'zero_ok'); false se inválido.
static bool lerInteiro(String resto, bool zero_ok, uint32_t *out) {
  resto.trim();
  if (resto.length() == 0) return false;
  long v = resto.toInt();
  if (v < 0 || (v == 0 && (!zero_ok || resto != "0"))) {
    Serial.printf("valor invalido: '%s'\n", resto.c_str());
    return false;
  }
  *out = (uint32_t)v;
  return true;
}

static void ajustarVelocidade(String resto) {
  uint32_t v;
  if (lerInteiro(resto, false, &v)) {
    velocidade = v;
    stepper_set_speed(&motX, velocidade);
  }
  mostrarPerfil();
}

static void ajustarAceleracao(String resto) {
  uint32_t a;
  if (lerInteiro(resto, true, &a)) {
    StepperPerfil p;
    stepper_get_profile(&motX, &p);
    p.acel = a;
    stepper_set_profile(&motX, &p);
  }
  mostrarPerfil();
}

static void ajustarPartida(String resto) {
  uint32_t v0;
  if (lerInteiro(resto, false, &v0)) {
    StepperPerfil p;
    stepper_get_profile(&motX, &p);
    p.v0 = v0;
    stepper_set_profile(&motX, &p);
  }
  mostrarPerfil();
}

// Movimento sem bloqueio em andamento? Avisa e devolve true.
static bool ocupado() {
  if (!stepper_busy(&motX)) return false;
  Serial.printf("motor andando (%lu passos/s): q para parar, l para soltar\n",
                (unsigned long)stepper_speed_now(&motX));
  return true;
}

// Para avisar quando um movimento sem bloqueio termina.
static bool andando = false;

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
    if (alvo != stepper_position(&motX)) {
      stepper_start_to(&motX, alvo, true);  // volta na hora; solta ao chegar
      andando = true;
      Serial.printf("indo para %.2f mm (q para parar)\n", mm);
      return;
    }
  }
  imprimir();
}

// ---------- medição de velocidade ----------

// Cronometra um movimento (bloqueante) e devolve o tempo em us.
static uint32_t cronometrar(int32_t passos) {
  uint32_t t0 = micros();
  stepper_move(&motX, passos);
  return micros() - t0;
}

static void relatarTrecho(const char *nome, int32_t passos, uint32_t us, uint32_t ideal_us) {
  float real_pps = passos / (us / 1e6f);
  Serial.printf("%-16s %8.3f s  (ideal %.3f s)  -> %7.1f passos/s  %+6.2f%%\n", nome, us / 1e6f,
                ideal_us / 1e6f, real_pps, (float)((int64_t)us - ideal_us) * 100.0f / ideal_us);
}

static void medirVelocidade(String resto) {
  resto.trim();
  float voltas = resto.length() ? lerMm(resto) : 1.0f;
  if (!(voltas > 0.0f) || voltas > 10.0f) {
    Serial.printf("voltas invalidas: '%s' (use 0 < voltas <= 10)\n", resto.c_str());
    return;
  }
  int32_t passos = (int32_t)lroundf(voltas * PASSOS_POR_VOLTA);
  StepperPerfil p;
  stepper_get_profile(&motX, &p);
  // Ideal: a própria rampa simulada (tiques / frequência da interrupção).
  RampaCfg rc = {STEPPER_TICK_HZ, p.v0, p.vmax, p.acel};
  uint32_t ideal = (uint32_t)(rampa_tiques_total(&rc, passos) * 1000000ULL / STEPPER_TICK_HZ);

  Serial.printf("-- medicao: %.2f volta(s) = %ld passos --\n", voltas, (long)passos);
  mostrarPerfil();
  Serial.printf("interrupcao dos motores a %u Hz no nucleo %d (o loop tambem)\n", STEPPER_TICK_HZ,
                xPortGetCoreID());

  // 1) encoder ligado, sentido f
  encoder_stats_reset(&encX);
  int32_t enc0 = encoder_count(&encX);
  uint32_t us_on = cronometrar(passos);
  delay(5);
  int32_t cont_on = encoder_count(&encX) - enc0;
  EncoderStats st;
  encoder_stats(&encX, &st);

  // 2) encoder parado, sentido b (volta ao ponto de partida)
  encoder_stop(&encX);
  uint32_t us_off = cronometrar(-passos);
  encoder_start(&encX);
  stepper_release(&motX);

  // O encoder não contou a volta: realinha com o motor.
  encoder_reset(&encX, (int32_t)lroundf(stepper_position(&motX) * contagensPorMm / passosPorMm));

  relatarTrecho("encoder LIGADO", passos, us_on, ideal);
  relatarTrecho("encoder PARADO", passos, us_off, ideal);
  Serial.printf("diferenca ligado - parado: %+ld us (%+.3f%%)\n", (long)us_on - (long)us_off,
                ((float)us_on - (float)us_off) * 100.0f / us_off);
  Serial.printf("encoder no trecho ligado: %ld contagens (esperado ~%ld)\n", (long)cont_on,
                (long)lroundf(passos * (float)CONTAGENS_POR_VOLTA / PASSOS_POR_VOLTA));

  if (st.amostras > 0) {
    float seg = us_on / 1e6f;
    Serial.printf("amostragem: nucleo %d, %lu amostras (%.0f/s, ideal %u/s)\n", st.core,
                  (unsigned long)st.amostras, st.amostras / seg, ENCODER_SAMPLE_RATE_HZ);
    Serial.printf("            duracao media %.2f us, maxima %lu us; maior intervalo %lu us (ideal %u)\n",
                  (float)st.soma_us / st.amostras, (unsigned long)st.max_us,
                  (unsigned long)st.max_intervalo_us, ENCODER_SAMPLE_PERIOD_US);
    Serial.printf("            ocupa %.3f%% do nucleo %d (so o corpo da amostra, sem o despacho do esp_timer)\n",
                  (float)st.soma_us * 100.0f / us_on, st.core);
  }
  Serial.println("obs.: o encoder ficou parado na volta; a contagem foi realinhada pelo motor");
  imprimir();
}

// ---------- testes automáticos: velocidade e curva da rampa ----------
//
// Medem o ERRO DE SEGUIMENTO: contagens que o encoder leu menos as que
// deveria ter lido pelos passos do motor (contagens/mm ÷ passos/mm). Se o
// motor perde passo, o encoder fica para trás e o erro cresce.

static const float TESTE_A_MM = 10.0f;  // pontos do vaivém (dentro do curso real!)
static const float TESTE_B_MM = 40.0f;
// Na bancada (2026-11-10): 600 ok, 800 travou com qualquer aceleração; passo fino entre eles.
static const uint32_t TESTE_VELS[] = {500, 550, 600, 650, 700, 750, 800};
static const int32_t TESTE_TOL_CONT = 5;  // tolerância: 5 contagens + 2% do esperado
static const float TESTE_TOL_PCT = 2.0f;

struct Amostra {
  uint32_t t_ms;
  uint32_t v;     // velocidade comandada (passos/s)
  int32_t motor;  // passos desde o início
  int32_t enc;    // contagens desde o início
};
static const uint16_t CURVA_MAX = 1000;
static Amostra curva[CURVA_MAX];
static uint16_t curvaN = 0;

struct ResultadoMov {
  bool ok;           // terminou sem cancelar
  uint32_t us;       // tempo real
  uint32_t ideal_us; // tempo da rampa simulada
  int32_t passos;
  int32_t esperado;  // contagens esperadas pelos passos
  int32_t lido;      // contagens lidas
  int32_t erro_max;  // maior |erro de seguimento| durante o movimento
  int32_t erro_fim;
};

static int32_t alvoMm(float mm) { return (int32_t)lroundf(-homingX.sentido * mm * passosPorMm); }
static int32_t modulo32(int32_t v) { return v < 0 ? -v : v; }

// Anda até 'alvo' com o perfil atual, medindo o erro de seguimento.
// Com 'gravar', guarda a curva a cada 'periodo_ms'.
static ResultadoMov medirMovimento(int32_t alvo, bool gravar, uint32_t periodo_ms) {
  ResultadoMov r = {};
  int32_t m0 = stepper_position(&motX);
  int32_t e0 = encoder_count(&encX);
  r.passos = alvo - m0;
  StepperPerfil p;
  stepper_get_profile(&motX, &p);
  RampaCfg rc = {STEPPER_TICK_HZ, p.v0, p.vmax, p.acel};
  r.ideal_us = (uint32_t)(rampa_tiques_total(&rc, r.passos) * 1000000ULL / STEPPER_TICK_HZ);
  const float k = contagensPorMm / passosPorMm;  // contagens por passo

  if (gravar) curvaN = 0;
  r.ok = true;
  uint32_t t0 = micros();
  uint32_t prox = 0;
  stepper_start_to(&motX, alvo, false);
  while (stepper_busy(&motX)) {
    if (cancelarPelaSerial(nullptr)) {
      stepper_abort(&motX);
      r.ok = false;
      break;
    }
    int32_t m = stepper_position(&motX) - m0;
    int32_t e = encoder_count(&encX) - e0;
    int32_t erro = e - (int32_t)lroundf(m * k);
    if (modulo32(erro) > r.erro_max) r.erro_max = modulo32(erro);
    uint32_t t_ms = (micros() - t0) / 1000;
    if (gravar && t_ms >= prox && curvaN < CURVA_MAX) {
      curva[curvaN++] = Amostra{t_ms, stepper_speed_now(&motX), m, e};
      prox = t_ms + periodo_ms;
    }
    delay(1);
  }
  r.us = micros() - t0;
  delay(5);  // última transição do encoder passa pelo debounce
  int32_t m = stepper_position(&motX) - m0;
  r.lido = encoder_count(&encX) - e0;
  r.esperado = (int32_t)lroundf(m * k);
  r.erro_fim = r.lido - r.esperado;
  if (modulo32(r.erro_fim) > r.erro_max) r.erro_max = modulo32(r.erro_fim);
  if (gravar && curvaN < CURVA_MAX) curva[curvaN++] = Amostra{r.us / 1000, 0, m, r.lido};
  return r;
}

static int32_t tolerancia(const ResultadoMov &r) {
  return TESTE_TOL_CONT + (int32_t)(modulo32(r.esperado) * TESTE_TOL_PCT / 100.0f);
}

static bool passou(const ResultadoMov &r) { return r.ok && r.erro_max <= tolerancia(r); }

// Homing (se ainda não tem) e vai para o ponto A.
static bool prepararTeste(float fim_mm) {
  if (fim_mm > CURSO_X_MM) {
    Serial.printf("o teste vai ate %.0f mm, alem do curso (%.0f mm): ajuste TESTE_*_MM\n", fim_mm,
                  CURSO_X_MM);
    return false;
  }
  if (!referenciado) {
    Serial.println("sem referencia: fazendo o homing antes");
    fazerHoming();
    if (!referenciado) return false;
  }
  ResultadoMov r = medirMovimento(alvoMm(TESTE_A_MM), false, 0);
  return r.ok;
}

static void varrerVelocidade(String resto) {
  uint32_t ate = TESTE_VELS[sizeof TESTE_VELS / sizeof TESTE_VELS[0] - 1];
  resto.trim();
  if (resto.length() && !lerInteiro(resto, false, &ate)) return;

  StepperPerfil orig;
  stepper_get_profile(&motX, &orig);
  Serial.printf("-- varredura de velocidade ate %lu passos/s: vaivem %.0f <-> %.0f mm --\n",
                (unsigned long)ate, TESTE_A_MM, TESTE_B_MM);
  Serial.printf("   partida %lu passos/s, aceleracao %lu passos/s2 (mude com n e a)\n",
                (unsigned long)orig.v0, (unsigned long)orig.acel);
  Serial.println("   Enter com qualquer tecla cancela");
  if (!prepararTeste(TESTE_B_MM)) {
    stepper_release(&motX);
    return;
  }

  Serial.println(" vmax | ida: real (ideal) s | volta: real (ideal) s | media passos/s | erro max (tol) | ");
  uint32_t limite = 0;
  bool falhou = false, cancelou = false;
  for (uint32_t v : TESTE_VELS) {
    if (v > ate) break;
    StepperPerfil p = orig;
    p.vmax = v;
    stepper_set_profile(&motX, &p);
    ResultadoMov ida = medirMovimento(alvoMm(TESTE_B_MM), false, 0);
    ResultadoMov volta = ida.ok ? medirMovimento(alvoMm(TESTE_A_MM), false, 0) : ida;
    if (!ida.ok || !volta.ok) {
      cancelou = true;
      break;
    }
    bool ok = passou(ida) && passou(volta);
    int32_t emax = ida.erro_max > volta.erro_max ? ida.erro_max : volta.erro_max;
    float media = modulo32(ida.passos) / (ida.us / 1e6f);
    Serial.printf(" %4lu |   %6.2f (%6.2f)    |   %6.2f (%6.2f)      |     %6.0f     |   %3ld (%3ld)     | %s\n",
                  (unsigned long)v, ida.us / 1e6f, ida.ideal_us / 1e6f, volta.us / 1e6f,
                  volta.ideal_us / 1e6f, media, (long)emax, (long)tolerancia(ida),
                  ok ? "ok" : "PERDEU PASSO");
    if (!ok) {
      falhou = true;
      break;
    }
    limite = v;
  }

  stepper_set_profile(&motX, &orig);
  stepper_release(&motX);
  if (cancelou) {
    Serial.println("-- varredura cancelada --");
    return;
  }
  // O homing mede quantos passos se perderam no total (e refaz a origem).
  Serial.println("conferencia: homing (chave perto de 0 passos = nada perdido)");
  fazerHoming();

  Serial.println("== resultado ==");
  if (limite == 0) {
    Serial.println("perdeu passo ja na primeira velocidade: baixe a aceleracao (a) ou a partida (n)");
  } else {
    Serial.printf("maior vmax sem perder passo: %lu passos/s (%.1f RPM, %.1f mm/s)%s\n",
                  (unsigned long)limite, limite * 60.0f / PASSOS_POR_VOLTA, limite / passosPorMm,
                  falhou ? "" : "  (nao chegou a falhar: limite real pode ser maior)");
    Serial.printf("sugestao de trabalho (80%%): v%lu  com a%lu\n", (unsigned long)(limite * 8 / 10),
                  (unsigned long)orig.acel);
  }
}

static void gravarCurva(String resto) {
  resto.trim();
  float mm = resto.length() ? lerMm(resto) : (TESTE_B_MM - TESTE_A_MM);
  if (!(mm > 0.0f)) {
    Serial.printf("distancia invalida: '%s'\n", resto.c_str());
    return;
  }
  Serial.printf("-- curva: %.1f mm a partir de %.0f mm (Enter com qualquer tecla cancela) --\n", mm,
                TESTE_A_MM);
  mostrarPerfil();
  if (!prepararTeste(TESTE_A_MM + mm)) {
    stepper_release(&motX);
    return;
  }

  // Período de gravação para caber em CURVA_MAX amostras.
  StepperPerfil p;
  stepper_get_profile(&motX, &p);
  RampaCfg rc = {STEPPER_TICK_HZ, p.v0, p.vmax, p.acel};
  int32_t passos = alvoMm(TESTE_A_MM + mm) - stepper_position(&motX);
  uint32_t ideal_ms = (uint32_t)(rampa_tiques_total(&rc, passos) * 1000ULL / STEPPER_TICK_HZ);
  uint32_t periodo = ideal_ms / (CURVA_MAX - 2) + 1;
  if (periodo < 10) periodo = 10;

  ResultadoMov r = medirMovimento(alvoMm(TESTE_A_MM + mm), true, periodo);
  medirMovimento(alvoMm(TESTE_A_MM), false, 0);  // volta ao ponto A
  stepper_release(&motX);
  if (!r.ok) {
    Serial.println("-- curva cancelada --");
    return;
  }

  Serial.printf("tempo %.3f s (ideal %.3f s), encoder %ld de %ld contagens, erro max %ld (tol %ld): %s\n",
                r.us / 1e6f, r.ideal_us / 1e6f, (long)r.lido, (long)r.esperado, (long)r.erro_max,
                (long)tolerancia(r), passou(r) ? "ok" : "PERDEU PASSO");
  Serial.printf("%u amostras a cada %lu ms. Copie as linhas abaixo para uma planilha (separador ;)\n",
                curvaN, (unsigned long)periodo);
  Serial.println("--- CSV ---");
  Serial.println("t_ms;v_motor;v_encoder;motor_passos;encoder_cont;erro_cont");
  const float k = contagensPorMm / passosPorMm;
  const uint16_t janela = (uint16_t)((100 + periodo - 1) / periodo);  // ~100 ms
  for (uint16_t i = 0; i < curvaN; i++) {
    const Amostra &a = curva[i];
    int32_t erro = a.enc - (int32_t)lroundf(a.motor * k);
    Serial.printf("%lu;%lu;", (unsigned long)a.t_ms, (unsigned long)a.v);
    if (i >= janela) {  // velocidade pelo encoder, em passos/s equivalentes
      const Amostra &b = curva[i - janela];
      float dt = (a.t_ms - b.t_ms) / 1000.0f;
      float v = dt > 0 ? fabsf((a.enc - b.enc) / k / dt) : 0.0f;
      Serial.printf("%.0f", v);
    }
    Serial.printf(";%ld;%ld;%ld\n", (long)a.motor, (long)a.enc, (long)erro);
  }
  Serial.println("--- fim do CSV ---");
}

static void mover(int32_t sentido, const String &resto) {
  int32_t passos = lerPassos(resto);
  if (passos == 0) return;
  // Volta na hora; a interrupção solta as bobinas ao terminar (não aquece).
  stepper_start(&motX, sentido * passos, true);
  andando = true;
  Serial.println("andando... (q para parar, l para soltar)");
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println("== Teste do motor X ==");
  Serial.println("f = volta p/ frente, b = volta p/ tras, q = para, l = solta, r = zera");
  Serial.println("f3 / b0.5 = numero de voltas (sem numero = 1)");
  Serial.println("v800 = vel. maxima, a1500 = aceleracao (a0 sem rampa), n300 = partida");
  Serial.println("teste de bobinas: 1..4 = so uma entrada, p/o = proxima/anterior fase");
  Serial.println("e = liga/desliga monitor do encoder (A e B crus)");
  Serial.println("h = homing, c3 = auto calibracao (3 ciclos), s = mostra a chave");
  Serial.println("d100 = calibra distancia, m99.6 = medida real, k = calibracao (k0 = nominal)");
  Serial.println("x50 = vai para 50 mm da chave (depois do homing)");
  Serial.println("t, t2 = mede a velocidade com o encoder ligado e parado");
  Serial.println("w = varredura de velocidade (acha o limite), g = grava a curva da rampa (CSV)");
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
  StepperPerfil perfil = {VEL_PARTIDA_PADRAO, VEL_MAX_PADRAO, ACEL_PADRAO};
  stepper_set_profile(&motX, &perfil);
  mostrarPerfil();
  imprimir();
}

// Executa uma linha completa: a letra é o comando, o resto é o número.
static void executar(const String &linha) {
  char c = linha[0];
  String resto = linha.substring(1);
  // Comandos que mexem no motor esperam o movimento atual terminar.
  if (strchr("fbhcdxtrwg1234po", c) && ocupado()) return;
  switch (c) {
    case 'f': mover(+1, resto); return;
    case 'b': mover(-1, resto); return;
    case 'q':
      stepper_stop(&motX);
      Serial.println("-- freando --");
      return;
    case 'v': ajustarVelocidade(resto); return;
    case 'a': ajustarAceleracao(resto); return;
    case 'n': ajustarPartida(resto); return;
    case 'h': fazerHoming(); break;
    case 'c': calibrar(resto); break;
    case 's': mostrarChave(); return;
    case 'd': calibrarDistancia(resto); return;
    case 'm': receberMedida(resto); return;
    case 'k': comandoCalibracao(resto); return;
    case 'x': irPara(resto); return;
    case 't': medirVelocidade(resto); return;
    case 'w': varrerVelocidade(resto); return;
    case 'g': gravarCurva(resto); return;
    case 'l':
      stepper_release(&motX);
      andando = false;
      Serial.println("-- solto --");
      break;
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
  if (andando && !stepper_busy(&motX)) {
    andando = false;
    Serial.print("-- parou -- ");
    imprimir();
  }
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
