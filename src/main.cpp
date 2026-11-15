// =============================================================
//  Bancada de teste: eixo X (motor + encoder) e eixo Z (motor, malha aberta)
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
// Homing e auto calibração do X (chave de fim de curso em HOME_X, lado 'f'):
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
//   x120   -> vai para X = 120 mm (precisa de homing); 'x' só mostra
//
// Coordenada X (todas as medidas em mm): posição do CENTRO DO BRAÇO.
//   Na chave do homing o centro está em X_HOME_MM (30 mm); o carro anda
//   CURSO_X_MM (190 mm) a partir da chave -> X vai de 30 a 220 mm.
//
// Eixo Z (malha aberta: sem encoder; chave de homing EMBAIXO, GPIO HOME_Z):
//   Z = descida a partir do topo: 0 = topo (seguro), CURSO_Z_MM (80) = embaixo.
//   Fuso igus de alto avanço, 8 entradas x ~3 mm = 24 mm/volta (85,33 passos/mm).
//   Esse fuso NÃO trava sozinho: as bobinas do Z ficam SEMPRE ligadas (o motor
//   esquenta; sem corrente o Z desce pelo peso).
//   ZH        -> homing do Z: DESCE até a chave, sobe 2 mm, volta devagar;
//                Z = 80 (fundo) na chave e continua segurando. O 'R' faz o ZH
//                antes, se faltar. O homing do X ('h') NÃO mexe no Z.
//                Confira que não há nada embaixo do braço antes.
//   Z+5 / Z-5 -> desce / sobe 5 mm (vale sem referência)
//   ZR        -> "aqui é o topo": Z = 0 (referência manual, sem a chave)
//   Z40       -> vai para Z = 40 mm (precisa de ZH ou ZR);  Z -> mostra
//   ZV300     -> velocidade do Z (passos/s, padrão Z_VEL_PADRAO);  ZL -> solta (o Z CAI!)
//   ZA600 / ZN150 -> aceleração / partida do Z (mais devagar = mais torque)
//   ZD, ZD50  -> calibração: desce 1 mm (folga), espera 5 s para marcar e
//                desce 50 mm;  ZM49.6 -> medida real;  ZK / ZK0 -> mostra / nominal
//   Sem encoder não há proteção contra colisão no Z: se perder passo,
//   o Z fica deslocado sem aviso. Refaça o ZH de vez em quando.
//
// Setpoints (programa de pontos X e Z, salvo na flash):
//   S                 -> lista o programa
//   S120 40           -> acrescenta o ponto X = 120 mm, Z = 40 mm
//   S120              -> X = 120 mm com Z = 0 (topo)
//   S120 40 1.5 2000  -> idem a 1,5 mm/s, espera 2000 ms no ponto
//                        (velocidade 0 = padrão = máxima de cada eixo; separe com espaço)
//   S-2  -> apaga o ponto 2;   S- -> apaga o programa todo
//   R, R3, R0 -> executa 1 vez, 3 vezes, sem fim (faz o homing do X se faltar;
//                e o do Z antes, se faltar)
//   P    -> pausa / continua;   q -> cancela (freando);  l -> cancela e solta o X
//   Um eixo por vez: se o ponto pede subir, o Z sobe primeiro e depois o X
//   anda; se pede descer, o X anda primeiro e o Z desce por último.
//   Em cada ponto confere o encoder do X: diferença > SEQ_TOL_MM cancela
//   (perdeu passo) e pede homing.
//
// Proteção contra colisão (lib/colisao), sempre ligada:
//   Durante qualquer movimento, a cada 1 mm (COLISAO_JANELA_CONT) o encoder
//   tem que andar pelo menos COLISAO_FRACAO_PCT do esperado. Abaixo disso
//   em COLISAO_CONFIRMAR janelas seguidas, o conjunto bateu: o motor
//   PARA NA HORA e SOLTA as bobinas (não fica empurrando), o programa é
//   cancelado, a referência é perdida e o ALARME liga (LED da placa
//   piscando + aviso na serial). Homing que trava também dá alarme.
//   Com alarme, nada mexe no motor até reconhecer:
//   A    -> reconhece o alarme (depois faça o homing: h)
//   C    -> mostra a proteção e o último movimento; C0 / C1 desliga / liga
//   Desligada também na varredura 'w', que força o motor a perder passo.
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
//   TX / TZ -> escolhe o motor testado (padrão X);  T -> mostra
//   T0   -> fim do teste: X solto, Z volta a segurar
//   Testar o Z perde a referência dele (com uma bobina só o Z pode descer).
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
#include "colisao.h"
#include "eixo.h"
#include "encoder.h"
#include "setpoint.h"
#include "stepper.h"

static const int32_t PASSOS_POR_VOLTA = 2048;  // 28BYJ-48, passo completo
// Perfil inicial da rampa (ajuste na bancada com v, a, n).
// Medido com 'w' (2026-11-10, a1500): 650 passos/s ok, 700 perde passo, e a
// aceleração (1000 a 2500) não mudou o limite.
static const uint32_t VEL_LIMITE = 650;  // passos/s (~19 RPM, ~2,5 mm/s): teto de tudo
// Padrão = sempre a máxima (decisão do Gab, 2026-11-12). Sem margem: se
// perder passo, a proteção contra colisão para e dá alarme.
static const uint32_t VEL_MAX_PADRAO = VEL_LIMITE;
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
// Eixo X montado (2026-11-12): o carro anda 190 mm a partir da chave e,
// na chave, o centro do braço está a 30 mm dela. X = centro do braço.
static const float CURSO_X_MM = 190.0f;
static const float X_HOME_MM = 30.0f;
static const float SEQ_TOL_MM = 1.0f;    // motor x encoder em cada ponto
// Colisão (2026-11-15, por janelas; ver lib/colisao): a cada 1 mm o encoder
// tem que andar pelo menos 40% do esperado. Bateu = encoder quase parado.
// A 1ª janela de cada movimento é ignorada (folga). Detecta em ~2 a 3 mm
// (~1 s a 650 passos/s). O limite antigo (erro TOTAL de 6 contagens) dava
// alarme falso em movimento longo: o erro se acumulava.
static const int32_t COLISAO_JANELA_CONT = 10;  // contagens = 1 mm
static const uint8_t COLISAO_FRACAO_PCT = 40;   // avanço mínimo na janela
static const uint8_t COLISAO_CONFIRMAR = 2;     // janelas ruins seguidas
static const uint32_t ALARME_REPETE_MS = 5000;   // repete o aviso na serial

// Eixo Z (2026-11-14): malha aberta, sem sensor de homing por enquanto.
// Fuso igus de alto avanço: 8 entradas, ~3 mm entre roscas, Ø ~6 mm.
// Avanço = entradas x passo = 8 x 3 = 24 mm por volta. Conferir com ZD.
static const float Z_AVANCO_MM = 24.0f;
static const float Z_PASSOS_MM_NOMINAL = PASSOS_POR_VOLTA / Z_AVANCO_MM;  // 85,33
static const float CURSO_Z_MM = 80.0f;
// Giro do motor que DESCE o Z: +1 = sentido do 'f'. Na bancada (2026-11-14) o Z
// andou ao contrário com +1: ficou -1. Se Z+5 subir, troque o sinal.
static const int8_t Z_DESCE = -1;
// Teto do ZV (mesmo motor do X). O Z NÃO subia bem a 650 (2026-11-14): o
// 28BYJ-48 tem mais torque devagar, então o padrão é Z_VEL_PADRAO.
// Nada disso foi medido no Z (sem encoder não há varredura automática):
// ajuste com ZV, ZA e ZN e confira com ciclos e régua.
static const uint32_t Z_VEL_LIMITE = 650;
static const uint32_t Z_VEL_PADRAO = 300;  // passos/s (~3,5 mm/s): padrão do Z
static const uint32_t Z_ACEL = 600;       // passos/s² (rampa suave: menos tranco)
static const uint32_t Z_PARTIDA = 150;    // passos/s: partida lenta = mais torque
// Fuso de alto avanço não trava: sem corrente o Z desce pelo peso.
static const bool Z_SEGURAR = true;
static const float Z_DIST_PADRAO = 50.0f;  // calibração ZD
static const float Z_DIST_PRE_MM = 1.0f;   // desce antes de marcar: assenta a folga
// Homing do Z (2026-11-14): chave EMBAIXO (HOME_Z), no fundo do curso.
// Z onde a chave aciona. A coordenada continua 0 = topo; o homing só põe a
// posição certa no fundo. Se a chave não ficar bem no fim do curso, ajuste aqui.
static const float Z_HOME_MM = CURSO_Z_MM;
// false = chave ainda não montada: ZH recusa e o 'h' não mexe no Z (use ZR).
static const bool Z_TEM_CHAVE = true;
static const float Z_HOME_FOLGA_MM = 20.0f;     // busca rápida: curso + isso sem achar = erro
static const float Z_HOME_RECUO_MM = 2.0f;      // sobe além de a chave soltar
static const float Z_HOME_RECUO_MAX_MM = 10.0f; // a chave tem que soltar antes disso
static const uint32_t Z_HOME_VEL_RAPIDA = 250;  // passos/s (~2,9 mm/s); sem rampa no homing
static const uint32_t Z_HOME_VEL_LENTA = 120;   // passos/s (~1,4 mm/s): precisão da origem

Encoder encX;
Stepper motX;
Stepper motZ;
Eixo eixoX;
Eixo eixoZ;  // sem encoder: só motor + chave
static EixoHomingCfg homingX;
static EixoHomingCfg homingZ;
// Último estado dos sensores já avisado na serial (fora do homing).
static bool chaveAnterior = false;
static bool chaveZAnterior = false;

// Depois de um homing: ele reiniciou o debounce da chave; sincroniza para
// o aviso fora do homing não repetir o que o homing já mostrou.
static void sincronizarChave() { chaveAnterior = debounce_level(&eixoX.chave) != 0; }
static void sincronizarChaveZ() { chaveZAnterior = debounce_level(&eixoZ.chave) != 0; }

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

// ---------- proteção contra colisão (definida mais abaixo) ----------
static ColisaoMonitor colisao;
static bool alarme = false;  // travado até o 'A'
static bool vigiarColisao();
static void dispararAlarme(const char *motivo, int32_t erro_cont);

// Coordenada X em mm (centro do braço), com a calibração atual.
static EixoMm eixoMm() {
  return EixoMm{X_HOME_MM, CURSO_X_MM, passosPorMm, (int8_t)-homingX.sentido};
}
static float vmaxMm() { return VEL_LIMITE / passosPorMm; }  // mm/s

// X do centro do braço pelo motor e pelo encoder (os dois zeram na chave).
static float posicaoMm() {
  EixoMm e = eixoMm();
  return eixo_passos_para_mm(&e, stepper_position(&motX));
}
static float encoderMm() {
  return X_HOME_MM + (float)(-homingX.sentido * encoder_count(&encX)) / contagensPorMm;
}

// ---------- eixo Z: coordenada (malha aberta) ----------
static float zPassosPorMm = Z_PASSOS_MM_NOMINAL;
static bool zReferenciado = false;   // ZR feito: Z em mm vale
static uint32_t velocidadeZ = Z_VEL_PADRAO;  // ZV muda (até Z_VEL_LIMITE)
static bool andandoZ = false;        // para avisar quando o Z para

static EixoMm eixoZmm() { return EixoMm{0.0f, CURSO_Z_MM, zPassosPorMm, Z_DESCE}; }
static float vmaxZmm() { return Z_VEL_LIMITE / zPassosPorMm; }  // mm/s
static float zMm() {
  EixoMm e = eixoZmm();
  return eixo_passos_para_mm(&e, stepper_position(&motZ));
}

// Limites dos setpoints: cada eixo no seu curso; velocidade até a do mais rápido.
static ProgLimites limites() {
  ProgLimites l;
  l.x = eixoMm();
  l.z = eixoZmm();
  l.vmax_mm_s = vmaxMm() > vmaxZmm() ? vmaxMm() : vmaxZmm();
  return l;
}

// Move o Z sem bloquear (as bobinas continuam ligadas no fim se Z_SEGURAR).
static void zIniciar(int32_t alvo) {
  if (alvo == stepper_position(&motZ)) return;
  stepper_start_to(&motZ, alvo, !Z_SEGURAR);
}

// Cancela h/c: qualquer tecla que não seja o fim da linha do próprio comando.
// Também é chamada a cada 1 ms pelas rotinas que bloqueiam (homing, d, c,
// testes): por isso vigia a colisão aqui.
static bool cancelarPelaSerial(void *) {
  if (vigiarColisao()) return true;
  while (Serial.available()) {
    int c = Serial.read();
    if (c != '\r' && c != '\n') return true;
  }
  return false;
}

static const char *giroNome(int8_t g) { return g > 0 ? "f" : (g < 0 ? "b" : "-"); }

// Impressão dos avisos do homing (chamada de dentro do eixo_home).
// ctx = nome do eixo ("X" ou "Z"). O Z não tem encoder: não imprime contagem.
static void avisoHoming(const EixoEvento *ev, void *ctx) {
  const char *eixo = ctx ? (const char *)ctx : "X";
  bool temEnc = eixo[0] == 'X';
  if (ev->tipo == EIXO_EV_CHAVE) {
    Serial.printf("[homing %s] SENSOR %s  (motor %ld", eixo, ev->chave ? "APERTADO" : "solto",
                  (long)ev->motor);
    if (temEnc) Serial.printf(", encoder %ld", (long)ev->enc);
    Serial.println(")");
    return;
  }
  if (ev->inicio && ev->chave) {
    Serial.printf("[homing %s] sensor JA APERTADO na partida: recua primeiro\n", eixo);
  }
  Serial.printf("[homing %s] estado: %s, girando '%s'  (motor %ld", eixo,
                homing_state_nome(ev->estado), giroNome(ev->giro), (long)ev->motor);
  if (temEnc) Serial.printf(", encoder %ld", (long)ev->enc);
  Serial.println(")");
  if (ev->estado == HOM_RECUO) Serial.printf("[homing %s]   -> esperando o sensor soltar\n", eixo);
  if (ev->estado == HOM_BUSCA_LENTA) {
    Serial.printf("[homing %s]   -> aperte o sensor de novo\n", eixo);
  }
}

// Homing do Z: chave EMBAIXO. Sem encoder não há como ver travamento: se a
// chave falhar, o motor desce até 'curso_max' (o 28BYJ-48 só pula passo no
// batente). Limites em passos nominais.
static int32_t zPassosNominais(float mm) { return (int32_t)lroundf(mm * Z_PASSOS_MM_NOMINAL); }

static void configurarHomingZ() {
  homingZ.sentido = Z_DESCE;  // chave embaixo: o giro que DESCE
  homingZ.fsm.curso_max = zPassosNominais(CURSO_Z_MM + Z_HOME_FOLGA_MM);
  homingZ.fsm.recuo = zPassosNominais(Z_HOME_RECUO_MM);
  homingZ.fsm.recuo_max = zPassosNominais(Z_HOME_RECUO_MAX_MM);
  homingZ.fsm.janela_trava = 0;  // sem encoder
  homingZ.vel_rapida = Z_HOME_VEL_RAPIDA;
  homingZ.vel_lenta = Z_HOME_VEL_LENTA;
  homingZ.segurar = Z_SEGURAR;  // o fuso não trava: termina segurando
  homingZ.cancelar = cancelarPelaSerial;
  homingZ.evento = avisoHoming;
  homingZ.ctx = (void *)"Z";
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
  homingX.segurar = false;  // o TR8x8 trava sozinho: solta (não aquece)
  homingX.ctx = (void *)"X";
}

static void imprimir() {
  Serial.printf("motor: %6ld   encoder: %5ld   erros: %lu", (long)stepper_position(&motX),
                (long)encoder_count(&encX), (unsigned long)encoder_errors(&encX));
  if (referenciado) Serial.printf("   X: %.2f mm (encoder %.2f mm)", posicaoMm(), encoderMm());
  if (zReferenciado) {
    Serial.printf("   Z: %.2f mm", zMm());
  } else {
    Serial.printf("   Z: sem ref (motor %ld)", (long)stepper_position(&motZ));
  }
  Serial.println();
}

// Fase do testador, separada da fase do motor. Começa em 3 para o
// primeiro 'p' mostrar a fase 0.
static uint8_t faseTeste = 3;

// Motor do testador (TX / TZ). O Z mexido pelo testador perde a referência:
// com uma bobina só (ou nenhuma) o fuso de alto avanço pode descer.
static Stepper *motTeste = &motX;
static bool zEmTeste = false;  // o testador mexeu nas bobinas do Z
static const char *nomeTeste() { return motTeste == &motZ ? "Z" : "X"; }

// Fim do teste: X solto; Z volta a segurar na fase dele (Z_SEGURAR).
static void terminarTeste() {
  stepper_release(&motX);
  if (zEmTeste) {
    if (Z_SEGURAR) {
      stepper_hold(&motZ);
    } else {
      stepper_release(&motZ);
    }
    zEmTeste = false;
  }
  faseTeste = 3;
}

// T -> mostra;  TX / TZ -> escolhe o motor;  T0 -> fim do teste.
static void comandoTeste(String resto) {
  resto.trim();
  resto.toUpperCase();
  if (resto == "X" || resto == "Z") {
    terminarTeste();
    motTeste = resto == "Z" ? &motZ : &motX;
  } else if (resto == "0") {
    terminarTeste();
    Serial.printf("-- fim do teste: X solto, Z %s --\n", Z_SEGURAR ? "segurando" : "solto");
    return;
  } else if (resto.length()) {
    Serial.printf("comando invalido: 'T%s' (T, TX, TZ, T0)\n", resto.c_str());
    return;
  }
  Serial.printf("testador no motor %s: IN1..IN4 = GPIO %u %u %u %u\n", nomeTeste(),
                stepper_pin(motTeste, 0), stepper_pin(motTeste, 1), stepper_pin(motTeste, 2),
                stepper_pin(motTeste, 3));
}

// Liga as bobinas pedidas e diz o que deve aparecer na placa.
static void testarBobinas(uint8_t coils) {
  if (motTeste == &motZ) {
    if (stepper_busy(&motZ)) {
      Serial.println("Z andando: espere parar");
      return;
    }
    if (!zEmTeste) {
      Serial.println("atencao: testando o Z. SEGURE o Z (pode descer); referencia do Z perdida."
                     " T0 volta a segurar");
      zEmTeste = true;
      zReferenciado = false;
      andandoZ = false;
    }
  }
  stepper_test_coils(motTeste, coils);
  Serial.printf("ligado %s:", nomeTeste());
  if (coils == 0) Serial.print("  nada");
  for (uint8_t i = 0; i < 4; i++) {
    if ((coils >> i) & 0x1) {
      Serial.printf("  IN%u (GPIO %u, LED %c)", i + 1, stepper_pin(motTeste, i), 'A' + i);
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
    // Travou (encoder parado com o motor girando) = bateu em algo.
    if (r.erro == HOM_ERRO_TRAVOU) dispararAlarme("TRAVOU no homing", 0);
  }
}

// Espera o Z parar (bloqueia). false se cancelarem pela serial (o Z para e segura).
static bool zEsperarParar() {
  while (stepper_busy(&motZ)) {
    if (cancelarPelaSerial(nullptr)) {
      stepper_abort(&motZ);
      return false;
    }
    delay(1);
  }
  return true;
}

// Posição do motor Z (passos) onde a chave de baixo aciona: Z = Z_HOME_MM.
static int32_t zPassosNaChave() {
  EixoMm e = eixoZmm();
  return eixo_mm_para_passos(&e, Z_HOME_MM);
}

// Homing do Z (ZH, e pelo 'h' e pelo 'R' quando o Z não tem referência).
// A chave fica EMBAIXO: desce até ela e termina ali, segurando, com
// Z = Z_HOME_MM (80 mm, o fundo). A coordenada não muda: 0 continua no topo.
// true = deu certo.
static bool fazerHomingZ() {
  if (!Z_TEM_CHAVE) {
    Serial.println("Z sem chave de homing (Z_TEM_CHAVE = false): leve ao topo e envie ZR");
    return false;
  }
  Serial.println("-- homing do Z: DESCE ate a chave (confira que nao ha nada embaixo;"
                 " Enter com qualquer tecla cancela) --");
  bool tinhaRef = zReferenciado;
  EixoHomeResult r;
  eixo_home(&eixoZ, &homingZ, &r);
  sincronizarChaveZ();
  zReferenciado = (r.erro == HOM_ERRO_NENHUM);
  andandoZ = false;
  if (zReferenciado) {
    // O eixo_home zerou o motor na chave; a chave é o fundo, não o zero.
    int32_t naChave = zPassosNaChave();
    stepper_reset(&motZ, naChave);
    Serial.printf("homing do Z ok em %lu passos (%.1f mm): Z = %.0f mm na chave (fundo)\n",
                  (unsigned long)r.passos_dados, r.passos_dados / zPassosPorMm, Z_HOME_MM);
    // Com referência antes: onde a chave estava x onde deveria estar.
    // ~0 = o Z não perdeu passo desde o último homing.
    if (tinhaRef) {
      int32_t dif = r.passos_na_chave - naChave;
      Serial.printf("   chave a %ld passos (%.2f mm) de onde estava antes\n", (long)dif,
                    dif / zPassosPorMm);
    }
  } else {
    Serial.printf("homing do Z FALHOU (%s) no estado '%s', depois de %lu passos. "
                  "Z parado e segurando, SEM referencia\n",
                  homing_error_nome(r.erro), homing_state_nome(r.parou_em),
                  (unsigned long)r.passos_dados);
    if (r.erro == HOM_ERRO_CURSO) {
      Serial.println("   nao achou a chave: confira a fiacao (s mostra as chaves) e o sentido (Z_DESCE)");
    }
  }
  return zReferenciado;
}

// Homing do X: só o X anda. O Z fica onde está (decisão do Gab, 2026-11-15:
// a chave do X fica embaixo e o braço não precisa subir para o X andar).
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
  Serial.printf("chave X: %s (GPIO %u = %s)\n", eixo_chave_crua(&eixoX) ? "ACIONADA" : "solta",
                HOME_X, digitalRead(HOME_X) ? "HIGH" : "LOW");
  Serial.printf("chave Z: %s (GPIO %u = %s)%s\n", eixo_chave_crua(&eixoZ) ? "ACIONADA" : "solta",
                HOME_Z, digitalRead(HOME_Z) ? "HIGH" : "LOW",
                Z_TEM_CHAVE ? "" : "  [Z_TEM_CHAVE = false: nao usada]");
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
  colisao_escala(&colisao, contagensPorMm / passosPorMm);
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
    EixoMm e = eixoMm();
    if (!eixo_mm_dentro(&e, mm)) {
      Serial.printf("posicao invalida: '%s' (use X de %.0f a %.0f mm)\n", resto.c_str(),
                    eixo_mm_min(&e), eixo_mm_max(&e));
      return;
    }
    int32_t alvo = eixo_mm_para_passos(&e, mm);
    if (alvo != stepper_position(&motX)) {
      stepper_start_to(&motX, alvo, true);  // volta na hora; solta ao chegar
      andando = true;
      Serial.printf("indo para X = %.2f mm (q para parar)\n", mm);
      return;
    }
  }
  imprimir();
}

// ---------- setpoints: programa em mm (lib/setpoint) ----------

static Programa programa;
static Seq seq = {};  // começa PARADO

static void salvarPrograma() {
  prefs.begin("prog", false);
  prefs.putBytes("x", &programa, sizeof programa);
  prefs.end();
}

static void carregarPrograma() {
  prog_limpar(&programa);
  prefs.begin("prog", false);
  if (prefs.isKey("x")) {
    if (prefs.getBytesLength("x") == sizeof programa) {
      prefs.getBytes("x", &programa, sizeof programa);
    } else {
      Serial.println("programa salvo no formato antigo (so X): apagado");
    }
  }
  prefs.end();
  ProgLimites l = limites();
  uint8_t i = 0;
  ProgErro r = prog_validar(&programa, &l, &i);
  if (r != PROG_OK) {
    Serial.printf("programa salvo invalido (ponto %u: %s): apagado\n", i + 1, prog_erro_nome(r));
    prog_limpar(&programa);
    salvarPrograma();
  }
}

// Velocidade do ponto em passos/s para cada eixo. 0 = padrão; acima do
// limite do eixo, fica no limite (o X é mais lento que o Z).
static uint32_t velocidadeDoPonto(const Setpoint *sp, float passos_mm, uint32_t padrao,
                                  uint32_t limite) {
  if (!(sp->vel_mm_s > 0.0f)) return padrao;
  uint32_t v = (uint32_t)lroundf(sp->vel_mm_s * passos_mm);
  if (v > limite) v = limite;
  return v ? v : 1;
}

static void listarPrograma() {
  EixoMm e = eixoMm();
  EixoMm z = eixoZmm();
  Serial.printf("programa X (centro do braco, %.0f a %.0f mm) Z (topo 0 a %.0f mm): %u ponto(s)\n",
                eixo_mm_min(&e), eixo_mm_max(&e), eixo_mm_max(&z), programa.n);
  for (uint8_t i = 0; i < programa.n; i++) {
    const Setpoint &p = programa.p[i];
    Serial.printf("  %2u: X %7.2f mm  Z %6.2f mm   ", i + 1, p.x_mm, p.z_mm);
    if (p.vel_mm_s > 0.0f) {
      Serial.printf("%.2f mm/s", p.vel_mm_s);
    } else {
      Serial.printf("padrao (X %.2f, Z %.2f mm/s)", velocidade / passosPorMm,
                    velocidadeZ / zPassosPorMm);
    }
    Serial.printf("   espera %lu ms\n", (unsigned long)p.espera_ms);
  }
  if (programa.n == 0) Serial.println("  (vazio) acrescente com S<x> <z>, ex.: S120 40");
}

// Programa rodando ou pausado: não deixa mudar (os índices mudariam).
static bool programaEmUso() {
  if (!seq_ativo(&seq) && seq_estado(&seq) != SEQ_PAUSADO) return false;
  Serial.println("programa em execucao: q cancela antes de mudar");
  return true;
}

// "120", "120 40", "120 40 1.5", "120 40 1,5 2000":
// X, Z (sem ele = 0, topo), velocidade (mm/s) e espera (ms).
static bool lerSetpoint(String s, Setpoint *out) {
  s.replace(',', '.');
  s.trim();
  const char *c = s.c_str();
  char *fim;
  out->x_mm = strtof(c, &fim);
  out->z_mm = 0.0f;
  out->vel_mm_s = 0.0f;
  out->espera_ms = 0;
  if (fim == c) return false;
  c = fim;
  float z = strtof(c, &fim);
  if (fim != c) {
    out->z_mm = z;
    c = fim;
    float v = strtof(c, &fim);
    if (fim != c) {
      out->vel_mm_s = v;
      c = fim;
      long esp = strtol(c, &fim, 10);
      if (fim != c) {
        if (esp < 0) return false;
        out->espera_ms = (uint32_t)esp;
        c = fim;
      }
    }
  }
  while (*c == ' ') c++;
  return *c == '\0';  // sobrou texto: inválido
}

static void comandoSetpoint(String resto) {
  resto.trim();
  if (resto.length() == 0) {
    listarPrograma();
    return;
  }
  if (programaEmUso()) return;

  if (resto[0] == '-') {
    String n = resto.substring(1);
    n.trim();
    if (n.length() == 0) {
      prog_limpar(&programa);
      Serial.println("-- programa apagado --");
    } else {
      long i = n.toInt();
      ProgErro r = (i >= 1 && i <= 255) ? prog_remover(&programa, (uint8_t)(i - 1))
                                        : PROG_INDICE_INVALIDO;
      if (r != PROG_OK) {
        Serial.printf("nao apagou: %s ('%s')\n", prog_erro_nome(r), n.c_str());
        return;
      }
      Serial.printf("-- ponto %ld apagado --\n", i);
    }
    salvarPrograma();
    listarPrograma();
    return;
  }

  Setpoint sp;
  if (!lerSetpoint(resto, &sp)) {
    Serial.printf("setpoint invalido: '%s' (use S<x> [z] [mm/s] [ms], ex.: S120 40 1.5 2000)\n",
                  resto.c_str());
    return;
  }
  ProgLimites l = limites();
  ProgErro r = prog_adicionar(&programa, &l, sp);
  if (r != PROG_OK) {
    Serial.printf("nao acrescentou: %s", prog_erro_nome(r));
    if (r == PROG_X_FORA_DO_CURSO) {
      Serial.printf(" (X de %.0f a %.0f mm)", eixo_mm_min(&l.x), eixo_mm_max(&l.x));
    }
    if (r == PROG_Z_FORA_DO_CURSO) {
      Serial.printf(" (Z de %.0f a %.0f mm)", eixo_mm_min(&l.z), eixo_mm_max(&l.z));
    }
    if (r == PROG_VEL_INVALIDA) Serial.printf(" (0 a %.2f mm/s)", l.vmax_mm_s);
    if (r == PROG_ESPERA_INVALIDA) Serial.printf(" (ate %lu ms)", (unsigned long)PROG_ESPERA_MAX_MS);
    if (r == PROG_CHEIO) Serial.printf(" (%u pontos)", PROG_MAX_PONTOS);
    Serial.println();
    return;
  }
  salvarPrograma();
  listarPrograma();
}

// Executa a ação que o sequenciador pediu: um eixo por fase.
static void executarAcao(SeqAcao a) {
  if (a == SEQ_FREAR) {
    stepper_stop(&motX);
    stepper_stop(&motZ);
    return;
  }
  if (a != SEQ_MOVER) return;
  const Setpoint *sp = seq_alvo(&seq);
  if (seq.fase == 0) {
    Serial.printf("[seq] ciclo %lu, ponto %u/%u -> X %.2f mm Z %.2f mm\n",
                  (unsigned long)seq.ciclo + 1, seq.i + 1, programa.n, sp->x_mm, sp->z_mm);
  }
  // Fase sem deslocamento: não liga o motor (o sequenciador vê parado = feita).
  if (seq_eixo(&seq) == SEQ_EIXO_X) {
    EixoMm e = eixoMm();
    int32_t alvo = eixo_mm_para_passos(&e, sp->x_mm);
    if (alvo == stepper_position(&motX)) return;
    Serial.printf("[seq]   X -> %.2f mm\n", sp->x_mm);
    stepper_set_speed(&motX, velocidadeDoPonto(sp, passosPorMm, velocidade, VEL_LIMITE));
    stepper_start_to(&motX, alvo, true);  // X: solta as bobinas ao chegar (TR8x8 trava)
  } else {
    EixoMm e = eixoZmm();
    int32_t alvo = eixo_mm_para_passos(&e, sp->z_mm);
    if (alvo == stepper_position(&motZ)) return;
    Serial.printf("[seq]   Z -> %.2f mm (%s)\n", sp->z_mm,
                  sp->z_mm < zMm() ? "sobe" : "desce");
    stepper_set_speed(&motZ, velocidadeDoPonto(sp, zPassosPorMm, velocidadeZ, Z_VEL_LIMITE));
    zIniciar(alvo);
  }
}

static void fimDoPrograma(const char *como) {
  stepper_set_speed(&motX, velocidade);  // volta as velocidades padrão
  stepper_set_speed(&motZ, velocidadeZ);
  Serial.printf("-- programa %s --  ", como);
  imprimir();
}

// Chamada a cada volta do loop().
static void atualizarSequencia() {
  if (!seq_ativo(&seq)) return;
  bool ocupado = stepper_busy(&motX) || stepper_busy(&motZ);

  if (seq_estado(&seq) == SEQ_MOVENDO && !ocupado && seq.fase == 1) {
    // Chegou no ponto: o encoder confirma que o carro do X está mesmo lá.
    float x = posicaoMm();
    float xe = encoderMm();
    Serial.printf("[seq] chegou: X %.2f mm (encoder %.2f mm) Z %.2f mm", x, xe, zMm());
    const Setpoint *sp = seq_alvo(&seq);
    if (sp->espera_ms) Serial.printf(", espera %lu ms", (unsigned long)sp->espera_ms);
    Serial.println();
    if (fabsf(xe - x) > SEQ_TOL_MM) {
      seq_cancelar(&seq);
      referenciado = false;
      Serial.printf("PERDEU PASSO: encoder %.2f mm longe do motor (limite %.1f mm). Faca o homing (h)\n",
                    xe - x, SEQ_TOL_MM);
      fimDoPrograma("cancelado");
      return;
    }
  }

  SeqEstado antes = seq_estado(&seq);
  executarAcao(seq_atualizar(&seq, ocupado, millis()));
  SeqEstado agora = seq_estado(&seq);
  if (agora == antes) return;
  if (agora == SEQ_PAUSADO) {
    Serial.printf("[seq] pausado em X %.2f mm Z %.2f mm: P continua, q cancela\n", posicaoMm(),
                  zMm());
  } else if (agora == SEQ_FIM) {
    char txt[40];
    snprintf(txt, sizeof txt, "terminado (%lu ciclo(s))", (unsigned long)seq.ciclo);
    fimDoPrograma(txt);
  }
}

static void rodarPrograma(String resto) {
  if (seq_estado(&seq) == SEQ_PAUSADO) {
    Serial.println("programa pausado: P continua, q cancela");
    return;
  }
  if (programa.n == 0) {
    Serial.println("programa vazio: acrescente pontos com S<x> <z>, ex.: S120 40");
    return;
  }
  if (stepper_busy(&motZ)) {
    Serial.println("Z andando: espere parar");
    return;
  }
  resto.trim();
  uint32_t ciclos = 1;
  if (resto.length() && !lerInteiro(resto, true, &ciclos)) return;
  if (!zReferenciado) {
    if (!Z_TEM_CHAVE) {
      Serial.println("Z sem referencia: leve o Z ao topo (Z-1, Z-5...) e envie ZR");
      return;
    }
    Serial.println("Z sem referencia: fazendo o homing do Z antes");
    if (!fazerHomingZ()) return;
  }
  if (!referenciado) {
    Serial.println("sem referencia: fazendo o homing antes");
    fazerHoming();
    if (!referenciado) return;
  }
  if (ciclos == 0) {
    Serial.printf("-- programa: %u ponto(s), repetindo sem fim (P pausa, q cancela) --\n", programa.n);
  } else {
    Serial.printf("-- programa: %u ponto(s), %lu ciclo(s) (P pausa, q cancela) --\n", programa.n,
                  (unsigned long)ciclos);
  }
  executarAcao(seq_iniciar(&seq, &programa, ciclos, zMm()));
}

static void pausarOuContinuar() {
  if (seq_estado(&seq) == SEQ_PAUSADO) {
    if (!referenciado) {
      Serial.println("a referencia se perdeu na pausa: faca o homing (h) e envie P de novo");
      return;
    }
    if (!zReferenciado) {
      Serial.println("o Z perdeu a referencia na pausa: ZH (ou ZR no topo) e envie P de novo");
      return;
    }
    if (stepper_busy(&motX) || stepper_busy(&motZ)) {
      Serial.println("motor andando: espere parar e envie P de novo");
      return;
    }
    Serial.println("-- continuando --");
    executarAcao(seq_continuar(&seq, zMm()));
    return;
  }
  if (seq_estado(&seq) == SEQ_PAUSANDO) return;  // já está freando
  if (!seq_ativo(&seq)) {
    Serial.println("nenhum programa rodando (R executa)");
    return;
  }
  Serial.println("-- pausando --");
  executarAcao(seq_pausar(&seq));
}

// q e l: cancela o programa, se houver. Devolve true se cancelou.
static bool cancelarPrograma() {
  if (!seq_ativo(&seq) && seq_estado(&seq) != SEQ_PAUSADO) return false;
  executarAcao(seq_cancelar(&seq));
  fimDoPrograma("cancelado");
  return true;
}

// ---------- proteção contra colisão: alarme ----------

static const char *alarmeMotivo = "";
static int32_t alarmeErro = 0;
static float alarmeX = 0.0f;
static bool alarmeTinhaReferencia = false;
static uint32_t alarmeAviso = 0;  // último aviso na serial (ms)

// Diagnóstico da colisão: a janela que disparou (contagens) e a velocidade.
static int32_t alarmeJanEsp = 0, alarmeJanMed = 0;
static uint32_t alarmeVel = 0;

static void avisarAlarme() {
  Serial.printf("!!! ALARME: %s", alarmeMotivo);
  if (alarmeErro) {
    Serial.printf(" (encoder %+ld contagens = %+.2f mm do esperado)", (long)alarmeErro,
                  alarmeErro / contagensPorMm);
  }
  if (alarmeTinhaReferencia) Serial.printf(" perto de X %.2f mm", alarmeX);
  Serial.println();
  if (alarmeJanEsp) {
    // O que o encoder fez na última janela diz o que provavelmente houve.
    Serial.printf("!!! na ultima janela o encoder andou %+ld de %+ld contagens esperadas, a %lu passos/s: ",
                  (long)alarmeJanMed, (long)alarmeJanEsp, (unsigned long)alarmeVel);
    if ((alarmeJanMed > 0) != (alarmeJanEsp > 0) && alarmeJanMed != 0) {
      Serial.println("encoder ao CONTRARIO do motor (fios A/B trocados?)");
    } else if (alarmeJanMed == 0) {
      Serial.println("encoder PARADO: bateu, motor perdendo passo, ou encoder solto/desligado");
    } else {
      Serial.println("encoder bem atras: motor perdendo passo (baixe a velocidade) ou batida");
    }
  }
  Serial.println("!!! X parado e solto, Z parado e segurando. Confira o eixo, tire o obstaculo e envie A");
  alarmeAviso = millis();
}

static void dispararAlarme(const char *motivo, int32_t erro_cont) {
  stepper_release(&motX);  // para NA HORA e tira a força: não fica empurrando
  stepper_abort(&motZ);    // Z para, mas SEGURA (solto, ele desceria pelo peso)
  andando = false;
  andandoZ = false;
  alarme = true;
  alarmeMotivo = motivo;
  alarmeErro = erro_cont;
  alarmeTinhaReferencia = referenciado;
  alarmeX = referenciado ? posicaoMm() : 0.0f;
  referenciado = false;  // perdeu passo: a posição em mm não vale mais
  if (seq_ativo(&seq) || seq_estado(&seq) == SEQ_PAUSADO) {
    seq_cancelar(&seq);  // a ação (frear) não importa: o motor já está solto
    stepper_set_speed(&motX, velocidade);
    stepper_set_speed(&motZ, velocidadeZ);
    Serial.println("-- programa cancelado pelo alarme --");
  }
  digitalWrite(ALARME_LED, HIGH);
  avisarAlarme();
}

// Chamar com frequência. true = alarme ativo (quem chamou deve parar).
static bool vigiarColisao() {
  if (alarme) return true;
  uint32_t vel = stepper_speed_now(&motX);
  if (!colisao_atualizar(&colisao, stepper_busy(&motX), stepper_moves(&motX),
                         stepper_position(&motX), encoder_count(&encX))) {
    return false;
  }
  alarmeJanEsp = colisao.esperado_j;
  alarmeJanMed = colisao.medido_j;
  alarmeVel = vel;
  dispararAlarme("COLISAO", 0);
  return true;
}

// C -> mostra;  C0 -> desliga a proteção (testes);  C1 -> liga de novo.
// Desligada não volta sozinha: o boot sempre liga.
static void comandoColisao(String resto) {
  resto.trim();
  if (resto == "0" || resto == "1") {
    colisao_habilitar(&colisao, resto == "1");
    if (resto == "0") {
      Serial.println("-- protecao contra colisao DESLIGADA: batidas NAO param o motor (C1 liga) --");
    }
  } else if (resto.length()) {
    Serial.printf("comando invalido: 'C%s' (C, C0, C1)\n", resto.c_str());
    return;
  }
  Serial.printf("protecao contra colisao: %s, janela %ld contagens (%.1f mm), avanco minimo %u%%, "
                "%u janelas seguidas\n",
                colisao.habilitado ? "ligada" : "DESLIGADA", (long)COLISAO_JANELA_CONT,
                COLISAO_JANELA_CONT / contagensPorMm, COLISAO_FRACAO_PCT, COLISAO_CONFIRMAR);
  Serial.printf("   ultimo movimento: pior avanco %ld%%, erro total max %ld contagens (%.2f mm)\n",
                (long)colisao_pior_pct(&colisao), (long)colisao_erro_max(&colisao),
                colisao_erro_max(&colisao) / contagensPorMm);
}

// No loop: pisca o LED (2 Hz) e repete o aviso enquanto o alarme durar.
static void sinalizarAlarme() {
  if (!alarme) return;
  digitalWrite(ALARME_LED, (millis() / 250) & 1);
  if (millis() - alarmeAviso >= ALARME_REPETE_MS) avisarAlarme();
}

static void reconhecerAlarme() {
  if (!alarme) {
    Serial.print("sem alarme. ");
    comandoColisao("");
    return;
  }
  alarme = false;
  alarmeJanEsp = alarmeJanMed = 0;
  colisao_rearmar(&colisao);
  digitalWrite(ALARME_LED, LOW);
  Serial.println("-- alarme reconhecido. Faca o homing (h) antes de usar posicoes em mm --");
}

// ---------- eixo Z: comandos (malha aberta) ----------

// Calibração do Z (só passos/mm: não tem encoder).
static bool zMedidaPendente = false;
static int32_t zPendPassos = 0;
static float zPendMm = 0.0f;

static void zCarregarCalibracao() {
  prefs.begin("calib", false);
  float p = prefs.getFloat("z_ppmm", Z_PASSOS_MM_NOMINAL);
  prefs.end();
  zPassosPorMm = (p > 0.0f) ? p : Z_PASSOS_MM_NOMINAL;
}

static void zSalvarCalibracao() {
  prefs.begin("calib", false);
  prefs.putFloat("z_ppmm", zPassosPorMm);
  prefs.end();
}

static void zMostrar() {
  StepperPerfil p;
  stepper_get_profile(&motZ, &p);
  Serial.printf("Z: %s", zReferenciado ? "" : "SEM REFERENCIA (ZH = homing, ou ZR no topo)  ");
  if (zReferenciado) Serial.printf("%.2f mm (topo 0 a %.0f mm)  ", zMm(), CURSO_Z_MM);
  Serial.printf("motor %ld passos, %s\n", (long)stepper_position(&motZ),
                stepper_busy(&motZ) ? "andando" : "parado");
  Serial.printf("   %.3f passos/mm (nominal %.2f = %.0f mm/volta), vel %lu passos/s (%.2f mm/s), "
                "bobinas %s\n",
                zPassosPorMm, Z_PASSOS_MM_NOMINAL, Z_AVANCO_MM, (unsigned long)p.vmax,
                p.vmax / zPassosPorMm, Z_SEGURAR ? "sempre ligadas" : "soltas ao parar");
  Serial.printf("   rampa Z: partida %lu passos/s, aceleracao %lu passos/s2%s (ZV, ZA, ZN ajustam)\n",
                (unsigned long)p.v0, (unsigned long)p.acel, p.acel ? "" : " (sem rampa)");
}

// ZD: desce 1 mm (assenta a folga), espera para marcar e desce 'mm'.
static void zCalibrarDistancia(String resto) {
  resto.trim();
  float mm = resto.length() ? lerMm(resto) : Z_DIST_PADRAO;
  float max = CURSO_Z_MM - Z_DIST_PRE_MM;
  if (zReferenciado) max -= zMm();  // não passa do fundo
  if (!(mm > 0.0f) || mm > max) {
    Serial.printf("distancia invalida: '%s' (use 0 < mm <= %.1f)\n", resto.c_str(), max);
    return;
  }
  zMedidaPendente = false;
  stepper_set_speed(&motZ, velocidadeZ);
  Serial.printf("-- calibracao do Z: desce %.1f mm (Enter com qualquer tecla cancela) --\n", mm);
  stepper_start(&motZ, Z_DESCE * (int32_t)lroundf(Z_DIST_PRE_MM * zPassosPorMm), !Z_SEGURAR);
  bool ok = zEsperarParar();
  if (ok) {
    Serial.printf(">> MARQUE a posicao do Z agora. Descendo %.1f mm em %lu s...\n", mm,
                  (unsigned long)(DIST_ESPERA_MS / 1000));
    ok = esperar(DIST_ESPERA_MS);
  }
  int32_t passos = (int32_t)lroundf(mm * zPassosPorMm);
  if (ok) {
    stepper_start(&motZ, Z_DESCE * passos, !Z_SEGURAR);
    ok = zEsperarParar();
  }
  if (!ok) {
    Serial.println("-- calibracao do Z cancelada --");
    zMostrar();
    return;
  }
  zMedidaPendente = true;
  zPendPassos = passos;
  zPendMm = mm;
  Serial.printf("desceu %ld passos (%.2f voltas)\n", (long)passos, (float)passos / PASSOS_POR_VOLTA);
  Serial.println(">> MECA o deslocamento real desde a marca e envie: ZM<mm>  (ex.: ZM49.6)");
}

static void zReceberMedida(const String &resto) {
  if (!zMedidaPendente) {
    Serial.println("nada para calibrar no Z: use ZD (ex.: ZD50) primeiro");
    return;
  }
  float medido = lerMm(resto);
  CalibDist r;
  // Sem encoder: contagens 0 e resolução 0 (só os passos/mm saem daqui).
  CalibDistErro e = calib_distancia(zPendPassos, 0, zPendMm, medido, DIST_TOLERANCIA_PCT, 0.0f, &r);
  if (e == CAL_DIST_ENTRADA_INVALIDA) {
    Serial.printf("%s: '%s'. Envie ZM<mm>, ex.: ZM49.6\n", calib_dist_erro_nome(e), resto.c_str());
    return;
  }
  Serial.printf("pedido %.2f mm, medido %.2f mm -> erro %+.2f%%\n", zPendMm, medido, r.erro_pct);
  if (e == CAL_DIST_FORA_DA_TOLERANCIA) {
    Serial.printf("NAO aplicado: %s (limite +-%.0f%%).\n", calib_dist_erro_nome(e),
                  DIST_TOLERANCIA_PCT);
    Serial.printf("   avanco medido = %.1f mm/volta: se for isso mesmo, mude Z_AVANCO_MM no main.cpp\n",
                  PASSOS_POR_VOLTA / r.passos_por_mm);
    return;
  }
  float ant = zPassosPorMm;
  zPassosPorMm = r.passos_por_mm;
  zSalvarCalibracao();
  zMedidaPendente = false;
  Serial.printf("Z passos/mm: %.3f -> %.3f (avanco %.2f mm/volta). Salvo.\n", ant, zPassosPorMm,
                PASSOS_POR_VOLTA / zPassosPorMm);
  // O zero (topo) vem do homing no fundo convertido com os passos/mm antigos:
  // com a calibração nova ele ficou deslocado. Precisa refazer o ZH.
  if (zReferenciado && Z_TEM_CHAVE) {
    zReferenciado = false;
    cancelarPrograma();
    Serial.println("   refaca o homing do Z (ZH): a posicao do topo mudou com a calibracao");
  }
}

// Família Z: Z, Z40, Z+5, Z-5, ZH, ZR, ZL, ZV300, ZA600, ZN150, ZD50, ZM49.6, ZK, ZK0.
static void comandoZ(String resto) {
  resto.trim();
  if (resto.length() == 0) {
    zMostrar();
    return;
  }
  char sub = resto[0];
  String arg = resto.substring(1);
  arg.trim();

  // Mostrar e medidas valem a qualquer momento.
  if (sub == 'M') {
    zReceberMedida(arg);
    return;
  }
  if (sub == 'K') {
    if (arg == "0") {
      zPassosPorMm = Z_PASSOS_MM_NOMINAL;
      zSalvarCalibracao();
      Serial.println("-- calibracao do Z voltou ao nominal --");
      if (zReferenciado && Z_TEM_CHAVE) {
        zReferenciado = false;
        cancelarPrograma();
        Serial.println("   refaca o homing do Z (ZH): a posicao do topo mudou");
      }
    }
    zMostrar();
    return;
  }
  if (sub == 'V') {
    uint32_t v;
    if (arg.length() && lerInteiro(arg, false, &v)) {
      if (v > Z_VEL_LIMITE) {
        Serial.printf("acima do limite (%lu passos/s): ficou no limite\n", (unsigned long)Z_VEL_LIMITE);
        v = Z_VEL_LIMITE;
      }
      velocidadeZ = v;
      stepper_set_speed(&motZ, velocidadeZ);
    }
    zMostrar();
    return;
  }
  // ZA600 = aceleração (passos/s², ZA0 = sem rampa); ZN150 = velocidade de partida.
  // Valem no próximo movimento. Mais devagar = mais torque no 28BYJ-48.
  if (sub == 'A' || sub == 'N') {
    uint32_t v;
    if (arg.length() && lerInteiro(arg, true, &v)) {
      StepperPerfil p;
      stepper_get_profile(&motZ, &p);
      if (sub == 'A') {
        p.acel = v;
      } else {
        if (v == 0 || v > Z_VEL_LIMITE) {
          Serial.printf("partida invalida (use 1 a %lu passos/s)\n", (unsigned long)Z_VEL_LIMITE);
          return;
        }
        p.v0 = v;
      }
      stepper_set_profile(&motZ, &p);
    }
    zMostrar();
    return;
  }

  // Daqui para baixo mexe no motor Z.
  if (alarme) {
    Serial.printf("ALARME ativo (%s): envie A para reconhecer\n", alarmeMotivo);
    return;
  }
  if (seq_ativo(&seq)) {
    Serial.println("programa rodando: P pausa, q cancela");
    return;
  }
  if (sub == 'L') {
    stepper_release(&motZ);
    andandoZ = false;
    zReferenciado = false;  // solto, o Z pode descer: a posição deixa de valer
    cancelarPrograma();
    Serial.println("-- Z solto: SEM corrente o Z pode descer pelo peso. Referencia perdida --");
    return;
  }
  if (stepper_busy(&motZ)) {
    Serial.printf("Z andando (%lu passos/s): q para\n", (unsigned long)stepper_speed_now(&motZ));
    return;
  }
  if (sub == 'R') {
    stepper_reset(&motZ, 0);
    zReferenciado = true;
    cancelarPrograma();  // pausado: os alvos em mm mudaram de origem
    if (Z_SEGURAR) stepper_hold(&motZ);
    Serial.println("-- Z referenciado: esta posicao e o topo (Z = 0) --");
    zMostrar();
    return;
  }
  if (sub == 'D') {
    zCalibrarDistancia(arg);
    return;
  }
  if (sub == 'H') {
    cancelarPrograma();  // pausado: os alvos em mm mudam de origem
    fazerHomingZ();
    zMostrar();
    return;
  }

  stepper_set_speed(&motZ, velocidadeZ);
  EixoMm e = eixoZmm();
  if (sub == '+' || sub == '-') {
    // Movimento relativo: + desce, - sobe.
    float mm = lerMm(arg);
    if (!(mm > 0.0f) || mm > CURSO_Z_MM) {
      Serial.printf("deslocamento invalido: '%s' (use Z+mm para descer ou Z-mm para subir, ate %.0f)\n",
                    resto.c_str(), CURSO_Z_MM);
      return;
    }
    float dz = sub == '+' ? mm : -mm;
    if (zReferenciado && !eixo_mm_dentro(&e, zMm() + dz)) {
      Serial.printf("fora do curso: Z iria para %.2f mm (0 a %.0f). Sem limite: ZL e ZR de novo\n",
                    zMm() + dz, CURSO_Z_MM);
      return;
    }
    int32_t passos = (int32_t)lroundf(dz * zPassosPorMm) * Z_DESCE;
    stepper_start(&motZ, passos, !Z_SEGURAR);
    andandoZ = true;
    Serial.printf("Z %s %.2f mm%s\n", dz > 0 ? "descendo" : "subindo", mm,
                  zReferenciado ? "" : " (sem referencia: sem limite de curso!)");
    return;
  }

  // Absoluto: Z40.
  if (!zReferenciado) {
    Serial.println("Z sem referencia: ZH (homing) ou leve ao topo (Z-1, Z-5...) e envie ZR");
    return;
  }
  float mm = lerMm(resto);
  const char *c = resto.c_str();
  if (!(*c >= '0' && *c <= '9') && *c != '.') {
    Serial.printf("comando Z invalido: 'Z%s' (Z, Z40, Z+5, Z-5, ZH, ZR, ZL, ZV, ZA, ZN, ZD50, ZM49.6, ZK)\n",
                  resto.c_str());
    return;
  }
  if (!eixo_mm_dentro(&e, mm)) {
    Serial.printf("posicao invalida: '%s' (use Z de 0 a %.0f mm)\n", resto.c_str(), CURSO_Z_MM);
    return;
  }
  int32_t alvo = eixo_mm_para_passos(&e, mm);
  if (alvo == stepper_position(&motZ)) {
    zMostrar();
    return;
  }
  zIniciar(alvo);
  andandoZ = true;
  Serial.printf("indo para Z = %.2f mm (q para parar)\n", mm);
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

// Pontos do vaivém em X (centro do braço): os mesmos 10 e 40 mm da chave de antes.
static const float TESTE_A_MM = 40.0f;
static const float TESTE_B_MM = 70.0f;
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

static int32_t alvoMm(float mm) {
  EixoMm e = eixoMm();
  return eixo_mm_para_passos(&e, mm);
}
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
  EixoMm e = eixoMm();
  if (!eixo_mm_dentro(&e, fim_mm)) {
    Serial.printf("o teste vai ate X = %.0f mm, alem do curso (%.0f a %.0f mm)\n", fim_mm,
                  eixo_mm_min(&e), eixo_mm_max(&e));
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

static void varrerVelocidadeSemProtecao(String resto);

// A varredura procura de propósito a velocidade em que o motor perde passo:
// com a proteção ligada, ela viraria alarme de colisão no primeiro erro.
static void varrerVelocidade(String resto) {
  colisao_habilitar(&colisao, false);
  Serial.println("   protecao contra colisao DESLIGADA durante a varredura");
  varrerVelocidadeSemProtecao(resto);
  colisao_habilitar(&colisao, true);
  Serial.println("   protecao contra colisao religada");
}

static void varrerVelocidadeSemProtecao(String resto) {
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
  Serial.println("== Bancada: eixo X + eixo Z ==");
  Serial.println("f = volta p/ frente, b = volta p/ tras, q = para, l = solta, r = zera");
  Serial.println("f3 / b0.5 = numero de voltas (sem numero = 1)");
  Serial.println("v800 = vel. maxima, a1500 = aceleracao (a0 sem rampa), n300 = partida");
  Serial.println("teste de bobinas: TX / TZ escolhe o motor, 1..4 = so uma entrada,");
  Serial.println("           p/o = proxima/anterior fase, T0 = fim do teste");
  Serial.println("e = liga/desliga monitor do encoder (A e B crus)");
  Serial.println("h = homing, c3 = auto calibracao (3 ciclos), s = mostra a chave");
  Serial.println("d100 = calibra distancia, m99.6 = medida real, k = calibracao (k0 = nominal)");
  Serial.printf("x120 = vai para X = 120 mm (centro do braco, %.0f a %.0f mm; depois do homing)\n",
                X_HOME_MM, X_HOME_MM + CURSO_X_MM);
  Serial.printf("Z (topo 0 a %.0f mm, malha aberta): ZH homing, Z+5 desce, Z-5 sobe, ZR = topo aqui,\n",
                CURSO_Z_MM);
  Serial.println("           Z40 vai (h faz o homing do Z antes do X, se faltar)");
  Serial.println("           ZV300 velocidade, ZA600 aceleracao, ZN150 partida, ZD50 / ZM49.6 calibra,");
  Serial.println("           ZK calibracao, ZL solta (cai!)");
  Serial.println("setpoints: S lista, S<x> <z> [mm/s] [ms] acrescenta (ex.: S120 40), S-2 / S- apaga");
  Serial.println("           R / R3 / R0 executa 1x / 3x / sem fim, P pausa/continua, q cancela");
  Serial.println("colisao: para, solta o motor e liga o ALARME; A reconhece (depois h); C mostra, C0/C1");
  Serial.println("t, t2 = mede a velocidade com o encoder ligado e parado");
  Serial.println("w = varredura de velocidade (acha o limite), g = grava a curva da rampa (CSV)");
  Serial.printf("encoder: amostra a cada %u us (%u Hz), debounce %u us = %u amostras\n",
                ENCODER_SAMPLE_PERIOD_US, ENCODER_SAMPLE_RATE_HZ, ENCODER_DEBOUNCE_US,
                ENCODER_DEBOUNCE_SAMPLES);

  encoder_begin(&encX, ENC_X_A, ENC_X_B, false);
  stepper_begin(&motX, MOT_X_IN1, MOT_X_IN2, MOT_X_IN3, MOT_X_IN4);
  // Chave ativa em HIGH: chave NA para 3V3 e pull-down de 10 kΩ externo
  // para GND. false = ativa em HIGH.
  eixo_begin(&eixoX, &motX, &encX, HOME_X, false, false);
  configurarHoming();
  pinMode(ALARME_LED, OUTPUT);
  digitalWrite(ALARME_LED, LOW);
  ColisaoCfg cc = {COLISAO_JANELA_CONT, COLISAO_FRACAO_PCT, COLISAO_CONFIRMAR};
  colisao_init(&colisao, &cc, CONTAGENS_MM_NOMINAL / PASSOS_MM_NOMINAL);
  carregarCalibracao();
  colisao_escala(&colisao, contagensPorMm / passosPorMm);
  Serial.printf("protecao contra colisao: ligada, a cada %.1f mm o encoder anda >= %u%% (C0 desliga)\n",
                COLISAO_JANELA_CONT / contagensPorMm, COLISAO_FRACAO_PCT);
  mostrarCalibracao();
  carregarPrograma();
  listarPrograma();
  chaveAnterior = eixo_chave_crua(&eixoX);
  Serial.printf("sensor do homing: %s\n", chaveAnterior ? "APERTADO" : "solto");
  StepperPerfil perfil = {VEL_PARTIDA_PADRAO, VEL_MAX_PADRAO, ACEL_PADRAO};
  stepper_set_profile(&motX, &perfil);
  mostrarPerfil();

  // Z: liga as bobinas já no boot (o fuso de alto avanço não segura sozinho).
  stepper_begin(&motZ, MOT_Z_IN1, MOT_Z_IN2, MOT_Z_IN3, MOT_Z_IN4);
  StepperPerfil pz = {Z_PARTIDA, velocidadeZ, Z_ACEL};
  stepper_set_profile(&motZ, &pz);
  zCarregarCalibracao();
  if (Z_SEGURAR) stepper_hold(&motZ);
  // Chave do Z: igual à do X (NA para 3V3 + pull-down externo de 10 kΩ).
  eixo_begin(&eixoZ, &motZ, nullptr, HOME_Z, false, false);
  configurarHomingZ();
  chaveZAnterior = eixo_chave_crua(&eixoZ);
  if (Z_TEM_CHAVE) {
    Serial.printf("sensor do homing Z: %s  (ZH faz o homing do Z)\n",
                  chaveZAnterior ? "APERTADO" : "solto");
  }
  zMostrar();
  imprimir();
}

// Executa uma linha completa: a letra é o comando, o resto é o número.
static void executar(const String &linha) {
  char c = linha[0];
  String resto = linha.substring(1);
  // Programa rodando: só ele mexe no motor (pausado, os comandos manuais valem).
  // Alarme: nada mexe no motor até reconhecer com 'A'.
  if (strchr("fbhcdxtrwg1234poRPT", c) && alarme) {
    Serial.printf("ALARME ativo (%s): confira o eixo e envie A para reconhecer\n", alarmeMotivo);
    return;
  }
  if (strchr("fbhcdxtrwg1234poRT", c) && seq_ativo(&seq)) {
    Serial.println("programa rodando: P pausa, q cancela");
    return;
  }
  // Comandos que mexem no motor esperam o movimento atual terminar.
  if (strchr("fbhcdxtrwg1234poRT", c) && ocupado()) return;
  switch (c) {
    case 'f': mover(+1, resto); return;
    case 'b': mover(-1, resto); return;
    case 'q':
      if (cancelarPrograma()) return;
      stepper_stop(&motX);
      stepper_stop(&motZ);
      Serial.println("-- freando --");
      return;
    case 'Z': comandoZ(resto); return;
    case 'S': comandoSetpoint(resto); return;
    case 'R': rodarPrograma(resto); return;
    case 'P': pausarOuContinuar(); return;
    case 'A': reconhecerAlarme(); return;
    case 'C': comandoColisao(resto); return;
    case 'T': comandoTeste(resto); return;
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
      stepper_abort(&motZ);  // Z para, mas continua segurando (ZL solta)
      cancelarPrograma();
      andando = false;
      andandoZ = false;
      Serial.println("-- X solto; Z parado e segurando (ZL solta o Z) --");
      break;
    case 'r':
      stepper_reset(&motX, 0);
      encoder_reset(&encX, 0);
      referenciado = false;  // zero manual: a origem da chave deixou de valer
      cancelarPrograma();    // pausado: o alvo em mm não vale mais
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
  if (c != chaveAnterior) {
    chaveAnterior = c;
    Serial.println(c ? "[sensor] homing X APERTADO" : "[sensor] homing X solto");
  }
  if (!Z_TEM_CHAVE) return;
  bool z = eixo_chave(&eixoZ);
  if (z != chaveZAnterior) {
    chaveZAnterior = z;
    Serial.println(z ? "[sensor] homing Z APERTADO" : "[sensor] homing Z solto");
  }
}

void loop() {
  if (monitorEncoder) monitorarEncoder();
  vigiarChave();
  vigiarColisao();  // antes do sequenciador: colisão cancela o programa
  sinalizarAlarme();
  atualizarSequencia();
  if (andando && !stepper_busy(&motX)) {
    andando = false;
    Serial.printf("-- parou (pior avanco %ld%%, erro total max %ld cont) -- ",
                  (long)colisao_pior_pct(&colisao), (long)colisao_erro_max(&colisao));
    imprimir();
  }
  if (andandoZ && !stepper_busy(&motZ)) {
    andandoZ = false;
    Serial.print("-- Z parou -- ");
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
