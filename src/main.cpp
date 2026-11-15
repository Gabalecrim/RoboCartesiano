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
//   Z = descida a partir do topo: 0 = topo (seguro), CURSO_Z_MM (40) = embaixo (deslocamento máximo do Z, 2026-11-15).
//   Fuso igus de alto avanço, 8 entradas x ~3 mm = 24 mm/volta (85,33 passos/mm).
//   As bobinas do Z são SOLTAS ao parar (Z_SEGURAR = false): ligadas o tempo
//   todo o motor superaquecia, e a caixa de redução do 28BYJ-48 já segura o Z.
//   ZH        -> homing do Z: DESCE até a chave, sobe 2 mm, volta devagar;
//                Z = 40 (fundo) na chave e solta as bobinas. O 'R' faz o ZH
//                antes, se faltar. O homing do X ('h') NÃO mexe no Z.
//                Confira que não há nada embaixo do braço antes.
//   Z+5 / Z-5 -> desce / sobe 5 mm (vale sem referência)
//   ZR        -> "aqui é o topo": Z = 0 (referência manual, sem a chave)
//   Z40       -> vai para Z = 40 mm (precisa de ZH ou ZR);  Z -> mostra
//   ZV300     -> velocidade do Z (passos/s, padrão Z_VEL_PADRAO);  ZL -> solta as bobinas
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
//   O X vai pela posição do ENCODER (lib/malha): parado, o erro no encoder
//   vira correção (até MALHA_CORRECOES); não chegou = programa cancelado.
//
// Posição do X pelo encoder (2026-11-15):
//   x, setpoints e programa usam o encoder como referência. O alvo em mm
//   vira contagens; os passos saem do erro medido. Depois de parar (e de
//   MALHA_ASSENTAR_MS), se |erro| > MALHA_TOL_CONT (0,1 mm) corrige.
//   Passo perdido não tira mais a referência: o encoder sabe onde o carro está.
//   f / b continuam em passos (diagnóstico). O Z continua em malha aberta.
//   Limites dos dois motores: VEL_LIMITE (500 passos/s), ACEL_LIMITE (1000 passos/s²).
//
// Travamento do X (lib/colisao detecta; SEM alarme desde 2026-11-15):
//   Durante qualquer movimento, a cada 1 mm (COLISAO_JANELA_CONT) o encoder
//   tem que andar pelo menos COLISAO_FRACAO_PCT do esperado. Abaixo disso
//   em COLISAO_CONFIRMAR janelas seguidas, o X TRAVOU (motor para,
//   ressincroniza e continua o que faltava, já com menos rotação).
//
// Setpoint de velocidade do X pelo encoder (lib/governo, 2026-11-15):
//   SETPOINT = velocidade pedida (v em passos/s, V em mm/s, ou a do ponto do
//   programa). A cada GOV_AMOSTRA_MS mede a velocidade REAL do carro pelo
//   encoder (janela de ~GOV_JANELA_MS) e compara com a do motor:
//     - real < GOV_QUEDA_PCT da do motor (perdendo passo) = 1 queda; com
//       N quedas SEGUIDAS (GQ) a rotação cai, ao vivo, para o menor entre o
//       fator de desaceleração (GD %) da do motor e a real (mais torque),
//       sem descer de GOV_VEL_MIN;
//     - a cada GOV_INTERVALO_MS (1 s) sem queda, tenta recuperar: sobe o
//       fator de aceleração (GA %) do que falta até o setpoint;
//     - depois de SUBIR, mantém a rotação pelo menos GS ms (>= 1000): quedas
//       nesse tempo não reduzem (mas seguram a próxima subida).
//   G    -> mostra;  GQ2 quedas para reduzir;  GA50 fator de aceleração (%);
//           GD70 fator de desaceleração (%);  GS1000 mantém depois de subir (ms).
//           Ficam salvos na placa.
//   V    -> mostra o setpoint, a velocidade real e a rotação;  V1.8 -> setpoint em mm/s
//   NADA PARA O SISTEMA por interpretação do encoder (2026-11-15): queda e
//   travada só baixam a rotação ao vivo; a malha do encoder corrige a
//   posição no fim e, se não chegar, só avisa. Homing: HT vem desligado.
//   Telemetria: sempre (andando 10/s, parado 4/s) sai uma linha "@T" (painel desenha):
//     @T <ms> <X mm pelo encoder> <encoder contagens> <vx passos/s> <vx mm/s> <limite>
//        <Z mm> <vz passos/s> <vz mm/s> <ref X><ref Z> <real passos/s> <setpoint passos/s>
//        <alvo X mm> <alvo X cont> <m/c/f = movendo/chegou/fora> <alvo Z mm>  ("-" = sem alvo)
//        <ok / SEMSINAL = encoder do X sem sinal>
//   C    -> mostra a detecção e o governo; C0 / C1 desliga / liga
//          (C0 desliga também o travamento do homing, para diagnosticar)
//
// Ajustes do homing X (valem até reiniciar; para ficar, configurarHoming()):
//   H mostra;  HV500 busca rápida;  HL150 busca lenta (passos/s);
//   HT256 janela de travamento em passos (0 = desliga);  HS+1 / HS-1 sentido;
//   HD diagnóstico: anda 2 mm para a chave e confere encoder e chave.
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
#include "governo.h"
#include "malha.h"
#include "setpoint.h"
#include "stepper.h"

static const int32_t PASSOS_POR_VOLTA = 2048;  // 28BYJ-48, passo completo
// Perfil inicial da rampa (ajuste na bancada com v, a, n).
// Medido com 'w' (2026-11-10, a1500): 650 passos/s ok, 700 perde passo, e a
// aceleração (1000 a 2500) não mudou o limite.
// Limites dos DOIS motores (decisão do Gab, 2026-11-15): 500 passos/s e
// 1000 passos/s². v, a, ZV, ZA, HV, HL e os setpoints não passam disso.
// (A varredura 'w' passa de propósito: ela procura onde o motor perde passo.)
static const uint32_t VEL_LIMITE = 500;   // passos/s (~14,6 RPM, ~1,95 mm/s no X)
static const uint32_t ACEL_LIMITE = 1000; // passos/s²
// Padrão = sempre a máxima (decisão do Gab, 2026-11-12).
static const uint32_t VEL_MAX_PADRAO = VEL_LIMITE;
static const uint32_t ACEL_PADRAO = ACEL_LIMITE;  // passos/s²
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
// Posição do X pelo encoder (2026-11-15, lib/malha): o alvo é em contagens;
// depois de cada movimento, parado, o erro no encoder vira correção.
// 1 contagem = 0,1 mm (KY-040: 80 contagens/volta, fuso de 8 mm).
static const int32_t MALHA_TOL_CONT = 1;      // chegou: |erro| <= 0,1 mm
static const uint8_t MALHA_CORRECOES = 4;     // correções antes de desistir
static const uint32_t MALHA_ASSENTAR_MS = 30; // espera o encoder assentar (debounce 1 ms)
// Detecção de travamento (2026-11-15, por janelas; ver lib/colisao): a cada
// 1 mm o encoder tem que andar pelo menos 40% do esperado. Travou = encoder
// quase parado. A 1ª janela de cada movimento é ignorada (folga).
static const int32_t COLISAO_JANELA_CONT = 10;  // contagens = 1 mm
static const uint8_t COLISAO_FRACAO_PCT = 40;   // avanço mínimo na janela
static const uint8_t COLISAO_CONFIRMAR = 2;     // janelas ruins seguidas
// Setpoint de velocidade do X pelo encoder (lib/governo). A 500 passos/s o
// encoder dá ~20 contagens/s: a janela de 0,5 s tem ~10 (±10% de resolução).
static const uint32_t GOV_AMOSTRA_MS = 50;    // amostra motor x encoder
static const uint32_t GOV_JANELA_MS = 500;    // janela mínima de medição
static const int32_t GOV_MIN_CONT = 5;        // contagens esperadas para avaliar
static const uint8_t GOV_QUEDA_PCT = 75;      // real abaixo disso da do motor = caiu
static const uint32_t GOV_VEL_MIN = 120;      // passos/s
static const uint32_t GOV_INTERVALO_MS = 1000;  // tenta recuperar o setpoint a cada 1 s
static const uint32_t GOV_SUBIDA_MIN = 25;    // passos/s mínimos por tentativa
// Configuráveis (G, painel), salvos na placa. Padrões:
static const uint8_t GOV_QUEDAS_PADRAO = 2;     // quedas seguidas para reduzir
static const uint8_t GOV_SUBIDA_PCT_PADRAO = 50;   // fator de aceleração (% do que falta)
static const uint8_t GOV_REDUZIR_PCT_PADRAO = 70;  // fator de desaceleração (% mantido)
static const uint32_t GOV_SEGURAR_MS_MIN = 1000;   // depois de subir, mantém pelo menos 1 s
static uint8_t govQuedas = GOV_QUEDAS_PADRAO;
static uint8_t govSubidaPct = GOV_SUBIDA_PCT_PADRAO;
static uint8_t govReduzirPct = GOV_REDUZIR_PCT_PADRAO;
static uint32_t govSegurarMs = GOV_SEGURAR_MS_MIN;
static const uint8_t HOM_TENTATIVAS = 3;      // homing que trava: refaz no limite novo
static const uint32_t TELEM_MS = 100;         // telemetria "@T" andando (10 por segundo)
static const uint32_t TELEM_PARADO_MS = 250;  // parado (4 por segundo): posição sempre atualizada
static const float ENC_SEM_SINAL_MM = 5.0f;  // motor andou isso sem o encoder mudar = encoder sem sinal

// Eixo Z (2026-11-14): malha aberta, sem sensor de homing por enquanto.
// Fuso igus de alto avanço: 8 entradas, ~3 mm entre roscas, Ø ~6 mm.
// Avanço = entradas x passo = 8 x 3 = 24 mm por volta. Conferir com ZD.
static const float Z_AVANCO_MM = 24.0f;
static const float Z_PASSOS_MM_NOMINAL = PASSOS_POR_VOLTA / Z_AVANCO_MM;  // 85,33
static const float CURSO_Z_MM = 40.0f;  // deslocamento máximo do Z (era 80; decisão do Gab, 2026-11-15)
// Giro do motor que DESCE o Z: +1 = sentido do 'f'. Na bancada (2026-11-14) o Z
// andou ao contrário com +1: ficou -1. Se Z+5 subir, troque o sinal.
static const int8_t Z_DESCE = -1;
// Teto do ZV (mesmo motor do X). O Z NÃO subia bem a 650 (2026-11-14): o
// 28BYJ-48 tem mais torque devagar, então o padrão é Z_VEL_PADRAO.
// Nada disso foi medido no Z (sem encoder não há varredura automática):
// ajuste com ZV, ZA e ZN e confira com ciclos e régua.
static const uint32_t Z_VEL_LIMITE = VEL_LIMITE;  // 500 passos/s
static const uint32_t Z_VEL_PADRAO = 300;  // passos/s (~3,5 mm/s): padrão do Z
static const uint32_t Z_ACEL = 600;       // passos/s² (rampa suave; teto ACEL_LIMITE)
static const uint32_t Z_PARTIDA = 150;    // passos/s: partida lenta = mais torque
// false (2026-11-15): com as bobinas sempre ligadas o motor do Z superaquecia.
// A caixa de redução do 28BYJ-48 já segura o Z parado, então as bobinas são
// soltas ao fim de todo movimento. true = volta a segurar com corrente.
static const bool Z_SEGURAR = false;

// Para o Z na hora: solto (padrão) ou segurando, conforme Z_SEGURAR.
static void zPararNaHora();
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

// ---------- travamento do X (definido mais abaixo) ----------
static ColisaoMonitor colisao;
static Gov govX;  // limite de velocidade do X (lib/governo)
static bool vigiarColisao();
static void governarX();
static void telemetria();
// Alvo do Z (passos) para o painel: quem manda o Z andar não guarda; com o Z
// andando, é a posição + o que falta. Fica o último até um novo movimento.
static int32_t zAlvoPassos = 0;
static bool zTemAlvo = false;
// Encoder do X sem sinal: o motor andou ENC_SEM_SINAL_MM e o encoder não
// mudou NENHUMA contagem (fato medido, não interpretação). Não para nada: só
// impede usar uma posição falsa (sem correção da malha, sem reduzir rotação,
// sem movimento novo pelo encoder). Volta sozinho quando o encoder contar.
static bool encSemSinal = false;
static bool homTravou = false;     // o encoder parou durante o homing do X

// ---------- diagnóstico do homing do X ----------
// Onde o homing do X estava: diz em que fase travou (e que travada é do
// homing, não de um movimento comum). Preenchido por avisoHoming (eixo "X").
struct HomingCtx {
  bool ativo;          // homing do X em andamento (ou acabou de falhar)
  HomingState fase;    // última fase em que o motor andava
  int8_t giro;         // giro nessa fase: +1 = f, -1 = b
  int32_t motor0;      // motor no início da fase
  int32_t enc0;        // encoder no início da fase
};
static HomingCtx homCtx = {false, HOM_PARADO, 0, 0, 0};
// Janela de travamento do homing (passos sem o encoder mudar = travou e o
// homing PARA). 0 = desligado (padrão desde 2026-11-15: não para por
// interpretação do encoder; o homing termina pela chave ou pelo curso).
// HT256 liga de novo, só para diagnóstico.
static int32_t homTravaPassos = 0;
static void aplicarTravaHoming();

// Coordenada X em mm (centro do braço), com a calibração atual.
static EixoMm eixoMm() {
  return EixoMm{X_HOME_MM, CURSO_X_MM, passosPorMm, (int8_t)-homingX.sentido};
}
static float vmaxMm() { return VEL_LIMITE / passosPorMm; }  // mm/s

// X do centro do braço: SÓ pelo encoder (zera na chave do homing).
static float encoderMm() {
  return X_HOME_MM + (float)(-homingX.sentido * encoder_count(&encX)) / contagensPorMm;
}

// ---------- eixo X: posição pelo encoder (lib/malha) ----------
static Malha malhaX;
static uint32_t xParadoDesde = 0;  // millis() da última vez que o motor estava andando

// mm (centro do braço) -> contagens do encoder (inverso de encoderMm).
static int32_t mmParaContagens(float mm) {
  return (int32_t)lroundf(-homingX.sentido * (mm - X_HOME_MM) * contagensPorMm);
}
// contagens do encoder -> mm (centro do braço); o mesmo que encoderMm() para um valor qualquer.
static float contagensParaMm(int32_t c) { return X_HOME_MM + (float)(-homingX.sentido * c) / contagensPorMm; }

static void xEscala() {
  malha_escala(&malhaX, passosPorMm / contagensPorMm);
  gov_escala(&govX, contagensPorMm / passosPorMm);
}

// X ocupado: motor andando ou a malha ainda corrigindo.
static bool xOcupado() { return stepper_busy(&motX) || malha_ativa(&malhaX); }

// Manda o X para 'alvo' contagens do encoder (sem bloquear). false = já estava
// lá, ou o encoder está sem sinal (andar por uma posição falsa jogaria o carro
// no fim do curso: o erro nunca diminui).
static bool xIrContagens(int32_t alvo) {
  if (encSemSinal) {
    Serial.println("[encoder X] SEM SINAL: movimento pelo encoder nao feito (a posicao lida e falsa)."
                   " Confira cabo, 3V3 e acoplamento; teste: e + girar o fuso a mao");
    return false;
  }
  int32_t passos = malha_ir(&malhaX, alvo, encoder_count(&encX));
  if (passos == 0) return false;
  stepper_start(&motX, passos, true);  // solta as bobinas ao chegar (TR8x8 trava)
  xParadoDesde = millis();
  return true;
}

static bool xIr(float mm) { return xIrContagens(mmParaContagens(mm)); }

// Chamada a cada volta do loop(): com o motor parado e o encoder assentado,
// mede o erro e corrige (ou termina).
static void xAtualizarMalha() {
  if (!malha_ativa(&malhaX)) return;
  if (encSemSinal) {  // sem encoder não há erro para corrigir: termina o movimento como está
    malha_cancelar(&malhaX);
    return;
  }
  if (stepper_busy(&motX)) {
    xParadoDesde = millis();
    return;
  }
  if (millis() - xParadoDesde < MALHA_ASSENTAR_MS) return;
  int32_t passos = malha_parou(&malhaX, encoder_count(&encX));
  if (passos != 0) {
    Serial.printf("[malha X] correcao %u: erro %+ld cont (%+.2f mm) -> %+ld passos\n",
                  malha_correcoes(&malhaX), (long)malha_erro(&malhaX),
                  -homingX.sentido * malha_erro(&malhaX) / contagensPorMm, (long)passos);
    stepper_start(&motX, passos, true);
    xParadoDesde = millis();
  } else if (malha_estado(&malhaX) == MALHA_FALHOU) {
    Serial.printf("[malha X] aviso: ficou a %+ld cont (%+.2f mm) do alvo depois de %u correcoes"
                  " (so aviso: nada parou)\n",
                  (long)malha_erro(&malhaX), -homingX.sentido * malha_erro(&malhaX) / contagensPorMm,
                  malha_correcoes(&malhaX));
  }
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

static void zPararNaHora() {
  if (Z_SEGURAR) {
    stepper_abort(&motZ);  // para e continua com corrente
  } else {
    stepper_release(&motZ);  // para e solta: a caixa de redução segura
  }
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
  governarX();   // também nas esperas bloqueantes (homing, testes, calibrações)
  telemetria();
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
  if (temEnc) {
    if (ev->tipo == EIXO_EV_ESTADO) {
      if (ev->inicio) homCtx.ativo = true;
      if (ev->giro != 0) {  // fase em que o motor anda: guarda o ponto de partida
        homCtx.fase = ev->estado;
        homCtx.giro = ev->giro;
        homCtx.motor0 = ev->motor;
        homCtx.enc0 = ev->enc;
      }
      if (ev->estado == HOM_OK) homCtx.ativo = false;
    }
  }
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
  homingZ.segurar = Z_SEGURAR;  // false: termina solto (a caixa de redução segura)
  homingZ.cancelar = cancelarPelaSerial;
  homingZ.evento = avisoHoming;
  homingZ.ctx = (void *)"Z";
}

static void configurarHoming() {
  homingX.sentido = +1;  // chave no lado 'f' (giro do comando f). Lado 'b': -1
  homingX.fsm.curso_max = 30 * PASSOS_POR_VOLTA;  // maior que o curso do eixo
  homingX.fsm.recuo = PASSOS_POR_VOLTA / 4;       // 1/4 de volta além de soltar
  homingX.fsm.recuo_max = 4 * PASSOS_POR_VOLTA;   // a chave tem que soltar antes disso
  homingX.fsm.janela_trava = homTravaPassos;      // 0 = não para por encoder (HT liga)
  homingX.vel_rapida = 500;                       // passos/s
  homingX.vel_lenta = 150;                        // passos/s: define a precisão da origem
  homingX.cancelar = cancelarPelaSerial;
  homingX.evento = avisoHoming;
  homingX.segurar = false;  // o TR8x8 trava sozinho: solta (não aquece)
  homingX.ctx = (void *)"X";
}

static void imprimir() {
  // A posição do X é SÓ a do encoder (os passos do motor não entram).
  Serial.printf("encoder X: %ld cont (erros %lu)%s", (long)encoder_count(&encX),
                (unsigned long)encoder_errors(&encX), encSemSinal ? " SEM SINAL" : "");
  if (referenciado) {
    Serial.printf("   X: %.2f mm", encoderMm());
  } else {
    Serial.print("   X: sem homing");
  }
  // Alvo do X (pelo encoder). Sem homing só em contagens: em mm não teria origem.
  if (malha_estado(&malhaX) != MALHA_PARADA) {
    int32_t a = malha_alvo(&malhaX);
    const char *est = malha_estado_nome(malha_estado(&malhaX));
    if (referenciado) {
      Serial.printf("   alvo X: %.2f mm (%s)", contagensParaMm(a), est);
    } else {
      Serial.printf("   alvo X: %ld cont (%s)", (long)a, est);
    }
  }
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
    if (v > VEL_LIMITE) {
      Serial.printf("acima do limite (%lu passos/s): ficou no limite\n", (unsigned long)VEL_LIMITE);
      v = VEL_LIMITE;
    }
    velocidade = v;
    stepper_set_speed(&motX, velocidade);
  }
  mostrarPerfil();
}

static void ajustarAceleracao(String resto) {
  uint32_t a;
  if (lerInteiro(resto, true, &a)) {
    if (a > ACEL_LIMITE) {
      Serial.printf("acima do limite (%lu passos/s2): ficou no limite\n", (unsigned long)ACEL_LIMITE);
      a = ACEL_LIMITE;
    }
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
    if (v0 > VEL_LIMITE) v0 = VEL_LIMITE;
    StepperPerfil p;
    stepper_get_profile(&motX, &p);
    p.v0 = v0;
    stepper_set_profile(&motX, &p);
  }
  mostrarPerfil();
}

// Movimento sem bloqueio em andamento? Avisa e devolve true.
static bool ocupado() {
  if (!xOcupado()) return false;
  Serial.printf("motor andando (%lu passos/s): q para parar, l para soltar\n",
                (unsigned long)stepper_speed_now(&motX));
  return true;
}

// Para avisar quando um movimento sem bloqueio termina.
static bool andando = false;

static void imprimirHome(const EixoHomeResult &r) {
  if (r.erro == HOM_ERRO_NENHUM) {
    Serial.printf("homing ok em %lu passos\n", (unsigned long)r.passos_dados);
  } else if (r.erro == HOM_ERRO_CANCELADO && homTravou) {
    // Quem parou foi a detecção de travamento (encoder parado), já mais devagar.
    Serial.printf("homing FALHOU (travou: encoder parado) na fase '%s', depois de %lu passos\n",
                  homing_state_nome(homCtx.fase), (unsigned long)r.passos_dados);
    Serial.println("   confira o eixo (ponto duro? acoplamento? encoder solto?) ou baixe HV/HL");
  } else {
    Serial.printf("homing FALHOU (%s) na fase '%s', depois de %lu passos\n",
                  homing_error_nome(r.erro),
                  homing_state_nome(r.parou_em == HOM_ERRO ? homCtx.fase : r.parou_em),
                  (unsigned long)r.passos_dados);
    if (r.erro == HOM_ERRO_CURSO) {
      Serial.printf("   andou o curso todo sem a chave acionar: confira a chave (s), o pino "
                    "(HOME_X = GPIO %u) e o sentido (HD testa)\n", HOME_X);
    } else if (r.erro == HOM_ERRO_CHAVE_PRESA) {
      Serial.println("   a chave nao soltou: chave NF no lugar de NA, fio em curto com o 3V3 "
                     "ou chave travada (s mostra o pino)");
    }
    if (r.erro == HOM_ERRO_TRAVOU) {
      Serial.println("   travou (encoder parado com o motor girando) mesmo devagar: confira o eixo");
    }
  }
}

// Espera o Z parar (bloqueia). false se cancelarem pela serial (o Z para e segura).
static bool zEsperarParar() {
  while (stepper_busy(&motZ)) {
    if (cancelarPelaSerial(nullptr)) {
      zPararNaHora();
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
// A chave fica EMBAIXO: desce até ela e termina ali (solto se !Z_SEGURAR), com
// Z = Z_HOME_MM (40 mm, o fundo). A coordenada não muda: 0 continua no topo.
// true = deu certo.
// Último homing do Z, para os testes (E): passos entre onde a chave estava e
// onde deveria estar (~0 = não perdeu passo). Só vale com referência antes.
static int32_t zUltimaDif = 0;
static bool zUltimaDifValida = false;

static bool fazerHomingZ() {
  zUltimaDifValida = false;
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
  zTemAlvo = false;  // a coordenada do Z mudou: o alvo antigo não vale
  andandoZ = false;
  if (zReferenciado) {
    // O eixo_home zerou o motor na chave; a chave é o fundo, não o zero.
    int32_t naChave = zPassosNaChave();
    stepper_reset(&motZ, naChave);
    Serial.printf("homing do Z ok em %lu passos (%.1f mm): Z = %.0f mm na chave (fundo)\n",
                  (unsigned long)r.passos_dados, r.passos_dados / zPassosPorMm, Z_HOME_MM);
    // Com referência antes: onde a chave estava x onde deveria estar.
    // ~0 = o Z não perdeu passo desde o último homing.
    zUltimaDifValida = tinhaRef;
    if (tinhaRef) {
      int32_t dif = r.passos_na_chave - naChave;
      zUltimaDif = dif;
      Serial.printf("   chave a %ld passos (%.2f mm) de onde estava antes\n", (long)dif,
                    dif / zPassosPorMm);
    }
  } else {
    Serial.printf("homing do Z FALHOU (%s) no estado '%s', depois de %lu passos. "
                  "Z parado, SEM referencia\n",
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
// Último homing do X, para os testes (E): onde a chave acionou na origem
// anterior. Com referência antes, ~0 = a origem repetiu.
static EixoHomeResult xUltimoHome = {};
static bool xUltimoTinhaRef = false;

static void fazerHoming() {
  malha_cancelar(&malhaX);
  Serial.println("-- homing (Enter com qualquer tecla cancela) --");
  xUltimoTinhaRef = referenciado;
  EixoHomeResult &r = xUltimoHome;
  // Travou (encoder parado): refaz o homing. A travada já baixou o limite do
  // governo (mais torque); as buscas andam no menor entre HV/HL e o limite.
  for (uint8_t tentativa = 1;; tentativa++) {
    homTravou = false;
    eixo_home(&eixoX, &homingX, &r);
    // Travamento do próprio homing (HT): o governo também precisa saber.
    if (r.erro == HOM_ERRO_TRAVOU) {
      gov_travou(&govX, 0);
      stepper_set_limit(&motX, gov_limite(&govX));
    }
    bool travou = r.erro == HOM_ERRO_TRAVOU || (r.erro == HOM_ERRO_CANCELADO && homTravou);
    if (!travou || tentativa >= HOM_TENTATIVAS) break;
    Serial.printf("[trava X] homing travou na fase '%s' (tentativa %u/%u): refazendo no limite de"
                  " %lu passos/s\n",
                  homing_state_nome(homCtx.fase), tentativa, HOM_TENTATIVAS,
                  (unsigned long)gov_limite(&govX));
  }
  homCtx.ativo = false;  // acabou: travada daqui em diante não é do homing
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
  xEscala();
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

// x120 -> vai para X = 120 mm;  x+10 / x-1 -> anda 10 mm / volta 1 mm.
// Tudo pelo ENCODER. O relativo vale sem homing (conta a partir da leitura
// atual do encoder); com homing, não passa do curso.
static void irPara(String resto) {
  resto.trim();
  if (resto.length() > 0 && (resto[0] == '+' || resto[0] == '-')) {
    float d = lerMm(resto.substring(1));
    if (!(d > 0.0f) || d > CURSO_X_MM) {
      Serial.printf("deslocamento invalido: '%s' (use x+mm ou x-mm, ate %.0f)\n", resto.c_str(),
                    CURSO_X_MM);
      return;
    }
    if (resto[0] == '-') d = -d;
    if (referenciado) {
      EixoMm e = eixoMm();
      float alvo = encoderMm() + d;
      if (!eixo_mm_dentro(&e, alvo)) {
        Serial.printf("fora do curso: X iria para %.2f mm (%.0f a %.0f)\n", alvo, eixo_mm_min(&e),
                      eixo_mm_max(&e));
        return;
      }
    }
    int32_t alvoCont = encoder_count(&encX) + (int32_t)lroundf(-homingX.sentido * d * contagensPorMm);
    if (xIrContagens(alvoCont)) {
      andando = true;
      Serial.printf("X %s %.2f mm pelo encoder (alvo %ld cont; q para parar)%s\n",
                    d > 0 ? "anda" : "volta", fabsf(d), (long)alvoCont,
                    referenciado ? "" : " (sem homing: sem limite de curso!)");
      return;
    }
    imprimir();
    return;
  }
  if (!referenciado) {
    Serial.println("sem referencia: faca o homing primeiro (h ou d); x+mm / x-mm valem sem homing");
    return;
  }
  if (resto.length() > 0) {
    float mm = lerMm(resto);
    EixoMm e = eixoMm();
    if (!eixo_mm_dentro(&e, mm)) {
      Serial.printf("posicao invalida: '%s' (use X de %.0f a %.0f mm)\n", resto.c_str(),
                    eixo_mm_min(&e), eixo_mm_max(&e));
      return;
    }
    if (xIr(mm)) {  // pelo encoder; volta na hora e solta ao chegar
      andando = true;
      Serial.printf("indo para X = %.2f mm pelo encoder (q para parar)\n", mm);
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
    malha_cancelar(&malhaX);  // senão, parado, ela corrigiria de volta ao alvo
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
    // Sempre chama xIr: mesmo sem mover, ela registra o resultado (CHEGOU).
    stepper_set_speed(&motX, velocidadeDoPonto(sp, passosPorMm, velocidade, VEL_LIMITE));
    if (xIr(sp->x_mm)) Serial.printf("[seq]   X -> %.2f mm\n", sp->x_mm);
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
  bool ocupado = xOcupado() || stepper_busy(&motZ);

  if (seq_estado(&seq) == SEQ_MOVENDO && !ocupado && seq.fase == 1) {
    // Chegou no ponto. O X foi pelo encoder: passo perdido já foi corrigido.
    // Se a malha não chegou na tolerância, só AVISA: o programa continua.
    Serial.printf("[seq] chegou: X %.2f mm (encoder %ld cont) Z %.2f mm", encoderMm(),
                  (long)encoder_count(&encX), zMm());
    const Setpoint *sp = seq_alvo(&seq);
    if (sp->espera_ms) Serial.printf(", espera %lu ms", (unsigned long)sp->espera_ms);
    Serial.println();
    if (malha_estado(&malhaX) == MALHA_FALHOU) {
      Serial.printf("[seq] aviso: X ficou a %.2f mm do ponto pelo encoder (continua)\n",
                    -homingX.sentido * malha_erro(&malhaX) / contagensPorMm);
    }
  }

  SeqEstado antes = seq_estado(&seq);
  executarAcao(seq_atualizar(&seq, ocupado, millis()));
  SeqEstado agora = seq_estado(&seq);
  if (agora == antes) return;
  if (agora == SEQ_PAUSADO) {
    Serial.printf("[seq] pausado em X %.2f mm Z %.2f mm: P continua, q cancela\n", encoderMm(),
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

// ---------- X: setpoint de velocidade pelo encoder (sem alarme) ----------
// (2026-11-15) O governo mede a velocidade real pelo encoder; caiu -> menos
// rotação (ao vivo: stepper_set_limit muda a rampa no meio do movimento);
// a cada 1 s sem queda tenta voltar ao setpoint. Travou de vez (encoder
// parado) -> o motor ressincroniza e continua com menos rotação.
// O limite vale entre movimentos: o próximo começa nele e continua tentando.

static uint32_t govUltimoMs = 0;  // última atualização do governo
static uint32_t govAplicado = 0;  // limite já mandado ao motor
static bool govSuspenso = false;  // varredura w: sem limite

static void govAplicar() {
  if (govSuspenso) return;
  uint32_t l = gov_limite(&govX);
  if (l == govAplicado) return;
  stepper_set_limit(&motX, l);
  govAplicado = l;
}

// Setpoint em mm/s (o governo trabalha em passos/s do motor).
static float spMmS() { return gov_setpoint_atual(&govX) / passosPorMm; }
static float realMmS() { return gov_vel_real(&govX) / passosPorMm; }

// Configuração do governo: fixos + os ajustáveis pelo G (painel).
static GovCfg govMontarCfg() {
  GovCfg c = {VEL_LIMITE,    GOV_VEL_MIN,  GOV_QUEDA_PCT,    govReduzirPct,
              GOV_JANELA_MS, GOV_MIN_CONT, GOV_INTERVALO_MS, GOV_SUBIDA_MIN,
              govQuedas,     govSubidaPct, govSegurarMs};
  return c;
}

static void govCarregar() {
  prefs.begin("gov", true);
  govQuedas = prefs.getUChar("quedas", GOV_QUEDAS_PADRAO);
  govSubidaPct = prefs.getUChar("subida", GOV_SUBIDA_PCT_PADRAO);
  govReduzirPct = prefs.getUChar("reduzir", GOV_REDUZIR_PCT_PADRAO);
  govSegurarMs = prefs.getULong("segurar", GOV_SEGURAR_MS_MIN);
  prefs.end();
  // Valores salvos estranhos voltam ao padrão.
  if (govQuedas < 1 || govQuedas > 20) govQuedas = GOV_QUEDAS_PADRAO;
  if (govSubidaPct < 5 || govSubidaPct > 100) govSubidaPct = GOV_SUBIDA_PCT_PADRAO;
  if (govReduzirPct < 10 || govReduzirPct > 95) govReduzirPct = GOV_REDUZIR_PCT_PADRAO;
  if (govSegurarMs < GOV_SEGURAR_MS_MIN || govSegurarMs > 30000) govSegurarMs = GOV_SEGURAR_MS_MIN;
}

static void govSalvar() {
  prefs.begin("gov", false);
  prefs.putUChar("quedas", govQuedas);
  prefs.putUChar("subida", govSubidaPct);
  prefs.putUChar("reduzir", govReduzirPct);
  prefs.putULong("segurar", govSegurarMs);
  prefs.end();
}

static void govMostrar() {
  Serial.printf("governo X: quedas para reduzir %u, fator de aceleracao %u%%, fator de desaceleracao"
                " %u%%, mantem %lu ms depois de subir\n",
                govQuedas, govSubidaPct, govReduzirPct, (unsigned long)govSegurarMs);
  Serial.printf("   queda = encoder < %u%% do motor numa janela de %lu ms; recupera a cada %lu ms;"
                " rotacao minima %lu passos/s; %lu reducao(oes) desde o boot\n",
                GOV_QUEDA_PCT, (unsigned long)GOV_JANELA_MS, (unsigned long)GOV_INTERVALO_MS,
                (unsigned long)GOV_VEL_MIN, (unsigned long)gov_quedas(&govX));
}

// G -> mostra;  GQ2 quedas para reduzir;  GA50 fator de aceleração (% do que
// falta, por tentativa);  GD70 fator de desaceleração (% mantido na queda);
// GS1000 mantém depois de subir (ms, >= 1000);  G0 volta aos padrões.
static void comandoGoverno(String resto) {
  resto.trim();
  resto.toUpperCase();
  if (resto.length()) {
    char sub = resto[0];
    String num = resto.substring(1);
    num.trim();
    long v = num.toInt();
    bool numOk = num.length() > 0 && (v != 0 || num == "0");
    switch (sub) {
      case 'Q':
        if (!numOk || v < 1 || v > 20) {
          Serial.printf("quedas invalidas: '%s' (1 a 20)\n", num.c_str());
          return;
        }
        govQuedas = (uint8_t)v;
        break;
      case 'A':
        if (!numOk || v < 5 || v > 100) {
          Serial.printf("fator de aceleracao invalido: '%s' (5 a 100 %%)\n", num.c_str());
          return;
        }
        govSubidaPct = (uint8_t)v;
        break;
      case 'D':
        if (!numOk || v < 10 || v > 95) {
          Serial.printf("fator de desaceleracao invalido: '%s' (10 a 95 %%)\n", num.c_str());
          return;
        }
        govReduzirPct = (uint8_t)v;
        break;
      case 'S':
        if (!numOk || v < (long)GOV_SEGURAR_MS_MIN || v > 30000) {
          Serial.printf("tempo invalido: '%s' (%lu a 30000 ms)\n", num.c_str(),
                        (unsigned long)GOV_SEGURAR_MS_MIN);
          return;
        }
        govSegurarMs = (uint32_t)v;
        break;
      case '0':
        govQuedas = GOV_QUEDAS_PADRAO;
        govSubidaPct = GOV_SUBIDA_PCT_PADRAO;
        govReduzirPct = GOV_REDUZIR_PCT_PADRAO;
        govSegurarMs = GOV_SEGURAR_MS_MIN;
        break;
      default:
        Serial.printf("comando invalido: 'G%s' (G, GQ2, GA50, GD70, GS1000, G0)\n", resto.c_str());
        return;
    }
    GovCfg c = govMontarCfg();
    gov_configurar(&govX, &c);  // vale na hora, sem perder a rotação atual
    govSalvar();
  }
  govMostrar();
}

// Chamar com frequência (loop e esperas bloqueantes): a cada GOV_AMOSTRA_MS
// mede motor x encoder, baixa ou recupera a rotação e aplica na hora.
// Encoder do X sem sinal? Soma os passos do motor desde a última vez que o
// encoder mudou; passou de ENC_SEM_SINAL_MM sem mudar nada = sem sinal.
static int32_t encUltimaLeitura = 0;
static int32_t encMotorRef = 0;
static uint32_t encPassosSemMudar = 0;

static void vigiarSinalEncoder() {
  int32_t e = encoder_count(&encX);
  int32_t m = stepper_position(&motX);
  if (e != encUltimaLeitura) {
    encUltimaLeitura = e;
    encMotorRef = m;
    encPassosSemMudar = 0;
    if (encSemSinal) {
      encSemSinal = false;
      Serial.println("[encoder X] voltou a contar: faca o homing (h) para a posicao valer de novo");
    }
    return;
  }
  encPassosSemMudar += (uint32_t)(m > encMotorRef ? m - encMotorRef : encMotorRef - m);
  encMotorRef = m;
  if (encSemSinal || encPassosSemMudar < (uint32_t)(ENC_SEM_SINAL_MM * passosPorMm)) return;
  encSemSinal = true;
  malha_cancelar(&malhaX);  // não corrige por uma posição falsa
  referenciado = false;     // a posição do X (encoder) não vale
  // Sem velocidade real, o governo não tem o que medir: volta ao setpoint.
  GovCfg c = govMontarCfg();
  gov_init(&govX, &c, contagensPorMm / passosPorMm);
  govAplicar();
  Serial.printf("[encoder X] SEM SINAL: o motor andou %.1f mm e o encoder nao mudou nenhuma contagem"
                " (fica em %ld). Posicao do X perdida. Confira o cabo do encoder (GPIO %u/%u), o 3V3"
                " e o acoplamento; teste: e + girar o fuso a mao\n",
                encPassosSemMudar / passosPorMm, (long)e, ENC_X_A, ENC_X_B);
}

static void governarX() {
  uint32_t agora = millis();
  if (agora - govUltimoMs < GOV_AMOSTRA_MS) return;
  govUltimoMs = agora;
  vigiarSinalEncoder();
  StepperPerfil p;
  stepper_get_profile(&motX, &p);
  gov_setpoint(&govX, p.vmax);  // setpoint = velocidade pedida para este movimento
  uint32_t antes = gov_limite(&govX);
  // Sem sinal do encoder não há velocidade real para comparar: o governo fica de fora.
  GovEvento ev = gov_amostra(&govX, agora, stepper_busy(&motX) && !govSuspenso && !encSemSinal,
                             stepper_position(&motX), encoder_count(&encX));
  govAplicar();
  if (ev == GOV_QUEDA_CONTADA) {
    Serial.printf("[vel X] queda %u/%u: encoder %.2f mm/s, motor %.2f mm/s (%ld%%): ainda nao reduz\n",
                  gov_seguidas(&govX), govQuedas, realMmS(), gov_vel_motor(&govX) / passosPorMm,
                  (long)gov_aproveitamento(&govX));
  } else if (ev == GOV_QUEDA) {
    Serial.printf("[vel X] CAIU (%u queda(s) seguida(s)): encoder %.2f mm/s, motor %.2f mm/s"
                  " (%ld%%, setpoint %.2f mm/s): rotacao %lu -> %lu passos/s (mais torque)\n",
                  govQuedas,
                  realMmS(), gov_vel_motor(&govX) / passosPorMm, (long)gov_aproveitamento(&govX),
                  spMmS(), (unsigned long)(antes < p.vmax ? antes : p.vmax),
                  (unsigned long)gov_limite(&govX));
  } else if (ev == GOV_SUBIU) {
    Serial.printf("[vel X] tentando recuperar: rotacao %lu passos/s (%.2f mm/s) de %.2f mm/s;"
                  " mantem pelo menos %lu ms\n",
                  (unsigned long)gov_limite(&govX), gov_limite(&govX) / passosPorMm, spMmS(),
                  (unsigned long)govSegurarMs);
  } else if (ev == GOV_RECUPEROU) {
    Serial.printf("[vel X] setpoint recuperado: %.2f mm/s (%lu passos/s)\n", spMmS(),
                  (unsigned long)gov_setpoint_atual(&govX));
  }
}

// Chamar com frequência. Sempre false: nunca manda parar.
static bool vigiarColisao() {
  if (encSemSinal) {  // encoder parado por falta de sinal não é travamento
    colisao_rearmar(&colisao);
    return false;
  }
  uint32_t vel = stepper_speed_now(&motX);
  if (!colisao_atualizar(&colisao, stepper_busy(&motX), stepper_moves(&motX),
                         stepper_position(&motX), encoder_count(&encX))) {
    return false;
  }
  // NUNCA para o motor (decisão do Gab, 2026-11-15: nada de parar por
  // interpretação do encoder). Só baixa a rotação ao vivo, sem interromper o
  // movimento nem o homing; a cada 1 s o governo tenta recuperar o setpoint.
  // Conta como queda (GQ quedas seguidas para reduzir; ignorada nos GS ms
  // depois de subir).
  bool reduziu = gov_travou(&govX, vel);
  govAplicar();
  colisao_rearmar(&colisao);  // o movimento continua: vigia de novo a partir daqui
  if (reduziu) {
    Serial.printf("[trava X] encoder parado a %lu passos/s: rotacao baixa para %lu passos/s"
                  " (sem parar; tenta recuperar a cada 1 s)\n",
                  (unsigned long)vel, (unsigned long)gov_limite(&govX));
  } else if (gov_segurando_ms(&govX) > 0) {
    Serial.printf("[trava X] encoder parado a %lu passos/s: mantendo a rotacao (acabou de subir)\n",
                  (unsigned long)vel);
  } else {
    Serial.printf("[trava X] encoder parado a %lu passos/s: queda %u/%u, ainda nao reduz\n",
                  (unsigned long)vel, gov_seguidas(&govX), govQuedas);
  }
  return false;
}

// Telemetria para o painel: SEMPRE uma linha "@T" (andando a cada TELEM_MS,
// parado a cada TELEM_PARADO_MS) com a posição pelo encoder e os alvos.
static uint32_t telemUltimo = 0;


static void telemetria() {
  uint32_t agora = millis();
  bool andando = stepper_busy(&motX) || stepper_busy(&motZ) || malha_ativa(&malhaX);
  // Sempre manda a posição: andando a cada TELEM_MS, parado a cada TELEM_PARADO_MS
  // (o carro mexido à mão também aparece).
  if (agora - telemUltimo < (andando ? TELEM_MS : TELEM_PARADO_MS)) return;
  telemUltimo = agora;
  if (stepper_busy(&motZ)) {
    zAlvoPassos = stepper_position(&motZ) + stepper_remaining(&motZ);
    zTemAlvo = true;
  }
  uint32_t vx = stepper_speed_now(&motX);
  uint32_t vz = stepper_speed_now(&motZ);
  uint32_t lim = gov_limite(&govX);
  uint32_t sp = gov_setpoint_atual(&govX);
  // Campo 3 = contagens cruas do encoder (a posição do X é só o encoder).
  Serial.printf("@T %lu %.2f %ld %lu %.2f %lu %.2f %lu %.2f %d%d %lu %lu", (unsigned long)agora,
                encoderMm(), (long)encoder_count(&encX), (unsigned long)vx, vx / passosPorMm,
                (unsigned long)(lim < sp ? lim : sp), zMm(), (unsigned long)vz, vz / zPassosPorMm,
                referenciado ? 1 : 0, zReferenciado ? 1 : 0,
                (unsigned long)lroundf(gov_vel_real(&govX)), (unsigned long)sp);
  // Alvo do X (pelo encoder: mm e contagens) e estado da malha: m = movendo,
  // c = chegou, f = ficou fora da tolerância, - = sem alvo. Depois o alvo do Z.
  MalhaEstado me = malha_estado(&malhaX);
  if (me == MALHA_PARADA) {
    Serial.print(" - - -");
  } else {
    int32_t a = malha_alvo(&malhaX);
    Serial.printf(" %.2f %ld %c", contagensParaMm(a), (long)a,
                  me == MALHA_MOVENDO ? 'm' : (me == MALHA_CHEGOU ? 'c' : 'f'));
  }
  if (zTemAlvo) {
    EixoMm ez = eixoZmm();
    Serial.printf(" %.2f", eixo_passos_para_mm(&ez, zAlvoPassos));
  } else {
    Serial.print(" -");
  }
  // Sinal do encoder X: ok, ou SEMSINAL (motor andou e o encoder não mudou nada).
  Serial.println(encSemSinal ? " SEMSINAL" : " ok");
}

// V -> mostra;  V1.8 -> setpoint de velocidade do X em mm/s (o mesmo que 'v' em passos/s).
static void comandoSetpointVel(String resto) {
  resto.trim();
  if (resto.length()) {
    float mm = lerMm(resto);
    float max = VEL_LIMITE / passosPorMm;
    if (!(mm > 0.0f)) {
      Serial.printf("setpoint invalido: '%s' (use 0 < mm/s <= %.2f)\n", resto.c_str(), max);
      return;
    }
    if (mm > max) {
      Serial.printf("acima do limite (%.2f mm/s = %lu passos/s): ficou no limite\n", max,
                    (unsigned long)VEL_LIMITE);
      mm = max;
    }
    velocidade = (uint32_t)lroundf(mm * passosPorMm);
    if (velocidade < 1) velocidade = 1;
    stepper_set_speed(&motX, velocidade);
  }
  StepperPerfil p;
  stepper_get_profile(&motX, &p);
  uint32_t lim = gov_limite(&govX);
  Serial.printf("setpoint X: %.2f mm/s (%lu passos/s); rotacao agora %lu passos/s%s;"
                " encoder %.2f mm/s (aproveitamento %ld%%)\n",
                p.vmax / passosPorMm, (unsigned long)p.vmax,
                (unsigned long)(lim < p.vmax ? lim : p.vmax),
                lim < p.vmax ? " (reduzida: tenta recuperar a cada 1 s)" : "", realMmS(),
                (long)gov_aproveitamento(&govX));
}

// C -> mostra;  C0 -> desliga a detecção (testes);  C1 -> liga de novo.
// Desligada não volta sozinha: o boot sempre liga.
static void comandoColisao(String resto) {
  resto.trim();
  if (resto == "0" || resto == "1") {
    colisao_habilitar(&colisao, resto == "1");
    aplicarTravaHoming();  // o travamento do homing segue a detecção
    if (resto == "0") {
      Serial.println("-- deteccao de travamento DESLIGADA: o X nao baixa a rotacao se travar (C1 liga) --");
      Serial.println("   o travamento do homing tambem fica desligado: se a chave falhar, o carro"
                     " vai ate o batente (fique no PARAR)");
    }
  } else if (resto.length()) {
    Serial.printf("comando invalido: 'C%s' (C, C0, C1)\n", resto.c_str());
    return;
  }
  Serial.printf("deteccao de travamento: %s, janela %ld contagens (%.1f mm), avanco minimo %u%%, "
                "%u janelas seguidas\n",
                colisao.habilitado ? "ligada" : "DESLIGADA", (long)COLISAO_JANELA_CONT,
                COLISAO_JANELA_CONT / contagensPorMm, COLISAO_FRACAO_PCT, COLISAO_CONFIRMAR);
  comandoSetpointVel("");
  govMostrar();
  Serial.printf("   ultimo movimento: pior avanco %ld%%, erro total max %ld contagens (%.2f mm)\n",
                (long)colisao_pior_pct(&colisao), (long)colisao_erro_max(&colisao),
                colisao_erro_max(&colisao) / contagensPorMm);
}

// ---------- homing do X: ajustes e diagnóstico (comando H) ----------
// Valem até reiniciar a placa. Para ficar, mude configurarHoming().

static void aplicarTravaHoming() {
  homingX.fsm.janela_trava = colisao.habilitado ? homTravaPassos : 0;
}

static void mostrarHoming() {
  Serial.printf("homing X: sentido %+d ('%s' vai para a chave), rapida %lu passos/s, lenta %lu passos/s, "
                "travamento %ld passos (%s)\n",
                homingX.sentido, giroNome(homingX.sentido), (unsigned long)homingX.vel_rapida,
                (unsigned long)homingX.vel_lenta, (long)homTravaPassos,
                homingX.fsm.janela_trava > 0 ? "ligado" : "DESLIGADO");
  Serial.printf("   chave X no GPIO %u: %s agora\n", HOME_X,
                eixo_chave_crua(&eixoX) ? "ACIONADA" : "solta");
}

// HD: anda 2 mm para a chave (ou afastando, se ela já está apertada) e diz
// o que o encoder e a chave fizeram. Quem olha o carro confirma o sentido.
static const float HD_MM = 2.0f;

static void diagnosticoHoming() {
  bool apertada = eixo_chave_crua(&eixoX);
  Serial.println("-- diagnostico do homing X (Enter com qualquer tecla cancela) --");
  Serial.printf("1) chave X: %s (GPIO %u = %s). Apertando com a mao tem que virar ACIONADA (HIGH)\n",
                apertada ? "ACIONADA" : "solta", HOME_X, digitalRead(HOME_X) ? "HIGH" : "LOW");
  int8_t dir = apertada ? (int8_t)-homingX.sentido : homingX.sentido;
  int32_t passos = (int32_t)lroundf(HD_MM * passosPorMm);
  Serial.printf("2) andando %.0f mm %s (giro '%s'): OLHE o carro\n", HD_MM,
                apertada ? "AFASTANDO da chave (ela ja esta apertada)" : "PARA a chave", giroNome(dir));
  int32_t enc0 = encoder_count(&encX);
  // Indo para a chave: para se ela acionar no caminho (mesmo debounce do homing).
  if (!apertada) {
    stepper_sensor_arm(&motX, HOME_X, !eixoX.ativa_em_baixo, true,
                       (uint16_t)((uint64_t)EIXO_CHAVE_DEBOUNCE_US * STEPPER_TICK_HZ / 1000000u));
  }
  stepper_start(&motX, dir * passos, true);
  bool cancelou = false;
  while (stepper_busy(&motX)) {
    if (cancelarPelaSerial(nullptr)) {
      stepper_release(&motX);
      cancelou = true;
      break;
    }
    delay(1);
  }
  stepper_sensor_disarm(&motX);
  delay(5);  // última transição passa pelo debounce do encoder
  int32_t pos;
  bool acionou = stepper_sensor_fired(&motX, &pos);
  stepper_release(&motX);
  referenciado = false;  // andou sem atualizar a origem da chave? por segurança, refazer o h
  if (cancelou) {
    Serial.println("-- diagnostico cancelado --");
    return;
  }
  int32_t enc = encoder_count(&encX) - enc0;
  float esp = dir * passos * contagensPorMm / passosPorMm;
  Serial.printf("   encoder: %+ld contagens (esperado ~%+.0f): ", (long)enc, esp);
  if (enc > -2 && enc < 2) {
    Serial.println("PARADO -> encoder solto/desligado, ou o motor nao girou (teste as bobinas)");
  } else if ((enc > 0) != (esp > 0)) {
    Serial.println("AO CONTRARIO -> troque os fios A e B do encoder (ou ENC_X_A/ENC_X_B no pins.h)");
  } else if (fabsf((float)enc) < 0.6f * fabsf(esp)) {
    Serial.println("bem menos que o esperado -> motor perdendo passo ou encoder patinando");
  } else {
    Serial.println("ok");
  }
  if (acionou) Serial.println("   a chave ACIONOU no caminho: o sentido para a chave esta certo");
  Serial.printf("3) o carro foi %s? Se foi para o lado contrario, o sentido esta trocado: envie HS%+d\n",
                apertada ? "para LONGE da chave" : "na direcao da chave", -homingX.sentido);
  Serial.println("   (HS vale ate reiniciar; para ficar, mude homingX.sentido em configurarHoming)");
}

// ---------- homing geral e testes de todos os eixos ----------
//   HA -> homing de todos os eixos: Z (desce até a chave) e depois X.
//   E  -> teste de todos: homing do que faltar, Z sobe ao topo, teste do X
//         e teste do Z.
//   EX -> teste do X: vai a TESTE_X_PONTOS pelo encoder (erro e correções
//         em cada um) e refaz o homing: a chave tem que acionar no mesmo
//         lugar (até TESTE_X_ORIGEM_TOL_CONT contagens).
//   EZ -> teste do Z: vai a TESTE_Z_PONTOS e refaz o homing. Sem encoder, a
//         distância entre a chave e onde ela deveria estar = passos perdidos.
//   Qualquer tecla cancela. Resultado: "== teste do X: OK ==" (ou FALHOU).

static const float TESTE_X_PONTOS[] = {60.0f, 200.0f, 125.0f, 40.0f};  // mm
static const float TESTE_Z_PONTOS[] = {0.0f, 30.0f, 10.0f, 35.0f};     // mm (acima do fundo, 40)
static const int32_t TESTE_X_ORIGEM_TOL_CONT = 2;  // ±0,2 mm
static const float TESTE_Z_ORIGEM_TOL_MM = 0.5f;

enum TesteRes { TESTE_OK, TESTE_FALHOU, TESTE_CANCELADO };
static const char *testeNome(TesteRes r) {
  return r == TESTE_OK ? "OK" : (r == TESTE_FALHOU ? "FALHOU" : "CANCELADO");
}

// Erro da malha do X (contagens, alvo - encoder) em mm no sentido da coordenada.
static float xErroMm(int32_t cont) { return -homingX.sentido * cont / contagensPorMm; }

// Espera a malha do X terminar (bloqueia). false = cancelado pela serial.
static bool xEsperarMalha() {
  while (xOcupado()) {
    if (cancelarPelaSerial(nullptr)) {  // também vigia o travamento
      malha_cancelar(&malhaX);
      stepper_release(&motX);
      return false;
    }
    xAtualizarMalha();
    delay(1);
  }
  return true;
}

// Resultado de teste para um homing do X que não deu certo.
static TesteRes xHomingFalhou() {
  return (xUltimoHome.erro == HOM_ERRO_CANCELADO && !homTravou) ? TESTE_CANCELADO : TESTE_FALHOU;
}

// Leva o Z a 'mm' e espera (bloqueia). false = cancelado.
static bool zIrEsperando(float mm) {
  EixoMm e = eixoZmm();
  stepper_set_speed(&motZ, velocidadeZ);
  zIniciar(eixo_mm_para_passos(&e, mm));
  return zEsperarParar();
}

// Nada rodando? Senão avisa e devolve false.
static bool podeTestar() {
  if (seq_ativo(&seq) || seq_estado(&seq) == SEQ_PAUSADO) {
    Serial.println("programa rodando ou pausado: q cancela");
    return false;
  }
  if (ocupado()) return false;
  if (stepper_busy(&motZ)) {
    Serial.println("Z andando: espere parar");
    return false;
  }
  return true;
}

static TesteRes homingGeral() {
  Serial.println("== homing de todos os eixos: Z (DESCE ate a chave) e depois X ==");
  if (Z_TEM_CHAVE) {
    if (!fazerHomingZ()) {
      Serial.println("== homing geral: FALHOU no Z ==");
      return TESTE_FALHOU;
    }
  } else if (!zReferenciado) {
    Serial.println("Z sem chave e sem referencia: leve ao topo e envie ZR");
    Serial.println("== homing geral: FALHOU no Z ==");
    return TESTE_FALHOU;
  }
  fazerHoming();
  if (!referenciado) {
    Serial.println("== homing geral: FALHOU no X ==");
    return xHomingFalhou();
  }
  Serial.printf("== homing geral: OK (X %.2f mm, Z %.2f mm) ==\n", encoderMm(), zMm());
  return TESTE_OK;
}

static TesteRes testeX() {
  Serial.println("== teste do X: pontos pelo encoder e repeticao da chave ==");
  if (!referenciado) {
    Serial.println("X sem referencia: homing antes");
    fazerHoming();
    if (!referenciado) {
      Serial.println("== teste do X: FALHOU (homing) ==");
      return TESTE_FALHOU;
    }
  }
  stepper_set_speed(&motX, velocidade);
  bool ok = true;
  float piorMm = 0.0f;
  uint32_t correcoes = 0;
  const size_t n = sizeof TESTE_X_PONTOS / sizeof TESTE_X_PONTOS[0];
  for (size_t i = 0; i < n; i++) {
    float alvo = TESTE_X_PONTOS[i];
    uint32_t t0 = millis();
    xIr(alvo);
    if (!xEsperarMalha()) {
      TesteRes r = TESTE_CANCELADO;
      Serial.printf("== teste do X: %s (ponto %.2f mm) ==\n", testeNome(r), alvo);
      return r;
    }
    bool chegou = malha_estado(&malhaX) == MALHA_CHEGOU;
    float erro = xErroMm(malha_erro(&malhaX));
    if (fabsf(erro) > fabsf(piorMm)) piorMm = erro;
    correcoes += malha_correcoes(&malhaX);
    if (!chegou) ok = false;
    Serial.printf("[teste X] %u/%u alvo %.2f mm: encoder %.2f mm (%ld cont), erro %+.2f mm, "
                  "%u correcao(oes), %.1f s -> %s\n",
                  (unsigned)(i + 1), (unsigned)n, alvo, encoderMm(), (long)encoder_count(&encX), erro,
                  malha_correcoes(&malhaX), (millis() - t0) / 1000.0f, chegou ? "ok" : "NAO CHEGOU");
  }
  Serial.printf("[teste X] pior erro %+.2f mm, %lu correcao(oes) no total\n", piorMm,
                (unsigned long)correcoes);

  // Volta à chave: com a origem certa ela aciona no mesmo lugar (~0).
  Serial.println("[teste X] refazendo o homing: a chave tem que acionar na origem");
  fazerHoming();
  if (!referenciado) {
    Serial.println("== teste do X: FALHOU (homing de volta) ==");
    return xHomingFalhou();
  }
  int32_t dc = xUltimoHome.enc_na_chave;
  bool origemOk = dc >= -TESTE_X_ORIGEM_TOL_CONT && dc <= TESTE_X_ORIGEM_TOL_CONT;
  Serial.printf("[teste X] chave a %+ld cont (%+.2f mm) da origem pelo encoder (limite +-%ld) -> %s\n",
                (long)dc, dc / contagensPorMm, (long)TESTE_X_ORIGEM_TOL_CONT,
                origemOk ? "ok" : "FORA");
  if (!origemOk) ok = false;
  TesteRes r = ok ? TESTE_OK : TESTE_FALHOU;
  Serial.printf("== teste do X: %s ==\n", testeNome(r));
  return r;
}

static TesteRes testeZ() {
  Serial.println("== teste do Z: pontos (malha aberta) e repeticao da chave ==");
  if (!zReferenciado) {
    if (!Z_TEM_CHAVE) {
      Serial.println("Z sem referencia: leve ao topo e envie ZR");
      Serial.println("== teste do Z: FALHOU (sem referencia) ==");
      return TESTE_FALHOU;
    }
    Serial.println("Z sem referencia: homing antes");
    if (!fazerHomingZ()) {
      Serial.println("== teste do Z: FALHOU (homing) ==");
      return TESTE_FALHOU;
    }
  }
  const size_t n = sizeof TESTE_Z_PONTOS / sizeof TESTE_Z_PONTOS[0];
  for (size_t i = 0; i < n; i++) {
    float alvo = TESTE_Z_PONTOS[i];
    uint32_t t0 = millis();
    if (!zIrEsperando(alvo)) {
      Serial.println("== teste do Z: CANCELADO ==");
      return TESTE_CANCELADO;
    }
    Serial.printf("[teste Z] %u/%u alvo %.2f mm: Z %.2f mm (motor %ld passos), %.1f s -> ok\n",
                  (unsigned)(i + 1), (unsigned)n, alvo, zMm(), (long)stepper_position(&motZ),
                  (millis() - t0) / 1000.0f);
  }
  bool ok = true;
  if (Z_TEM_CHAVE) {
    Serial.println("[teste Z] refazendo o homing: a chave tem que acionar onde estava");
    if (!fazerHomingZ()) {
      Serial.println("== teste do Z: FALHOU (homing de volta) ==");
      return TESTE_FALHOU;
    }
    if (zUltimaDifValida) {
      float mm = zUltimaDif / zPassosPorMm;
      bool origemOk = fabsf(mm) <= TESTE_Z_ORIGEM_TOL_MM;
      Serial.printf("[teste Z] chave a %+ld passos (%+.2f mm) de onde deveria (limite +-%.1f mm) -> %s\n",
                    (long)zUltimaDif, mm, TESTE_Z_ORIGEM_TOL_MM, origemOk ? "ok" : "FORA (perdeu passo)");
      if (!origemOk) ok = false;
    }
    Serial.println("[teste Z] voltando ao topo (Z = 0)");
    if (!zIrEsperando(0.0f)) {
      Serial.println("== teste do Z: CANCELADO ==");
      return TESTE_CANCELADO;
    }
  } else {
    Serial.println("[teste Z] sem chave (Z_TEM_CHAVE = false): sem conferencia da origem");
  }
  TesteRes r = ok ? TESTE_OK : TESTE_FALHOU;
  Serial.printf("== teste do Z: %s ==\n", testeNome(r));
  return r;
}

static void testeTodos() {
  Serial.println("== teste de todos os eixos (qualquer tecla cancela) ==");
  if (!referenciado || !zReferenciado) {
    TesteRes h = homingGeral();
    if (h != TESTE_OK) {
      Serial.printf("== teste de todos os eixos: %s (homing) ==\n", testeNome(h));
      return;
    }
  }
  Serial.println("[teste] Z sobe ao topo antes de o X andar");
  if (!zIrEsperando(0.0f)) {
    Serial.println("== teste de todos os eixos: CANCELADO ==");
    return;
  }
  TesteRes x = testeX();
  if (x == TESTE_CANCELADO) {
    Serial.println("== teste de todos os eixos: CANCELADO ==");
    return;
  }
  TesteRes z = testeZ();
  bool ok = x == TESTE_OK && z == TESTE_OK;
  Serial.printf("== teste de todos os eixos: %s (X %s, Z %s) ==\n",
                z == TESTE_CANCELADO ? "CANCELADO" : (ok ? "OK" : "FALHOU"), testeNome(x),
                testeNome(z));
}

// E -> todos;  EX -> só o X;  EZ -> só o Z.
static void comandoEnsaio(String resto) {
  resto.trim();
  resto.toUpperCase();
  if (resto.length() && resto != "X" && resto != "Z") {
    Serial.printf("comando invalido: 'E%s' (E, EX, EZ)\n", resto.c_str());
    return;
  }
  if (!podeTestar()) return;
  cancelarPrograma();
  if (resto == "X") {
    testeX();
  } else if (resto == "Z") {
    testeZ();
  } else {
    testeTodos();
  }
  stepper_set_speed(&motX, velocidade);
  stepper_set_speed(&motZ, velocidadeZ);
}

// H -> mostra;  HV500 rápida;  HL150 lenta;  HT256 janela de travamento (0 = desliga);
// HS+1 / HS-1 sentido;  HD diagnóstico (anda 2 mm);  HA homing de todos os eixos.
static void comandoHoming(String resto) {
  resto.trim();
  if (resto.length() == 0) {
    mostrarHoming();
    return;
  }
  char sub = resto[0];
  String num = resto.substring(1);
  num.trim();
  long v = num.toInt();
  switch (sub) {
    case 'V':
    case 'L':
      if (v <= 0 || v > (long)VEL_LIMITE) {
        Serial.printf("velocidade invalida: '%s' (use 1 a %lu passos/s)\n", num.c_str(),
                      (unsigned long)VEL_LIMITE);
        return;
      }
      if (sub == 'V') homingX.vel_rapida = (uint32_t)v;
      else homingX.vel_lenta = (uint32_t)v;
      break;
    case 'T':
      if (v < 0 || v > 8 * PASSOS_POR_VOLTA || (num.length() == 0)) {
        Serial.printf("janela invalida: '%s' (0 = desliga, ate %ld passos)\n", num.c_str(),
                      (long)(8 * PASSOS_POR_VOLTA));
        return;
      }
      homTravaPassos = (int32_t)v;
      aplicarTravaHoming();
      break;
    case 'S':
      if (v != 1 && v != -1) {
        Serial.printf("sentido invalido: '%s' (HS+1 = chave no lado 'f', HS-1 = lado 'b')\n", num.c_str());
        return;
      }
      if (v != homingX.sentido) {
        homingX.sentido = (int8_t)v;
        referenciado = false;  // a coordenada em mm muda de lado
        cancelarPrograma();
        Serial.println("-- sentido do homing trocado: faca o homing (h) de novo --");
      }
      break;
    case 'D':
      if (seq_ativo(&seq)) {
        Serial.println("programa rodando: P pausa, q cancela");
        return;
      }
      if (ocupado()) return;
      diagnosticoHoming();
      imprimir();
      return;
    case 'A':
      if (!podeTestar()) return;
      homingGeral();
      imprimir();
      return;
    default:
      Serial.printf("comando invalido: 'H%s' (H, HV, HL, HT, HS, HD, HA)\n", resto.c_str());
      return;
  }
  mostrarHoming();
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
        if (v > ACEL_LIMITE) {
          Serial.printf("acima do limite (%lu passos/s2): ficou no limite\n",
                        (unsigned long)ACEL_LIMITE);
          v = ACEL_LIMITE;
        }
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
  if (seq_ativo(&seq)) {
    Serial.println("programa rodando: P pausa, q cancela");
    return;
  }
  if (sub == 'L') {
    stepper_release(&motZ);
    andandoZ = false;
    cancelarPrograma();
    if (Z_SEGURAR) {
      zReferenciado = false;  // sem corrente o Z pode descer: a posição deixa de valer
      Serial.println("-- Z solto: SEM corrente o Z pode descer pelo peso. Referencia perdida --");
    } else {
      // A caixa de redução segura: a referência continua valendo.
      Serial.println("-- Z parado e solto (a caixa de reducao segura; referencia mantida) --");
    }
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

  // O encoder ficou desligado na volta: a posição dele não vale mais. NÃO
  // copia dos passos do motor (a referência do X é só o encoder): refaça o homing.
  if (referenciado) {
    referenciado = false;
    Serial.println("-- o encoder ficou desligado na volta: posicao do X perdida, faca o homing (h) --");
  }

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

// Alvo do motor (em passos) para levar o carro a 'mm', medido a partir de onde
// o ENCODER diz que ele está: a referência é o encoder, os passos só executam.
static int32_t alvoMm(float mm) {
  EixoMm e = eixoMm();
  return stepper_position(&motX) + eixo_mm_para_passos(&e, mm) -
         eixo_mm_para_passos(&e, encoderMm());
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
  // Vai ao começo do teste pelo ENCODER (não pelos passos do motor).
  xIr(TESTE_A_MM);
  return xEsperarMalha();
}

static void varrerVelocidadeSemProtecao(String resto);

// A varredura procura de propósito a velocidade em que o motor perde passo:
// com a detecção ligada, ela baixaria a rotação no primeiro erro (e o governo
// limitaria a velocidade): os dois ficam desligados durante a varredura.
static void varrerVelocidade(String resto) {
  colisao_habilitar(&colisao, false);
  govSuspenso = true;
  stepper_set_limit(&motX, 0);
  Serial.println("   deteccao de travamento e governo DESLIGADOS durante a varredura");
  varrerVelocidadeSemProtecao(resto);
  colisao_habilitar(&colisao, true);
  govSuspenso = false;
  govAplicado = 0;  // força reaplicar o limite
  govAplicar();
  Serial.println("   deteccao de travamento e governo religados");
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
  // Por que a placa (re)iniciou: queda de tensão (BROWNOUT) quando os
  // motores arrancam também para tudo e parece um alarme.
  {
    esp_reset_reason_t rr = esp_reset_reason();
    const char *nome = "OUTRO";
    switch (rr) {
      case ESP_RST_POWERON: nome = "LIGOU (energia ou botao EN)"; break;
      case ESP_RST_EXT: nome = "RESET externo"; break;
      case ESP_RST_SW: nome = "reset por software"; break;
      case ESP_RST_PANIC: nome = "PANIC (o programa travou)"; break;
      case ESP_RST_INT_WDT:
      case ESP_RST_TASK_WDT:
      case ESP_RST_WDT: nome = "WATCHDOG (o programa ficou preso)"; break;
      case ESP_RST_BROWNOUT: nome = "BROWNOUT (queda de tensao: fonte/USB fraca para os motores)"; break;
      case ESP_RST_DEEPSLEEP: nome = "deep sleep"; break;
      default: break;
    }
    Serial.printf("reinicio: %s\n", nome);
  }
  Serial.println("f = volta p/ frente, b = volta p/ tras, q = para, l = solta, r = zera");
  Serial.println("f3 / b0.5 = numero de voltas (sem numero = 1)");
  Serial.println("v800 = vel. maxima, a1500 = aceleracao (a0 sem rampa), n300 = partida");
  Serial.println("teste de bobinas: TX / TZ escolhe o motor, 1..4 = so uma entrada,");
  Serial.println("           p/o = proxima/anterior fase, T0 = fim do teste");
  Serial.println("e = liga/desliga monitor do encoder (A e B crus)");
  Serial.println("h = homing, c3 = auto calibracao (3 ciclos), s = mostra a chave");
  Serial.println("H = ajustes do homing X: HV500 rapida, HL150 lenta, HT256 travamento (0 desliga),");
  Serial.println("           HS+1 / HS-1 sentido da chave, HD diagnostico (anda 2 mm e confere)");
  Serial.println("V = setpoint de velocidade do X em mm/s (V1.8); pelo encoder: caiu -> menos rotacao, tenta recuperar a cada 1 s");
  Serial.println("G = governo: GQ2 quedas para reduzir, GA50 fator de aceleracao %, GD70 fator de desaceleracao %, GS1000 mantem (ms), G0 padroes");
  Serial.println("HA = homing de todos os eixos (Z desce ate a chave, depois o X)");
  Serial.println("E = teste de todos os eixos, EX so o X, EZ so o Z (homing, pontos e repeticao da chave)");
  Serial.println("d100 = calibra distancia, m99.6 = medida real, k = calibracao (k0 = nominal)");
  Serial.printf("x120 = vai para X = 120 mm (centro do braco, %.0f a %.0f mm; depois do homing)\n",
                X_HOME_MM, X_HOME_MM + CURSO_X_MM);
  Serial.println("x+10 / x-1 = anda / volta em mm pelo ENCODER (vale sem homing, a partir da leitura atual)");
  Serial.printf("Z (topo 0 a %.0f mm, malha aberta): ZH homing, Z+5 desce, Z-5 sobe, ZR = topo aqui,\n",
                CURSO_Z_MM);
  Serial.println("           Z40 vai (h faz o homing do Z antes do X, se faltar)");
  Serial.println("           ZV300 velocidade, ZA600 aceleracao, ZN150 partida, ZD50 / ZM49.6 calibra,");
  Serial.println("           ZK calibracao, ZL solta as bobinas");
  Serial.println("setpoints: S lista, S<x> <z> [mm/s] [ms] acrescenta (ex.: S120 40), S-2 / S- apaga");
  Serial.println("           R / R3 / R0 executa 1x / 3x / sem fim, P pausa/continua, q cancela");
  Serial.println("travou (encoder parado): o X baixa a rotacao e continua, sem alarme; C mostra, C0/C1");
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
  // Pull-down interno junto com o externo: sem o resistor (ou com o fio no
  // pino errado) a entrada não fica no ar dando "apertado" falso.
  pinMode(HOME_X, INPUT_PULLDOWN);
  configurarHoming();
  pinMode(ALARME_LED, OUTPUT);
  digitalWrite(ALARME_LED, LOW);
  ColisaoCfg cc = {COLISAO_JANELA_CONT, COLISAO_FRACAO_PCT, COLISAO_CONFIRMAR};
  colisao_init(&colisao, &cc, CONTAGENS_MM_NOMINAL / PASSOS_MM_NOMINAL);
  carregarCalibracao();
  colisao_escala(&colisao, contagensPorMm / passosPorMm);
  govCarregar();
  GovCfg gc = govMontarCfg();
  gov_init(&govX, &gc, contagensPorMm / passosPorMm);
  govMostrar();
  stepper_set_limit(&motX, gov_limite(&govX));
  MalhaCfg mc = {MALHA_TOL_CONT, MALHA_CORRECOES};
  malha_init(&malhaX, &mc, passosPorMm / contagensPorMm);
  Serial.printf("X pela posicao do ENCODER: tolerancia %.2f mm, ate %u correcoes\n",
                MALHA_TOL_CONT / contagensPorMm, MALHA_CORRECOES);
  Serial.printf("limites dos motores: %lu passos/s, %lu passos/s2\n", (unsigned long)VEL_LIMITE,
                (unsigned long)ACEL_LIMITE);
  Serial.printf("deteccao de travamento: ligada, a cada %.1f mm o encoder anda >= %u%% (C0 desliga)\n",
                COLISAO_JANELA_CONT / contagensPorMm, COLISAO_FRACAO_PCT);
  aplicarTravaHoming();
  mostrarHoming();
  mostrarCalibracao();
  carregarPrograma();
  listarPrograma();
  chaveAnterior = eixo_chave_crua(&eixoX);
  Serial.printf("sensor do homing: %s\n", chaveAnterior ? "APERTADO" : "solto");
  StepperPerfil perfil = {VEL_PARTIDA_PADRAO, VEL_MAX_PADRAO, ACEL_PADRAO};
  stepper_set_profile(&motX, &perfil);
  mostrarPerfil();

  // Z: só liga as bobinas no boot se Z_SEGURAR (padrão: soltas, a caixa segura).
  stepper_begin(&motZ, MOT_Z_IN1, MOT_Z_IN2, MOT_Z_IN3, MOT_Z_IN4);
  StepperPerfil pz = {Z_PARTIDA, velocidadeZ, Z_ACEL};
  stepper_set_profile(&motZ, &pz);
  zCarregarCalibracao();
  if (Z_SEGURAR) stepper_hold(&motZ);
  // Chave do Z: igual à do X (NA para 3V3 + pull-down externo de 10 kΩ).
  eixo_begin(&eixoZ, &motZ, nullptr, HOME_Z, false, false);
  pinMode(HOME_Z, INPUT_PULLDOWN);  // idem ao X
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
      malha_cancelar(&malhaX);
      stepper_stop(&motX);
      stepper_stop(&motZ);
      Serial.println("-- freando --");
      return;
    case 'Z': comandoZ(resto); return;
    case 'S': comandoSetpoint(resto); return;
    case 'R': rodarPrograma(resto); return;
    case 'P': pausarOuContinuar(); return;
    case 'C': comandoColisao(resto); return;
    case 'H': comandoHoming(resto); return;
    case 'T': comandoTeste(resto); return;
    case 'E': comandoEnsaio(resto); break;
    case 'v': ajustarVelocidade(resto); return;
    case 'V': comandoSetpointVel(resto); return;
    case 'G': comandoGoverno(resto); return;
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
      malha_cancelar(&malhaX);
      stepper_release(&motX);
      zPararNaHora();  // Z para (solto, a menos que Z_SEGURAR)
      cancelarPrograma();
      andando = false;
      andandoZ = false;
      Serial.println(Z_SEGURAR ? "-- X solto; Z parado e segurando (ZL solta o Z) --"
                               : "-- X e Z parados e soltos --");
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
  // O homing bloqueia: chegou aqui, ele acabou.
  homCtx.ativo = false;
  if (monitorEncoder) monitorarEncoder();
  vigiarChave();
  vigiarColisao();    // travou: baixa a rotação e continua (ou desiste)
  xAtualizarMalha();  // antes do sequenciador: ele espera a malha terminar
  governarX();        // limite de velocidade sobe com o tempo andando sem travar
  telemetria();       // "@T" para o painel desenhar posição e velocidade
  atualizarSequencia();
  if (andando && !xOcupado()) {
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
