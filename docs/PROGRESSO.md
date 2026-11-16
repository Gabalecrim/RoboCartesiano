# Progresso do projeto

Registro de onde paramos, para o grupo retomar. Atualizar a cada sessão.

**Última sessão:** 14/11/2026

---

## Resumo do estado

| Módulo | Lógica (PC) | Na placa | Situação |
|---|---|---|---|
| 1. Encoder | quadratura 10/10, debounce 15/15 | debounce **não testado na placa** (antes: perdia ~12%) | Checkpoint 2 do Passo 3 |
| 2. Motor de passo | 8/8, rampa 14/14 | rampa + sem bloqueio **não testados na placa** | checkpoint do Passo 3 do Módulo 2 |
| 3. Homing + auto calibração | homing 17/17 | **não testado** (falta instalar a chave no GPIO 36) | instalar a chave e fazer o checkpoint do Módulo 3 |
| 4. Calibração de distância | calib 10/10 | **não testado** | `d100` + `m<medida>` na placa |
| 5. Setpoints em mm (X e Z) | setpoint 25/25 | **não testado** | checkpoint "Setpoints em mm (X e Z)" do README |
| 6. Proteção contra colisão (X) | colisao 15/15 | **não testado** | checkpoint "Proteção contra colisão" do README |
| 7. Eixo Z (malha aberta + homing) | (usa setpoint, calib e homing) | **não testado**, motor e chave do Z ainda não ligados | checkpoint "Eixo Z" do README |

O firmware (`pio run -e esp32dev`) compila **sem erros e sem avisos** (`-Wall -Wextra`).
Os testes do PC (`pio test -e native`) rodam: 118/118 (gcc instalado via WinLibs).

---

## O que foi feito em 15/11/2026

### Alarme no homing do X (sempre disparava)
Dois caminhos davam alarme no homing, e só um deles desligava com `C0`:
1. **Colisão** (lib/colisao), vigiada a cada 1 ms também durante o homing.
2. **Travamento do homing** (`janela_trava` = 256 passos sem o encoder mudar): virava
   "TRAVOU no homing" e **ignorava o `C0`**, por isso não havia como desligar.

Causas prováveis, a conferir na bancada com o `HD`:
- **Sentido**: se o giro `f` afasta o carro da chave, a busca rápida vai até o batente do outro lado e bate.
- **Chave não lida**: `HOME_X = 32` no pins.h, mas a documentação antiga cita o GPIO 36. Fio no
  pino errado = a chave nunca aciona = o carro bate no fim do curso. Agora há pull-down
  interno também (a entrada não fica no ar).
- **Partida a 500 passos/s sem rampa** (o homing não usa rampa; o resto parte de 300): o 28BYJ-48
  pode não arrancar. Testar com `HV400` / `HV300`.
- **Folga de inversão** maior que 1 mm no recuo: o encoder não mexe no começo e o travamento dispara. `HT512`.
- **Encoder ao contrário** (A/B trocados): a colisão dispara em qualquer movimento longo.

**Resultado na bancada (15/11):** com a proteção desligada o homing terminou normal; o alarme
era **"TRAVOU no homing"** = falso positivo (chave, sentido e motor ok). Correção em
`lib/eixo/eixo.c`: o travamento ignora a 1ª janela de cada trecho (folga de inversão), como a
colisão já fazia. **A testar** com a proteção ligada (`C1` + `h`); se ainda travar, `HT512`.

**Depois (15/11):** continua dando alarme nos movimentos manuais e a faixa não aparecia no
painel. O painel foi testado no Edge (headless) com as linhas do firmware e a faixa funciona;
então a linha `!!! ALARME` não chegava ou a placa REINICIAVA (o boot apaga a faixa).
- O estado do alarme agora vai na linha de estado (`... ALARME: <motivo>`) e no `C`
  (`alarme: ATIVO (...)` / `alarme: nenhum`); o painel segue isso, com chip "alarme" no topo.
- No boot sai `reinicio: <motivo>` (BROWNOUT, PANIC, WATCHDOG...); o painel mostra faixa amarela
  se a placa reiniciar sozinha. **Pendente:** ver qual é (alarme de colisão ou reinício) e, se
  for colisão, a linha `!!! na ultima janela ...` diz se o encoder está ao contrário (`HD` confirma).

Mudanças:
- `C0` desliga também o travamento do homing (`C1` religa).
- No alarme do homing, a serial diz a fase, os passos do motor e as contagens do encoder nessa fase, a chave e a **causa provável**.
- "homing FALHOU (cancelado)" virou "parado pelo ALARME de colisão" quando foi o alarme.
- Comandos novos: `H` mostra, `HV`/`HL` velocidades, `HT` janela de travamento, `HS+1`/`HS-1`
  sentido, `HD` diagnóstico (anda 2 mm para a chave e confere o encoder e a chave). Valem até reiniciar.
- Painel: faixa de alarme com todos os detalhes e botão "Reconhecer e desligar a proteção";
  quadro "Diagnóstico do homing X" na aba Operar (chave, teste de sentido/encoder, ajustes,
  homing sem proteção); chip "proteção" no topo; o jog segue o sentido do homing.

## O que foi feito em 14/11/2026

### Eixo Z em malha aberta
- Motor Z nos GPIOs 14, 25, 26, 27 (os 25/26 eram do encoder Z, que saiu: malha aberta;
  o Gab trocou o IN1 de 23 para 14). O 14 dá pulsos no boot: no máximo um tranco no Z.
- **Homing do Z** (`ZH`): chave NA **embaixo** (Z = 80 mm na chave; o zero continua no topo), GPIO 4 (escolha do Gab), pull-down externo de 10 kΩ.
  Usa `lib/homing` + `lib/eixo` sem encoder (`enc = NULL`, sem janela de travamento) e
  termina solto (`EixoHomingCfg.segurar = Z_SEGURAR`). o `R` faz o `ZH` antes, se faltar.
  `Z_TEM_CHAVE = false` volta para a referência manual (`ZR`).
- Fuso igus de alto avanço, 8 entradas x ~3 mm = 24 mm/volta (85,33 passos/mm),
  **a confirmar** com `ZD50` + `ZM<medida>`. Curso 80 mm, Z = 0 no topo.
- Bobinas do Z **soltas ao parar** (2026-11-15, `Z_SEGURAR = false`): sempre ligadas o
  motor superaquecia, e a caixa de redução do 28BYJ-48 já segura o Z. Fim de movimento,
  homing, `q`, `l`, alarme e cancelamento soltam o Z. `ZL` solta sem perder a referência.
  `Z_SEGURAR = true` volta ao comportamento antigo (segurar com corrente).
- Referência manual (`ZR` no topo) continua valendo; sem proteção contra colisão no Z.
- Setpoints agora têm X e Z (`S<x> <z> [mm/s] [ms]`); o sequenciador anda um eixo por
  vez (Z sobe primeiro / X anda primeiro quando o Z desce). Programa do formato antigo é apagado.
- O homing do X (`h`) não mexe no Z (2026-11-15: a chave do X fica embaixo, o braço não precisa subir).
- Painel (`tools/painel.html`): campo Z, coluna Z, quadro do Z (homing, sobe/desce, zerar, ir).

### X pela posição do encoder (2026-11-15)
- `lib/malha` (pura, `test_malha`): o alvo do X é em **contagens do encoder**; os passos
  saem do erro medido. Parado (+30 ms para o encoder assentar), `|erro| > 1 contagem`
  (0,1 mm) vira correção, até 4. Não chegou → aviso `[malha X] NAO CHEGOU` e o programa cancela.
- Vale para `x`, setpoints e programa. `f`/`b`, homing e calibrações continuam em passos.
- Passo perdido não tira mais a referência (antes: `SEQ_TOL_MM` cancelava e pedia homing).
- Estado na serial: `X: <encoder> mm (motor <passos> mm)`; o painel lê esse formato.
- Corrige só parado: o KY-040 tem 25,6 passos por contagem e debounce de 1 ms; corrigir em
  movimento brigaria com a rampa.
- **Limites dos motores** (X e Z): 500 passos/s e 1000 passos/s² (`VEL_LIMITE`, `ACEL_LIMITE`).
  X padrão = os limites; Z padrão continua 300 passos/s e 600 passos/s². A varredura `w`
  passa do limite de propósito.

### Homing geral e teste dos eixos (2026-11-15)
- Painel mostrava "sem homing" depois do homing: o firmware passou a imprimir
  `X: … mm (motor … mm)` e o painel antigo procurava `(encoder …)`. O painel aceita os dois.
  Recarregue a página (Ctrl+F5) depois de atualizar o `painel.html`.
- `HA`: homing de todos os eixos (Z desce até a chave, depois o X). Mesma ordem do `R`.
- `E`: homing do que faltar, Z sobe ao topo, teste do X e teste do Z. `EX` / `EZ`: só um.
  - X: 4 pontos pelo encoder (60, 200, 125, 40 mm) com erro e correções em cada um; depois
    refaz o homing: a chave tem que acionar a ±2 contagens (±0,2 mm) da origem.
  - Z: 4 pontos (0, 60, 20, 70 mm) e refaz o homing: a chave tem que acionar a ±0,5 mm de
    onde deveria (sem encoder, é assim que aparece passo perdido). Termina no topo.
  - Resumo: `== teste ...: OK / FALHOU / CANCELADO ==`. Painel: aba Operar (botões) e aba
    Testes (lista ponto a ponto).

### Travou: baixa a rotação em vez de alarme (2026-11-15)
- O **alarme de colisão saiu** (alarmava demais em falso). O comando `A`, o LED piscando e a
  faixa vermelha do painel não existem mais.
- A detecção continua a mesma (`lib/colisao`, janelas de 1 mm). Travou (encoder parado com o
  motor girando) → o X para, multiplica velocidade, partida e aceleração por 60% (mínimos
  120 passos/s, 60 passos/s, 200 passos/s²) e **continua o resto do movimento**. A malha do
  encoder corrige no fim o que o motor contou sem andar. O perfil normal volta quando o X para.
- Até 5 travadas no mesmo movimento; na 6ª desiste: para, solta e cancela o programa. A
  referência **continua valendo** (a posição vem do encoder).
- Homing que trava: refaz mais devagar (rápida e lenta x 60%), até 3 tentativas; as
  velocidades do `H` voltam no fim.
- Serial: linhas `[trava X] ...`. Painel: chip "travadas" (amarelo = baixou a rotação,
  vermelho = desistiu). `C` mostra o total de travadas desde o boot; `C0`/`C1` continuam.

### Governo da velocidade + movimento ao vivo (2026-11-15)
- Começou a travar mais com a versão anterior (perfil reduzido a cada travada, só voltava no
  fim do movimento). Agora: `lib/governo` (pura, `test_governo`) mantém um **limite** de
  velocidade do X:
  - travou → limite = 60% da velocidade em que travou (mínimo 120 passos/s); o motor para,
    ressincroniza e continua o que faltava já no limite (a partida também fica nele);
  - andando sem travar 300 ms → sobe 25 passos/s a cada 250 ms até 500 (metade do ritmo
    acima de onde travou da última vez). **Ao vivo**, no meio do movimento.
  - O limite vale entre movimentos; parado não sobe.
- `lib/rampa`: `rampa_mudar_vmax` (acelera/desacelera até a nova vmax no meio do movimento);
  `lib/stepper`: `stepper_set_limit` (limite por cima do perfil) e `stepper_remaining`.
- Desiste com 9 travadas no mesmo movimento. Homing que trava refaz no limite novo (3x).
- Telemetria `@T` a cada 100 ms andando (também nas rotinas que bloqueiam). Painel: quadro
  "Movimento ao vivo" (aba Operar) com X encoder/motor, Z, velocidades, limite e as travadas
  marcadas, últimos 30 s. As linhas `@T` não vão para o registro.
- `C` mostra o governo (limite, última travada, total). A varredura `w` desliga o governo.

### Setpoint de velocidade pelo encoder (2026-11-15, substitui a subida por tempo acima)
- `lib/governo` reescrita: **setpoint** = velocidade pedida (`v` passos/s, `V` mm/s novo, ou a do
  ponto do programa). A cada 50 ms compara encoder x motor numa janela de ≥ 0,5 s (e ≥ 5
  contagens esperadas; em rotação baixa a janela cresce até 2 s):
  - encoder < 75% do motor (perdendo passo) → **rotação cai ao vivo** para o menor entre 70%
    da do motor e a velocidade real (mínimo 120 passos/s);
  - **a cada 1 s** sem queda tenta recuperar: sobe metade do que falta até o setpoint
    (≥ 25 passos/s); chegou → `[vel X] setpoint recuperado`.
  - Encoder parado de vez (`lib/colisao`) continua: ressincroniza e segue com menos rotação.
- Serial: `[vel X] CAIU ...`, `[vel X] tentando recuperar ...`, `[vel X] setpoint recuperado`.
- `@T` ganhou 2 campos no fim: velocidade real pelo encoder e setpoint (passos/s).
- Painel (Movimento ao vivo): setpoint, X real (encoder), rotação do motor e permitida; campo
  para o setpoint em mm/s (`V`); gráfico com setpoint (tracejado), motor, real e as quedas.

### Nada para o sistema por interpretação do encoder (2026-11-15)
- Decisão do Gab: detecção de travamento/queda **só baixa a rotação e avisa**. Removido:
  - parar e reiniciar o movimento quando trava (agora baixa a rotação ao vivo, sem parar);
  - desistir depois de N travadas (`TRAVA_MAX`) e cancelar o programa;
  - cancelar o homing quando o encoder para (o homing termina pela chave ou pelo curso);
  - cancelar o programa quando a malha não chega na tolerância (vira `[seq] aviso`).
- Janela de travamento do homing (`HT`) vem **desligada** (0). `HT256` liga só para diagnóstico.
- Continuam parando só: `q`, `l`, Esc/PARAR do painel, chave de homing e curso máximo do homing.

### Controle de rotação configurável (2026-11-15)
- Depois de **subir**, a rotação fica pelo menos 1 s sem mudar (`GS`, mínimo 1000 ms):
  quedas e travadas nesse tempo não reduzem, mas seguram a próxima subida.
- **Quedas para reduzir** (`GQ`, padrão 2): janelas seguidas (sem sobreposição) com o encoder
  < 75% do motor antes de baixar; uma janela boa zera a conta. Travada também conta.
- **Fator de aceleração** (`GA`, padrão 50%): % do que falta até o setpoint subido a cada 1 s.
- **Fator de desaceleração** (`GD`, padrão 70%): % da rotação mantida na queda.
- `G` mostra, `G0` volta aos padrões. Salvos na placa (Preferences "gov").
- Painel (Movimento ao vivo → Controle de rotação): os 4 campos, Aplicar, Ler e Padrões.
- Serial nova: `[vel X] queda 1/2 ... ainda nao reduz`, `[trava X] ... mantendo a rotacao`.

### Posição do X só pelo encoder (2026-11-15)
- Os passos do motor **não** são mais referência de posição do X em lugar nenhum:
  - linha de estado: `encoder X: <cont> cont (erros N)   X: <mm> mm` (ou `X: sem homing`);
  - `@T`: o 3º campo agora é a contagem crua do encoder (era o X pelo motor);
  - `[seq] chegou: X <mm> mm (encoder <cont> cont)`; teste do X sem "motor";
  - novo `x+10` / `x-1`: anda/volta em mm pelo encoder, vale sem homing (com homing,
    respeita o curso). O jog do painel usa isso (antes era f/b em voltas do motor);
  - testes de diagnóstico (`w`, `g`): o ponto de partida vem do encoder (`alvoMm` conta a
    partir da leitura do encoder);
  - `t` (encoder desligado na volta) não copia mais a posição do motor para o encoder:
    perde a referência e pede homing.
- Continuam em passos só o que mede o próprio motor: `f`/`b`, calibração de passos/mm
  (`d`, `c`), e a comparação motor x encoder do governo e da detecção de travamento.
- Z continua em passos (não tem encoder).
- Painel: chip "encoder X" com a leitura crua; "Movimento ao vivo" mostra "Encoder X
  (leitura)" em contagens; o gráfico de posição não tem mais a linha do motor.

### Posição atual e alvo sempre visíveis (2026-11-15)
- `@T` sai **sempre**: andando 10/s, parado 4/s (antes só andando: parado ou mexido à mão,
  a posição não atualizava). Campos novos no fim: alvo X (mm e contagens), estado da malha
  (`m` indo, `c` chegou, `f` fora da tolerância, `-` sem alvo) e alvo do Z.
- Linha de estado: `alvo X: 125.00 mm (chegou)` (sem homing, em contagens).
- Painel (Movimento ao vivo): quadros grandes "Posição atual do X (encoder)", "Alvo do X",
  "Falta (alvo − atual)" e "Z atual / alvo"; chip "alvo X" no topo; alvos tracejados no
  gráfico de posição.

### Painel: posição que não atualizava (2026-11-15)
- Sem homing o painel escrevia só "sem homing" (e escondia a linha do X no gráfico): agora
  mostra sempre o X em mm pelo encoder, marcado "(sem homing)"/"relativo" (zero ao ligar).
- Linha de estado depois de `-- parou ... --` não era lida (regex com `^`): corrigido.
- Erro ao tratar uma linha encerrava a leitura da serial inteira: agora cada linha tem
  try/catch e o erro vai para o registro.
- Chip "dados da placa": "chegando" (verde) quando as linhas `@T` chegam; "nenhum (grave o
  firmware novo)" ou "PAROU de chegar" (vermelho) quando não.

### Encoder do X sem sinal + Z com 40 mm (2026-11-15)
- Log da bancada: homing X com o motor andando 19628 passos (~77 mm) e `encoder X: 0 cont
  (erros 0)` o tempo todo → o encoder (GPIO 34/35) **não está mandando nenhuma transição**.
  Firmware do encoder não mudou: é cabo, 3V3 ou acoplamento. Diagnóstico: `e` e girar o fuso.
- Com o encoder parado em 0, a malha achava que o carro nunca saía do lugar: cada `x220`
  mandava o curso inteiro de novo (e o governo baixava a rotação sem motivo).
- Novo: **encoder sem sinal** = o motor andou 5 mm (`ENC_SEM_SINAL_MM`) e o encoder não mudou
  nenhuma contagem (fato medido). Não para nada; só impede usar a posição falsa: sem correção
  da malha, sem governo, sem detecção de travamento, recusa movimento novo pelo encoder e tira
  a referência. Volta sozinho quando o encoder contar (`[encoder X] voltou a contar`).
- `@T` ganhou o último campo `ok`/`SEMSINAL`; linha de estado mostra `SEM SINAL`. Painel: chip
  "encoder X" vermelho e aviso embaixo da posição atual.
- Painel: "Framing error" (bytes do boot da placa ao abrir a porta) não encerra mais a leitura.
- **Curso do Z = 40 mm** (`CURSO_Z_MM`; a chave do homing é o fundo, Z = 40). Teste do Z vai a
  0, 30, 10, 35 mm. Programa salvo com Z > 40 é apagado ao ligar (validação).
- Chave X oscilando (`[sensor] homing X APERTADO/solto` dezenas de vezes) com o Z andando: o
  carro fica parado exatamente no ponto em que a chave aciona (fim do homing) e a vibração do Z
  faz ela bater; ou interferência do cabo do Z no GPIO 32. A conferir.

### Eixo Y (2026-11-15)
- Módulo à parte: `src/eixo_y.cpp` / `eixo_y.h`. O X e o Z não mudaram; o `main.cpp` só ganhou
  7 linhas (include, `yIniciar` no setup, `yLoop` no loop, comando `Y`, ajuda, e `q`/`l`
  param/soltam o Y também).
- Pinos (`pins.h`): motor Y 13/16/17/23, encoder Y 33/36 (já reservados), chave Y 39 (só
  entrada: pull-down EXTERNO de 10 kΩ obrigatório).
- Mesma lógica do X, com as mesmas libs: posição só pelo encoder (`lib/malha`), setpoint de
  velocidade pelo encoder (`lib/governo`, ajustes próprios salvos em "govY"), encoder sem
  sinal, nada para por interpretação do encoder, homing pela chave (janela de travamento 0).
- **A conferir**: mecânica igual à do X (28BYJ-48, TR8x8 = 256 passos/mm, KY-040 = 10
  contagens/mm), curso 190 mm, Y = 0 na chave, chave no lado `f` (`Y_SENTIDO_CHAVE`). Tudo em
  constantes no topo do `eixo_y.cpp`.
- Comandos: `Y`, `YH`, `Y120`, `Y+10`/`Y-1`, `YR`, `YV1.5`, `YQ`, `YL`, `YF1`/`YB1`, `YE`, `YT`,
  `YG`/`YGQ`/`YGA`/`YGD`/`YGS`/`YG0`. Telemetria própria `@Y` (a `@T` do X/Z não mudou).
- Painel: quadro "Eixo Y" (aba Operar) com posição/alvo/falta/velocidade, comandos, jog pelo
  encoder, gráficos de posição e velocidade, setpoint e governo do Y; chip "Y" no topo.
- Ainda não entra no Y: programa de setpoints (`S`/`R`) — continua só X e Z.

### Eixo Y com tudo o que o X tem (2026-11-15)
- Firmware (`eixo_y.cpp`): testador de bobinas (`YI1..4`, `YIP`/`YIO`, `YI0`), monitor do encoder
  (`YE`), tempo com encoder ligado x parado (`YTV1`), diagnóstico do homing (`YHD`), ajustes do
  homing (`YHV`/`YHL`/`YHT`/`YHS`), chave (`YS`), velocidade/aceleração/partida (`YV`/`YA`/`YN`),
  calibração de distância (`YD100` → `YM99.6`, salva em "calibY"), `YK`/`YK0`, auto calibração do
  homing (`YCAL3`), varredura (`YW`), curva da rampa em CSV (`YP30`), detecção liga/desliga
  (`YC0`/`YC1`), zerar (`YZ`).
- `HA` agora faz Z, X e **Y**; `E` testa X, Z e **Y**; novo `EY`; `s` mostra a chave do Y.
- Painel: diagnóstico do homing Y (aba Operar); velocidade, calibração e auto calibração do Y
  (aba Ajustes); bobinas do Y com LEDs, encoder/chave/detecção do Y e velocidade/rampa do Y
  (aba Testes); "Só o Y" no teste dos eixos; chip "chave Y" no topo.
- Mensagens do Y escritas para não cair nos regex do X (ex.: `2) Y andando`, `Y andou o curso`).

### Y no testador de bobinas e no programa (2026-11-15)
- Testador de bobinas compartilhado: `TY` escolhe o motor Y (além de `TX`/`TZ`); `1..4`, `p`/`o`,
  girar e `T0` valem para ele. No painel, botão **Y** no mesmo quadro (o quadro separado saiu).
- Programa de setpoints com Y: `Setpoint` ganhou `tem_y`/`y_mm` e `ProgLimites` ganhou `y`
  (`PROG_Y_FORA_DO_CURSO`); `test_setpoint` com 3 testes novos. Comando `S<x> <z> [mm/s] [ms]
  [y<mm>]` (ex.: `S120 40 y95`); sem `y` o Y não mexe naquele ponto.
- O Y anda na **fase do X** (os dois horizontais): Z sobe antes / desce depois, como antes.
  O sequenciador espera o Y; pausa/cancelar freiam o Y; o `R` faz o homing do Y se algum ponto
  usa o Y; velocidade do ponto vale para o Y também.
- **O programa salvo antes do Y é apagado ao ligar** (formato mudou): cadastre os pontos de novo.
- Painel (aba Programa): campo Y (vazio = não mexe), coluna Y na tabela, "Usar posição atual"
  e "Copiar" preenchem o Y.

### Encoder Y travado andando para frente (2026-11-15)
- Bancada: o carro do Y anda, mas indo para frente (afastando da chave, giro `b`) o encoder fica
  oscilando 0/−1; voltando conta só ~40% e os erros de quadratura sobem (3 → 239).
- Novo `YED` (painel: Testes → Encoder e chave do Y): 1 volta para cada lado a 150 passos/s, lê A e
  B direto nos pinos e conta mudanças por canal, transições válidas +/− e impossíveis. Conclui:
  canal morto (A = GPIO 33, B = GPIO 36), vai-e-volta (eixo do encoder treme: acoplamento
  patinando), ruído, ou conta a menos.

### A decidir / medir
- Rodar `YED` no Y e decidir: fio, acoplamento ou ruído.
- Ajustar `GOV_QUEDA_PCT` (75%) na bancada: o KY-040 tem ±10% de resolução na janela de 0,5 s
  a 500 passos/s. Queda falsa sem travar → baixe para 65%.
- Ver nas linhas `[trava X]` em que velocidade o X trava sem carga: se for sempre perto de
  500 passos/s, baixar `VEL_LIMITE` resolve na origem.
- Testar a malha na bancada: `x100`, `x50`... e ver as linhas `[malha X] correcao`.
- Velocidade máxima real do Z (650 passos/s assumido, igual ao X).
- Confirmar com a garra montada que o Z não desce sem corrente (se descer: `Z_SEGURAR = true`).
- Saídas livres para o Y e a garra: 13, 16, 17, 23 (faltam 5); o 39 está livre só como entrada.
- Testador de motor no painel: `TX` / `TZ` escolhe o motor de 1..4, p, o; `T0` termina.
- Z andava ao contrário: `Z_DESCE = -1`. Não subia bem a 650 passos/s: padrão 300, partida 150,
  aceleração 600 (comandos novos `ZA` e `ZN`); homing do Z a 250 / 120.
- Painel com todos os comandos da serial (X manual, rampa, homing/calibração, Z, testes).
- Colisão por janelas de 1 mm (15/11): o erro total acumulava e o alarme disparava o tempo todo
  em movimento longo. Comando `C` (mostra), `C0` / `C1` (desliga / liga). Testar na bancada.

---

## O que foi feito em 12/11/2026

### Setpoints em mm (`lib/setpoint`)
- Coordenada X = **centro do braço**, em mm: 30 mm na chave (`X_HOME_MM`), curso
  de 190 mm (`CURSO_X_MM`), então X vai de 30 a 220 mm. `x`, `w` e `g` passaram a
  usar essa coordenada (pontos do `w`: 40 e 70 mm = os mesmos 10 e 40 mm da chave).
- Programa de até 32 pontos (X, velocidade mm/s, espera ms), salvo na flash.
- Sequenciador (máquina de estados pura): PARADO → MOVENDO → ESPERANDO → ... → FIM,
  com pausa (freia e continua do mesmo ponto), cancelamento e ciclos (`R0` = sem fim).
- Em cada ponto, motor × encoder: diferença > 1 mm cancela e pede homing.
- Comandos novos: `S`, `S<mm> [mm/s] [ms]`, `S-n`, `S-`, `R[n]`, `P`.

### Proteção contra colisão (`lib/colisao`)
- Erro de seguimento (encoder × passos) vigiado em todo movimento: > 6 contagens
  (0,6 mm) em 2 leituras seguidas = colisão. Motor para e solta, programa cancela,
  referência perdida, ALARME (LED GPIO 2 piscando + serial). `A` reconhece; depois `h`.
- `stepper_moves()` (contador de movimentos) para o monitor refazer a referência a
  cada movimento, mesmo quando um zero (homing) acontece entre dois movimentos.
- Homing com `TRAVOU` também dá alarme. Desligada só na varredura `w`.
- Limite a ajustar na bancada: `A` sem alarme mostra o erro máximo do último movimento.

---

## O que foi feito em 10/11/2026

### Bancada
- Comandos agora são por **linha** (`f3`, `b0.5`, `v300`...): o Serial Monitor do VS Code
  precisa de **Line ending = CRLF** (ou LF). Com `None`, nada é executado.
- `f`/`b` soltam as bobinas ao terminar: parado, o 28BYJ-48 dissipava ~1 W e esquentava.
  A redução 64:1 + fuso seguram a posição. O eixo **Z** precisa de teste (peso constante).
- Encoder medido com `b3` (esperado −240): −203 e −219, com 16 e 11 erros. Cada erro
  perde 2 contagens e explica quase toda a falta. À mão, com o motor solto, a taxa de erro
  foi parecida → a causa é o **repique do KY-040**, não ruído das bobinas.
- Do jeito que está, o encoder serve para **detectar travamento**, não como posição em mm.

### Módulo 1, Passo 3: amostragem + debounce
- `lib/encoder/` trocou interrupção por borda por **amostragem a cada 500 µs** (`esp_timer`).
- Novo `lib/debounce/` (lógica pura, FSM ESTAVEL/CONFIRMANDO): 1 ms = 2 amostras.
- Validação continua na quadratura: se A e B firmam na mesma amostra, conta erro.
- `debounce_update()` implementado; `test_debounce` 15/15.
- PCNT descartado para esta função: filtro máximo ~12,8 µs.

---

## O que foi feito em 20/10/2026

### Código convertido para C
- `encoder` e `quadrature` passaram de C++ (classe com métodos) para C (`struct` + funções).
- A interface do encoder manteve as mesmas 4 operações, só mudou a forma de chamar:

| Antes (C++) | Agora (C) |
|---|---|
| `enc.begin(A, B)` | `encoder_begin(&enc, A, B, false)` |
| `enc.count()` | `encoder_count(&enc)` |
| `enc.errors()` | `encoder_errors(&enc)` |
| `enc.reset()` | `encoder_reset(&enc, 0)` |
| `q.update(ab)` | `quadrature_update(&q, ab)` |

- Em C não há valor padrão nos parâmetros nem nos campos: passar todos os argumentos e
  declarar com `Quadrature q = {0};`.
- `main.cpp` continua em C++ porque usa `Serial` (classe do Arduino). Os headers têm
  `extern "C"` para funcionar dos dois lados.
- Corrigido o typo `int8t_t` em `quadrature_update` (impedia a compilação).

### Módulo 2: motor de passo (28BYJ-48 + ULN2003)
Versão simples e didática:
- **Só passo completo**, 2 bobinas ligadas por vez, 4 fases, 2048 passos/volta.
- **Movimento bloqueante**: `stepper_move(&m, n)` só volta quando termina.
- Exercícios feitos pelo grupo em `lib/stepseq/stepseq.c`:
  1. Tabela de fases: `0b0011, 0b0110, 0b1100, 0b1001` (IN1+IN2 → IN2+IN3 → IN3+IN4 → IN4+IN1).
  2. `stepseq_step()`: fase `(phase + dir) & 0x3`, posição `+= dir`.
- Revisado: correto. Os 8 testes foram conferidos à mão; falta confirmar rodando.

### Conceitos vistos
- **ISR** (rotina de interrupção): curta, sem `Serial`/`delay`, `IRAM_ATTR`, seção crítica
  (`portENTER_CRITICAL`) para dividir dados com o `loop()`.
- **Máscara `& 0x3`**: guarda os 2 bits de baixo; faz a fase "dar a volta" (`-1 & 0x3 = 3`,
  por causa do complemento de dois).
- **Delta**: a variação por transição (+1, −1, 0, inválida); a posição é a soma dos deltas.
- **Reset**: define a origem (homing) e sincroniza `prev` com o estado real dos pinos.
- **Meio passo ≠ mais velocidade**: 8 fases dobram a resolução (4096 passos/volta) mas,
  na mesma taxa de passos, giram com metade da velocidade. Velocidade se ganha com **rampa**.

### Módulo 3: homing e auto calibração
- Origem pela **chave de fim de curso** (GPIO 36, NA para 3V3, **ativa em HIGH**, pull-down externo de 10 kΩ para GND),
  com o **encoder vigiando travamento** (256 passos sem mudar = erro).
- Chave no lado **`f`** (`homingX.sentido = +1`): homing anda em `f`, calibração volta em `b`.
- Serial mostra cada estado do homing (com o giro) e quando o sensor é apertado/solto,
  também fora do homing (`[sensor] homing APERTADO`).
- FSM pura em `lib/homing`: busca rápida (500 passos/s) → recuo (até soltar + ¼ volta) →
  busca lenta (150 passos/s). Erros: curso, chave presa, não achou, travou, cancelado.
- `lib/eixo`: homing e auto calibração no hardware; `stepper_hold/stepper_step` novos.
- Bancada: `h` homing, `c`/`c3` auto calibração (10 voltas em `b` + homing por ciclo),
  `s` mostra a chave. Tecla + Enter cancela.

### Módulo 4: calibração de distância
- Fuso **TR8x8** (8 mm/volta): nominal 256 passos/mm e 10 contagens/mm.
- `d100`: homing → afasta 2 mm (folga) → 5 s para marcar → anda 100 mm.
  `m99.6`: medida real → novos passos/mm e contagens/mm, salvos na flash (`Preferences`).
- `k`/`k0` mostra/volta ao nominal; `x50` vai para 50 mm (precisa de homing).
- Conta em `calib_distancia()` (lib/calib, testada no PC). Recusa medida > 10% do pedido.
- `CURSO_X_MM` = 190 mm (curso real, 12/11/2026).

### Motor: rampa e movimento sem bloqueio
- `lib/rampa`: perfil trapezoidal em inteiros (Q32 passos/tique), sem `float` (roda na
  interrupção). Testado no PC: passos exatos, tempo do trapézio, triângulo, simetria, parar.
- `lib/stepper`: interrupção de timer a 20 kHz (núcleo 1) gera os passos de até 3 motores;
  bobinas escritas direto no registrador do GPIO; parada por sensor com debounce na
  interrupção (2 ms) e posição guardada no ponto em que disparou.
- `lib/homing` agora planeja por **trechos** (até acionar / até soltar / recuo extra),
  executados pelo motor sem bloqueio; `eixo.c` vigia encoder (travou) e cancelamento.
- Bancada: `f`/`b`/`x` não bloqueiam; `q` freia; `v`/`a`/`n` ajustam a rampa
  (padrão: partida 300, máxima 650 passos/s = o limite medido, aceleração 1500 passos/s²).
- Varredura `w` na bancada: 650 passos/s ok, 700 perde passo, com aceleração de 1000,
  1500 ou 2500: o limite é o torque do 28BYJ-48 a 5 V, não a rampa. Padrão: 650 (o máximo, decisão de 12/11/2026; antes 520 = 80%).
- Funções da interrupção conferidas em `.iram0.text` no binário.
- Ainda bloqueiam (esperam terminar, mas com o motor na interrupção): `h`, `c`, `d`, `t`.

### Diagnóstico
- `t`/`t2`: cronometra N voltas com o encoder ligado e parado e mostra o custo da
  amostragem (núcleo, duração, maior intervalo). Para confirmar se o encoder atrasa o motor.

---

## Pendências

1. ~~Instalar o `gcc` no Windows~~ ✔ rodou em 10/11/2026: `test_quadrature` 10/10,
   `test_stepseq` 8/8.
2. ~~Instalar o Git e versionar o projeto~~ ✔ feito em 21/10/2026 (repositório local, branch
   `main`, no GitHub em https://github.com/Gabalecrim/RoboCartesiano).
3. ~~Pinos do encoder X divergentes~~ ✔ `pins.h` com encoder X em 34/35 e motor X em
   18/19/21/22.
4. **Faltam pinos de saída** para 3 motores (12 fios) + garra com os encoders nos pinos
   atuais. Resolver na etapa dos 3 eixos (opções: encoder X em 34/35, registrador de
   deslocamento 74HC595).
5. **Algoritmo genético**: com um único alimentador fixo a ordem não muda a distância total.
   Decidir bandeja com vários slots ou mais de um tipo de pino **antes** do módulo do GA.
6. ~~Limpeza: comentários `TODO` que sobraram nas tabelas de `quadrature.c` e `stepseq.c`~~ ✔ 10/11/2026

---

## Próximos passos (na ordem)

1. ~~Implementar `debounce_update`~~ ✔ 10/11/2026 (15/15).
2. Gravar e refazer na placa: à mão 3 voltas ≈ ±240 e `f3` ≈ +240, quase sem erros novos.
   Se ainda houver muitos erros: capacitores de 10 nF em CLK e DT para GND, ou
   `-DENCODER_DEBOUNCE_US=1500`.
3. Testar se o eixo **Z** segura a posição com as bobinas soltas.
4. Instalar a chave do eixo X (lado `f`) e fazer o checkpoint do Módulo 3 (README).
5. **Motor, Passo 3**: movimento sem bloquear (`loop()` continua rodando) e rampa de aceleração.
6. Encoder em três eixos (um timer por encoder; avaliar um timer só para os três).
7. Depois: movimento XYZ em mm → garra → receitas → sequenciador → Modbus TCP/IHM → GA.

---

## Como retomar

```
pio run -e esp32dev              # compila o firmware
pio run -e esp32dev -t upload    # grava na placa
pio device monitor               # Monitor Serial (115200)
pio test -e native               # testes no PC (precisa de gcc)
```

Comandos da bancada atual (`src/main.cpp`): `f` volta p/ frente, `b` volta p/ trás,
`l` solta as bobinas, `r` zera motor e encoder.

Forma de trabalho (desde 10/11/2026): o código é entregue completo, com contrato (`.h`),
testes e explicação; não há mais exercícios `TODO`.
