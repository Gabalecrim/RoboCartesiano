# Pick and Place cartesiano (ESP32)

Projeto da Jornada de Aprendizagem: robô XYZ que posiciona pinos num grid conforme receitas, com sequência otimizada por algoritmo genético e IHM no iX Developer via Modbus TCP.

O código é construído **um módulo por vez**, e cada módulo é testado antes de seguir.

## Estrutura

```
platformio.ini            dois ambientes: esp32dev (placa) e native (testes no PC)
include/pins.h            TODA a pinagem do projeto
lib/quadrature/           lógica pura do encoder (sem hardware, testável no PC)
lib/debounce/             filtro de repique (FSM), lógica pura, testável no PC
lib/encoder/              encoder no ESP32 (amostragem por timer + debounce)
lib/stepseq/              lógica pura do motor de passo (fases + contagem)
lib/rampa/                rampa de aceleração (perfil trapezoidal, inteiros), lógica pura
lib/stepper/              motor no ESP32 (ULN2003), sem bloqueio, por interrupção
lib/homing/               FSM do homing, lógica pura
lib/calib/                estatística e calibração de distância, lógica pura
lib/eixo/                 eixo no ESP32: motor + encoder + fim de curso
src/main.cpp              programa de teste da etapa atual (C++ só por causa do Serial)
test/test_quadrature/     testes automáticos da lógica do encoder
test/test_debounce/       testes do debounce e da cadeia debounce + quadratura
test/test_stepseq/        testes automáticos da lógica do motor
test/test_homing/         testes do homing (eixo simulado) e da estatística
test/test_calib/          testes da calibração de distância (passos/mm)
test/test_rampa/          testes da rampa de aceleração
docs/PROGRESSO.md         onde paramos, decisões e próximos passos
```

Os módulos são escritos em **C**: cada um é uma `struct` + funções `modulo_*(&obj, ...)`.

A separação entre `quadrature` e `encoder` é proposital: **a lógica que decide para onde o eixo girou não depende da placa**. Isso permite testar no PC, sem montar nada, e depois trocar a forma de ler os pinos (interrupção → amostragem por timer com debounce) sem mexer na lógica.

## Módulo 1: Encoder

### Passo 1: fazer os testes passarem (no PC)

Abram `lib/quadrature/quadrature.c`. Há dois exercícios marcados com `TODO`:

1. Preencher a tabela de transição de 16 posições.
2. Implementar `quadrature_update()`.

Rodem os testes:

```
pio test -e native
```

No começo, 8 dos 10 testes falham. Esse é o ponto de partida. Cada teste tem um nome que descreve o comportamento esperado; leiam o teste que falhou para entender o que falta.

> **Windows:** o ambiente `native` precisa de um compilador C (`gcc`) no PATH (por exemplo, MinGW-w64 / MSYS2). No Linux e no Mac já costuma estar instalado.

✅ **Checkpoint 1:** `10 Tests 0 Failures`.

### Passo 2: ligar o encoder e testar na placa

Ligação do KY-040 no eixo X:

| KY-040 | ESP32 |
|---|---|
| `+` | 3V3 (não usar 5 V) |
| `GND` | GND |
| `CLK` (A) | GPIO 34 |
| `DT` (B) | GPIO 35 |
| `SW` | não usado |

```
pio run -e esp32dev -t upload
pio device monitor
```

✅ **Checkpoint 2:**
- uma volta completa num sentido mostra **+80**;
- voltar a mesma volta mostra **0**;
- girar rápido não aumenta o contador de erros (alguns poucos erros no KY-040 mecânico são aceitáveis; muitos indicam ruído ou fiação ruim);
- se o sentido estiver invertido em relação ao movimento do eixo, troquem A e B em `pins.h`.

### Passo 3: amostragem + debounce (FSM)

Na bancada, a leitura por interrupção perdeu cerca de 12% das contagens: o contato do KY-040 repica, e repiques nos dois canais viram transições inválidas. Agora `lib/encoder/` lê os pinos em **intervalo fixo** e filtra cada canal antes da quadratura:

```
pino A -> debounce A -+
                      +-> quadrature_update()  (validação: salto inválido = erro)
pino B -> debounce B -+
```

| Parâmetro (`encoder.h`) | Valor | Observação |
|---|---|---|
| `ENCODER_SAMPLE_PERIOD_US` | 500 µs | intervalo entre amostras |
| `ENCODER_SAMPLE_RATE_HZ` | 2 kHz | **derivado** do período, não se configura à parte |
| `ENCODER_DEBOUNCE_US` | 1 ms | o nível precisa durar isso para valer |
| `ENCODER_DEBOUNCE_SAMPLES` | 2 | derivado: 1 ms / 500 µs |

Para mudar sem editar o código: `build_flags = -DENCODER_DEBOUNCE_US=1500` no `platformio.ini`. O build para se o debounce não for múltiplo do período.

Folga: o motor gira o encoder a ~23 contagens/s (uma a cada ~43 ms); com 1 ms de debounce, o limite fica perto de 1000 contagens/s.

O PCNT do ESP32 foi descartado para isto: o filtro dele vai só até ~12,8 µs, curto demais para repique de contato.

A máquina de estados está em `lib/debounce/debounce.c` (desenho em `debounce.h`). Testes:

```
pio test -e native -f test_debounce
```

✅ **Checkpoint 1:** `test_debounce` 15/15.

✅ **Checkpoint 2 (placa):** anotar `erros` antes e depois de cada teste (`r` não zera os erros).
- à mão, 3 voltas marcadas: ~±240 e quase nenhum erro novo;
- `r` + `f3`: encoder ~+240 e quase nenhum erro novo (antes: ~−203 a −219 e 11 a 16 erros por `b3`).

### Passo 4: três eixos

Três instâncias de `Encoder`, uma por eixo, imprimindo `X Y Z`.

## Módulo 2: Motor de passo (28BYJ-48 + ULN2003)

```
lib/stepseq/              lógica pura: fases das bobinas e contagem de passos
lib/stepper/              motor no ESP32 (4 pinos do ULN2003, sem bloqueio, com rampa)
test/test_stepseq/        testes automáticos da lógica do motor
```

### Passo 1: fazer os testes passarem (no PC)

Abram `lib/stepseq/stepseq.c`. Há dois exercícios marcados com `TODO`:

1. Completar a tabela das 4 fases (a fase 0 já está feita).
2. Implementar `stepseq_step()`.

```
pio test -e native
```

✅ **Checkpoint 1:** os 8 testes de `test_stepseq` passam.

### Passo 2: ligar o motor X na placa

| ULN2003 | ESP32 |
|---|---|
| `IN1` | GPIO 18 |
| `IN2` | GPIO 19 |
| `IN3` | GPIO 21 |
| `IN4` | GPIO 22 |
| `+` | 5 V de fonte externa (não usar o 3V3 do ESP32) |
| `-` | GND, **comum com o GND do ESP32** |

✅ **Checkpoint 2:**
- `f` dá uma volta (motor: 2048) e o encoder mostra ≈ **+80**;
- `b` volta e os dois voltam para ≈ **0**;
- se o encoder contar negativo com `f`, troquem IN1↔IN4 e IN2↔IN3 em `pins.h`.

### Passo 3: rampa de aceleração e movimento sem bloqueio

```
lib/rampa/                perfil trapezoidal em inteiros (lógica pura, testável no PC)
lib/stepper/              interrupção de timer que gera os passos de até 3 motores
test/test_rampa/          testes da rampa (tempo, simetria, triângulo, parar)
```

**Rampa.** O motor parte em `v0`, acelera `a` passos/s² até `vmax` e freia a tempo de chegar em `v0` no último passo. Movimento curto vira triângulo (não chega em `vmax`).

```
 vmax |     ________
      |    /        \
 v0   |___/          \___
      +--------------------> passos
```

**Sem bloqueio.** Uma interrupção de timer a **20 kHz** (`STEPPER_TICK_HZ`), no núcleo 1, chama a rampa de cada motor a cada tique e escreve as 4 bobinas de uma vez no registrador do GPIO. O `loop()` só manda comandos:

| Função | Faz |
|---|---|
| `stepper_start(&m, passos, soltar)` | começa a andar e **volta na hora** |
| `stepper_start_to(&m, alvo, soltar)` | idem, para uma posição absoluta |
| `stepper_busy(&m)` | ainda andando? |
| `stepper_stop(&m)` / `stepper_abort(&m)` | freia pela rampa / para já |
| `stepper_move(&m, passos)` | atalho bloqueante (start + espera) |
| `stepper_sensor_arm(...)` | a interrupção para o motor quando um pino fica no estado pedido e guarda a posição (usado no homing) |

Por que **inteiros** na rampa: no ESP32 a interrupção não pode usar ponto flutuante. A velocidade fica em Q32 "passos por tique": a cada tique `acc += v`; quando `acc` estoura, é um passo. A frenagem começa quando `faltam · 2 · dv <= v² − v0²` (sem divisão).

Comandos da bancada:

| Comando | Faz |
|---|---|
| `f`, `b3`, `x50` | andam sem bloquear; a serial avisa `-- parou --` no fim |
| `q` | freia pela rampa |
| `l` | para na hora e solta as bobinas |
| `v800` | velocidade máxima (passos/s) |
| `a1500` | aceleração (passos/s²); `a0` = sem rampa |
| `n300` | velocidade de partida |
| `v` | mostra o perfil (e quanto tempo/distância leva para acelerar) |

Enquanto o motor anda, comandos que mexem nele (`f b h c d x t r 1-4 p o`) respondem `motor andando...`. `v`, `a`, `n`, `s`, `e`, `k`, `m` funcionam a qualquer momento.

✅ **Checkpoint:**
1. `f3` volta na hora; `v` durante o movimento responde; no fim aparece `-- parou --`.
2. `f20` e `q` no meio: o motor freia suave e para.
3. Subir `v` (800, 1000, 1200...) com `a1500` até perder passo; anotar o limite. Conferir com `x0`/`x50` e medida, ou com `d100`.
4. `t2` com rampa: tempo real perto do ideal (o ideal agora vem da própria rampa).

### Testes automáticos da rampa (`w` e `g`)

Os dois medem o **erro de seguimento**: contagens lidas pelo encoder menos as esperadas pelos passos do motor (contagens/mm ÷ passos/mm). Motor que perde passo deixa o encoder para trás. Tolerância: 5 contagens + 2% do esperado. Precisam de homing (fazem se faltar). Os pontos ficam em `TESTE_A_MM` / `TESTE_B_MM` (10 e 40 mm): **dentro do curso real**.

| Comando | Faz |
|---|---|
| `w` / `w1400` | vaivém 10 ↔ 40 mm subindo a vmax (500, 550 … 800, ou até o número); para na primeira que perder passo; homing no fim para conferir; mostra o limite e sugere 80% dele |
| `g` / `g50` | grava 30 mm (ou o valor) a cada 10 ms e imprime CSV `t_ms;v_motor;v_encoder;motor_passos;encoder_cont;erro_cont` para plotar a curva |

A aceleração e a partida são as atuais (`a`, `n`): para comparar acelerações, mude `a` e rode `w` de novo.

## Resolução (KY-040 no eixo de saída, fuso M8 passo 1,25 mm)

| | Contagens/volta | Resolução |
|---|---|---|
| Motor 28BYJ-48 (passo completo) | 2048 | ≈ 0,6 µm |
| Encoder KY-040 (x4) | 80 | ≈ 15,6 µm |

Cada contagem do encoder ≈ 25 passos do motor. O encoder **confere** o movimento (detecta perda de passo e travamento); quem posiciona é o motor de passo.

## Módulo 3: Homing e auto calibração

```
lib/homing/               FSM do homing, lógica pura (testável no PC)
lib/calib/                estatística da calibração (min, max, média), lógica pura
lib/eixo/                 eixo no ESP32: motor + encoder + fim de curso
test/test_homing/         testes do homing com um eixo simulado
```

A **chave de fim de curso** define a origem e o **encoder vigia**: se o motor manda passos e o encoder não muda, o eixo travou (batente, eixo preso, motor sem força) e o homing para com erro.

### Ligação da chave (eixo X, lado `f`)

```
GND --[10 kΩ]--+-- GPIO 36
               +-- chave NA -- 3V3        (acionada = HIGH)
```

O GPIO 36 é só entrada e **não tem pull-down interno**: o resistor de 10 kΩ para GND é obrigatório. Sem ele, com a chave aberta, o pino fica flutuando e o homing aciona em lugar aleatório. Nunca ligue o pino direto no 3V3 e no GND ao mesmo tempo (curto): um dos lados passa sempre pelo resistor.

A polaridade é o 5º argumento de `eixo_begin()` no `main.cpp`: `false` = ativa em HIGH (atual), `true` = ativa em LOW (pull-up + chave para GND).

### Homing (`h`)

| Estado | Faz | Sai quando |
|---|---|---|
| busca rápida | anda em `f` a 500 passos/s | a chave aciona |
| recuo | anda em `b` | a chave solta |
| recuo extra | anda em `b` | ¼ de volta |
| busca lenta | anda em `f` a 150 passos/s | a chave aciona → **origem = 0** |

O lado da chave é um parâmetro: `homingX.sentido = +1` (lado `f`) em `configurarHoming()`, no `main.cpp`. Com a chave do lado `b`, use `-1`; homing e calibração invertem juntos.

O homing **precisa que o sensor solte** no recuo: apertou → o motor recua → solta → aproxima devagar → apertou de novo = fim. Testando com um fio no lugar da chave, encoste no 3V3, **tire**, e encoste de novo quando aparecer `aperte o sensor de novo`. A serial mostra cada passo:

```
[homing] estado: busca rapida, girando 'f'  (motor 0, encoder 0)
[homing] SENSOR APERTADO  (motor 3120, encoder 121)
[homing] estado: recuo, girando 'b'  (motor 3122, encoder 121)
[homing]   -> esperando o sensor soltar
[homing] SENSOR solto  (motor 2900, encoder 113)
[homing] estado: busca lenta, girando 'f'  (motor 2388, encoder 93)
[homing]   -> aperte o sensor de novo
[homing] SENSOR APERTADO  (motor 2950, encoder 115)
[homing] estado: ok, girando '-'  (motor 2952, encoder 115)
```

Se aparecer `sensor JA APERTADO na partida`, o homing começa recuando (girando ao contrário). Com o fio solto, isso indica pino flutuando: falta o resistor de 10 kΩ para GND.

Se a chave já estiver acionada, começa pelo recuo. As duas aproximações fazem a origem cair sempre no mesmo ponto: a busca lenta chega sempre do mesmo lado e na mesma velocidade.

| Erro | Significa |
|---|---|
| chave não encontrada no curso máximo | andou 30 voltas sem achar a chave |
| chave não soltou no recuo | recuou 4 voltas com a chave acionada (chave presa ou fiação em curto) |
| chave não encontrada na busca lenta | achou na rápida, mas não na lenta (contato ruim) |
| eixo travado (encoder parado) | 256 passos seguidos sem o encoder mudar |
| cancelado | alguém mandou uma tecla + Enter |

### Auto calibração (`c`, `c3`...)

1. Homing: define a origem.
2. Anda 10 voltas afastando da chave (sentido `b`) e mede as contagens do encoder.
3. Homing de novo: mede em que passo e em que contagem a chave acionou.
4. Repete 2 e 3 pelo número de ciclos pedido.

| Resultado | Ideal | Se não for |
|---|---|---|
| chave (passos) | ~0 em todos os ciclos | perda de passo do motor ou chave imprecisa |
| variação da chave | poucos passos | repetibilidade da origem (em graus do eixo) |
| encoder na chave | ~0 | o encoder perdeu contagens na ida e volta |
| encoder por volta | ~80 (a ida em `b` deve dar ~−800) | repique ou contagem errada do encoder |

Cada ciclo leva cerca de 1,5 min (10 voltas a 500 passos/s, ida e volta).

✅ **Checkpoint:**
1. `s` com a chave solta mostra `solta` e `LOW`; apertando, `ACIONADA` e `HIGH`.
2. `h` termina em `homing ok`, e o motor fica em 0.
3. Segurar o eixo durante o `h` dá `eixo travado`.
4. `c3`: chave a poucos passos de 0 e encoder perto de 80 por volta.

## Módulo 4: Calibração de distância (mm)

Fuso **TR8x8**: avanço de **8 mm por volta**.

| | Nominal |
|---|---|
| motor | 2048 passos/volta ÷ 8 mm = **256 passos/mm** |
| encoder | 80 contagens/volta ÷ 8 mm = **10 contagens/mm** |
| 100 mm | 12,5 voltas = 25.600 passos (~51 s a 500 passos/s) |

O valor real é diferente do nominal (redução do 28BYJ-48 não é exatamente 2048, passo do fuso, folgas). A calibração mede o deslocamento de verdade e corrige.

### Como fazer

1. `d100` (ou só `d`): homing, afasta 2 mm para **tirar a folga** da inversão de sentido, e espera 5 s.
2. Quando aparecer `MARQUE a posicao do carro agora`, marque (ou zere o paquímetro/relógio comparador).
3. O eixo anda 100 mm. Meça quanto o carro andou desde a marca.
4. Envie `m` + o valor medido: `m99.6` (vírgula também vale: `m99,6`).

```
pedido 100.00 mm, medido 99.60 mm -> erro -0.40%
encoder viu 100.2% do giro do motor
passos/mm:    256.000 -> 257.028
contagens/mm: 10.000 -> 10.060
salvo na memoria (vale depois do reset). Confira com x<mm>, ex.: x50
```

- **passos/mm** = passos dados ÷ mm medidos. É o que o movimento em mm usa.
- **contagens/mm** = contagens do encoder ÷ mm medidos: calibra a leitura do encoder em mm.
- **encoder viu X%**: contagens lidas ÷ esperadas pelos passos. Longe de 100% = encoder perdendo (ou sobrando) contagens; a calibração em mm só compensa se a perda for sempre igual.
- Medida mais de **10%** longe do pedido é recusada (ex.: digitou `m10` em vez de `m100`). Envie de novo.
- Os valores ficam salvos na flash do ESP32 (`Preferences`) e são carregados no boot.

| Comando | Faz |
|---|---|
| `d`, `d100` | calibração de distância (padrão 100 mm) |
| `m99.6` | informa a medida real e aplica |
| `k` / `k0` | mostra a calibração / volta ao nominal |
| `x50` / `x` | vai para 50 mm da chave / só mostra a posição (precisa de homing) |

Depois do homing, a linha de estado mostra a posição em mm pelo motor e pelo encoder.

`CURSO_X_MM` (200 mm) no `main.cpp` limita o `d` e o `x`: **ajustar ao curso real do eixo**.

✅ **Checkpoint:**
1. `d100`, medir, `m<valor>`: aceita e mostra os valores novos.
2. `x50` e medir a partir da chave: ~50 mm. `x0` volta à chave.
3. Repetir `d100`: o erro deve cair para perto de 0%.
4. Reset (botão EN): o boot mostra a calibração salva.

## Diagnóstico: medição de velocidade (`t`)

Responde se a amostragem do encoder atrasa o motor. `t` (1 volta) ou `t2` (2 voltas), na velocidade atual (`v`):

1. N voltas em `f` com o encoder **ligado**, cronometradas com `micros()`;
2. N voltas em `b` com o timer do encoder **parado**;
3. compara os dois tempos com o ideal ((passos + 1) × intervalo) e mostra o custo da amostragem.

```
encoder LIGADO      3.415 s  (ideal 3.415 s)  ->   599.8 passos/s   +0.03%
encoder PARADO      3.415 s  (ideal 3.415 s)  ->   599.8 passos/s   +0.02%
diferenca ligado - parado: +40 us (+0.001%)
amostragem: nucleo 0, 6830 amostras (2000/s, ideal 2000/s)
            duracao media 1.10 us, maxima 4 us; maior intervalo 560 us (ideal 500)
            ocupa 0.220% do nucleo 0 ...
```
(números ilustrativos)

- **diferença ligado − parado** perto de 0: o encoder não atrapalha o motor.
- **% em relação ao ideal**: o atraso do próprio laço do motor (escrita das bobinas, espera ocupada).
- **núcleo**: a amostragem deve rodar no núcleo 0 e o motor no 1.
- **maior intervalo**: muito acima de 500 µs = amostras atrasadas (risco de perder contagem em velocidade alta).
- Na volta com o encoder parado ele não conta; no fim a contagem é realinhada pela posição do motor.
