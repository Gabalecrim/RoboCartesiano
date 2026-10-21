# Pick and Place cartesiano (ESP32)

Projeto da Jornada de Aprendizagem: robô XYZ que posiciona pinos num grid conforme receitas, com sequência otimizada por algoritmo genético e IHM no iX Developer via Modbus TCP.

O código é construído **um módulo por vez**, e cada módulo é testado antes de seguir.

## Estrutura

```
platformio.ini            dois ambientes: esp32dev (placa) e native (testes no PC)
include/pins.h            TODA a pinagem do projeto
lib/quadrature/           lógica pura do encoder (sem hardware, testável no PC)
lib/encoder/              encoder no ESP32 (pinos + interrupção)
lib/stepseq/              lógica pura do motor de passo (fases + contagem)
lib/stepper/              motor no ESP32 (ULN2003)
src/main.cpp              programa de teste da etapa atual (C++ só por causa do Serial)
test/test_quadrature/     testes automáticos da lógica do encoder
test/test_stepseq/        testes automáticos da lógica do motor
docs/PROGRESSO.md         onde paramos, decisões e próximos passos
```

Os módulos são escritos em **C**: cada um é uma `struct` + funções `modulo_*(&obj, ...)`.

A separação entre `quadrature` e `encoder` é proposital: **a lógica que decide para onde o eixo girou não depende da placa**. Isso permite testar no PC, sem montar nada, e depois trocar a forma de ler os pinos (interrupção → PCNT) sem mexer na lógica.

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

### Passo 3: PCNT (próxima etapa)

Trocar o "por dentro" de `lib/encoder/` para usar o periférico **PCNT** do ESP32, que decodifica a quadratura em hardware, com filtro de ruído e sem interrupção. A interface pública (`begin`, `count`, `errors`, `reset`) **não pode mudar**: o `main.cpp` deve continuar funcionando sem alteração.

### Passo 4: três eixos

Três instâncias de `Encoder`, uma por eixo, imprimindo `X Y Z`.

## Módulo 2: Motor de passo (28BYJ-48 + ULN2003)

```
lib/stepseq/              lógica pura: fases das bobinas e contagem de passos
lib/stepper/              motor no ESP32 (4 pinos do ULN2003, movimento bloqueante)
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
| `IN1` | GPIO 16 |
| `IN2` | GPIO 17 |
| `IN3` | GPIO 21 |
| `IN4` | GPIO 22 |
| `+` | 5 V de fonte externa (não usar o 3V3 do ESP32) |
| `-` | GND, **comum com o GND do ESP32** |

✅ **Checkpoint 2:**
- `f` dá uma volta (motor: 2048) e o encoder mostra ≈ **+80**;
- `b` volta e os dois voltam para ≈ **0**;
- se o encoder contar negativo com `f`, troquem IN1↔IN4 e IN2↔IN3 em `pins.h`.

### Passo 3 (depois): movimento sem bloquear e rampa de aceleração

## Resolução (KY-040 no eixo de saída, fuso M8 passo 1,25 mm)

| | Contagens/volta | Resolução |
|---|---|---|
| Motor 28BYJ-48 (passo completo) | 2048 | ≈ 0,6 µm |
| Encoder KY-040 (x4) | 80 | ≈ 15,6 µm |

Cada contagem do encoder ≈ 25 passos do motor. O encoder **confere** o movimento (detecta perda de passo e travamento); quem posiciona é o motor de passo.
