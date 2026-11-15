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
| 6. Proteção contra colisão (X) | colisao 12/12 | **não testado** | checkpoint "Proteção contra colisão" do README |
| 7. Eixo Z (malha aberta + homing) | (usa setpoint, calib e homing) | **não testado**, motor e chave do Z ainda não ligados | checkpoint "Eixo Z" do README |

O firmware (`pio run -e esp32dev`) compila **sem erros e sem avisos** (`-Wall -Wextra`).
Os testes do PC (`pio test -e native`) rodam: 115/115 (gcc instalado via WinLibs).

---

## O que foi feito em 14/11/2026

### Eixo Z em malha aberta
- Motor Z nos GPIOs 14, 25, 26, 27 (os 25/26 eram do encoder Z, que saiu: malha aberta;
  o Gab trocou o IN1 de 23 para 14). O 14 dá pulsos no boot: no máximo um tranco no Z.
- **Homing do Z** (`ZH`): chave NA **embaixo** (Z = 80 mm na chave; o zero continua no topo), GPIO 4 (escolha do Gab), pull-down externo de 10 kΩ.
  Usa `lib/homing` + `lib/eixo` sem encoder (`enc = NULL`, sem janela de travamento) e
  termina segurando (`EixoHomingCfg.segurar`). `h` e `R` fazem o `ZH` antes, se faltar.
  `Z_TEM_CHAVE = false` volta para a referência manual (`ZR`).
- Fuso igus de alto avanço, 8 entradas x ~3 mm = 24 mm/volta (85,33 passos/mm),
  **a confirmar** com `ZD50` + `ZM<medida>`. Curso 80 mm, Z = 0 no topo.
- Bobinas do Z **sempre ligadas** (fuso de alto avanço não trava). Alarme do X e `l`
  param o Z sem soltar. `ZL` solta (o Z cai e perde a referência).
- Referência manual (`ZR` no topo) continua valendo; sem proteção contra colisão no Z.
- Setpoints agora têm X e Z (`S<x> <z> [mm/s] [ms]`); o sequenciador anda um eixo por
  vez (Z sobe primeiro / X anda primeiro quando o Z desce). Programa do formato antigo é apagado.
- O homing do X sobe o Z ao topo antes (com `ZH`, se o Z estiver sem referência).
- Painel (`tools/painel.html`): campo Z, coluna Z, quadro do Z (homing, sobe/desce, zerar, ir).

### A decidir / medir
- Velocidade máxima real do Z (650 passos/s assumido, igual ao X).
- Se o Z desce sozinho sem corrente com a garra montada (decide `Z_SEGURAR`).
- Saídas livres para o Y e a garra: 13, 16, 17, 23 (faltam 5); o 39 está livre só como entrada.
- Testador de motor no painel: `TX` / `TZ` escolhe o motor de 1..4, p, o; `T0` termina.
- Z andava ao contrário: `Z_DESCE = -1`. Não subia bem a 650 passos/s: padrão 300, partida 150,
  aceleração 600 (comandos novos `ZA` e `ZN`); homing do Z a 250 / 120.
- Painel com todos os comandos da serial (X manual, rampa, homing/calibração, Z, testes).

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
