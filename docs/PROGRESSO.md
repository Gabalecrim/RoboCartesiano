# Progresso do projeto

Registro de onde paramos, para o grupo retomar. Atualizar a cada sessão.

**Última sessão:** 20/10/2026

---

## Resumo do estado

| Módulo | Lógica (PC) | Na placa | Situação |
|---|---|---|---|
| 1. Encoder | exercícios feitos | compila, **não testado** | falta rodar os testes e o Checkpoint 2 |
| 2. Motor de passo | exercícios feitos | compila, **não testado** | falta rodar os testes e o Checkpoint 2 |

O firmware (`pio run -e esp32dev`) compila **sem erros e sem avisos** (`-Wall -Wextra`).
Os testes do PC (`pio test -e native`) **ainda não rodaram**: falta o `gcc` no Windows (ver Pendências).

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

---

## Pendências

1. **Instalar o `gcc` no Windows** (MSYS2 → MinGW-w64, colocar no PATH) e rodar
   `pio test -e native`. Esperado: `test_quadrature` 10/10 e `test_stepseq` 8/8.
2. ~~Instalar o Git e versionar o projeto~~ ✔ feito em 21/10/2026 (repositório local, branch
   `main`, no GitHub em https://github.com/Gabalecrim/RoboCartesiano).
3. **Pinos do encoder X divergentes**: a decisão registrada era GPIO **34/35**, mas
   `include/pins.h` está com **18/19**. Confirmar com o grupo qual vale.
4. **Faltam pinos de saída** para 3 motores (12 fios) + garra com os encoders nos pinos
   atuais. Resolver na etapa dos 3 eixos (opções: encoder X em 34/35, registrador de
   deslocamento 74HC595).
5. **Algoritmo genético**: com um único alimentador fixo a ordem não muda a distância total.
   Decidir bandeja com vários slots ou mais de um tipo de pino **antes** do módulo do GA.
6. Limpeza pequena: apagar os comentários `TODO` que sobraram nas tabelas de
   `quadrature.c` e `stepseq.c`.

---

## Próximos passos (na ordem)

1. **Checkpoint 1 dos dois módulos** no PC (depende da pendência 1).
2. **Checkpoint 2 do encoder** na placa: KY-040 no eixo X, uma volta = +80, ida e volta = 0.
3. **Checkpoint 2 do motor** na placa: ULN2003 em GPIO 16/17/21/22, **5 V externo e GND comum**.
   Comando `f` → motor 2048 e encoder ≈ +80; `b` → os dois voltam a ≈ 0.
   Se o encoder contar negativo, trocar IN1↔IN4 e IN2↔IN3 em `pins.h`.
4. **Motor, Passo 3**: movimento sem bloquear (`loop()` continua rodando) e rampa de aceleração.
5. Encoder com PCNT e três eixos.
6. Depois: homing → movimento XYZ em mm → garra → receitas → sequenciador → Modbus TCP/IHM → GA.

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

Forma de trabalho combinada: para cada módulo, primeiro o conceito, depois o contrato (`.h`)
e os testes; o núcleo fica como exercício `TODO` para o grupo; em seguida, revisão.
