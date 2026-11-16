#pragma once
// =============================================================
//  Eixo Y da bancada (2026-11-15): motor + encoder + chave de homing
// =============================================================
//
// Módulo separado do main.cpp: o X e o Z continuam iguais. Mesma lógica do
// X, com as mesmas bibliotecas:
//   - posição SÓ pelo encoder do Y (lib/malha: alvo em contagens, corrige parado)
//   - setpoint de velocidade pelo encoder (lib/governo: queda -> menos rotação,
//     tenta recuperar a cada 1 s, mantém 1 s depois de subir)
//   - nada para por interpretação do encoder (só avisa e ajusta a rotação)
//   - encoder sem sinal (motor anda 5 mm e o encoder não muda) não usa a
//     posição falsa
//
// Comandos (família Y, ver yAjuda): Y, YH, Y120, Y+10, Y-1, YR, YV1.5,
// YQ, YL, YF1, YB1, YE, YT, YG, YGQ2, YGA50, YGD70, YGS1000, YG0.
// Telemetria: linha "@Y" (ver eixo_y.cpp), à parte da "@T" do X/Z.

#include <Arduino.h>

void yIniciar();               // no fim do setup()
void yLoop();                  // a cada volta do loop()
void yComando(String resto);   // linha que começou com 'Y'
void yParar();                 // q: freia o Y pela rampa
void ySoltar();                // l: para na hora e solta as bobinas
bool yOcupado();               // motor andando ou malha corrigindo
void yAjuda();                 // linhas da ajuda

// Para o testador de bobinas (TY) e o programa de setpoints (S/R) do main.cpp.
#include "setpoint.h"
#include "stepper.h"
Stepper *yMotor();             // motor do Y (testador de bobinas)
bool yReferenciado();          // posição em mm do Y vale (YH ou YR)
// Vai para 'mm' pelo encoder, a 'vel_mm_s' (0 = setpoint do Y). false = já
// estava lá ou não pôde (encoder sem sinal / fora do curso).
bool yIrMm(float mm, float vel_mm_s);
void yRestaurarVelocidade();   // volta o setpoint do Y (fim do programa)
EixoMm yEixoMm();              // curso do Y para conferir setpoints

// Para o homing geral (HA), o teste geral (E / EY) e o 's' do main.cpp.
bool yHomingGeral();           // homing do Y; true = deu certo
int yTesteGeral();             // teste do Y: 0 = OK, 1 = FALHOU, 2 = CANCELADO
float yPosicaoMm();            // Y pelo encoder (mm)
void yMostrarChave();          // "chave Y: ACIONADA/solta (GPIO 39 = ...)"
