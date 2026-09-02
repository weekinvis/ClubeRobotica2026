#pragma once
#include <Arduino.h>

#define S1 15
#define S2 14
#define S3 27
#define S4 4
#define S5 2
#define S6 12
#define S7 13

#define IN1 34
#define IN2 32
#define IN3 3
#define IN4 25

#define NUM_SENSORES 7
#define LIMIAR_PADRAO 2000
#define HISTERESE 250
#define LINHA_ALTA true

#define T_CALIBRACAO 5000
#define T_LARGADA 2000

#define VEL_BASE 140
#define VEL_GIRO 150
#define VEL_BUSCA 100
#define VEL_RE 120

#define KP 0.55f
#define KI 0.0f
#define KD 4.0f