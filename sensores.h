#pragma once
#include "pinos.h"

void sensoresInit();
void sensoresCalibrar(uint32_t ms);
void sensoresLer();
const uint8_t* sensoresBits();
bool sensoresAlgum();
int16_t sensoresErro();
void sensoresImprimir();
