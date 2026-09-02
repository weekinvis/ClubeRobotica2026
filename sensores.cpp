#include "sensores.h"

static const uint8_t PINOS[NUM_SENSORES] = {S1, S2, S3, S4, S5, S6, S7};
static uint16_t limiar[NUM_SENSORES];
static uint8_t bits[NUM_SENSORES];
static uint8_t qtd = 0;
static int16_t erro = 0;

void sensoresInit() 
{
  analogReadResolution(12);
  for (uint8_t i = 0; i < NUM_SENSORES; i++) 
  {
    pinMode(PINOS[i], INPUT);
    limiar[i] = LIMIAR_PADRAO;
    bits[i] = 0;
  }
}

void sensoresCalibrar(uint32_t ms) 
{
  uint16_t mn[NUM_SENSORES], mx[NUM_SENSORES];
  for (uint8_t i = 0; i < NUM_SENSORES; i++) 
  { 
    mn[i] = 4095; 
    mx[i] = 0; 
  }

  uint32_t fim = millis() + ms;
  while (millis() < fim) 
  {
    for (uint8_t i = 0; i < NUM_SENSORES; i++) 
    {
      uint16_t v = analogRead(PINOS[i]);
      if (v < mn[i]) 
      {
        mn[i] = v;
      }
      if (v > mx[i]) 
      {
        mx[i] = v;
      }
    }
    delay(2);
  }

  for (uint8_t i = 0; i < NUM_SENSORES; i++)
  {
    if (mx[i] > mn[i] + 100) 
    {
      limiar[i] = (mn[i] + mx[i]) / 2;
    }
  }
}

void sensoresLer() {
  qtd = 0;
  int32_t soma = 0;

  for (uint8_t i = 0; i < NUM_SENSORES; i++) {
    uint16_t v = analogRead(PINOS[i]);
    if (!LINHA_ALTA)
    { 
      v = 4095 - v;
    }
    if (bits[i]) 
    { 
      if (v + HISTERESE < limiar[i]) 
      {
        bits[i] = 0; 
      }
    }
    else         
    { 
      if (v > limiar[i] + HISTERESE) 
      {
        bits[i] = 1; 
      }
    }

    if (bits[i]) 
    { 
      qtd++; 
      soma += (int32_t)i * 100 - 300; 
    }
  }

  if (qtd) erro = soma / qtd;
}

const uint8_t* sensoresBits() 
{ 
  return bits; 
}
bool sensoresAlgum() 
{ 
  return qtd > 0; 
}
int16_t sensoresErro() 
{ 
  return erro; 
}

void sensoresImprimir() {
  for (uint8_t i = 0; i < NUM_SENSORES; i++) 
  {
    Serial.print(bits[i] ? '1' : '0');
  }
  Serial.printf(" erro=%d\n", erro);
}
