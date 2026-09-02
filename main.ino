#include <Arduino.h>
#include "pinos.h"
#include "sensores.h"
#include "motores.h"
#include "fsm.h"

static uint32_t proximo = 0;

void setup() {
  Serial.begin(115200);

  delay(300);

  motoresInit();

  sensoresInit();

  sensoresCalibrar(T_CALIBRACAO);


  fsmInit();

  proximo = micros();
  
}

void loop() {
  if ((int32_t)(micros() - proximo) < 0) return;
  proximo += 4000;

  sensoresLer();
  fsmAtualizar();

  static uint32_t log = 0;
  if (millis() - log > 200) 
  {
    log = millis();
    Serial.printf("%-20s ", fsmNome(fsmEstado()));
    sensoresImprimir();
  }
}