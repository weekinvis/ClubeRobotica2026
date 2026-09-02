#pragma once
#include "pinos.h"

enum Estado : uint8_t {
  E0  = 0,   // parado 2s, aguardando largada
  E1,        // lendo preto, espera qualquer sensor acender
  E2,        // partida, arranca para frente
  E3,        // seguir linha (PID) - hub central da maquina
  E4,        // encruzilhada, todos os sensores acesos

  E5D,       // quad nos dois lados, entrou pela DIREITA
  E5E,       // quad nos dois lados, entrou pela ESQUERDA

  E6,        // leu quad DIR
  E7,        // parou de ler quad DIR
  E8,        // virando pra DIR
  E9,        // procurando linha DIR
  E10,       // leu quad DIR2
  E11,       // parou de ler quad DIR2
  E12,       // virando pra DIR2
  E13,       // procurando linha DIR2
  E14,       // seguindo DIR2
  E15,       // leu quad DIR3
  E16,       // parou de ler quad DIR3
  E17,       // virando pra DIR3
  E18,       // procurando linha DIR3
  E19,       // seguindo DIR3
  E20,       // leu linha DIR3
  E21,       // leu quad DIR4
  E22,       // parou de ler quad DIR4
  E23,       // virando pra DIR4
  E24,       // procurando linha DIR4
  E25,       // seguindo DIR4
  E26,       // leu linha DIR4

  E27,       // leu quad ESQ
  E28,       // parou de ler quad ESQ
  E29,       // virando pra ESQ
  E30,       // procurando linha ESQ
  E31,       // leu quad ESQ2
  E32,       // parou de ler quad ESQ2
  E33,       // virando pra ESQ2
  E34,       // procurando linha ESQ2
  E35,       // seguindo ESQ2
  E36,       // leu quad ESQ3
  E37,       // parou de ler quad ESQ3
  E38,       // virando pra ESQ3
  E39,       // procurando linha ESQ3
  E40,       // seguindo ESQ3
  E41,       // leu linha ESQ3
  E42,       // leu quad ESQ4
  E43,       // parou de ler quad ESQ4
  E44,       // virando pra ESQ4
  E45,       // procurando linha ESQ4
  E46,       // seguindo ESQ4
  E47,       // leu linha ESQ4

  E48,       // leu quad ESQ durante trajeto DIR, prepara re
  E49,       // voltando de re para pegar a ESQ
  E50,       // leu quad DIR durante trajeto ESQ, prepara re
  E51,       // voltando de re para pegar a DIR

  E52,       // possivel pista invertida, confirma em 100ms
  E53,       // pista invertida confirmada
  E54,       // seguindo para posicionar na faixa
  E55,       // parado 5s na faixa de pedestres
  E56,       // partida na faixa
  E57,       // andar reto ate achar a faixa
  E58,       // andar sobre a faixa
  E59,       // passou da faixa, volta ao percurso

  NUM_ESTADOS
};

void fsmInit();
void fsmAtualizar();
void fsmIrPara(Estado e);
Estado fsmEstado();
const char* fsmNome(Estado e);