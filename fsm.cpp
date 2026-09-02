#include "fsm.h"
#include "sensores.h"
#include "motores.h"

enum Acao : uint8_t { PARADO, PID, FRENTE, GIRA_D, GIRA_E, BUSCA_D, BUSCA_E, RE };

struct Transicao {
  Estado de;
  const char* padrao;
  uint32_t ms;
  Estado para;
};

static const Transicao TABELA[] = {
  {E0,  nullptr,   T_LARGADA, E1},
  {E1,  "*",       0,    E2},
  {E2,  nullptr,   0,    E3},

  {E3,  "0000000", 0,    E1},
  {E3,  "1111111", 0,    E4},
  {E3,  "11n0n11", 0,    E52},
  {E3,  "10n1n01", 0,    E5D},
  {E3,  "00n1n01", 0,    E6},
  {E3,  "001n0n1", 0,    E6},
  {E3,  "10n1n00", 0,    E27},
  {E3,  "1n0n100", 0,    E27},

  {E4,  "00n1n00", 0,    E3},
  {E4,  "1111111", 0,    E6},

  {E5D, "1111111", 0,    E4},
  {E5D, "0nn1nn0", 0,    E7},
  {E5E, "1111111", 0,    E4},
  {E5E, "0nn1nn0", 0,    E28},

  {E6,  "10n1n01", 0,    E5D},
  {E6,  "0nn1nn0", 0,    E7},
  {E7,  "00n1n01", 0,    E10},
  {E7,  "10n1n00", 0,    E48},
  {E7,  "nnn1111", 0,    E8},
  {E8,  nullptr,   250,  E9},
  {E9,  "00n1n00", 0,    E3},

  {E10, "nnn1111", 0,    E8},
  {E10, "0nn1nn0", 0,    E11},
  {E11, "00n1n01", 0,    E15},
  {E11, "nnn1111", 0,    E12},
  {E12, nullptr,   250,  E13},
  {E13, "00n1n00", 0,    E14},
  {E14, "nnn1111", 0,    E8},

  {E15, "0nn1nn0", 0,    E16},
  {E16, "00n1n01", 0,    E21},
  {E16, "nnn1111", 0,    E17},
  {E17, nullptr,   250,  E18},
  {E18, "00n1n00", 0,    E19},
  {E19, "nnn1111", 0,    E20},
  {E20, "0nn1nn0", 0,    E14},

  {E21, "nnn1111", 0,    E17},
  {E21, "0nn1nn0", 0,    E22},
  {E22, "nnn1111", 0,    E23},
  {E23, nullptr,   250,  E24},
  {E24, "00n1n00", 0,    E25},
  {E25, "nnn1111", 0,    E26},
  {E26, "0nn1nn0", 0,    E10},

  {E27, "10n1n01", 0,    E5E},
  {E27, "0nn1nn0", 0,    E28},
  {E28, "10n1n00", 0,    E31},
  {E28, "00n1n01", 0,    E50},
  {E28, "1111nnn", 0,    E29},
  {E29, nullptr,   250,  E30},
  {E30, "00n1n00", 0,    E3},

  {E31, "1111nnn", 0,    E29},
  {E31, "0nn1nn0", 0,    E32},
  {E32, "10n1n00", 0,    E36},
  {E32, "1111nnn", 0,    E33},
  {E33, nullptr,   250,  E34},
  {E34, "00n1n00", 0,    E35},
  {E35, "1111nnn", 0,    E29},

  {E36, "0nn1nn0", 0,    E37},
  {E37, "10n1n00", 0,    E42},
  {E37, "1111nnn", 0,    E38},
  {E38, nullptr,   250,  E39},
  {E39, "00n1n00", 0,    E40},
  {E40, "1111nnn", 0,    E41},
  {E41, "0nn1nn0", 0,    E35},

  {E42, "1111nnn", 0,    E38},
  {E42, "0nn1nn0", 0,    E43},
  {E43, "1111nnn", 0,    E44},
  {E44, nullptr,   250,  E45},
  {E45, "00n1n00", 0,    E46},
  {E46, "1111nnn", 0,    E47},
  {E47, "0nn1nn0", 0,    E31},

  {E48, "0nnnn1n", 0,    E49},
  {E49, "1111nnn", 0,    E29},
  {E50, "n1nnnn0", 0,    E51},
  {E51, "nnn1111", 0,    E8},

  {E52, "11n0n11", 0,    E53},
  {E52, nullptr,   100,  E3},
  {E53, "00n1n00", 0,    E54},
  {E54, "0000000", 0,    E55},
  {E55, nullptr,   5000, E56},
  {E56, nullptr,   0,    E57},
  {E57, "*",       0,    E58},
  {E58, "0000000", 0,    E59},
  {E59, "*",       0,    E3},
};

static const uint8_t ACAO[NUM_ESTADOS] = {
  PARADO, 
  PARADO, 
  FRENTE, 
  PID, 
  FRENTE, 
  FRENTE, 
  FRENTE,
  PID, 
  FRENTE, 
  GIRA_D, 
  BUSCA_D,
  PID, 
  FRENTE, 
  GIRA_D, 
  BUSCA_D, 
  PID,
  PID, 
  FRENTE, 
  GIRA_D, 
  BUSCA_D, 
  PID, 
  PID,
  PID, 
  FRENTE, 
  GIRA_D, 
  BUSCA_D, 
  PID, 
  PID,
  PID, 
  FRENTE, 
  GIRA_E,
  BUSCA_E, 
  PID, 
  FRENTE,
  GIRA_E, 
  BUSCA_E, 
  PID,
  PID, 
  FRENTE, 
  GIRA_E, 
  BUSCA_E, 
  PID, 
  PID,
  PID, 
  FRENTE, 
  GIRA_E, 
  BUSCA_E, 
  PID, 
  PID,
  PID, 
  RE, 
  PID, 
  RE,
  PID, 
  PID, 
  PID, 
  PARADO, 
  FRENTE, 
  FRENTE, 
  PID, 
  FRENTE,
};

static const char* NOME[NUM_ESTADOS] = {
  "Parado 2s", 
  "Lendo preto", 
  "Partida", 
  "Seguir Linha", 
  "Encruzilhada",
  "quad 2 lados DIR", 
  "quad 2 lados ESQ",
  "leu quad DIR", 
  "parou quad DIR", 
  "virando DIR", 
  "procurando DIR",
  "leu quad DIR2", 
  "parou quad DIR2", 
  "virando DIR2", 
  "procurando DIR2", 
  "Seguindo DIR2",
  "leu quad DIR3", 
  "parou quad DIR3", 
  "virando DIR3", 
  "procurando DIR3", 
  "Seguindo DIR3", 
  "linha DIR3",
  "leu quad DIR4", 
  "parou quad DIR4",
  "virando DIR4", 
  "procurando DIR4", 
  "Seguindo DIR4", 
  "linha DIR4",
  "leu quad ESQ", 
  "parou quad ESQ", 
  "virando ESQ", 
  "procurando ESQ",
  "leu quad ESQ2", 
  "parou quad ESQ2", 
  "virando ESQ2", 
  "procurando ESQ2", 
  "Seguindo ESQ2",
  "leu quad ESQ3", 
  "parou quad ESQ3", 
  "virando ESQ3", 
  "procurando ESQ3", 
  "Seguindo ESQ3", 
  "linha ESQ3",
  "leu quad ESQ4", 
  "parou quad ESQ4", 
  "virando ESQ4", 
  "procurando ESQ4", 
  "Seguindo ESQ4", 
  "linha ESQ4",
  "quad ESQ RE", 
  "RE para ESQ", 
  "quad DIR RE", 
  "RE para DIR",
  "possivel invertida", 
  "invertida", 
  "posicionando", 
  "Parar 5s",
  "Partida faixa", 
  "Andar reto", 
  "Andar faixa", 
  "Passou faixa",
};

static const size_t N_TRANS = sizeof(TABELA) / sizeof(TABELA[0]);

static Estado estado = E0;
static uint32_t entrouEm = 0;
static float integral = 0;
static int16_t erroAnt = 0;

static bool casa(const char* p, const uint8_t* b) {
  for (uint8_t i = 0; i < NUM_SENSORES; i++) {
    if (p[i] == 'n') 
    {
      continue;
    }
    if ((p[i] == '1') != (b[i] != 0)) 
    {
      return false;
    }
  }
  return true;
}

static void executar(uint8_t a) {
  switch (a) {
    case PARADO:  
      motoresParar(); 
      break;

    case FRENTE:  
      motoresDefinir(VEL_BASE, VEL_BASE); 
      break;

    case RE:      
    motoresDefinir(-VEL_RE, -VEL_RE); 
    break;

    case GIRA_D:  
      motoresDefinir(VEL_GIRO, -VEL_GIRO); 
      break;

    case GIRA_E:  
      motoresDefinir(-VEL_GIRO, VEL_GIRO); 
      break;

    case BUSCA_D: 
    motoresDefinir(VEL_BUSCA, -VEL_BUSCA); break;

    case BUSCA_E: 
      motoresDefinir(-VEL_BUSCA, VEL_BUSCA); 
      break;

    case PID: 
      int16_t e = sensoresErro();
      integral = constrain(integral + e, -8000.0f, 8000.0f);
      int corr = (int)(KP * e + KI * integral + KD * (e - erroAnt));
      erroAnt = e;
      motoresDefinir(VEL_BASE + corr, VEL_BASE - corr);
      break;
    
  }
}

void fsmInit() {
  estado = E0;
  entrouEm = millis();
  integral = 0;
  erroAnt = 0;
}

void fsmIrPara(Estado novo) {
  if (novo == estado) return;
  Serial.printf("%s -> %s\n", NOME[estado], NOME[novo]);
  estado = novo;
  entrouEm = millis();
  integral = 0;
  erroAnt = sensoresErro();
}

void fsmAtualizar() {
  const uint8_t* b = sensoresBits();

  for (size_t i = 0; i < N_TRANS; i++) {
    const Transicao& t = TABELA[i];
    if (t.de != estado) continue;

    bool ok;
    if (!t.padrao)               ok = (millis() - entrouEm >= t.ms);
    else if (t.padrao[0] == '*') ok = sensoresAlgum();
    else                         ok = casa(t.padrao, b);

    if (ok) { fsmIrPara(t.para); break; }
  }

  executar(ACAO[estado]);
}

Estado fsmEstado() 
{ 
  return estado; 
}

const char* fsmNome(Estado e) 
{ 
  return NOME[e]; 
}