#include "motores.h"

static void acionar(int vel, uint8_t a, uint8_t b) {
  vel = constrain(vel, -255, 255);
  if (vel >= 0) 
  { 
    analogWrite(a, vel); 
    analogWrite(b, 0); 
  }
  else          
  { 
    analogWrite(a, 0);   
    analogWrite(b, -vel); 
  }
}

void motoresInit() 
{
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  motoresParar();
}

void motoresDefinir(int esq, int dir) 
{
  acionar(esq, IN1, IN2);
  acionar(dir, IN3, IN4);
}

void motoresParar() 
{
  analogWrite(IN1, 0); 
  analogWrite(IN2, 0);
  analogWrite(IN3, 0); 
  analogWrite(IN4, 0);
}

void motoresFrear() {
  analogWrite(IN1, 255); 
  analogWrite(IN2, 255);
  analogWrite(IN3, 255); 
  analogWrite(IN4, 255);
}
