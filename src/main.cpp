#include <Arduino.h>
#include "ControlDeMotores.h"

struct Motor encendido = {true,0,0,120,0,0,0};
struct Motor apagado = {false,0,0,0,0,0,0};

struct Motor configDePrueba[6] = {encendido,apagado,apagado,apagado,apagado,apagado};


int tiempoAnterior ;

 // Serial.println("Se han actualizado los PID");
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup()
{
  Serial.begin(9600);
  configurarMotores(configDePrueba);
  tiempoAnterior = millis();
}

void loop()
{
  if(millis()-tiempoAnterior >= 1000){
    Serial.print(motores[motor1].rpm);Serial.print(" ");Serial.print(motores[motor1].rpmFiltrado);Serial.print(" ");Serial.print(motores[motor1].setPointVelocidad);
    Serial.print(" ");Serial.println(motores[motor1].cicloDeTrabajo);
    tiempoAnterior = millis();
  }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

