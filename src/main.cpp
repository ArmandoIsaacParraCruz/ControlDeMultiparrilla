#include <Arduino.h>
#include "ControlDeMotores.h"

struct Motor encendido;
struct Motor apagado;


int tiempoAnterior ;

void initStructsDePrueba(){
  encendido.activado = true;
  encendido.minutosParaMantenerSetpoints.push(1);
  encendido.setpoints.push(500);
  encendido.minutosParaMantenerSetpoints.push(2);
  encendido.setpoints.push(100);
  apagado.activado = false;
}

 // Serial.println("Se han actualizado los PID");
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup()
{
  initStructsDePrueba();
  struct Motor configDePrueba[6] = {encendido,apagado,apagado,apagado,apagado,apagado};
  Serial.begin(9600);
  configurarMotores(configDePrueba);
  tiempoAnterior = millis();
}

void loop()
{
  if(millis()-tiempoAnterior >= 1000){
    Serial.print(motores[motor1].rpm);Serial.print(" ");Serial.print(motores[motor1].rpmFiltrado);Serial.print(" ");Serial.print(motores[motor1].setpointActual);
    Serial.print(" ");Serial.print(motores[motor1].cicloDeTrabajo);Serial.print(" ");Serial.println(millis()/1000.0);
    tiempoAnterior = millis();
  }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

