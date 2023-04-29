#include <Arduino.h>
#include "ControlDeMotores.h"

struct configuracionParaMotor encendido;
struct configuracionParaMotor apagado;


int tiempoAnterior ;

void initStructsDePrueba(){
  encendido.activado = true;
  encendido.setpoints.push(600);
  encendido.minutosParaMantenerSetpoints.push(1);
  encendido.setpoints.push(200);
  encendido.minutosParaMantenerSetpoints.push(2);
  encendido.setpoints.push(700);
  encendido.minutosParaMantenerSetpoints.push(3);
  encendido.setpoints.push(100);
  encendido.minutosParaMantenerSetpoints.push(4);
  encendido.setpoints.push(900);
  encendido.minutosParaMantenerSetpoints.push(5);
  encendido.setpoints.push(300);
  encendido.minutosParaMantenerSetpoints.push(6);
  apagado.activado = false;
}

 // Serial.println("Se han actualizado los PID");
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup()
{
  initStructsDePrueba();
  struct configuracionParaMotor configDePrueba[6] = {encendido,apagado,apagado,apagado,apagado,apagado};
  Serial.begin(9600);
  configurarMotores(configDePrueba);
  tiempoAnterior = millis();
}

void loop()
{
  if(millis()-tiempoAnterior >= 1000){
    Serial.print(motores[motor1].rpmFiltrado);Serial.print(" ");Serial.print(motores[motor1].setpointActual);
    Serial.print(" ");Serial.print(motores[motor1].cicloDeTrabajoActual);Serial.print(" ");Serial.println(millis()/1000.0);
    tiempoAnterior = millis();
  }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

