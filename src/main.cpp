#include <Arduino.h>
#include "ControlDeMotores.h"

struct Motor encendido = {true,0,0,0,0};
struct Motor apagado = {false,0,0,0,0};

struct Motor configDePrueba[6] = {encendido,apagado,apagado,apagado,apagado,apagado};


bool unaVez = true;



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup()
{
  Serial.begin(9600);
  configurarMotores(configDePrueba);
  
}

void loop()
{
  Serial.println(obtenerRevoluciones(motor1));
  delay(1000);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

