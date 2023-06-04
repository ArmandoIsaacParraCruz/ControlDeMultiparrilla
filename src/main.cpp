#include <Arduino.h>
#include <ControlDeMultiparrilla.h>

uint64_t tiempoAnterior;
uint64_t tiempoDeInicio;

void setup()
{
  Serial.begin(115200);
  Serial.println("Iniciando");
  inicializar_control_de_multiparrilla();
  comenzar_prueba = false;
  tiempoAnterior = millis();
  while(!comenzar_prueba) {
    if(millis()-tiempoAnterior >= 1000){
   			Serial.println("Esperando...");
    		tiempoAnterior = millis();
		}
  }
  tiempoAnterior = millis();
  while(millis() - tiempoAnterior >= 1500) {
     tiempoAnterior = millis();
  }
  Serial.println("listo para comenzar");
  funcion_de_prueba();
  tiempoDeInicio = millis();
  tiempoAnterior = millis();
}

void loop()
{
  if(millis()-tiempoAnterior >= 1000){
    Serial.print("Velocidad [rpm]:");Serial.print(motores[motor1].rpmFiltrado);Serial.print("  Velocidad deseada[rpm]:");Serial.print(motores[motor1].setpointActual);
    Serial.print("  Tiempo[s]:");Serial.println((millis()- tiempoDeInicio)/1000.0);
    tiempoAnterior = millis();
  }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



/*#include <Arduino.h>
#include <ControlDeMultiparrilla.h>

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
    Serial.print("Velocidad [rpm]:");Serial.print(motores[motor1].rpmFiltrado);Serial.print("  Velocidad deseada[rpm]:");Serial.print(motores[motor1].setpointActual);
    Serial.print("  Tiempo[s]:");Serial.println(millis()/1000.0);
    tiempoAnterior = millis();
  }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
*/