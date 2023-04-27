#include "ControlDeMotores.h"

struct Motor motores[6];

void configurarMotores(struct Motor *configDePrueba)
{
    for(uint8_t i = 0; i < CANT_MOTORES; i++){
        motores[i].activado = configDePrueba[i].activado;
        if(motores[i].activado){
            pinMode(pinMotores[i], OUTPUT);
            ledcSetup(i, FRECUENCIA_PWM_POR_DEFECTO, RESOLUCION_PWM_POR_DEFECTO);
            ledcAttachPin(pinMotores[i], i);
            motores[i].setPointVelocidad = configDePrueba[i].setPointVelocidad;
            pinMode(pinSensoresHall[i], INPUT_PULLUP);
            attachInterrupt(digitalPinToInterrupt(pinSensoresHall[i]), interrupcionesSensorHall[i], RISING);
            motores[i].tiempoAnterior = micros();
            motores[i].revoluciones = 0;
            motores[i].rpm = 0;
        }
    }
}


void IRAM_ATTR interrupcionSensorHall1()
{
    agregaUnaVuelta(motor1);
}

void IRAM_ATTR interrupcionSensorHall2()
{
    agregaUnaVuelta(motor2);
}
void IRAM_ATTR interrupcionSensorHall3()
{
    agregaUnaVuelta(motor3);
}
void IRAM_ATTR interrupcionSensorHall4()
{
    agregaUnaVuelta(motor4);
}
void IRAM_ATTR interrupcionSensorHall5()
{
    agregaUnaVuelta(motor5);
}
void IRAM_ATTR interrupcionSensorHall6()
{
    agregaUnaVuelta(motor6);
}

void agregaUnaVuelta(uint8_t numMotor)
{
    bool motorActivado = motores[numMotor].activado;
    if(motorActivado){
        bool condicionAntirrebote = micros()-motores[numMotor].tiempoAnterior >= TIEMPO_ANTIRREBOTE_SENSOR_HALL;
        bool interrupcionConfirmada = digitalRead(pinSensoresHall[numMotor]);
        if(condicionAntirrebote && interrupcionConfirmada){
            motores[numMotor].revoluciones += 1;
            motores[numMotor].tiempoAnterior = micros();
        }
    }
}


void (*interrupcionesSensorHall[6])() = {
    interrupcionSensorHall1,
    interrupcionSensorHall2,
    interrupcionSensorHall3,
    interrupcionSensorHall4,
    interrupcionSensorHall5,
    interrupcionSensorHall6
};

bool obtenerEstadoMotores(uint8_t numMotor){
    return motores[numMotor].activado;
}
double obtenerVelocidad(uint8_t numMotor){
    return motores[numMotor].rpm;
}
double obtenerSetPointVelocidad(uint8_t numMotor){
    return motores[numMotor].setPointVelocidad;
}
double obtenerRevoluciones(uint8_t numMotor){
    return motores[numMotor].revoluciones;
}
double obtenerTiempoAnterior(uint8_t numMotor){
    return motores[numMotor].tiempoAnterior;
}


void asignaCicloDeTrabajo(uint8_t numMotor,uint32_t cicloDeTrabajo){
    ledcWrite(numMotor, cicloDeTrabajo);
}

