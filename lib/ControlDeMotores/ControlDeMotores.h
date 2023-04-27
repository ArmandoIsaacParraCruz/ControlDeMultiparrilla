#ifndef ControlDeMotores_h
#define ControlDeMotores_h
#include <Arduino.h>
#include <PID_v1.h>
#include <driver/ledc.h>
#include <esp_timer.h>



#define CANT_MOTORES 6
#define TIEMPO_ANTIRREBOTE_SENSOR_HALL  500


enum{motor1,motor2,motor3,motor4,motor5,motor6};


const uint8_t pinMotores[CANT_MOTORES] = {5,18,19,21,3,1};
const uint8_t pinSensoresHall[CANT_MOTORES] = {26,34,35,32,33,25};
const double constantesKp[CANT_MOTORES] = {0.1,0.1,0.1,0.1,0.1,0.1};
const double constanteski[CANT_MOTORES] = {0.02,0.02,0.02,0.02,0.02,0.02};
const double constantesKd[CANT_MOTORES] = {0,0,0,0,0,0};


struct Motor{
    bool activado;
    double rpm;
    double setPointVelocidad;
    volatile uint32_t revoluciones;
    uint16_t tiempoAnterior;
};

extern struct Motor motores[6];
extern PID PIDsParaMotores[CANT_MOTORES];

void configurarMotores(struct Motor*);

void IRAM_ATTR interrupcionSensorHall1();
void IRAM_ATTR interrupcionSensorHall2();
void IRAM_ATTR interrupcionSensorHall3();
void IRAM_ATTR interrupcionSensorHall4();
void IRAM_ATTR interrupcionSensorHall5();
void IRAM_ATTR interrupcionSensorHall6();


extern void (*interrupcionesSensorHall[CANT_MOTORES])();

void agregaUnaVuelta(uint8_t);

bool obtenerEstadoMotor(uint8_t);
double obtenerVelocidad(uint8_t);
double obtenerSetPointVelocidad(uint8_t);
double obtenerRevoluciones(uint8_t);
double obtenerTiempoAnterior(uint8_t);

#endif