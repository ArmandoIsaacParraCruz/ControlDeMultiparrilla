#include "ControlDeMotores.h"

struct Motor motores[CANT_MOTORES];

PID PIDsParaMotores[CANT_MOTORES] = {
    PID(&motores[motor1].rpmFiltrado,&motores[motor1].cicloDeTrabajo, &motores[motor1].setPointVelocidad, constantesKp[motor1], constanteski[motor1], constantesKd[motor1], DIRECT),
    PID(&motores[motor2].rpmFiltrado,&motores[motor2].cicloDeTrabajo, &motores[motor2].setPointVelocidad, constantesKp[motor2], constanteski[motor2], constantesKd[motor2], DIRECT),
    PID(&motores[motor3].rpmFiltrado,&motores[motor3].cicloDeTrabajo, &motores[motor3].setPointVelocidad, constantesKp[motor3], constanteski[motor3], constantesKd[motor3], DIRECT),
    PID(&motores[motor4].rpmFiltrado,&motores[motor4].cicloDeTrabajo, &motores[motor4].setPointVelocidad, constantesKp[motor4], constanteski[motor4], constantesKd[motor4], DIRECT),
    PID(&motores[motor5].rpmFiltrado,&motores[motor5].cicloDeTrabajo, &motores[motor5].setPointVelocidad, constantesKp[motor5], constanteski[motor5], constantesKd[motor5], DIRECT),
    PID(&motores[motor6].rpmFiltrado,&motores[motor6].cicloDeTrabajo, &motores[motor6].setPointVelocidad, constantesKp[motor6], constanteski[motor6], constantesKd[motor6], DIRECT)
};

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

void (*interrupcionesSensorHall[CANT_MOTORES])() = {
    interrupcionSensorHall1,
    interrupcionSensorHall2,
    interrupcionSensorHall3,
    interrupcionSensorHall4,
    interrupcionSensorHall5,
    interrupcionSensorHall6
};

void configurarMotores(struct Motor *configDePrueba)
{
    bool realizaUnaVez = false;
    for(uint8_t i = 0; i < CANT_MOTORES; i++){
        motores[i].activado = configDePrueba[i].activado;
        if(motores[i].activado){
            pinMode(pinMotores[i], OUTPUT);
            ledcSetup(i, FRECUENCIA_PWM, RESOLUCION_PWM);
            ledcAttachPin(pinMotores[i], i);

            pinMode(pinSensoresHall[i], INPUT_PULLUP);
            attachInterrupt(digitalPinToInterrupt(pinSensoresHall[i]), interrupcionesSensorHall[i], RISING);

            motores[i].revoluciones = VALOR_INICIAL;
            motores[i].rpm = VALOR_INICIAL;
            motores[i].rpmFiltrado = VALOR_INICIAL;
            motores[i].setPointVelocidad = configDePrueba[i].setPointVelocidad;
            motores[i].tiempoAnterior = micros();

            PIDsParaMotores[i].SetMode(AUTOMATIC);
            PIDsParaMotores[i].SetOutputLimits(VALOR_INICIAL, VALOR_LIMITE_PWM);


            if(!realizaUnaVez){
                esp_timer_handle_t temporizador;
                esp_timer_create_args_t argumentosTemporizador = {
                    .callback = &actualizaPIDs, 
                    .arg = NULL,
                    .dispatch_method = ESP_TIMER_TASK
                };
                esp_timer_create(&argumentosTemporizador, &temporizador);
                esp_timer_start_periodic(temporizador, PERIODO_DE_MUESTREO);
            }
        }
        else{
            PIDsParaMotores[i].SetMode(MANUAL);
        }
    }
}


void agregaUnaVuelta(uint8_t numMotor)
{
    if(motores[numMotor].activado){
        bool condicionAntirrebote = micros()-motores[numMotor].tiempoAnterior >= TIEMPO_ANTIRREBOTE_SENSOR_HALL;
        bool interrupcionConfirmada = digitalRead(pinSensoresHall[numMotor]);
        if(condicionAntirrebote && interrupcionConfirmada){
            motores[numMotor].revoluciones += 1;
            motores[numMotor].tiempoAnterior = micros();
        }
    }
}




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


void asignaCicloDeTrabajo(uint8_t numMotor,double cicloDeTrabajo){
    if(motores[numMotor].activado){
        ledcWrite(numMotor, cicloDeTrabajo);
    } 
}

void actualizaPIDs(void *arg){
    for(uint8_t i = 0; i < CANT_MOTORES; i++){
        if(motores[i].activado){
            calculaRPM(i);
            PIDsParaMotores[i].Compute();
            asignaCicloDeTrabajo(i, motores[i].cicloDeTrabajo);
        }   
    }
}

void calculaRPM(uint8_t numMotor){
    motores[numMotor].rpm = motores[numMotor].revoluciones * RPM_EN_100_MILISEGUNDOS;
    motores[numMotor].rpmFiltrado = alpha * motores[numMotor].rpm + (1.0 - alpha) * motores[numMotor].rpmFiltrado;
    motores[numMotor].revoluciones = VALOR_INICIAL;
}

