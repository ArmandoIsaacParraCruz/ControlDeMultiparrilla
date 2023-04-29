#include "ControlDeMotores.h"

struct Motor motores[CANT_MOTORES];
//EL SIGUIENTE BLOQUE DE CÓDIGO 
PID PIDsParaMotores[CANT_MOTORES] = {
    PID(&motores[motor1].rpmFiltrado,&motores[motor1].cicloDeTrabajo, &motores[motor1].setpointActual, constantesKp[motor1], constanteski[motor1], constantesKd[motor1], DIRECT),
    PID(&motores[motor2].rpmFiltrado,&motores[motor2].cicloDeTrabajo, &motores[motor2].setpointActual, constantesKp[motor2], constanteski[motor2], constantesKd[motor2], DIRECT),
    PID(&motores[motor3].rpmFiltrado,&motores[motor3].cicloDeTrabajo, &motores[motor3].setpointActual, constantesKp[motor3], constanteski[motor3], constantesKd[motor3], DIRECT),
    PID(&motores[motor4].rpmFiltrado,&motores[motor4].cicloDeTrabajo, &motores[motor4].setpointActual, constantesKp[motor4], constanteski[motor4], constantesKd[motor4], DIRECT),
    PID(&motores[motor5].rpmFiltrado,&motores[motor5].cicloDeTrabajo, &motores[motor5].setpointActual, constantesKp[motor5], constanteski[motor5], constantesKd[motor5], DIRECT),
    PID(&motores[motor6].rpmFiltrado,&motores[motor6].cicloDeTrabajo, &motores[motor6].setpointActual, constantesKp[motor6], constanteski[motor6], constantesKd[motor6], DIRECT)
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

void activaPWMDelMotorI(uint8_t i)
{
    pinMode(pinMotores[i], OUTPUT);
    ledcSetup(i, FRECUENCIA_PWM, RESOLUCION_PWM);
    ledcAttachPin(pinMotores[i], i);
}

void activaInterrupcionExternaDelMotorI(uint8_t i)
{
    pinMode(pinSensoresHall[i], INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(pinSensoresHall[i]), interrupcionesSensorHall[i], RISING);
}

void inicializaValoresDelMotorI(uint8_t i){
    motores[i].revoluciones = VALOR_INICIAL;
    motores[i].rpm = VALOR_INICIAL;
    motores[i].rpmFiltrado = VALOR_INICIAL;
    motores[i].setpointActual = motores[i].setpoints.front();
    motores[i].setpoints.pop();
    motores[i].minutosParaMantenerSetpointActual = motores[i].minutosParaMantenerSetpoints.front();
    motores[i].minutosParaMantenerSetpoints.pop();
    motores[i].tiempoAnterior = micros();
}

void activaPIDDelMotorI(uint8_t i)
{
    PIDsParaMotores[i].SetMode(AUTOMATIC);
    PIDsParaMotores[i].SetOutputLimits(VALOR_INICIAL, VALOR_LIMITE_PWM);
}

void activaTimerParaActualizarLosPIDs(){
    esp_timer_handle_t temporizadorPIDs;
    esp_timer_create_args_t argumentosTemporizadorPIDs = {
        .callback = &actualizaPIDs, 
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK
    };
    esp_timer_create(&argumentosTemporizadorPIDs, &temporizadorPIDs);
    esp_timer_start_periodic(temporizadorPIDs, PERIODO_DE_MUESTREO);
}

void activaTimerParaActualizarLosSetpoints()
{
    esp_timer_handle_t temporizadorSetpoint;
    esp_timer_create_args_t argumentosTemporizadorSetpoint = {
        .callback = &actualizaSetPoints,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK
    };
    esp_timer_create(&argumentosTemporizadorSetpoint, &temporizadorSetpoint);
    esp_timer_start_periodic(temporizadorSetpoint, TIEMPO_PARA_ACTUALIZAR_SETPOINT);
}

void configurarMotores(struct Motor *configDePrueba)
{
    bool realizaUnaVez = false;
    for(uint8_t i = 0; i < CANT_MOTORES; i++){
        motores[i].activado = configDePrueba[i].activado;
        if(motores[i].activado){
            
            activaPWMDelMotorI(i);
            activaInterrupcionExternaDelMotorI(i);
            
            while(!configDePrueba[i].setpoints.empty() && !configDePrueba[i].minutosParaMantenerSetpoints.empty()){
                motores[i].setpoints.push(configDePrueba[i].setpoints.front());
                configDePrueba[i].setpoints.pop();
                motores[i].minutosParaMantenerSetpoints.push(configDePrueba[i].minutosParaMantenerSetpoints.front());
                configDePrueba[i].minutosParaMantenerSetpoints.pop();
            }

            inicializaValoresDelMotorI(i);

            activaPIDDelMotorI(i);

            if(!realizaUnaVez){
                activaTimerParaActualizarLosPIDs();
                activaTimerParaActualizarLosSetpoints();
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
    return motores[numMotor].rpmFiltrado;
}
double obtenerSetPointVelocidad(uint8_t numMotor){
    return motores[numMotor].setpointActual;
}



void asignaCicloDeTrabajo(uint8_t numMotor,double cicloDeTrabajo){
    if(motores[numMotor].activado){
        ledcWrite(numMotor, cicloDeTrabajo);
    } 
}

void IRAM_ATTR actualizaPIDs(void *arg){
    for(uint8_t i = 0; i < CANT_MOTORES; i++){
        if(motores[i].activado){
            calculaRPM(i);
            PIDsParaMotores[i].Compute();
            ledcWrite(i, motores[i].cicloDeTrabajo);
        }   
    }
}

void calculaRPM(uint8_t numMotor){
    motores[numMotor].rpm = motores[numMotor].revoluciones * RPM_EN_100_MILISEGUNDOS;
    motores[numMotor].rpmFiltrado = alpha * motores[numMotor].rpm + (1.0 - alpha) * motores[numMotor].rpmFiltrado;
    motores[numMotor].revoluciones = VALOR_INICIAL;
}

void IRAM_ATTR actualizaSetPoints(void *arg)
{
    for(uint8_t i = 0; i < CANT_MOTORES; i++){
        if(motores[i].activado){
             //Serial.println(motores[i].minutosParaMantenerSetpoints.size());
          //double minutosTranscurridos = millis()/(1000.0*60.0); 
            double minutosTranscurridos = millis()/(1000.0*10.0); 
            if(minutosTranscurridos >= motores[i].minutosParaMantenerSetpointActual){
                if(!motores[i].minutosParaMantenerSetpoints.empty() && !motores[i].setpoints.empty()){
                    motores[i].minutosParaMantenerSetpointActual = motores[i].minutosParaMantenerSetpoints.front();
                    motores[i].minutosParaMantenerSetpoints.pop();
                    motores[i].setpointActual = motores[i].setpoints.front();
                    motores[i].setpoints.pop();
                }else{
                    PIDsParaMotores[i].SetMode(MANUAL);
                    motores[i].cicloDeTrabajo = VALOR_INICIAL;
                    asignaCicloDeTrabajo(i,motores[i].cicloDeTrabajo);
                    motores[i].setpointActual = VALOR_INICIAL;
                    motores[i].rpmFiltrado = VALOR_INICIAL;
                    motores[i].activado = false;
                }
            }
        }
    }
}
