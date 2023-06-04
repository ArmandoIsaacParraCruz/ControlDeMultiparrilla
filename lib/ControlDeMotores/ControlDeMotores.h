//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
UNIVERSIDAD AUÓNOMA METROPOLITANA - AZCAPOTZALCO
Alumno: Armando Isaac Parra Cruz
Matrícula: 2193002552
Descripción: El siguiente código se encarga controlar la velocidad 6 motores de una multiparrilla agitadora de 6 plazas en función del tiempo, permitiendo
elegir la velocidad de cada motor de forma independiente en cada minuto.
En el código se implementaron 6 PIDs de la librería "PID_v1.h" para el control de cada uno de los motores.
Para la detectección del giro de los motores se utilizaron sensores hall digitales que mandan un pulso por cada revolución de los motores
e interrupciones externas. Se agregó un filtro pasa bajo media móvil exponencial y un filtro antirrebote para eliminar el ruido en
la obtención de las revoluciones por minuto de cada motor. También se agregaron sentencias condicionales para evitar en la medida de lo posible 
cambios abruptos en las velocidades de los motores.Por último, se añadieron timers para actualizar el valor de las rpm y las salidas de los 
PIDs de forma periódica. 
*/
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////DECLARACIÓN E INCLUSIÓN DE LIBRERIAS////////////////////////////////////////////////////////////////////////////////////////////////////////
//En este apartado se define la librería "ControlDeMotores.h" que es desarrollado a lo largo del código, 
//y se incluyen las dependencias necesarias para su funcionamiento.
#ifndef ControlDeMotores_h
#define ControlDeMotores_h 
#include <Arduino.h>
#include <queue>
#include <PID_v1.h>
#include <driver/ledc.h>
#include <esp_timer.h>
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////CREACIÓN DE MACROS PARA DEFINIR CONSTANTES EN EL PROGRAMA/////////////////////////////////////////////////////////////////////////////////////////
#define CANT_MOTORES   6                            //Es la cantidad de motores que dispone la multiparrilla.
#define TIEMPO_ANTIRREBOTE_SENSOR_HALL 500          //Es el tiempo mínimo que debe de pasar para evitar detectar una falsa interrupción externa por el sensor hall.
#define FRECUENCIA_PWM 15000                        //Es la frecuencia PWM para controlar los motores = 15 kHz.
#define RESOLUCION_PWM 8                            //Es el rango de valores para definir la frecuencia PWM, de 0 a 255.
#define TIEMPO_PARA_ACTUALIZAR_PIDs 100000          //Cada 100 milisegundos se activa una interrupcion por software que actualizará las salidas de los controladores PIDs.
#define TIEMPO_PARA_ACTUALIZAR_SETPOINT 60000000    //Cada minuto se verificará si se tiene que actualizar el valor del setpoint de acuerdo a lo configurado por el usuario.
#define VALOR_INICIAL 0                             //Valor para inicializar o resetear variables.
#define VALOR_LIMITE_PWM 255                        //El valor límite por una resolución de 8 bits es de 255.
#define RPM_EN_100_MILISEGUNDOS 600                 //Valor para convertir revoluciones por 100 milisegundos a revoluciones por minuto.
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//El motor 1 tendrá un valor de 0, el motor 2 un valor de 1, y así sucesivamente.
enum{motor1,motor2,motor3,motor4,motor5,motor6};

///////////////////////////////////////DECLARACIÓN DE LOS PINES Y CONSTANTES PID DE CADA MOTOR////////////////////////////////////////////////////////////////////////////////////////////////
const double alpha  = 0.01;                                                 //Constante de suavizado para el filtro pasa bajo media móvil exponencial.
const uint8_t pinMotores[CANT_MOTORES] = {5,18,19,21,3,1};                  //Pines para las señales PWM de cada motor. 
                                                                            //El pin 5 para el motor 1, el pin 18 para el motor 2, y así respectivamente
const uint8_t pinSensoresHall[CANT_MOTORES] = {26,34,35,32,33,25};          //Pines para las interrupciones externas y detección del giro de cada motor.
                                                                            //El pin 26 para el motor 1, el pin 34 para el motor 2, etc. 
const double constantesKp[CANT_MOTORES] = {0.1,0.1,0.1,0.1,0.1,0.1};        //Constantes kp para el PID de cada motor en orden {kp para el motor 1,kp para el motor 2, etc}.
const double constanteski[CANT_MOTORES] = {0.01,0.02,0.02,0.02,0.02,0.02};  //Constantes ki para el PID de cada motor.
const double constantesKd[CANT_MOTORES] = {0.01,0,0,0,0,0};                 //Constantes kd para el PID de cada motor.
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////ESTRUCTURA PARA GUARDAR LA INFORMACIÓN RELEVANTE DE CADA MOTOR////////////////////////////////////////////////////////////////////////////////////////
struct Motor{
    bool activado;                                      //Indica si el motor estará en funcionamiento durante la operación de la multiparrilla.
    double rpm;                                         //Guarda el valor de las rpm del motor.
    double rpmFiltrado;                                 //Guarda el valor de las rpm del motor una vez que ya hayan pasado por el filtro pasa bajo media móvil exponencial.
    double setpointActual;                              //Almacena el valor del setpoint.
    double setpointAnterior;                            //Almacena el valor del setpoint anterior.
    double minutosParaMantenerSetpointActual;           //Guarda los minutos que durará el setpoin actual.
    volatile uint32_t revoluciones;                     //Almacena las revoluciones que ha dado el motor.
    uint16_t tiempoAnterior;                            //Almacena el tiempo en el que se registró la última revolución válida.
    double cicloDeTrabajoActual;                        //Guarda el ciclo de trabajo con el que el motor está funcionando en ese instante.
    double cicloDeTrabajoAnterior;                      //Guarda el ciclo de trabajo con el que el motor funcionaba antes de cambiar el setpoint.
    std::queue<double>minutosParaMantenerSetpoints;     //Guarda la cantidad de minutos en el que debe de durar el setpoint.
    std::queue<double>setpoints;                        //Guarda los setpoints que se programaron para la operación de la parrilla.
};
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////ESTRUCTURA PARA LA CONFIGURACIÓN DE CADA MOTOR///////////////////////////////////////////////////////////////////////////////////////////
struct configuracionParaMotor{  
    bool activado;                                      //Activa o desactiva un motor al inicio de la configuración de los motores.
    std::queue<double>minutosParaMantenerSetpoints;     //Almacena los minutos de duración de un setpoint que seguirá el PID del motor a configurar.
    std::queue<double>setpoints;                        //Almacena los setpoints que seguirá el PID del motor a configurar
};
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////SE DECLARA UN ARREGLO DE ESTRUCTURAS DE 6 MOTORES Y 6 PIDs///////////////////////////////////////////////////////////////////////////////////////
extern struct Motor motores[6];
extern PID PIDsParaMotores[6];
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////FUNCIONES PARA LA CONFIGURACIÓN DE LOS MOTORES////////////////////////////////////////////////////////////////////////////////////////
//Cada una de estas funciones activan o inicializan funciones necesarias para el control de cada motor. Las primeras 4 funciones reciben como parámetro el número del i-ésimo motor ya que
//se les debe configurar aspectos específicos para controlarlo (como el pin de salida PWM), en cambio la penúltima y la antepenúltima funcion son necesarias para todos los motores.
void activaPWMDelMotorI(uint8_t);
void activaInterrupcionExternaDelMotorI(uint8_t);
void inicializaValoresDelMotorI(uint8_t);
void activaPIDDelMotorI(uint8_t);
void activaTimerParaActualizarLosPIDs();
void activaTimerParaActualizarLosSetpoints();
void configurarMotores(struct configuracionParaMotor*); //Esta función está encargada de la configuración de todos los motores haciendo uso de las funciones anteriores.
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////FUNCIONES LLAMADAS AL ACTIVARSE LAS INTERRUPCIONES EXTERNAS//////////////////////////////////////////////////////////////////////////////
//Alguna de estas funciones son llamadas dependiendo del sensor hall que ha activado.
void IRAM_ATTR interrupcionSensorHall1();
void IRAM_ATTR interrupcionSensorHall2();
void IRAM_ATTR interrupcionSensorHall3();
void IRAM_ATTR interrupcionSensorHall4();
void IRAM_ATTR interrupcionSensorHall5();
void IRAM_ATTR interrupcionSensorHall6();

extern void (*interrupcionesSensorHall[CANT_MOTORES])();//Se declara un arreglo de funciones para la configuración de los motores.
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////FUNCIONES EN COMÚN PARA CADA MOTOR EN FUNCIONAMIENTO//////////////////////////////////////////////////////////////////////////////////
void agregaUnaVuelta(uint8_t);                      //Cuando se ha detectado una vuelta de cierto motor se llama a esta función para aumentar en una aunidad el número 
                                                    //de vueltas que ha dado un motor.
bool obtenerEstadoMotor(uint8_t);                   //Devuelve el estado del i-ésimo motor: true para activado y false para desactivado.
double obtenerVelocidad(uint8_t);                   //Devuelve los rpmFiltrado de i-ésimo motor.
double obtenerSetPointVelocidad(uint8_t);           //Devuelve el setpoint que se está en el PID del i-ésimo motor en el momento en el que se llama a la función.
void asignaCicloDeTrabajo(uint8_t,double);          //Asigna el ciclo de trabajo para el i-ésimo motor.
void IRAM_ATTR actualizaPIDs(void *arg);            //Esta funcion es llamada cada 100 milisegundos por timer para actualizar la salida de los PIDs.
void calculaRPM(uint8_t);                           //Obtiene el valor de rpmFiltrado del i-ésimo motor.
void IRAM_ATTR actualizaSetPoints(void *arg);       //Esta función es llamada cada minuto para actualizar el valor del setpoint según definió el usuario.
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#endif
