#ifndef ControlDeMultiparrilla_h
#define ControlDeMultiparrilla_h
#include <Arduino.h>
#include <WiFi.h>
#include "ESPNowW.h"
#include "ControlDeMotores.h"

#define CANTIDAD_DE_PLAZAS 6
#define CANTIDAD_MAXIMA_DE_RUTINAS 20

extern bool comenzar_prueba;

extern uint8_t mac_multiparrilla[];
extern uint8_t mac_HMI[];

struct Multiparrilla {
    bool disponible_para_recibir_rutinas;
    bool sensor_infrarrojo; 
    bool plazas_activadas[CANTIDAD_DE_PLAZAS];
    uint16_t setpoints_temperatura[CANTIDAD_MAXIMA_DE_RUTINAS*2];
    char tipo_de_funcion_de_temperatura[CANTIDAD_MAXIMA_DE_RUTINAS];
    uint16_t setpoints_agitacion[CANTIDAD_MAXIMA_DE_RUTINAS]; 
    uint32_t minutos_para_mantener_setpoints[CANTIDAD_MAXIMA_DE_RUTINAS];
    uint8_t numero_de_rutinas_configuradas;
};


extern Multiparrilla multiparrilla;

void recibir_mensaje(const uint8_t *direccion_mac, const uint8_t *mensaje, int longitud_del_mensaje);

void inicializar_control_de_multiparrilla();

void funcion_de_prueba();


    


/*
//////////////////////////////////////////////RECEPTOR///////////////////////////////////////////////////////////////////////////
#include <Arduino.h>
#ifdef ESP8266
#include <ESP8266WiFi.h>
#elif ESP32
#include <WiFi.h>
#endif
#include "ESPNowW.h"

uint8_t mac_multiparrilla[] = {0x40, 0x91, 0x51, 0xAB , 0x1B, 0xC0};
uint8_t mac_HMI[] = {0x0C, 0xB8, 0x15, 0xC1, 0x9A, 0xD4};

void confirmarRecepcion(){
    uint8_t recibido[] = {'R','E', 'C','I','B','I','D','O'};
    ESPNow.send_message(mac_HMI, recibido, sizeof(recibido));
}

void onRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
    Serial.print("Last Packet Recv Data: ");
    // if it could be a string, print as one
    if (data[data_len - 1] == 0)
        Serial.printf("%c\n", data);
    // additionally print as hex
    for (int i = 0; i < data_len; i++) {
        Serial.printf("%c", data[i]);
    }
    Serial.println(" ");
    confirmarRecepcion();
}

void setup() {
    Serial.begin(115200);
    Serial.println("ESPNow receiver Demo");
    WiFi.mode(WIFI_MODE_STA);
    ESPNow.set_mac(mac_multiparrilla);
    WiFi.disconnect();
    ESPNow.init();
    ESPNow.reg_recv_cb(onRecv);
    ESPNow.add_peer(mac_HMI);
}

void loop() {}
*/
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#endif