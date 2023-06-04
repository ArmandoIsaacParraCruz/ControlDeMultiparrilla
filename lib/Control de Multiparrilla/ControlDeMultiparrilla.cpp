#include <ControlDeMultiparrilla.h>
uint8_t mac_multiparrilla[] = {0x40, 0x91, 0x51, 0xAB , 0x1B, 0xC0};
uint8_t mac_HMI[] = {0x0C, 0xB8, 0x15, 0xC1, 0x9A, 0xD4};

Multiparrilla multiparrilla;
struct configuracionParaMotor rutina_de_prueba;
bool comenzar_prueba;

void inicializar_control_de_multiparrilla()
{
    Serial.begin(115200);
    Serial.println("ESPNow receiver Demo");
    WiFi.mode(WIFI_MODE_STA);
    ESPNow.set_mac(mac_multiparrilla);
    WiFi.disconnect();
    ESPNow.init();
    ESPNow.reg_recv_cb(recibir_mensaje);
    ESPNow.add_peer(mac_HMI);
}



void recibir_mensaje(const uint8_t *direccion_mac, const uint8_t *mensaje, int longitud_del_mensaje)
{
	Serial.println(longitud_del_mensaje);
    if(longitud_del_mensaje == sizeof(uint32_t)){
       		Serial.println("Mensaje de prueba");
    } else {
		memcpy(&multiparrilla, mensaje, sizeof(Multiparrilla));
        /*Serial.print("Sensor infrarrojo: ");
    	if(multiparrilla.sensor_infrarrojo) {
        	Serial.println("Ingrarrojo");
    	} else {
        	Serial.println("Termopares");
    	}
    	Serial.println("Plazas_activadas:");
    	for(uint8_t i = 0; i < CANTIDAD_DE_PLAZAS; ++i) {
        	Serial.print(i+1);Serial.print(": ");Serial.println(multiparrilla.plazas_activadas[i]);
    	}

    	Serial.print("Rutinas configuradas: "); Serial.println(multiparrilla.numero_de_rutinas_configuradas);

    	Serial.println(" ");
    	Serial.println("Setpoints de temperatura:");
    	for(uint8_t i = 0; i < CANTIDAD_MAXIMA_DE_RUTINAS; ++i) {
        	if(multiparrilla.setpoints_temperatura[i * 2] == 0) {
         	   Serial.print("Desactivado");
        	} else {
            	Serial.print(multiparrilla.setpoints_temperatura[i]);
        	}
        	Serial.print("  ");
        	if(multiparrilla.setpoints_temperatura[i * 2 + 1] == 0) {
            	Serial.println("Desactivado");
       		} else {
        	    Serial.println(multiparrilla.setpoints_temperatura[i * 2 + 1]);
       	 }
        
    	}

    	Serial.println(" ");
    	Serial.println("Funciones de temperatura:");
    	for(uint8_t i = 0; i < CANTIDAD_MAXIMA_DE_RUTINAS; ++i) {
        	Serial.println(multiparrilla.tipo_de_funcion_de_temperatura[i]);
    	}

    	Serial.println(" ");
    	Serial.println("Setpoints de agitación");
    	for(uint8_t i = 0; i < CANTIDAD_MAXIMA_DE_RUTINAS; ++i) {
			if(multiparrilla.setpoints_agitacion[i] == 0) {
				Serial.println("DESACTIVADO");
			} else {
				Serial.println(multiparrilla.setpoints_agitacion[i]);
			}
    	}

    	Serial.println(" ");
    	Serial.println("Minutos para mantener setpoints");
    	for(uint8_t i = 0; i < CANTIDAD_MAXIMA_DE_RUTINAS; ++i) {
        	Serial.println(multiparrilla.minutos_para_mantener_setpoints[i]);
   		}*/
		comenzar_prueba = true;
    }
    
}

void funcion_de_prueba()
{
	uint8_t i = 0;
	rutina_de_prueba.activado = true;
	while(i <= multiparrilla.numero_de_rutinas_configuradas && i < CANTIDAD_MAXIMA_DE_RUTINAS) {
		rutina_de_prueba.setpoints.push(multiparrilla.setpoints_agitacion[i]);
  		rutina_de_prueba.minutosParaMantenerSetpoints.push(multiparrilla.minutos_para_mantener_setpoints[i]);
		i++;
	}
	struct configuracionParaMotor apagado;
	apagado.activado = false;
  	struct configuracionParaMotor configDePrueba[6] = {rutina_de_prueba,apagado,apagado,apagado,apagado,apagado};
 	configurarMotores(configDePrueba);
}





