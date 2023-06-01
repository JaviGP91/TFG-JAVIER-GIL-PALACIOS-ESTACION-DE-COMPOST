/*LIBRERIAS*/
#include "AUTOpairing.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include "funciones.h" // fichero del proyecto

/*PINES*/
const int Power_s_temp = 12; // Pin GPIO para alimentar los sensores de temperatura
const int Power_s_hum = 13; // Pin GPIO para alimentar sensor de humedad



/*VARIABLES*/
float S1_T=0, S2_T=0, S3_T=0;
float temp[3];

//-----------------------------------------------------------

void procesa_mensajes (String topic, String payload)
{
  Serial.println("Mensaje recibido...");
  Serial.print("Topic: ");
  Serial.println(topic);
  Serial.print("Payload: ");
  Serial.println(payload);
}


void setup() {
// Iniciar alimentación de sensores
pinMode(Power_s_temp, OUTPUT);
pinMode(Power_s_hum, OUTPUT);
digitalWrite(Power_s_temp, HIGH);
digitalWrite(Power_s_hum, HIGH);

// Iniciar puerto serie
  Serial.begin(115200);

  Serial.println();
  Serial.println("SETUP...");
  adquisicion_direcciones_temp();

  AUTOpairing::set_channel(6);  // canal donde empieza el scaneo
  AUTOpairing::set_timeOut(5000,true); // tiempo máximo en ms 
  AUTOpairing::set_debug(true);   // depuración, inicializar Serial antes
  AUTOpairing::set_deepSleep(5);  //configura la duración de la suspensión profunda en la clase AUTOpairing en 10 segundos. Esto significa que el dispositivo se mantendrá en modo de bajo consumo durante ese período de tiempo antes de reanudar la ejecución normal.
  AUTOpairing::set_callback(procesa_mensajes);  
  
  AUTOpairing::begin();
}

void loop() {
digitalWrite(Power_s_temp, HIGH);
digitalWrite(Power_s_hum, HIGH);
AUTOpairing::mantener_conexion();
  
  if (AUTOpairing::envio_disponible()) { 
      char mensaje[256];
      
      pedir_temperaturas(temp);
      float humedad=pedir_humedad();
      sprintf(mensaje, "{\"topic\":\"datos\",\"T1\":%4.2f,\"T2\":%4.2f,\"T3\":%4.2f, \"hum\":%4.2f }", temp[0], temp[1], temp[2], humedad);
      AUTOpairing::espnow_send_check(mensaje); // hará deepsleep por defecto

  }
    
      //digitalWrite(Power_s_temp, LOW);
      //digitalWrite(Power_s_hum, LOW);
}
