/*LIBRERIAS*/
#include "AUTOpairing.h"
#include <ArduinoJson.h>
#include <ESP8266httpUpdate.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266WiFi.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "funciones.h" // fichero del proyecto



/*PINES*/
const int Power_s_temp = 12; // Pin GPIO para alimentar los sensores de temperatura
const int Power_s_hum = 13; // Pin GPIO para alimentar sensor de humedad

/*VARIABLES*/
float S1_T=0, S2_T=0, S3_T=0;
float temp[3];
float humedad;

// Objeto de la clase AUTOpairing_t para la comunicación por ESP-NOW.
AUTOpairing_t clienteAP; 

// Estructura utilizada para almacenar la configuración del programa, (tiempo de sueño profundo y el tiempo de espera de conexión).
struct configuracion     
{
  uint32_t sleep;
  uint32_t timeout;
} mi_configuracion;

//--------------------- ACTUALIZACION FOTA ----------------------

//const char* ssid = "huerticawifi";
//const char* password = "4cc3sshu3rt1c4";
const char* ssid = "sagemcom67E0_EXT";
const char* password = "UWZKFTHRWZZTYY";
#define OTA_URL    "https://huertociencias.uma.es/esp8266-ota-update"
#define HTTP_OTA_VERSION   String(__FILE__).substring(String(__FILE__).lastIndexOf('\\')+1) + ".d1_mini_pro"

// IDE > 2.0 Mac
//#define HTTP_OTA_VERSION   String(__FILE__).substring(String(__FILE__).lastIndexOf('/')+1)

WiFiClientSecure WClient;
    
void procesa_mensajes (String topic, String payload)
{
  Serial.println("Mensaje recibido...");
  Serial.print("Topic: ");
  Serial.println(topic);
  Serial.print("Payload: ");
  Serial.println(payload);
  if(topic=="fota")
  { 
    delay(100);
    Serial.println();
    Serial.print("Conectando a WiFi con SSID: ");
    Serial.println(ssid);
    //wifi_promiscuous_enable(0);
    WiFi.disconnect();
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    unsigned long ahora= millis();
    while (WiFi.status() != WL_CONNECTED && millis()-ahora<15000) {
      delay(100);
      Serial.print(".");
    }
    if(WiFi.status() != WL_CONNECTED)
    {
      Serial.println("WiFi NO conectado");
      clienteAP.gotoSleep();
    }
    Serial.println("");
    Serial.println("WiFi conectado");
    Serial.println("Direccion IP: ");
    Serial.println(WiFi.localIP());
    WClient.setTimeout(12); // timeout argument is defined in seconds for setTimeout
    WClient.setInsecure();  // no chequea certificado...
    Serial.println( "Intentando la actualización FOTA..." );
    Serial.println(OTA_URL);
    Serial.println(HTTP_OTA_VERSION);
    
    switch(ESPhttpUpdate.update(WClient,OTA_URL, HTTP_OTA_VERSION)) {
    case HTTP_UPDATE_FAILED:
      Serial.printf("Error en actualización HTTP: Error #(%d): %s\n", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
      break;
    case HTTP_UPDATE_NO_UPDATES:
      Serial.println(F("No hay nueva actualiación"));
      break;
    case HTTP_UPDATE_OK:
      Serial.println(F("Actualiación OK"));
      break;
    }
    clienteAP.gotoSleep();
  } // end FOTA
  
  if(topic=="config") // el mesaje debe tener topic = config y payload = {"sleep": # , "timeout": # }
  {
    StaticJsonDocument<512> doc; // el tamaño tiene que ser adecuado para el mensaje
    // Deserialize the JSON document
    DeserializationError error = deserializeJson(doc, payload.c_str(),payload.length());

    // Compruebo si no hubo error
    if (error) {
      Serial.print("Error fallo en deserializeJson(): ");
      Serial.println(error.c_str());
    }
    else
    if(doc.containsKey("sleep") && doc.containsKey("timeout") )  // comprobar si existe el campo/clave que estamos buscando
    { 
     int valor = doc["sleep"];
     Serial.print("JSON sleep = ");
     Serial.println(valor);
     mi_configuracion.sleep=valor;
     valor = doc["timeout"];
     Serial.print("JSON timeout = ");
     Serial.println(valor);
     mi_configuracion.timeout=valor;
     clienteAP.set_config((uint8_t*)&mi_configuracion);
    }
    else
    {
      Serial.println("ERROR: claves \"sleep\" & \"timeout\" no aparecen en JSON");
    }
  } // end config
}

//-------------------------------------------------------------------
//-----------------------   SETUP   ---------------------------------
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
  

  clienteAP.init_config_size(sizeof(mi_configuracion));
  
  if (clienteAP.get_config((uint8_t*)&mi_configuracion)==false)
  { // si no hay configuración guardada la pongo por defecto
    mi_configuracion.sleep=10;
    mi_configuracion.timeout=3000;
  }
  Serial.printf(" > DeepSleep : %d\n",mi_configuracion.sleep );
  Serial.printf(" > TimeOut   : %d\n",mi_configuracion.timeout );
  clienteAP.set_timeOut(mi_configuracion.timeout,true); // tiempo máximo
  clienteAP.set_deepSleep(mi_configuracion.sleep);  //tiempo dormido en segundos
  clienteAP.set_channel(6);  // canal donde empieza el scaneo
  clienteAP.set_debug(true);   // depuración, inicializar Serial antes
  clienteAP.set_callback(procesa_mensajes);  //por defecto a NULL -> no se llama a ninguna función
  
  clienteAP.begin();
  adquisicion_direcciones_temp();
}

//-------------------------------------------------------------------
//-----------------------   LOOP    ---------------------------------
void loop() {
  //digitalWrite(Power_s_temp, HIGH);
  //digitalWrite(Power_s_hum, HIGH);
  clienteAP.mantener_conexion();
  
  if (clienteAP.envio_disponible()) { 
      char mensaje[256];
      
      pedir_temperaturas(temp);
      digitalWrite(Power_s_temp, LOW); // apagamos los sensores de temperatura
      humedad=pedir_humedad();
      digitalWrite(Power_s_hum, LOW);  // apagamos sensor de humedad
      sprintf(mensaje, "{\"topic\":\"datos\",\"T1\":%4.2f,\"T2\":%4.2f,\"T3\":%4.2f, \"hum\":%4.2f }", temp[0], temp[1], temp[2], humedad);
      clienteAP.espnow_send_check(mensaje); // hará deepsleep por defecto
  }
    
  //digitalWrite(Power_s_temp, LOW);
  //digitalWrite(Power_s_hum, LOW);
}
