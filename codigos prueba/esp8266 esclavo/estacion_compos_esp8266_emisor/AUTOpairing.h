/*
 * Librería para comunicación ESPNOW<->MQTT a través de pasarela con auto-emparejamiento
 * Recuerda la configuración de emparejamiento usando FLASH o memoria RTC
 * 
 * 
 * Basado en el trabajo de:
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/?s=esp-now
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
  Based on JC Servaye example: https://https://github.com/Servayejc/esp8266_espnow

*/

#ifndef AUTOpairing_H
#define AUTOpairing_H

#include <ESP8266WiFi.h>
#include <espnow.h>
#include <EEPROM.h>
#include <string>
#include <queue> 

class TmensajeMQTT
{
  public:
  String topic;
  String payload;
  TmensajeMQTT ( String t, String p)
  { 
    topic=t;
    payload=p;
  }
};


// cola de mensajes mqtt recibidos
std::queue<TmensajeMQTT> cola_mensajes;

#include "Arduino.h"
#include "funciones.h" // fichero del proyecto
#include "AUTOpairing_common.h"

uint8_t broadcastAddressX[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};


#define MAGIC_CODE 0xA5A5A5A5
#define INVALID_CODE 0


class AUTOpairing
{
  
  enum PairingStatus {PAIR_REQUEST, PAIR_REQUESTED, PAIR_PAIRED, };
  
  struct struct_rtc
  {
    uint32_t code;
    struct_pairing data;
  } ;

  
  static int mensajes_sent;
  static PairingStatus pairingStatus;
  static struct_rtc rtcData;
  static struct_pairing pairingData;
  static bool mensaje_enviado; // para saber cuando hay que dejar de enviar porque ya se hizo y estamos esperando confirmación
  static bool terminar; // para saber cuando hay que dejar de enviar porque ya se hizo y estamos esperando confirmación
  static unsigned long previousMillis_scanChannel;    // will store last time channel was scanned 
  static unsigned long start_time;  // para controlar el tiempo de escaneo
  static bool esperando;  // esperando mensajes

  static void (*user_callback)(String, String); 

 //-----------------------------------------------------------
 static void printMAC(const uint8_t * mac_addr)
 {
   char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
    mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    if(debug) Serial.print(macStr);
 }

 // para modificar por el usuario usando funciones

  static unsigned long timeOut;
  static bool debug;
  static bool timeOutEnabled;
  static bool usar_FLASH;
  static uint8_t channel;  // canal para empezar a escanear
  static int segundos_en_deepSleep;   // tiempo dormido en segundos
  static int panAddress;
  
 public:
 //-----------------------------------------------------------
 static void set_pan(uint8_t _pan=1)
 {
    panAddress=_pan;
 }
 //-----------------------------------------------------------
 static int get_pan()
 {
    return panAddress;
 }
 //-----------------------------------------------------------
 static void set_timeOut(unsigned long _timeOut=3000, bool _enable=true)
 {
    timeOut=_timeOut;
    timeOutEnabled=_enable;
 }

 //-----------------------------------------------------------
 static void set_channel(uint8_t _channel=6)
 {
    channel=_channel;
 }

 //-----------------------------------------------------------
 static void set_FLASH(bool _usar_FLASH=false)
 {
    usar_FLASH=_usar_FLASH;
 }

 //-----------------------------------------------------------
 static void set_debug(bool _debug=true)
 {
    debug=_debug;
 }

 //-----------------------------------------------------------
 static void set_deepSleep(int _segundos_en_deepSleep=10)
 {
    segundos_en_deepSleep=_segundos_en_deepSleep;
 }

 //-----------------------------------------------------------
 static void begin()
  {
    previousMillis_scanChannel=0;
    start_time=millis();
   
    if(debug) Serial.println();

    //init check FLASH or RTC MEM usar_flsh = false para utilizar la memoria rtc
    // usamos rtc memory
    if(usar_FLASH)
    {
      EEPROM.begin(sizeof(rtcData));
      EEPROM.get(0, rtcData); 
      if(debug) Serial.print("Magic code on FLASH: ");
    }
    else 
    {
      ESP.rtcUserMemoryRead(0,(uint32_t *)&rtcData, sizeof(rtcData));
      if(debug) Serial.print("Código en RTC MEM: ");
    }
    if(debug) Serial.print(rtcData.code,HEX);
    if(debug) Serial.print(" - esperando: ");
    if(debug) Serial.println((unsigned long)MAGIC_CODE,HEX);
    
    if(rtcData.code==MAGIC_CODE)
    {
      // Verifica si el código en rtcData es igual a MAGIC_CODE
      // Copia los datos desde rtcData.data a la variable pairingData
      memcpy(&pairingData, &(rtcData.data), sizeof(pairingData));
      // Imprime un mensaje indicando que el emparejamiento se ha realizado desde la memoria RTC del usuario
      if(debug) Serial.print("emparejamiento realizado desde la memoria RTC del usuario ");
      // Imprime la dirección MAC almacenada en la variable pairingData.macAddr
      printMAC(pairingData.macAddr);
      // Imprime el canal utilizado por el servidor
      if(debug) Serial.print(" en el canal " );
      if(debug) Serial.print(pairingData.channel);    // channel used by the server
      // Imprime el tiempo transcurrido desde el inicio del proceso de emparejamiento
      if(debug) Serial.print(" en ");
      if(debug) Serial.print(millis()-start_time);
      if(debug) Serial.println("ms");
     
      // Configura el dispositivo como una estación Wi-Fi
      WiFi.mode(WIFI_STA);
      // Imprime la dirección MAC del dispositivo
      if(debug) Serial.print(" MY DIRECCIÓN MAC: ");   
      if(debug) Serial.println(WiFi.macAddress());
      // Habilita el modo promiscuo de Wi-Fi
      wifi_promiscuous_enable(1);
      // Establece el canal de Wi-Fi al utilizado por el servidor
      wifi_set_channel(pairingData.channel);
      // Deshabilita el modo promiscuo de Wi-Fi
      wifi_promiscuous_enable(0);
      // Desconecta cualquier red Wi-Fi a la que esté conectado actualmente el dispositivo
      WiFi.disconnect();
  
      // Inicializa ESP-NOW
      if (esp_now_init() != 0) 
      {
        // Imprime un mensaje de error si hay un problema al inicializar ESP-NOW
        if(debug) Serial.println("Error al inicializar ESP-NOW");
        return;
      }
  
      // Establece el rol del dispositivo en ESP-NOW como combinado
      esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
       
      // Registra una función de devolución de llamada que se llamará cuando se reciba un dato
      esp_now_register_recv_cb(AUTOpairing::OnDataRecv);
      // Registra una función de devolución de llamada que se llamará cuando se envíe un dato
      esp_now_register_send_cb(AUTOpairing::OnDataSent);
      // Agrega el servidor a la lista de pares en ESP-NOW  
      esp_now_add_peer(pairingData.macAddr, ESP_NOW_ROLE_COMBO, pairingData.channel, NULL, 0); // add the server to the peer list 
      // Establece el estado de emparejamiento como "PAIR_PAIRED" para indicar que el dispositivo se ha emparejado correctamente
      pairingStatus = PAIR_PAIRED ;
    }
  }
  // Callback when data is sent
  static void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) 
  {
   
   // Esta función se ejecuta cuando se envía un paquete de datos a través de ESP-NOW
   // Imprime el estado de envío del último paquete y la dirección MAC del destinatario
   if(debug) Serial.print("Estado de envío del último paquete: ");
    printMAC(mac_addr);
   if (sendStatus == 0)
   {
    // Si el estado de envío es 0, indica un éxito en la entrega del paquete
    if(debug) Serial.println(" Éxito de entrega");
    if(pairingStatus == PAIR_PAIRED && mensaje_enviado)
    {
      // Si el dispositivo está emparejado y se ha enviado un mensaje previamente
       mensaje_enviado=false;
       if (terminar && !esperando); 
       gotoSleep();
    }
   }
   else{
    // Si el estado de envío no es 0, indica un error en la entrega del paquete
    if(debug) Serial.println(" Error de entrega");
    if(pairingStatus == PAIR_PAIRED && mensaje_enviado)
    {
       // Si el dispositivo está emparejado y se ha enviado un mensaje previamente
       // Invalida la configuración en la memoria RTC del usuario
      rtcData.code=INVALID_CODE;
      rtcData.data.channel= 2;
      if(usar_FLASH) {  
            // Si se usa memoria flash para almacenar los datos
            EEPROM.put(0, rtcData);
            EEPROM.commit();
      } else {          
            // Si se usa la memoria RTC del usuario para almacenar los datos
            ESP.rtcUserMemoryWrite(0,(uint32_t *)&rtcData, sizeof(rtcData));  
      } 
      if(debug) Serial.println(" INFO de emparejamiento invalidada");
      pairingStatus = PAIR_REQUEST; // volvemos a intentarlo?
      mensaje_enviado=false;
      terminar=false;
      //delay(100);
      //gotoSleep();
    }
  }
  
}

//-----------------------------------------------------------
// Callback when data is received
 static void OnDataRecv(uint8_t * mac, uint8_t *incommingData, uint8_t len) {
  // Esta función se ejecuta cuando se recibe un paquete de datos a través de ESP-NOW
  // Obtiene el tipo de mensaje del primer byte de los datos entrantes
  uint8_t type = incommingData[0];
  if(debug) Serial.print("tipo de mensaje recibido = ");
  if(debug) Serial.print(type,BIN);
  if(debug) Serial.println(messType2String(type));
  if(debug) Serial.print("Tamaño del mensaje : ");
  if(debug) Serial.print(len);
  if(debug) Serial.print(" de ");
  printMAC(mac);
  if(debug) Serial.println();
  uint8_t i;
  // Variables para almacenar el tema y el contenido del mensaje MQTT
  String topic;
  String payload;
  
  switch (type & MASK_MSG_TYPE) 
  {

    case NODATA: 
      // No hay mensajes MQTT
      esperando = false;
      if(debug) Serial.println("No hay mensajes MQTT"); 
      if (terminar); 
      gotoSleep();
    break;  
    
    case DATA:
      // Recibido mensaje MQTT, trasladarlo al callback()
      if(debug) Serial.println("Mensaje recibido MQTT"); 
      for(i=0; i<len; i++) if(incommingData[i]=='|') break;
      topic=String(std::string((char*)incommingData,i).c_str());
      payload = String(std::string((char*)(incommingData+i+1),len-i-1).c_str());
      // Si se ha definido una función de devolución de llamada del usuario, se llama a esa función
      if(user_callback!=NULL) user_callback(topic,payload);
    break;

    case PAIRING:
      // Recibido mensaje de emparejamiento
      memcpy(&pairingData, incommingData, sizeof(pairingData));
      if (pairingData.id == GATEWAY) 
      {                
        // El mensaje proviene del servidor
        if(debug) Serial.print("Emparejamiento hecho para ");
        printMAC(pairingData.macAddr);
        if(debug) Serial.print(" en el canal " );
        if(debug) Serial.print(pairingData.channel);    // channel used by the server
        if(debug) Serial.print(" en ");
        if(debug) Serial.print(millis()-start_time);
        if(debug) Serial.println("ms");
        //esp_now_del_peer(pairingData.macAddr);
        //esp_now_del_peer(mac);
        // Añade el servidor a la lista de pares de ESP-NOW
        esp_now_add_peer(pairingData.macAddr, ESP_NOW_ROLE_COMBO, pairingData.channel, NULL, 0); // add the server to the peer list 
        pairingStatus = PAIR_PAIRED ;  // Establece el estado de emparejamiento
        // Guarda los datos de emparejamiento en la memoria RTC del usuario o en la memoria flash
        rtcData.code=MAGIC_CODE;
        memcpy(&(rtcData.data), &pairingData, sizeof(pairingData));
        if(usar_FLASH) 
        {  
          // Si se usa la memoria flash para almacenar los datos
          if(debug) Serial.println("Escribimos emparejamiento en FLASH");
          EEPROM.put(0, rtcData);
          EEPROM.commit();
        } 
        else 
        {
          // Si se usa la memoria RTC del usuario para almacenar los datos
          if(debug) Serial.println("Escribimos emparejamiento en RTC MEM");        
          ESP.rtcUserMemoryWrite(0,(uint32_t *)&rtcData, sizeof(rtcData));  
        } 
       if(debug) Serial.println("Paired at millis = "+String(millis()));
      }
    break;
  }  
}

//void func ( void (*f)(int) );
//-----------------------------------------------------------
static void set_callback( void (*_user_callback)(String, String) ) 
{
  user_callback=_user_callback;
}
  
//-----------------------------------------------------------
static void check_messages()
  {
    mensaje_enviado=true;
    esperando=true;
    timeOut+=500;
    uint8_t mensaje_esp;
    mensaje_esp=CHECK;
    if(debug) Serial.print("Enviando petición de compronación de mensajes... ");
    esp_now_send(pairingData.macAddr, (uint8_t *) &mensaje_esp, 1);
  }

//-----------------------------------------------------------
static bool espnow_send_check(char * mensaje, bool fin=true, uint8_t _msgType=DATA)
  {
    esperando=true;
    timeOut+=500;
    return espnow_send(mensaje, fin, _msgType | CHECK);
  }
//-----------------------------------------------------------
static bool espnow_send(char * mensaje, bool fin=true, uint8_t _msgType=DATA)
  {
    _msgType = _msgType | ((panAddress << PAN_OFFSET) & MASK_PAN);
    if(debug) Serial.print("enviar tipo de mensaje = ");
    if(debug) Serial.print(_msgType,BIN);
    if(debug) Serial.println(messType2String(_msgType));
    mensaje_enviado=true;
    terminar=fin;
    mensajes_sent++;
    int size = strlen(mensaje);
    if (size> 249)
    {
      if(debug) Serial.print("Longitud del mensaje demasiado larga: ");
      if(debug) Serial.println(size);
      return false;
    }
    struct_espnow mensaje_esp;
    mensaje_esp.msgType=_msgType;
    memcpy(mensaje_esp.payload, mensaje, size); 
    if(debug) Serial.print("Longitud del mensaje: ");
    if(debug) Serial.println(size);
    if(debug) Serial.print("mensaje: ");
    if(debug) Serial.println(mensaje);
    esp_now_send(pairingData.macAddr, (uint8_t *) &mensaje_esp, size+1);
    return true;
  }

//-----------------------------------------------------------
static int mensajes_enviados()
{
  return mensajes_sent;
}

//-----------------------------------------------------------
static bool emparejado()
{
  return (pairingStatus==PAIR_PAIRED) ;
}

//-----------------------------------------------------------
static bool envio_disponible()
{
  return (pairingStatus==PAIR_PAIRED && mensaje_enviado==false && terminar==false) ;
}

//-----------------------------------------------------------
static bool mantener_conexion()
{
  unsigned long currentMillis;
  
  if(!cola_mensajes.empty())
  {
    TmensajeMQTT mensaje = cola_mensajes.front();
    user_callback(mensaje.topic,mensaje.payload);
    cola_mensajes.pop();
  }
  else if( terminar && !esperando);
  gotoSleep();

  // Verifica si ha pasado el tiempo de espera sin emparejar o sin enviar
  if(millis()-start_time > timeOut && timeOutEnabled){
    if(debug) Serial.println("SE PASO EL TIEMPO SIN EMPAREJAR o SIN ENVIAR");
    if(debug) Serial.println("millis = "+String(millis())+" limite: "+String(timeOut));
    gotoSleep();
  }
  
  switch(pairingStatus) 
  {
    case PAIR_REQUEST:
    // Solicitud de emparejamiento en el canal actual
    if(debug) Serial.print(" Solicitud de emparejamiento en el canal "  );
    if(debug) Serial.println(channel);
  
    // Limpia ESP-NOW
    esp_now_deinit();
    WiFi.mode(WIFI_STA);
    // Establece el canal WiFi   
    wifi_promiscuous_enable(1);
    wifi_set_channel(channel);
    wifi_promiscuous_enable(0);
    //WiFi.printDiag(Serial);
    WiFi.disconnect();

    // Inicializa ESP-NOW
    if (esp_now_init() != 0) 
    {
     if(debug) Serial.println("Error al inicializar ESP-NOW");
    }
    esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
    // Establece las rutinas de devolución de llamada
    esp_now_register_send_cb(AUTOpairing::OnDataSent);
    esp_now_register_recv_cb(AUTOpairing::OnDataRecv);
    
    // Establece los datos de emparejamiento para enviar al servidor
    pairingData.msgType = PAIRING;
    pairingData.id = ESPNOW_DEVICE;     
    previousMillis_scanChannel = millis();
    // Envía la solicitud
    esp_now_send(broadcastAddressX, (uint8_t *) &pairingData, sizeof(pairingData));
    pairingStatus = PAIR_REQUESTED;
    break;

    case PAIR_REQUESTED:
    // Tiempo de espera para recibir una respuesta del servidor
    currentMillis = millis(); 
    if(currentMillis - previousMillis_scanChannel > 100) {
      previousMillis_scanChannel = currentMillis;
      // Tiempo de espera expirado, intenta en el siguiente canal
      channel ++;
      if (channel > 11) {
        channel = 0;
      }
      pairingStatus = PAIR_REQUEST; 
    }
    break;

    case PAIR_PAIRED:
    //if(debug) Serial.println("Paired!");
    // Emparejado correctamente
    break;
  }
  return (pairingStatus==PAIR_PAIRED) ;
} 


//--------------------------------------------------------
/*static void gotoSleep() {
  // add some randomness to avoid collisions with multiple devices
  if(debug) Serial.println("Apaga y vamonos");
  ESP.deepSleepInstant(segundos_en_deepSleep * 1000000, RF_NO_CAL);

} */

static void gotoSleep() {
  // add some randomness to avoid collisions with multiple devices
  if (debug) Serial.println("Apaga y vámonos");
  pinMode(16, INPUT_PULLUP); // Configura el GPIO16 (D0) como entrada con resistencia de pull-up interna
  // Entra en el sueño profundo y espera a ser despertado por el cambio de nivel en el GPIO16
  ESP.deepSleepInstant(segundos_en_deepSleep * 1000000, RF_NO_CAL);
  // Apagar alimentación de sensores
  
}




// end of class  
};

//-----------------------------------------------------------
//-----------------------------------------------------------

// statics:

 AUTOpairing::struct_rtc AUTOpairing::rtcData;
 struct_pairing AUTOpairing::pairingData;
 AUTOpairing::PairingStatus AUTOpairing::pairingStatus = PAIR_REQUEST;
 bool AUTOpairing::mensaje_enviado=false; // para saber cuando hay que dejar de enviar porque ya se hizo y estamos esperando confirmación
 bool AUTOpairing::terminar=false; // para saber cuando hay que dejar de enviar porque ya se hizo y estamos esperando confirmación
 bool AUTOpairing::esperando=false; 
 bool AUTOpairing::timeOutEnabled=true; 
 bool AUTOpairing::usar_FLASH=false;
 int AUTOpairing::segundos_en_deepSleep = 10;   // tiempo dormido en segundos
 int AUTOpairing::panAddress = 1;
 unsigned long AUTOpairing::start_time=0;  // para controlar el tiempo de escaneo
 unsigned long AUTOpairing::previousMillis_scanChannel=0;   
 unsigned long AUTOpairing::timeOut=3000;
 bool AUTOpairing::debug=true;
 uint8_t AUTOpairing::channel = 6;  // canal para empezar a escanear
 int AUTOpairing::mensajes_sent=0; 
 void (*AUTOpairing::user_callback)(String, String)=NULL;

#endif
