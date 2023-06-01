/*
 * Librería para comunicación ESPNOW<->MQTT a través de pasarela con auto-emparejamiento
 * Recuerda la configuración de emparejamiento usando FLASH o memoria RTC
 * Recuerda configuración del usuario usando FLASH
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
#include "Arduino.h"
#include "AUTOpairing_common.h"

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

uint8_t broadcastAddressX[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

#define MAGIC_CODE1 0xA5A5
#define MAGIC_CODE2 0xC7C7
#define INVALID_CODE 0

#ifndef MAX_CONFIG_SIZE
#define MAX_CONFIG_SIZE 64
#endif

class AUTOpairing
{
  
  enum PairingStatus {PAIR_REQUEST, PAIR_REQUESTED, PAIR_PAIRED, };
  
  struct struct_rtc{
    uint16_t code1;
    uint16_t code2;
    struct_pairing data;
    uint8_t config[MAX_CONFIG_SIZE]; // max config size
  } ;

  static int config_size;
  static int mensajes_sent;
  static PairingStatus pairingStatus;
  static struct_rtc rtcData;
  static struct_pairing pairingData;
  static bool mensaje_enviado; // para saber cuando hay que dejar de enviar porque ya se hizo y estamos esperando confirmación
  static bool terminar; // para saber cuando hay que dejar de enviar porque ya se hizo y estamos esperando confirmación
  static unsigned long previousMillis_scanChannel;    // will store last time channel was scanned 
  static unsigned long start_time;  // para controlar el tiempo de escaneo
  static bool esperando;  // esperando mensajes
  static bool EEPROM_init;

  static void (*user_callback)(String, String); 

//-----------------------------------------------------------
static void printMAC(const uint8_t * mac_addr){
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
static bool init_config_size(uint8_t size)
{
  config_size = size;
  if(!EEPROM_init) { EEPROM.begin(sizeof(rtcData)); EEPROM_init=true; }
  if (size>MAX_CONFIG_SIZE) 
  {
    Serial.printf("Espacio reservado en FLASH demasiado pequeño: %d\n Por favor incremente el valor MAX_CONFIG_SIZE",MAX_CONFIG_SIZE );
    return false;
  }
  else
  return true;
}

//-----------------------------------------------------------
static bool get_config(uint8_t* config)
{
  //init check FLASH 
  
  EEPROM.get(0, rtcData); 
  if(debug) Serial.print("Magic code en FLASH: ");
  if(debug) Serial.print(rtcData.code2,HEX);
  if(debug) Serial.print(" - esperando: ");
  if(debug) Serial.println((unsigned long)MAGIC_CODE2,HEX);
  
  if(rtcData.code2==MAGIC_CODE2)
  {
    memcpy(config, &(rtcData.config), config_size);
    if(debug) Serial.println("Configuración leída de FLASH");
    return true;
  }
  else
  {
    if(debug) Serial.println("Sin configuración en FLASH");
    return false;
  }
}

//-----------------------------------------------------------
static void set_config(uint8_t* config)
{
  rtcData.code2=MAGIC_CODE2;
  memcpy(&(rtcData.config), config, config_size);
  if(debug) Serial.println("Escribimos configuración en FLASH");
  EEPROM.put(0, rtcData);
  EEPROM.commit(); 
}

//-----------------------------------------------------------
static void begin()
  {
    previousMillis_scanChannel=0;
    start_time=millis();
    
    //init check FLASH or RTC MEM
    // usamos rtc memory
    if(usar_FLASH)
    {
      if(!EEPROM_init) { EEPROM.begin(sizeof(rtcData)); EEPROM_init=true; }
      EEPROM.get(0, rtcData); 
      if(debug) Serial.print("Magic code en FLASH: ");
    }
    else 
    {
      ESP.rtcUserMemoryRead(0,(uint32_t *)&rtcData, sizeof(rtcData));
      if(debug) Serial.print("Magic code en RTC MEM: ");
    }
    if(debug) Serial.print(rtcData.code1,HEX);
    if(debug) Serial.print(" - esperando: ");
    if(debug) Serial.println((unsigned long)MAGIC_CODE1,HEX);
    
    if(rtcData.code1==MAGIC_CODE1)
    {
      memcpy(&pairingData, &(rtcData.data), sizeof(pairingData));
      if(debug) Serial.print("Emparejamiento recuperado de la memoria RTC del usuario ");
      printMAC(pairingData.macAddr);
      if(debug) Serial.print(" en el canal " );
      if(debug) Serial.print(pairingData.channel);    // channel used by the server
      // Imprime el tiempo transcurrido desde el inicio del proceso de emparejamiento
      if(debug) Serial.print(" en ");
      if(debug) Serial.print(millis()-start_time);
      if(debug) Serial.println("ms");
     
    // Set device as a Wi-Fi Station
      WiFi.mode(WIFI_STA);
      if(debug) Serial.print(" MI DIRECCIÓN MAC: ");   
      if(debug) Serial.println(WiFi.macAddress());
      wifi_promiscuous_enable(1);
      wifi_set_channel(pairingData.channel);
      wifi_promiscuous_enable(0);
      WiFi.disconnect();
  
      // Init ESP-NOW
      if (esp_now_init() != 0) {
        if(debug) Serial.println("Error al inicializar ESP-NOW");
        return;
      }
  
      // Set ESP-NOW Role
      esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
       
      // Register for a callback function that will be called when data is received
      esp_now_register_recv_cb(AUTOpairing::OnDataRecv);
      esp_now_register_send_cb(AUTOpairing::OnDataSent);
  
      esp_now_add_peer(pairingData.macAddr, ESP_NOW_ROLE_COMBO, pairingData.channel, NULL, 0); // add the server to the peer list 
      pairingStatus = PAIR_PAIRED ;            // set the pairing status
    }
}


//-----------------------------------------------------------
// Callback when data is sent
static void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  if(debug) Serial.print("Estado de envío del último paquete a ");
  printMAC(mac_addr);
  if (sendStatus == 0){
    if(debug) Serial.println(" >> Éxito de entrega");
    if(pairingStatus == PAIR_PAIRED && mensaje_enviado)
    {
       mensaje_enviado=false;
       if (terminar && !esperando) gotoSleep();
    }
  }
  else{
    if(debug) Serial.println(" >> Error de entrega");
    if(pairingStatus == PAIR_PAIRED && mensaje_enviado)
    {
      //no hemos conseguido hablar con la pasarela emparejada...
      // invalidamos config en flash;
      rtcData.code1=INVALID_CODE;
      rtcData.data.channel= 2;
      if(usar_FLASH) {  
            EEPROM.put(0, rtcData);
            EEPROM.commit();
      } else {          
            ESP.rtcUserMemoryWrite(0,(uint32_t *)&rtcData, sizeof(rtcData));  
      } // end if usar_FLASH
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
  String topic;
  String payload;
  
  switch (type & MASK_MSG_TYPE) {

  case NODATA: 
        esperando = false;
        if(debug) Serial.println("No hay mensajes MQTT"); 
        if (terminar && cola_mensajes.empty()) gotoSleep();
    break;  
    
  case DATA:
        // recibimos mensaje mqtt, trasladarlo al callback() 
        if(debug) Serial.println("Mensaje recibido MQTT"); 
        for(i=0; i<len; i++) if(incommingData[i]=='|') break;
        topic=String(std::string((char*)incommingData+1,i-1).c_str());
        payload = String(std::string((char*)(incommingData+i+1),len-i-1).c_str());
        if(user_callback!=NULL) cola_mensajes.push(TmensajeMQTT(topic, payload));
        //user_callback(topic,payload);
    break;

  case PAIRING:
    memcpy(&pairingData, incommingData, sizeof(pairingData));
    if (pairingData.id == GATEWAY) {                // the message comes from server
        if(debug) Serial.print("Emparejamiento hecho para ");
        printMAC(pairingData.macAddr);
        if(debug) Serial.print(" en el canal " );
        if(debug) Serial.print(pairingData.channel);    // channel used by the server
        if(debug) Serial.print(" en ");
        if(debug) Serial.print(millis()-start_time);
        if(debug) Serial.println("ms");
      //esp_now_del_peer(pairingData.macAddr);
      //esp_now_del_peer(mac);
      esp_now_add_peer(pairingData.macAddr, ESP_NOW_ROLE_COMBO, pairingData.channel, NULL, 0); // add the server to the peer list 
      pairingStatus = PAIR_PAIRED ;            // set the pairing status
      //guardar en FLASH
      rtcData.code1=MAGIC_CODE1;
      memcpy(&(rtcData.data), &pairingData, sizeof(pairingData));
      if(usar_FLASH) {  
            if(debug) Serial.println("Escribimos emparejamiento en FLASH");
            EEPROM.put(0, rtcData);
            EEPROM.commit();
      } else {
            if(debug) Serial.println("Escribimos emparejamiento en RTC MEM");        
            ESP.rtcUserMemoryWrite(0,(uint32_t *)&rtcData, sizeof(rtcData));  
      } // end if usar_FLASH
       if(debug) Serial.println("Emparejado en milisegundo = "+String(millis()));
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
    if(debug) Serial.print("sending message type = ");
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
  else if( terminar && !esperando) gotoSleep();
    
  if(millis()-start_time > timeOut && timeOutEnabled )
  {
    if(debug) Serial.println("SE PASO EL TIEMPO SIN EMPAREJAR o SIN ENVIAR");
    if(debug) Serial.println("millis = "+String(millis())+" limite: "+String(timeOut));
    gotoSleep();
  }
  
  switch(pairingStatus) {
  case PAIR_REQUEST:
    if(debug) Serial.print(" Solicitud de emparejamiento en el canal "  );
    if(debug) Serial.println(channel);
  
    // clean esp now
    esp_now_deinit();
    WiFi.mode(WIFI_STA);
    // set WiFi channel   
    wifi_promiscuous_enable(1);
    wifi_set_channel(channel);
    wifi_promiscuous_enable(0);
    //WiFi.printDiag(Serial);
    WiFi.disconnect();

    // Init ESP-NOW
    if (esp_now_init() != 0) {
      if(debug) Serial.println("Error al inicializar ESP-NOW");
    }
    esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
    // set callback routines
    esp_now_register_send_cb(AUTOpairing::OnDataSent);
    esp_now_register_recv_cb(AUTOpairing::OnDataRecv);
    
    // set pairing data to send to the server
    pairingData.msgType = PAIRING;
    pairingData.id = ESPNOW_DEVICE;     
    previousMillis_scanChannel = millis();
    // send request
    esp_now_send(broadcastAddressX, (uint8_t *) &pairingData, sizeof(pairingData));
    pairingStatus = PAIR_REQUESTED;
    break;

  case PAIR_REQUESTED:
    // time out to allow receiving response from server
    currentMillis = millis(); 
    if(currentMillis - previousMillis_scanChannel > 100) {
      previousMillis_scanChannel = currentMillis;
      // time out expired,  try next channel
      channel ++;
      if (channel > 11) {
        channel = 0;
      }
      pairingStatus = PAIR_REQUEST; 
    }
    break;

  case PAIR_PAIRED:
    //if(debug) Serial.println("Paired!");
    break;
  }
  return (pairingStatus==PAIR_PAIRED) ;
} 


//--------------------------------------------------------
static void gotoSleep() {
  // add some randomness to avoid collisions with multiple devices
  if(debug) Serial.println("Apaga y vamonos");
  ESP.deepSleepInstant(segundos_en_deepSleep * 1000000, RF_NO_CAL);
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
 bool AUTOpairing::EEPROM_init=false;
 int AUTOpairing::segundos_en_deepSleep = 10;   // tiempo dormido en segundos
 int AUTOpairing::panAddress = 1;
 int AUTOpairing::config_size = MAX_CONFIG_SIZE;
 unsigned long AUTOpairing::start_time=0;  // para controlar el tiempo de escaneo
 unsigned long AUTOpairing::previousMillis_scanChannel=0;   
 unsigned long AUTOpairing::timeOut=3000;
 bool AUTOpairing::debug=true;
 uint8_t AUTOpairing::channel = 6;  // canal para empezar a escanear
 int AUTOpairing::mensajes_sent=0; 
 void (*AUTOpairing::user_callback)(String, String)=NULL;

#endif
