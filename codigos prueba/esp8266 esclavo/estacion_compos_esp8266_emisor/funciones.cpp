//librerias

#include "funciones.h"
#include<OneWire.h>
#include<DallasTemperature.h>

//variables
/* DECLARACIÓN DE PINES */
#define ONE_WIRE_BUS 4 // DECLARACIÓN DE GPIO 4 (D2) COMO BUS DE COMUNICACIÓN CON LOS 3 SENSORES DE TEMPERATURA
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);





/* DECLARACIÓN DE VARIABLES */
int numberOfDevices;
/*adquisición de las direcciones de los sensores conectados al bus*/
DeviceAddress tempDeviceAddress; 
/* LLAMADA A FUNCIÓN DE ADQUISICIÓN DE DIRECCIONES DE SENSORES CONECTADOS AL BUS DE TEMPERATURA*/


// FUNCIÓN PARA IMPRIMIR LAS DIRECCIONES DE LOS SENSORES ENCONTRADOS
void printAddress(DeviceAddress deviceAddress){
    for (uint8_t i = 0; i < 8; i++){
      if (deviceAddress[i] < 16) Serial.print("0");
      Serial.print(deviceAddress[i], HEX);
    } 
  }
    // FUNCIÓN DE ADQUISICIÓN DE DIRECCIONES DE SENSORES CONECTADOS AL BUS DE TEMPERATURA
  void adquisicion_direcciones_temp(){
    sensors.begin();
    // Variable para almacenar cuantos dispositivos tengo conectados al bus
    numberOfDevices = sensors.getDeviceCount();
  
    // Encontrar dispositivos conectados al bus
    Serial.print("Localización de dispositivos...");
    Serial.print("encontrados ");
    Serial.print(numberOfDevices, DEC);
    Serial.println(" Sensores de temperatura.");

    // Recorrer cada dispositivo y obteber sus direcciónes
    for(int i=0;i<numberOfDevices; i++){
    
         if(sensors.getAddress(tempDeviceAddress, i)){
          Serial.print("Sensor  ");
          Serial.print(i, DEC);
          Serial.print(" con dirección: ");
          printAddress(tempDeviceAddress);
          Serial.println();
          } else {
          Serial.print("Dispositivos sin dirección ");
          Serial.print(i, DEC);
          Serial.print(" pero no pudo detectar la dirección. Comprobar la alimentación y el cableado");
          }
         } 
    sensors.requestTemperatures();       
  } //b


  void pedir_temperaturas(float temp[]){

    /*ENVIO DE PETICIÓN PARA ADQUIRIR TEMPERATURAS*/
    
  
    for(int i=0;i<numberOfDevices; i++){
    // Search the wire for address
      if(sensors.getAddress(tempDeviceAddress, i)){
      // Output the device ID
      Serial.print("Temperatura sensor: ");
      Serial.println(i,DEC);
      // Print the data
      float tempC = sensors.getTempC(tempDeviceAddress);
      temp[i]=tempC;
      Serial.print("Temp C: ");
      Serial.print(tempC);
      Serial.print(" Temp F: ");
      Serial.println(DallasTemperature::toFahrenheit(tempC)); // Converts tempC to Fahrenheit
    }
  }
  //delay(1000);
}

float  pedir_humedad()
{
  //declaración de pines
  const int analogInPin = A0;  // ESP8266 Analog Pin ADC0 = A0

  //inicialización de pines
  float sensorValue = 0;  // Inicializar valor de lectura analógica

  
  /*calibración sensor de humedad*/
  float ValorHumedadMinima=780; // Registre el valor del sensor cuando la sonda esté expuesta al aire como "ValorHumedadMinima". Este es el valor límite del suelo seco "Humedad: 0% HR"
  float ValorHumedadMaxima=375; // Registre el valor del sensor cuando la sonda esté expuesta al agua como "ValorHumedadMaxima". Este es el valor límite del suelo húmedo "Humedad: 100% HR"
  float HUM_RANGO (ValorHumedadMinima - ValorHumedadMaxima);

 delay(900);
  // lectura de sensor humedad
  sensorValue = analogRead(analogInPin);
  float porcentaje_humedad = 100 * (1 - (sensorValue - ValorHumedadMaxima) / HUM_RANGO);
 
  // Imprimir medida por el monitor de l puerto serie
  //  Serial.print("sensor humedad = ");
  //  Serial.print(sensorValue);
  //  Serial.print('\n');
  Serial.print("sensor humedad RH% = ");
  Serial.print(porcentaje_humedad);
  Serial.print('\n');
  return porcentaje_humedad;
  
}
