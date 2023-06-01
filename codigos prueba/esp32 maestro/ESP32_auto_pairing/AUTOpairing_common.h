#ifndef AUTOpairing_common_H
#define AUTOpairing_common_H

#define DATA    0b00000001
#define PAIRING 0b00000000
#define SUBS    0b00000010
#define CHECK   0b10000000
#define NODATA  0b00000011

struct struct_pairing {       // new structure for pairing
    uint8_t msgType;
    uint8_t id;
    uint8_t macAddr[6];
    uint8_t channel;
    uint8_t padding[3];
};

struct struct_espnow {
    uint8_t msgType;
    uint8_t payload[249];
};

//-----------------------------------------------------
// devuelve 2 caracteres HEX para un byte
inline String one_b2H(uint8_t data)
{
  return (String(data, HEX).length()==1)? String("0")+String(data, HEX) : String(data, HEX);
}

String byte2HEX (uint8_t *mac)
{
  String _deviceMac="";
  for (int i=0; i<6; i++) _deviceMac += one_b2H(mac[i]);
  for (auto & c: _deviceMac) c = toupper(c);
    
  return _deviceMac;
}

//-----------------------------------------------------
// compara dos MACs
inline bool igualMAC(uint8_t *mac1, uint8_t *mac2){
  for(int i=0; i<6; i++) if(mac1[i] != mac2[i]) return false;
  return true;
}

//-----------------------------------------------------
// devuelve 6 bytes de la MAC en texto

void HEX2byte(uint8_t *mac, char* texto){ 
    char * pos=texto;
     /* WARNING: no sanitization or error-checking whatsoever */
    for (size_t count = 0; count < 6 ; count++) {
        sscanf(pos, "%2hhx", &mac[count]);
        pos += 2;
    }
}
#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)   \
  ((byte) & 0x80 ? '1' : '0'), \
  ((byte) & 0x40 ? '1' : '0'), \
  ((byte) & 0x20 ? '1' : '0'), \
  ((byte) & 0x10 ? '1' : '0'), \
  ((byte) & 0x08 ? '1' : '0'), \
  ((byte) & 0x04 ? '1' : '0'), \
  ((byte) & 0x02 ? '1' : '0'), \
  ((byte) & 0x01 ? '1' : '0') 
#endif
