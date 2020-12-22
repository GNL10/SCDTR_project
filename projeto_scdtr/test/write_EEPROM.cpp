#include <EEPROM.h>
#include <Arduino.h>


void setup(){
    Serial.begin(115200);

    int address = 0;

    byte id = 1;
    float C1 = 0.0000015; //0.000001; // Capacitor capacitance[uF]
    float m = -0.793;
    float b = 5.1;
    
    // WRITING TO EEPROM
    EEPROM.put(address, id);
    address += sizeof(id);
    EEPROM.put(address, C1);
    address += sizeof(C1);
    EEPROM.put(address, m);
    address += sizeof(m);
    EEPROM.put(address, b);
    address += sizeof(b);

    Serial.print("##### FINISHED LOADING EEPROM #####");
}

void loop() {

}