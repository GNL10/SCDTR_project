#include <EEPROM.h>
#include <Arduino.h>


void setup(){
    Serial.begin(115200);

    int address = 0;

    uint8_t id = 2;
    float C1 = 0.000001; //0.0000015; // Capacitor capacitance[uF]
    float m = -0.70062; //-0.68424;
    float b = 4.83251; //4.77815;
    
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