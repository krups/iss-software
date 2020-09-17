#include "TcInterface.h"

#if defined(ADAFRUIT_TRINKET_M0) || defined(__MK64FX512__)

TcInterface::TcInterface( char* tc_types,
        int cs_tc1 = CS_TC1, 
        int cs_tc2 = CS_TC2, 
        int mux0 = MUX0, 
        int mux1 = MUX1, 
        int tc1_fault = TC1_FAULT,
        int tc2_fault = TC2_FAULT
      ) 
      :tc_types(tc_types), cs_tc1(cs_tc1), cs_tc2(cs_tc2),
      max1(cs_tc1), max2(cs_tc2),
      mux0(mux0), mux1(mux1),
      tc1_fault(tc1_fault), tc2_fault(tc2_fault)
{}

void TcInterface::enable(void){
    pinMode(mux0, OUTPUT);
    pinMode(mux1, OUTPUT);
    // Adafruit_MAX31856::begin() calls Adafruit_SPIDevice::begin()
    // For hardware SPI (which we are using), this calls SPIClass::begin()
    // this sets chip select, MOSI, and SCK to output. MISO is always set as input in master mode.
    if(!max1.begin() || !max2.begin()){
        Serial.println("Could not initialize all thermocouples");
    }
    set_types_from_chars(tc_types);
    //max1.setConversionMode(MAX31856_ONESHOT);
    //max2.setConversionMode(MAX31856_ONESHOT);
}

void TcInterface::disable(void){
    // Cede control so other processor can control MAX chips
    pinMode(cs_tc1, INPUT_PULLUP);
    pinMode(cs_tc2, INPUT_PULLUP);
    pinMode(mux0, INPUT_PULLUP);
    pinMode(mux1, INPUT_PULLUP);
}

void TcInterface::read_all(float* arr){  
    for(int i = 1; i <= 8; i ++){
        Adafruit_MAX31856 max = get_max_from_tc(i);
        max.setThermocoupleType((max31856_thermocoupletype_t) tc_type_lookup[i-1]);
        select_tc(i);

        delay(100);
        
        uint8_t fault = max.readFault();
        if (fault) {
          if (fault & MAX31856_FAULT_CJRANGE) Serial.println("Cold Junction Range Fault");
          if (fault & MAX31856_FAULT_TCRANGE) Serial.println("Thermocouple Range Fault");
          if (fault & MAX31856_FAULT_CJHIGH)  Serial.println("Cold Junction High Fault");
          if (fault & MAX31856_FAULT_CJLOW)   Serial.println("Cold Junction Low Fault");
          if (fault & MAX31856_FAULT_TCHIGH)  Serial.println("Thermocouple High Fault");
          if (fault & MAX31856_FAULT_TCLOW)   Serial.println("Thermocouple Low Fault");
          if (fault & MAX31856_FAULT_OVUV)    Serial.println("Over/Under Voltage Fault");
          if (fault & MAX31856_FAULT_OPEN)    Serial.println("Thermocouple Open Fault");
          Serial.print(" on TC "); Serial.println(i);
        }
        
        //Blocks for ~100 ms
        float temp = max.readThermocoupleTemperature();
        
        
        
        
        arr[i -1] = temp;
    }
}
void TcInterface::triggerOneShot(int tc){
    Adafruit_MAX31856 max = get_max_from_tc(tc);
    max.setConversionMode(MAX31856_ONESHOT_NOWAIT);
    // Set thermocouple type
    max.setThermocoupleType((max31856_thermocoupletype_t) tc_type_lookup[tc-1]);
    // Select thermocouple with mux
    select_tc(tc);
    max.triggerOneShot();
}

bool TcInterface::conversionComplete(int tc){
    Adafruit_MAX31856 max = get_max_from_tc(tc);
    return max.conversionComplete();
}

float TcInterface::readThermocoupleTemperature(int tc){
    Adafruit_MAX31856 max = get_max_from_tc(tc);
    // Select thermocouple with mux
    select_tc(tc);
    return max.readThermocoupleTemperature();
}

uint16_t TcInterface::check_faults(void){
    uint8_t hb = max1.readFault();
    uint8_t lb = max2.readFault();
    return ((uint16_t) hb << 8) | lb;
}

void TcInterface::select_tc(int tc){
    if(tc <= 4){
        tc = tc - 1;
    }
    else{
        tc = tc - 5;
    }

    switch (tc)
    {
    case 0:
        digitalWrite(MUX0, LOW);
        digitalWrite(MUX1, LOW);
        break;
    case 1:
        digitalWrite(MUX0, LOW);
        digitalWrite(MUX1, HIGH);
        break;
    case 2:
        digitalWrite(MUX0, HIGH);
        digitalWrite(MUX1, LOW);
        break;
    case 3:
        digitalWrite(MUX0, HIGH);
        digitalWrite(MUX1, HIGH);
        break;
    default:
        break;
    }
}

Adafruit_MAX31856& TcInterface::get_max_from_tc(int tc){
    if (tc <= 4){
        return max1;
    }
    else{
        return max2;
    }
}


void TcInterface::set_types_from_chars(char* tc_types){
    for(int i = 0; i < 8; i++){
        switch (tc_types[i])
        {
            case 'B':
                tc_type_lookup[i] = MAX31856_TCTYPE_B;
                break;
            case 'E':
                tc_type_lookup[i] = MAX31856_TCTYPE_E;
                break;
            case 'J':
                tc_type_lookup[i] = MAX31856_TCTYPE_J;
                break;
            case 'K':
                tc_type_lookup[i] = MAX31856_TCTYPE_K;
                break;
            case 'N':
                tc_type_lookup[i] = MAX31856_TCTYPE_N;
                break;
            case 'R':
                tc_type_lookup[i] = MAX31856_TCTYPE_R;
                break;
            case 'S':
                tc_type_lookup[i] = MAX31856_TCTYPE_S;
                break;
            case 'T':
                tc_type_lookup[i] = MAX31856_TCTYPE_T;
                break;
            default:
                // Type is already defaulted to K by begin() function
                Serial.println("Cannot set thermocouple type. Default is K");
                break;
        } 
    }
}

#endif
