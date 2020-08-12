#include "Tc.h"

Tc::Tc( char* tc_types,
        int cs_tc1 = CS_TC1, 
        int cs_tc2 = CS_TC2, 
        int mux0 = MUX0, 
        int mux1 = MUX1, 
        int tc1_fault = TC1_FAULT,
        int tc2_fault = TC2_FAULT
      ) : max1(cs_tc1), max2(cs_tc2)
{
    if(!max1.begin() | !max2.begin()){
        Serial.println("Could not initialize all thermocouples");
    }
    set_types_from_chars(tc_types);
}

void Tc::read_all(float* arr){
    max1.setConversionMode(MAX31856_ONESHOT);
    max2.setConversionMode(MAX31856_ONESHOT);

    for(int i = 1; i <= 8; i ++){
        Adafruit_MAX31856 max = get_max_from_tc(i);
        max.setThermocoupleType((max31856_thermocoupletype_t) tc_type_lookup[i-1]);
        select_tc(i);
        //Blocks for ~100 ms
        float temp = max.readThermocoupleTemperature();
        arr[i -1] = temp;
    }
}
void Tc::triggerOneShot(int tc){
    Adafruit_MAX31856 max = get_max_from_tc(tc);
    max.setConversionMode(MAX31856_ONESHOT_NOWAIT);
    // Set thermocouple type
    max.setThermocoupleType((max31856_thermocoupletype_t) tc_type_lookup[tc-1]);
    // Select thermocouple with mux
    select_tc(tc);
    max.triggerOneShot();
}

bool Tc::conversionComplete(int tc){
    Adafruit_MAX31856 max = get_max_from_tc(tc);
    return max.conversionComplete();
}

float Tc::readThermocoupleTemperature(int tc){
    Adafruit_MAX31856 max = get_max_from_tc(tc);
    // Select thermocouple with mux
    select_tc(tc);
    return max.readThermocoupleTemperature();
}

uint16_t Tc::check_faults(void){
    uint8_t hb = max1.readFault();
    uint8_t lb = max2.readFault();
    return ((uint16_t) hb << 8) | lb;
}

void Tc::select_tc(int tc){
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

Adafruit_MAX31856 Tc::get_max_from_tc(int tc){
    if (tc <= 4){
        return max1;
    }
    else{
        return max2;
    }
}


void Tc::set_types_from_chars(char* tc_types){
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
