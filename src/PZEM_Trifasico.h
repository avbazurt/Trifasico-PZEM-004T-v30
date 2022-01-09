#pragma once
#include <PZEM004Tv30.h>

class PZEM_Trifasico
{
private:
    struct ParametrosFase
    {
        float VLN; //VOLTAJE LINEA A NEUTRO [V]
        float VLL; //VOLTAJE LINEA A LINEA [VLL]
        float I;   //CORRIENTE
        float P;    //POTENCIA REACTIVA
        float Q;
        float S;
        float angule;
        float E;
        float f;
        float FP;
    };

    
    bool GetMedicionMonofasica(PZEM004Tv30 fase,ParametrosFase& struct_fase);
    bool SimuMedicionMonofasica(ParametrosFase& struct_fase);


    float asin(float c);
    float acos(float c);
    float atan(float c);

    bool _simulation = false;

public:

    // Constructor
    PZEM_Trifasico(HardwareSerial &PZEM_SERIAL, int PZEM_RX_PIN, int PZEM_TX_PIN,
                    int PZEM_ADDRESS_FASE_A = 0x00,
                    int PZEM_ADDRESS_FASE_B = 0x01,
                    int PZEM_ADDRESS_FASE_C = 0x02);

    // Funciones
    int GetMedicionTrifasica();

    void setModeSimulation();


    // Objetos
    PZEM004Tv30 faseA;
    PZEM004Tv30 faseB;
    PZEM004Tv30 faseC;

    // Variables
    ParametrosFase DatosFaseA;
    ParametrosFase DatosFaseB;
    ParametrosFase DatosFaseC;

    float P3;
    float Q3;
    float S3;
};