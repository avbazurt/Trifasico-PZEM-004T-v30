#include "PZEM_Trifasico.h"

#define PZEM_OK 0
#define PZEM_ERROR_FASE_A 1
#define PZEM_ERROR_FASE_B 2
#define PZEM_ERROR_FASE_C 3
#define PZEM_ERROR_FASE_AB 4
#define PZEM_ERROR_FASE_BC 5
#define PZEM_ERROR_FASE_AC 6
#define PZEM_ERROR_FASE_ABC 7

PZEM_Trifasico::PZEM_Trifasico(HardwareSerial &PZEM_SERIAL, int PZEM_RX_PIN, int PZEM_TX_PIN,
                               int PZEM_ADDRESS_FASE_A,
                               int PZEM_ADDRESS_FASE_B,
                               int PZEM_ADDRESS_FASE_C) : faseA(PZEM_SERIAL, PZEM_RX_PIN, PZEM_TX_PIN, PZEM_ADDRESS_FASE_A),
                                                          faseB(PZEM_SERIAL, PZEM_RX_PIN, PZEM_TX_PIN, PZEM_ADDRESS_FASE_B),
                                                          faseC(PZEM_SERIAL, PZEM_RX_PIN, PZEM_TX_PIN, PZEM_ADDRESS_FASE_C)
{
    // pzem.setCalibration();
    DatosFaseA = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    DatosFaseB = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    DatosFaseC = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    //Inicializo
    P3 = 0;
    Q3 = 0;
    S3 = 0;
}

float PZEM_Trifasico::asin(float c)
// Solucion utilizada de https://www.instructables.com/Arduino-Trigonometric-Inverse-Functions/
{
    float out;
    out = ((c + (c * c * c) / 6 + (3 * c * c * c * c * c) / 40 + (5 * c * c * c * c * c * c * c) / 112 +
            (35 * c * c * c * c * c * c * c * c * c) / 1152 + (c * c * c * c * c * c * c * c * c * c * c * 0.022) +
            (c * c * c * c * c * c * c * c * c * c * c * c * c * .0173) + (c * c * c * c * c * c * c * c * c * c * c * c * c * c * c * .0139) +
            (c * c * c * c * c * c * c * c * c * c * c * c * c * c * c * c * c * 0.0115) + (c * c * c * c * c * c * c * c * c * c * c * c * c * c * c * c * c * c * c * 0.01)));
    // asin
    if (c >= .96 && c < .97)
    {
        out = 1.287 + (3.82 * (c - .96));
    }
    if (c >= .97 && c < .98)
    {
        out = (1.325 + 4.5 * (c - .97));
    } // arcsin
    if (c >= .98 && c < .99)
    {
        out = (1.37 + 6 * (c - .98));
    }
    if (c >= .99 && c <= 1)
    {
        out = (1.43 + 14 * (c - .99));
    }
    return out;
}

float PZEM_Trifasico::acos(float c)
// Solucion utilizada de https://www.instructables.com/Arduino-Trigonometric-Inverse-Functions/
{
    float out;
    out = asin(sqrt(1 - c * c));
    return out;
}

float PZEM_Trifasico::atan(float c)
// Solucion utilizada de https://www.instructables.com/Arduino-Trigonometric-Inverse-Functions/
{
    float out;
    out = asin(c / (sqrt(1 + c * c)));
    return out;
}

void PZEM_Trifasico::setModeSimulation()
{
    _simulation = true;
}

bool PZEM_Trifasico::SimuMedicionMonofasica(ParametrosFase &struct_fase)
{
    struct_fase.VLN = 110 + random(-100, 100) / 10;
    struct_fase.I = (1.15 * random(0, 200)) / 10;

    struct_fase.FP = (1.15 * random(0, 90)) / 100;
    struct_fase.angule = acos(struct_fase.FP);

    struct_fase.P = struct_fase.VLN * struct_fase.I * struct_fase.FP;
    struct_fase.Q = struct_fase.P * sin(struct_fase.angule);
    struct_fase.S = struct_fase.VLN * struct_fase.I;

    struct_fase.f = 60;

    return true;
}

bool PZEM_Trifasico::GetMedicionMonofasica(PZEM004Tv30 fase, ParametrosFase &struct_fase)
{
    // Obtener Medicion Factor Potencia
    struct_fase.FP = fase.pf();
    if (isnan(struct_fase.FP))
    {
        Serial.println("Error reading power factor");
        struct_fase.FP = -1;
        return false;
    }
    else
    {
        struct_fase.angule = acos(struct_fase.FP);
    }

    // Obtener Medicion Voltaje
    struct_fase.VLN = fase.voltage();
    if (isnan(struct_fase.VLN))
    {
        Serial.println("Error reading voltage");
        struct_fase.VLN = -1;
        return false;
    }

    // Obtener Medicion Corriente
    struct_fase.I = fase.current();
    if (isnan(struct_fase.I))
    {
        Serial.println("Error reading current");
        struct_fase.I = -1;
        return false;
    }

    // Obtener Medicion Potencia
    struct_fase.P = fase.power();
    if (isnan(struct_fase.P))
    {
        Serial.println("Error reading power");
        struct_fase.P = -1;
        struct_fase.Q = -1;
        struct_fase.S = -1;
        return false;
    }
    else
    {
        struct_fase.P = struct_fase.P / 1000;
        struct_fase.Q = struct_fase.P * sin(struct_fase.angule);
        struct_fase.S = struct_fase.VLN * struct_fase.I;
    }

    // Obtener Medicion Energia
    struct_fase.E = fase.energy();
    if (isnan(struct_fase.E))
    {
        Serial.println("Error reading energy");
        struct_fase.E = -1;
        return false;
    }

    // Obtener Medicion Frecuencia
    struct_fase.f = fase.frequency();
    if (isnan(struct_fase.f))
    {
        Serial.println("Error reading frequency");
        struct_fase.f = -1;
        return false;
    }

    return true;
}

int PZEM_Trifasico::GetMedicionTrifasica()
{
    bool _faseA;
    bool _faseB;
    bool _faseC;

    if (_simulation)
    {
        _faseA = SimuMedicionMonofasica(DatosFaseA);
        _faseB = SimuMedicionMonofasica(DatosFaseB);
        _faseC = SimuMedicionMonofasica(DatosFaseC);
    }
    else
    {
        _faseA = GetMedicionMonofasica(faseA, DatosFaseA);
        _faseB = GetMedicionMonofasica(faseB, DatosFaseB);
        _faseC = GetMedicionMonofasica(faseC, DatosFaseC);
    }

    //Esta condicion nos indica que la medicion fue exitosa
    if (_faseA && _faseB && _faseC)
    {
        P3 = (DatosFaseA.P + DatosFaseB.P + DatosFaseC.P);
        Q3 = (DatosFaseA.Q + DatosFaseB.Q + DatosFaseC.Q);
        S3 = (DatosFaseA.S + DatosFaseB.S + DatosFaseC.S) / 1000;
        return PZEM_OK;
    }

    //Esta condicion nos indica que falla fase A
    else if (!_faseA && _faseB && _faseC)
    {
        return PZEM_ERROR_FASE_A;
    }

    //Esta condicion nos indica que falla fase B
    else if (_faseA && !_faseB && _faseC)
    {
        return PZEM_ERROR_FASE_B;
    }

    //Esta condicion nos indica que falla fase C
    else if (_faseA && _faseB && !_faseC)
    {
        return PZEM_ERROR_FASE_C;
    }

    //Esta condicion nos indica que falla fase A y B
    else if (!_faseA && !_faseB && _faseC)
    {
        return PZEM_ERROR_FASE_AB;
    }

    //Esta condicion nos indica que falla fase B y C
    else if (_faseA && !_faseB && !_faseC)
    {
        return PZEM_ERROR_FASE_BC;
    }

    //Esta condicion nos indica que falla fase A y C
    else if (!_faseA && _faseB && !_faseC)
    {
        return PZEM_ERROR_FASE_AC;
    }

    //Esta condicion nos indica que falla fase A B y C
    else if (!_faseA && !_faseB && !_faseC)
    {
        return PZEM_ERROR_FASE_ABC;
    }
}