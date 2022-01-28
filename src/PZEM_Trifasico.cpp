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

    MaxFaseA = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    MaxFaseB = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    MaxFaseC = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    // Inicializo

    TriFase = {0, 0, 0, 0, 0, 0};
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

    DatosFaseA.Simu._num_change = random(60, 150);
    DatosFaseA.Simu._currentSimu = 1.01 * random(1, 1000) / 100;
    DatosFaseA.Simu._fpSimu = 1.01 * random(87, 90) / 100;

    DatosFaseB.Simu._num_change = random(120, 240);
    DatosFaseB.Simu._currentSimu = 1.01 * random(1, 1000) / 100;
    DatosFaseB.Simu._fpSimu = 1.01 * random(87, 90) / 100;

    DatosFaseC.Simu._num_change = random(120, 240);
    DatosFaseC.Simu._currentSimu = 1.01 * random(1, 1000) / 100;
    DatosFaseC.Simu._fpSimu = 1.01 * random(87, 90) / 100;

    Serial.println("Cambios");
    Serial.println(DatosFaseA.Simu._num_change);
    Serial.println(DatosFaseB.Simu._num_change);
    Serial.println(DatosFaseC.Simu._num_change);

}

bool PZEM_Trifasico::ValidateData(float medicion, float &valor, float &max)
{
    if (isnan(medicion))
    {
        valor = -1;
        return false;
    }

    valor = medicion;
    if (valor > max)
    {
        max = valor;
    }
    return true;
}

bool PZEM_Trifasico::SimuMedicionMonofasica(ParametrosFase &struct_fase, ParametrosFase &max_fase)
{
    if (struct_fase.Simu._contador >= struct_fase.Simu._num_change)
    {
        struct_fase.Simu._num_change = random(60, 120);
        struct_fase.Simu._contador = 0;

        do
        {
            struct_fase.Simu._incrementoCurrente = 1.01 * random(-150,150)/ 100;
        } while (struct_fase.Simu._currentSimu + struct_fase.Simu._incrementoCurrente > 9 or struct_fase.Simu._currentSimu + struct_fase.Simu._incrementoCurrente <= 0);
        struct_fase.Simu._currentSimu = struct_fase.Simu._currentSimu + struct_fase.Simu._incrementoCurrente;

        do
        {
            struct_fase.Simu._incrementoFP = 1.01 * random(-1, 1) / 100;
        } while (struct_fase.Simu._fpSimu + struct_fase.Simu._incrementoFP > 0.97 or struct_fase.Simu._fpSimu + struct_fase.Simu._incrementoFP < 0.8);
        struct_fase.Simu._fpSimu = struct_fase.Simu._fpSimu + struct_fase.Simu._incrementoFP;
    }

    ValidateData(110 + random(-25, 25) / 10, struct_fase.VLN, max_fase.VLN);
    ValidateData(struct_fase.Simu._currentSimu + 1.01 * random(-150, 150) / 1000, struct_fase.I, max_fase.I);
    ValidateData(struct_fase.Simu._fpSimu + 1.01 * random(-2, 2) / 100, struct_fase.FP, max_fase.FP);

    ValidateData(acos(struct_fase.FP), struct_fase.angule, max_fase.angule);

    ValidateData(struct_fase.VLN * struct_fase.I * struct_fase.FP, struct_fase.P, max_fase.P);
    ValidateData(struct_fase.P * sin(struct_fase.angule), struct_fase.Q, max_fase.Q);
    ValidateData(struct_fase.VLN * struct_fase.I, struct_fase.S, max_fase.S);

    struct_fase.f = 60;
    max_fase.f = 60;

    Serial.println(struct_fase.Simu._contador);
    struct_fase.Simu._contador++;

    return true;
}

bool PZEM_Trifasico::GetMedicionMonofasica(PZEM004Tv30 fase, ParametrosFase &struct_fase, ParametrosFase &max_fase)
{
    // Medicion de Factor de potencia --------------------------------
    if (!ValidateData(fase.pf(), struct_fase.FP, max_fase.FP))
    {
        Serial.println("Error reading power factor");
        return false;
    }
    else
    {
        ValidateData(acos(struct_fase.FP), struct_fase.angule, max_fase.angule);
    }

    // Medicion de Voltaje-------------------------------------------
    if (!ValidateData(fase.voltage(), struct_fase.VLN, max_fase.VLN))
    {
        Serial.println("Error reading voltage");
        return false;
    }

    // Obtener Medicion Corriente -------------------------------------------
    if (!ValidateData(fase.current(), struct_fase.I, max_fase.I))
    {
        Serial.println("Error reading current");
        return false;
    }

    // Obtener Medicion Potencia ---------------------------------------------
    if (!ValidateData(fase.power(), struct_fase.P, max_fase.P))
    {
        Serial.println("Error reading power");
        return false;
    }
    else
    {
        ValidateData(struct_fase.P / 1000, struct_fase.P, max_fase.P);
        ValidateData(struct_fase.P * sin(struct_fase.angule), struct_fase.Q, max_fase.Q);
        ValidateData(struct_fase.VLN * struct_fase.I, struct_fase.S, max_fase.S);
    }

    // Obtener Medicion Energia ---------------------------------------------------
    if (!ValidateData(fase.energy(), struct_fase.E, max_fase.E))
    {
        Serial.println("Error reading energy");
        return false;
    }

    // Obtener Medicion Frecuencia ---------------------------------------------------
    if (!ValidateData(fase.frequency(), struct_fase.FP, max_fase.FP))
    {
        Serial.println("Error reading frequency");
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
        _faseA = SimuMedicionMonofasica(DatosFaseA, MaxFaseA);
        _faseB = SimuMedicionMonofasica(DatosFaseB, MaxFaseB);
        _faseC = SimuMedicionMonofasica(DatosFaseC, MaxFaseC);
    }
    else
    {
        _faseA = GetMedicionMonofasica(faseA, DatosFaseA, MaxFaseA);
        _faseB = GetMedicionMonofasica(faseB, DatosFaseB, MaxFaseB);
        _faseC = GetMedicionMonofasica(faseC, DatosFaseC, MaxFaseC);
    }

    // Esta condicion nos indica que la medicion fue exitosa
    if (_faseA && _faseB && _faseC)
    {
        ValidateData((DatosFaseA.P + DatosFaseB.P + DatosFaseC.P), TriFase.P3, TriFase.P3_MAX);
        ValidateData((DatosFaseA.Q + DatosFaseB.Q + DatosFaseC.Q), TriFase.Q3, TriFase.Q3_MAX);
        ValidateData((DatosFaseA.S + DatosFaseB.S + DatosFaseC.S) / 1000, TriFase.S3, TriFase.S3_MAX);
        return PZEM_OK;
    }

    // Esta condicion nos indica que falla fase A
    else if (!_faseA && _faseB && _faseC)
    {
        return PZEM_ERROR_FASE_A;
    }

    // Esta condicion nos indica que falla fase B
    else if (_faseA && !_faseB && _faseC)
    {
        return PZEM_ERROR_FASE_B;
    }

    // Esta condicion nos indica que falla fase C
    else if (_faseA && _faseB && !_faseC)
    {
        return PZEM_ERROR_FASE_C;
    }

    // Esta condicion nos indica que falla fase A y B
    else if (!_faseA && !_faseB && _faseC)
    {
        return PZEM_ERROR_FASE_AB;
    }

    // Esta condicion nos indica que falla fase B y C
    else if (_faseA && !_faseB && !_faseC)
    {
        return PZEM_ERROR_FASE_BC;
    }

    // Esta condicion nos indica que falla fase A y C
    else if (!_faseA && _faseB && !_faseC)
    {
        return PZEM_ERROR_FASE_AC;
    }

    // Esta condicion nos indica que falla fase A B y C
    else if (!_faseA && !_faseB && !_faseC)
    {
        return PZEM_ERROR_FASE_ABC;
    }
}