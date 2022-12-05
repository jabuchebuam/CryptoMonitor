#ifndef MODEL_HPP
#define MODEL_HPP

#include "stdint.h"

class ModelListener;

class Model
{
public:
    Model();

    void bind(ModelListener* listener)
    {
        modelListener = listener;
    }

    void tick();
    void GetValue();

    void ReadVar();

    uint32_t GetMinVal(uint8_t index);
    uint32_t GetMaxVal(uint8_t index);
    void SetMinVal(uint8_t index, uint32_t val);
    void SetMaxVal(uint8_t index, uint32_t val);
    uint32_t* Get_Vect_Val(uint8_t index);
protected:

    uint32_t min[10];
    uint32_t max[10];

    uint32_t vect_val[13];

    uint32_t data[10];
    uint32_t GetVarRTC();

    ModelListener* modelListener;
};

#endif // MODEL_HPP
