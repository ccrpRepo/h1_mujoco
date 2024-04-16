#ifndef _PINODY_H_
#define _PINODY_H_

#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/rnea.hpp>

struct Pinody
{
public: 
    Pinody(pinocchio::Model* model,
           pinocchio::Data* data)
    {
        _model = model;
        _data = data;
    }

    ~Pinody(){}

    pinocchio::Model* _model;
    pinocchio::Data* _data;

private: 
};


#endif