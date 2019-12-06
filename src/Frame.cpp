//
// Created by jojo on 06.12.19.
//

#include "Frame.h"

using namespace BAMapping;

void Frame::getMutable(double* param)
{
    for(int i = 0; i < 3; i++)
    {
        *(param + i) = m_angleAxis.angle()*m_angleAxis.axis()[i];
    }
    for(int i = 0; i < 3; i++)
    {
        *(param + i + 3) = m_translation[i];
    }
}
