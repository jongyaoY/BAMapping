#include "Point.h"

Point::Point():color(1,0,0)
{
    pointSize = 0.005;
    m_p = new double[3];
}

Point::~Point()
{
    delete[] m_p;
}

