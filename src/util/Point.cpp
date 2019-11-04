#include "Point.h"

Point::Point():color(1,0,0)
{
    pointSize = 0.005;
    m_p = new double[3];
}

Point::Point(double x, double y, double z)
{
    m_p = new double[3];
    m_p[0] = x;
    m_p[1] = y;
    m_p[2] = z;
}
Point::~Point()
{
//    if(m_p != NULL)
//        delete[] m_p;
}

