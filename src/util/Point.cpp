#include "Point.h"

Point::Point():color(1,0,0),pointSize(0.005),m_point(0,0,0)
{
}

Point::Point(double x, double y, double z):color(1,0,0),
                                           pointSize(0.005),
                                           m_point(x,y,z)
{
}
Point::~Point()
{

}


const Point::Position Point::getConstPoint()
{
    return m_point;
}

void Point::getMutable(double* param)
{
    *(param + 0) = m_point[0];
    *(param + 1) = m_point[1];
    *(param + 2) = m_point[2];
}

void Point::setPoint(double x, double y, double z)
{
    m_point = Position(x,y,z);
}
