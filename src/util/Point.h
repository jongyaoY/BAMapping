#ifndef POINT_H
#define POINT_H
#include <Eigen/Eigen>

class Point
{
public:
    typedef Eigen::Vector3f Position;
    typedef Eigen::Vector3i Color;

    Point();
    Point(double x, double y, double z);
    ~Point();
    void setPoint(double x, double y, double z);
    void getMutable(double* param);
    const Position getConstPoint();

private:
    Position m_point;

};

typedef std::vector<Point> PointVector;
typedef std::map<unsigned int,Point*> PointPtrMap;

#endif // POINT_H
