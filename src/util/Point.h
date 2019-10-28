#ifndef POINT_H
#define POINT_H
#include <Eigen/Eigen>

class Point
{
public:
    typedef Eigen::Vector3f Pose;
    typedef Eigen::Vector3i Color;

    Point();

    Pose p;
//    void setPose(Pose p);
//    void setColor(Color c);
//    inline Pose getPose(){return p;}
//    inline Color getColor(){return color;}
    Color color;
    float pointSize; //in meter
protected:
};

typedef std::vector<Point> PointVector;

#endif // POINT_H
