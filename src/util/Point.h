#ifndef POINT_H
#define POINT_H
#include <Eigen/Eigen>

class Point
{
public:
    typedef Eigen::Vector3f Pose;
    typedef Eigen::Vector3i Color;

    Point();
    ~Point();
    inline double* mutablePose() { return m_p; }

    Pose pose;
    double *m_p;
    Color color;
    float pointSize; //in meter
protected:
};

typedef std::vector<Point> PointVector;

#endif // POINT_H
