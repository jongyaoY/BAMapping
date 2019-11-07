#ifndef VIEWER_H
#define VIEWER_H

#include <pangolin/pangolin.h>
#include <Eigen/Core>

#include "util/Point.h"
#include "util/Frame.h"

class Viewer
{
public:
    Viewer();
    void visualize();
    inline void setRefPoints(PointVector points){m_refPoints = points;}
    inline void setPoints(PointVector points){m_points = points;}
    inline void setFrames(FrameVector frames){m_frames = frames;}
private:
    void drawPoint(const Point point);
    void drawRefPoint(const Point point);
    void drawFrame(const Frame frame);

    PointVector m_points;
    PointVector m_refPoints;
    FrameVector m_frames;
};

#endif // VIEWER_H
