#ifndef GRAPH_H
#define GRAPH_H

#include "Frame.h"
#include "Point.h"
class Graph
{
public:
    Graph();
    void getOptParameters(double *cam_param,double* point_param);
    void getOptParameters(double** cam_param, double** point_param);
    void update(double *cam_param,double* point_param);
    void update(double** cam_param,double** point_param);

    int getFrameBlockSize();
    int getPointBlockSize();

    void addFrame(const Frame frame);
    void addPoint(const Point point);
    void setFrames(FramePtrVector framePtrs);
    void setPoints(PointPtrMap pointPtrs);
    inline FramePtrVector& getFrames() {return m_framePtrs;}
    inline PointPtrMap& getPoints() {return m_pointPtrs;}

    const FrameVector getConstFrames();
    const PointVector getConstPoints();
private:
    FramePtrVector m_framePtrs;
    PointPtrMap m_pointPtrs;
    unsigned int m_numObs;

    int mPointBlockSize;
    int mFrameBlockSize;
};

#endif // GRAPH_H
