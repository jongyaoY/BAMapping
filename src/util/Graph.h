#ifndef GRAPH_H
#define GRAPH_H

#include "Frame.h"
#include "Point.h"
class Graph
{
public:
    Graph();
    void addFrame(Frame frame);
    void setFrames(FramePtrVector framePtrs);
    void setPoints(PointPtrMap pointPtrs);
    inline FramePtrVector& getFrames() {return m_framePtrs;}
    inline PointPtrMap& getPoints() {return m_pointPtrs;}
    int point_block_size;
    int frame_block_size;
private:
    FramePtrVector m_framePtrs;
    PointPtrMap m_pointPtrs;
};

#endif // GRAPH_H
