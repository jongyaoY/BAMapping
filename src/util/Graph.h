#ifndef GRAPH_H
#define GRAPH_H

#include "Frame.h"
#include "Point.h"
class Graph
{
public:
    Graph();

    inline FrameVector* getFrames() {return &m_frameVec;}
    inline PointVector* getPoints() {return &m_pointVec;}
    int point_block_size;
    int frame_block_size;
private:
    FrameVector m_frameVec;
    PointVector m_pointVec;
};

#endif // GRAPH_H
