#include "Graph.h"

Graph::Graph()
{

}
void Graph::addFrame(Frame frame)
{
    Frame* pFrame = new Frame(frame);
    m_framePtrs.push_back(pFrame);
}
void Graph::setFrames(FramePtrVector framePtrs)
{
    for(auto pFrame : framePtrs)
    {
        Frame* pframe = new Frame;
        pframe = pFrame;
        m_framePtrs.push_back(pframe);
    }
}

void Graph::setPoints(PointPtrMap pointPtrs)
{
    for(unsigned int i = 0; i < pointPtrs.size(); i++)
    {
        Point* pPoint = new Point;
        pPoint = pointPtrs[i];
        m_pointPtrs.emplace(i,pPoint);
    }
}

