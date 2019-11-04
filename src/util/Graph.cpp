#include "Graph.h"

Graph::Graph()
{
    m_numObs = 0;
}
void Graph::addFrame(const Frame frame)
{
    Frame* pFrame = new Frame(frame);
    m_framePtrs.push_back(pFrame);
    m_numObs += pFrame->getObservationSize();
}

void Graph::addPoint(const Point point)
{
    Point* p = new Point(point);
    unsigned int index = m_pointPtrs.size();
    m_pointPtrs.emplace(index,p);
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

const FrameVector Graph::getConstFrames()
{
    FrameVector frameVec;
    for(auto pFrame : m_framePtrs)
    {
        Frame frame = *pFrame;
        frameVec.push_back(frame);
    }
    return frameVec;
}

const PointVector Graph::getConstPoints()
{
    PointVector pointVec;
    for(auto idpPoint : m_pointPtrs)
    {
        Point point;
        point = *idpPoint.second;
        pointVec.push_back(point);
    }
    return pointVec;
}
