#include "Graph.h"

Graph::Graph()
{
    m_numObs = 0;
    mPointBlockSize = 3;
    mFrameBlockSize = Frame::getParamBlockSize();

}

Graph Graph::getSubGraph(unsigned int begin,unsigned int end)
{
    Graph subGraph;
    if(begin >= m_framePtrs.size() || end >= m_framePtrs.size())
        return subGraph;
    FrameVector frames = getConstFrames();
    for(int i = begin; i < end; i++)
    {
        subGraph.addFrame(frames[i]);
    }
    PointVector points = getConstPoints();
    for(auto point : points)
    {
        subGraph.addPoint(point);
    }

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


void Graph::getOptParameters(double *cam_param, double *point_param)
{
    int cam_size = 9*m_framePtrs.size();
    int point_size = 3*m_pointPtrs.size();

    double* cam;
    double* point;
    for(int cam_id = 0; cam_id < m_framePtrs.size(); cam_id++)
    {
        cam = cam_param + 9*cam_id;
        m_framePtrs[cam_id]->getMutable(cam);
    }
    for(int point_id = 0; point_id < m_pointPtrs.size(); point_id++)
    {
        point = point_param + 3*point_id;
        m_pointPtrs[point_id]->getMutable(point);
    }
}
void Graph::getOptParameters(double** cam_param,double** point_param,bool withIntrinsics)
{
    int cam_size = m_framePtrs.size();
    int point_size = m_pointPtrs.size();

    double* cam;
    double* point;
    for(int cam_id = 0; cam_id < m_framePtrs.size(); cam_id++)
    {
        cam = cam_param[cam_id];
        m_framePtrs[cam_id]->getMutable(cam,withIntrinsics);
    }
    for(int point_id = 0; point_id < m_pointPtrs.size(); point_id++)
    {
        point = point_param[point_id];
        m_pointPtrs[point_id]->getMutable(point);
    }
}

int Graph::getFrameBlockSize(bool withIntrinsics)
{
    if(!withIntrinsics)
        return 6;

    return mFrameBlockSize;
}
int Graph::getPointBlockSize()
{
    return mPointBlockSize;
}

void Graph::update(double *cam_param,double* point_param)
{
    for(int i = 0; i < m_framePtrs.size(); i++)
    {
        double* cam = cam_param + 9*i;
        double norm = cam[0]*cam[0] + cam[1]*cam[1] + cam[2]*cam[2];
        Eigen::AngleAxisd axisAngle(norm,Eigen::Vector3d(cam[0]/norm,cam[1]/norm,cam[2]/norm));

        m_framePtrs[i]->setAngleAxisAndPoint(axisAngle,Eigen::Vector3d(cam[3],cam[4],cam[5]));
    }

    for(int i = 0; i < m_pointPtrs.size(); i++)
    {
        double* point = point_param + 3*i;
        m_pointPtrs[i]->setPoint(point[0],point[1],point[2]);
    }

}
void Graph::update(double** cam_param,double** point_param)
{
    for(int i = 0; i < m_framePtrs.size(); i++)
    {
        double* cam = cam_param[i];
        double norm = cam[0]*cam[0] + cam[1]*cam[1] + cam[2]*cam[2];
        norm = sqrt(norm);
        Eigen::AngleAxisd axisAngle(norm,Eigen::Vector3d(cam[0]/norm,cam[1]/norm,cam[2]/norm));

        m_framePtrs[i]->setAngleAxisAndPoint(axisAngle,Eigen::Vector3d(cam[3],cam[4],cam[5]));
    }

    for(int i = 0; i < m_pointPtrs.size(); i++)
    {
        double* point = point_param[i];
        m_pointPtrs[i]->setPoint(point[0],point[1],point[2]);
    }
}
