//
// Created by jojo on 06.12.19.
//

#include "Graph.h"
using namespace BAMapping;
void Graph::addFrame(const Frame frame)
{
//    mFrameVec.push_back(frame);
}
void Graph::addPoint(const Point point)
{
//    mPointVec.push_back(point);
}

const FrameVector Graph::getConstFrames()
{
    FrameVector frameVec;

    return frameVec;
}

const PointVector Graph::getConstPoints()
{
    PointVector pointVec;
    return pointVec;
}

void Graph::getOptParameters(double **cam_param, double **point_param)
{
    int cam_size = mpFrameVec.size();
    int point_size = mpPointVec.size();

    double* cam;
    double* point;
    for(int cam_id = 0; cam_id < cam_size; cam_id++)
    {
        cam = cam_param[cam_id];
        mpFrameVec[cam_id]->getMutable(cam);
    }
    for(int point_id = 0; point_id < point_size; point_id++)
    {
        point = point_param[point_id];
        mpPointVec[point_id]->getMutable(point);
    }
}

void Graph::update(double **cam_param, double **point_param)
{

}

size_t Graph::getFrameVectorSize()
{
    return mpFrameVec.size();
}

size_t Graph::getPointVectorSize()
{
    return mpPointVec.size();
}

size_t Graph::getFrameBlockSize()
{
    return 6;
}

size_t Graph::getPointBlockSize()
{
    return 3;
}

const std::map<Graph::pair,Eigen::Vector3d> Graph::getEdges()
{
    return mEdges;
}
