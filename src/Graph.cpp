//
// Created by jojo on 06.12.19.
//

#include "Graph.h"
using namespace BAMapping;

void Graph::addFrame(Frame* pFrame)
{
    auto observations = pFrame->getObservations();
    size_t frame_id = mpFrameVec.size();

    for(auto observation : observations)
    {
        size_t point_id = mpPointVec.size();
        size_t global_point_id = observation.first;
        Eigen::Vector3d obs = observation.second;
        auto ids = std::make_pair(frame_id,point_id);
        addPoint(mpGlobalIndexedPoints[global_point_id]);
        addEdge(ids,obs);
        mTrackedPointIndexes.push_back(global_point_id);
    }
    mpFrameVec.push_back(pFrame);
}

void Graph::addPoint(Point* pPoint)
{
    mpPointVec.push_back(pPoint);
}

const FrameVector Graph::getConstFrames()
{
    FrameVector frameVec;
    for(auto pFrame : mpFrameVec)
    {
        frameVec.push_back(*pFrame);
    }
    return frameVec;
}

const PointVector Graph::getConstPoints()
{
    PointVector pointVec;
    for(auto pPoint : mpPointVec)
    {
        pointVec.push_back(*pPoint);
    }
    return pointVec;
}

const FrameVector Graph::getConstGlobalFrames()
{
    FrameVector frameVec;
    for(auto indexedpFrame : mpGlobalIndexedFrames)
    {
        frameVec.push_back(*indexedpFrame.second);
    }

    return frameVec;
}

const PointVector Graph::getConstGlobalPoints()
{
    PointVector pointVec;
    for(auto indexpPoint : mpGlobalIndexedPoints)
    {
        pointVec.push_back(*indexpPoint.second);
    }
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
    for(int i = 0; i < mpFrameVec.size(); i++)
    {
        double* cam = cam_param[i];
        double norm = cam[0]*cam[0] + cam[1]*cam[1] + cam[2]*cam[2];
        norm = sqrt(norm);
        Eigen::AngleAxisd axisAngle(norm,Eigen::Vector3d(cam[0]/norm,cam[1]/norm,cam[2]/norm));

        mpFrameVec[i]->setAngleAxisAndPoint(axisAngle,Eigen::Vector3d(cam[3],cam[4],cam[5]));
    }
    for(int i = 0; i < mpPointVec.size(); i++)
    {
        double* point = point_param[i];
        mpPointVec[i]->setPoint(Eigen::Vector3d(point[0],point[1],point[2]));
    }
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

void Graph::addGlobalFrame(Frame frame)
{
    Frame* pFrame = new Frame(frame);//Todo copy frame
    auto observations = pFrame->getObservations();

    size_t global_frame_id = mpGlobalIndexedFrames.size();

    for(auto observation : observations)
    {
        size_t global_point_id = observation.first;
        Eigen::Vector3d obs = observation.second;
        auto ids = std::make_pair(global_frame_id,global_point_id);
        mObservations.emplace(ids,obs);
    }
    mpGlobalIndexedFrames.emplace(global_frame_id,pFrame);
}

void Graph::addEdge(const Graph::pair ids, const Eigen::Vector3d observation)
{
    mEdges.emplace(ids,observation);
}

void Graph::addGlobalPoint(const Point point)
{
    Point* pPoint = new Point(point);
    size_t point_id = mpGlobalIndexedPoints.size();
    mpGlobalIndexedPoints.emplace(point_id,pPoint);
}

void Graph::splitInto(unsigned int num_subgraphs)
{
    int frameProSubgraph = mpFrameVec.size()/num_subgraphs;
    int frame_id = 0;
    for(int subgraph_id = 0; subgraph_id < num_subgraphs; subgraph_id++)
    {
        Graph::Ptr pSubgraph(new Graph);
        pSubgraph->setParent(mPointer);
        mpChildGraphVec.push_back(pSubgraph);
        for(int i = 0; i < frameProSubgraph; i++, frame_id++)
        {
            pSubgraph->addFrame(mpFrameVec[frame_id]);
        }
    }
}

void Graph::setAsRootGraph(Graph* pRootGraph)
{
    if(pRootGraph!=NULL)
        mpRootGraph = pRootGraph;
    for(auto pIndexedFrame : mpGlobalIndexedFrames)
    {
        pRootGraph->addFrame(pIndexedFrame.second);
    }
}

void Graph::setParent(Graph::Ptr parent)
{
    mpParentGraph = parent;
}

Graph::Graph()
{
    mPointer = Ptr(this);
}

std::vector<Graph::Ptr> Graph::getSubmaps()
{
    return mpChildGraphVec;
}

Graph* Graph::mpRootGraph;
std::map<size_t,Frame*> Graph::mpGlobalIndexedFrames;
std::map<size_t,Point*> Graph::mpGlobalIndexedPoints;
std::map<Graph::pair,Eigen::Vector3d> Graph::mObservations;