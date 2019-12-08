//
// Created by jojo on 06.12.19.
//

#include <iostream>
#include "Graph.h"
using namespace BAMapping;

void Graph::addFrame(Frame* pFrame)
{
    auto observations = pFrame->getObservations();
    size_t frame_id = mpFrameVec.size();

    for(auto observation : observations)
    {
        size_t global_point_id = observation.first;
        mTrackedPointIndexes.push_back(global_point_id);
    }
    mpFrameVec.push_back(pFrame);
}

void Graph::addPoint(size_t global_index,Point* pPoint)
{
    auto it = std::find(mTrackedPointIndexes.begin(),mTrackedPointIndexes.end(),global_index);
    if(it != mTrackedPointIndexes.end()) //point already added
        return;
    mpPointVec.push_back(pPoint);
}
void Graph::addPoints()
{
    mTrackedPointIndexes.sort();
    mTrackedPointIndexes.unique();
    for(auto global_point_index : mTrackedPointIndexes)
    {
        mpPointVec.push_back(mpGlobalIndexedPoints[global_point_index]);
    }
}

const FrameVector Graph::getLocalConstFrames()
{
    return mLocalFrameVec;
}

const PointVector Graph::getLocalConstPoints()
{
    return mLocalPointVec;
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
    Frame* pFrame = new Frame(frame);
    auto observations = pFrame->getObservations();

    size_t global_frame_id = mpGlobalIndexedFrames.size();
    pFrame->mGlobalIndex = global_frame_id;

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
    pPoint->mGlobalIndex = point_id;
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
        pSubgraph->addPoints();
        pSubgraph->addEdges();
        pSubgraph->alignToBaseFrame();
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

void Graph::alignToBaseFrame()
{
    Frame baseFrame;
    if(mpFrameVec.empty())
    {
        std::cout<<"mpFrameVec empty\n";
        return;
    }
    baseFrame = *mpFrameVec[0];
    auto Tcw = mpFrameVec[0]->getConstTcw();
    Eigen::Affine3d Tcw_affine;
    Tcw_affine.matrix() = Tcw;
    Eigen::Affine3d baseNode;
    baseNode.matrix() = Eigen::Matrix4d::Identity();
    baseFrame.setFromAffine3d(baseNode);
    mLocalFrameVec.push_back(baseFrame);
    for(int i = 1; i < mpFrameVec.size(); i++)
    {
        Frame localFrame = *mpFrameVec[i];
        auto Twi = mpFrameVec[i]->getConstTwc();
        Eigen::Affine3d Tci;
        Tci.matrix() = Tcw * Twi;
        localFrame.setFromAffine3d(Tci);
        mLocalFrameVec.push_back(localFrame);
    }
    for(int i = 0; i<mpPointVec.size(); i++)
    {
        Point localPoint = *mpPointVec[i];
        Eigen::Vector3d localPose = Tcw_affine * localPoint.getPoseInWorld();
        localPoint.setPoint(localPose);
        mLocalPointVec.push_back(localPoint);
    }

}

void Graph::trimPointsAndEdges(unsigned int threshold)
{
    auto it = mpPointVec.begin();
    for(;it!=mpPointVec.end();it++)
    {
        auto pPoint = *it;
        if(pPoint->getObservationSize()<threshold)
        {
            //todo
        }
    }
}

void Graph::addEdges()
{
    size_t point_id = 0;
    for(auto pPoint: mpPointVec)
    {
        auto global_point_id = pPoint->mGlobalIndex;
        size_t frame_id = 0;
        for(auto pFrame : mpFrameVec)
        {
            if(pFrame->isPointObserved(global_point_id))
            {
                auto obs = pFrame->getObservationByPointIndex(global_point_id);
                mEdges.emplace(std::make_pair(frame_id,point_id),obs);
            }
            frame_id++;
        }
        point_id++;
    }

}

void Graph::addObservationsToPoints()
{
    for(auto observation : mObservations)
    {
        size_t global_frame_id = observation.first.first;
        size_t global_point_id = observation.first.second;
        auto obs = observation.second;
        mpGlobalIndexedPoints[global_point_id]->addObservation(global_frame_id,obs);
    }
}

Graph* Graph::mpRootGraph;
std::map<size_t,Frame*> Graph::mpGlobalIndexedFrames;
std::map<size_t,Point*> Graph::mpGlobalIndexedPoints;
std::map<Graph::pair,Eigen::Vector3d> Graph::mObservations;