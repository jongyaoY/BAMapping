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
        mLocalFrameVec[cam_id].getMutable(cam);
    }
    for(int point_id = 0; point_id < point_size; point_id++)
    {
        point = point_param[point_id];
        mLocalPointVec[point_id].getMutable(point);

    }
}

void Graph::updateLocal(double **cam_param, double **point_param)
{
    for(int i = 0; i < mpFrameVec.size(); i++)
    {
        double* cam = cam_param[i];
        double norm = cam[0]*cam[0] + cam[1]*cam[1] + cam[2]*cam[2];
        norm = sqrt(norm);
        if(norm==0)
        {
            Eigen::AngleAxisd axisAngle(0,Eigen::Vector3d(0,0,0));
            mLocalFrameVec[i].setAngleAxisAndPoint(axisAngle,Eigen::Vector3d(cam[3],cam[4],cam[5]));
        }
        else
        {
            Eigen::AngleAxisd axisAngle(norm,Eigen::Vector3d(cam[0]/norm,cam[1]/norm,cam[2]/norm));
            mLocalFrameVec[i].setAngleAxisAndPoint(axisAngle,Eigen::Vector3d(cam[3],cam[4],cam[5]));
        }

    }
    for(int i = 0; i < mpPointVec.size(); i++)
    {
        double* point = point_param[i];
        mLocalPointVec[i].setPoint(Eigen::Vector3d(point[0],point[1],point[2]));
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
        pSubgraph->mGraphIndex = subgraph_id;
        pSubgraph->alignToBaseFrame();
    }


    findSeparatorPoints();
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
    auto Twc0 = mpFrameVec[0]->getConstTwc();
    auto Tc0w = mpFrameVec[0]->getConstTcw();
    Eigen::Affine3d Tc0w_affine;
    Tc0w_affine.matrix() = Tc0w;
    Eigen::Affine3d baseNode;
    baseNode.matrix() = Eigen::Matrix4d::Identity();
    baseFrame.setFromAffine3d(baseNode);
    mLocalFrameVec.push_back(baseFrame);
    for(int i = 1; i < mpFrameVec.size(); i++)
    {
        Frame localFrame = *mpFrameVec[i];
        auto Tciw = mpFrameVec[i]->getConstTcw();
        Eigen::Affine3d Tcic0;
        Tcic0.matrix() = Tciw * Twc0;
        localFrame.setFromAffine3d(Tcic0);
        mLocalFrameVec.push_back(localFrame);
    }
    for(int i = 0; i<mpPointVec.size(); i++)
    {
        auto pPoint = mpPointVec[i]->getpMirrorPointWithAffine3d(Tc0w_affine,mGraphIndex);

        mLocalPointVec.push_back(*pPoint);
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
const std::list<size_t> Graph::getTrackedPointIndexes()
{
    return mTrackedPointIndexes;
}

void Graph::findSeparatorPoints()
{
    std::list<size_t> result_list;
    for(int i = 0;i<mpChildGraphVec.size()-1;i++)
    {
        for(int j = i+1;j<mpChildGraphVec.size();j++)
        {
            std::list<size_t> list1 = mpChildGraphVec[i]->getTrackedPointIndexes();
            std::list<size_t> list2 = mpChildGraphVec[j]->getTrackedPointIndexes();
            std::list<size_t> intersect_list;
            std::set_intersection(list1.begin(),list1.end(),
                                  list2.begin(),list2.end(),
                                  std::back_inserter(intersect_list));
            result_list.insert(result_list.end(),intersect_list.begin(),intersect_list.end());
        }
    }
    for(auto global_point_id : result_list)
    {
        mpSeparatorPointVec.push_back(mpGlobalIndexedPoints[global_point_id]);
    }
}

Graph::Ptr Graph::getParent() const
{
    return mpParentGraph;
}
std::vector<Graph::Ptr> Graph::getChildren() const
{
    return mpChildGraphVec;
}
std::vector<Point*> Graph::getpSeparators() const
{
    return mpSeparatorPointVec;
}

void Graph::getInterGraphOptParameters(double** cam_param, double** point_param)
{
    int cam_size = mpChildGraphVec.size();
    int point_size = mpSeparatorPointVec.size();

    double* cam;
    double* point;
    for(int cam_id = 0; cam_id < cam_size; cam_id++)
    {
        cam = cam_param[cam_id];
        mpChildGraphVec[cam_id]->getBaseFrame().getMutable(cam);
    }
    for(int point_id = 0; point_id < point_size; point_id++)
    {
        point = point_param[point_id];
        mpSeparatorPointVec[point_id]->getMutable(point);
    }
}

Frame Graph::getBaseFrame()
{
    if(!mLocalFrameVec.empty())
    {
        auto baseFrame = mLocalFrameVec[0];
        //base frame must be transformed back to global coordinate
        auto global_id = baseFrame.mGlobalIndex;
        auto Tc0w = mpGlobalIndexedFrames[global_id]->getConstTcw();
        

        auto Tc0_w = Tc0_c0 * Tc0w;
        Eigen::Affine3d Tc0_w_affine;
        Tc0_w_affine.matrix() = Tc0_w;
        baseFrame.setFromAffine3d(Tc0_w_affine);
//        return baseFrame;
        return *mpGlobalIndexedFrames[global_id];//todo
    }

}

void Graph::addInterObservations()
{
    size_t point_id = 0;
    for(auto pPoint : mpSeparatorPointVec)
    {
        for(auto pIndexedLocalPoint : pPoint->mpLocalMirrorPoints)
        {
            size_t graph_id = pIndexedLocalPoint.first;
            auto obs = pIndexedLocalPoint.second->getPoseInWorld();// actually returns the pose relative to base node--inter-measurement
            mInterObservations.emplace(std::make_pair(graph_id,point_id),obs);
        }
        point_id++;
    }
}

size_t Graph::getSupGraphSize()
{
    return mpChildGraphVec.size();
}

size_t Graph::getSeparatorSize()
{
    return mpSeparatorPointVec.size();
}

const std::map<Graph::pair,Eigen::Vector3d> Graph::getInterObservations()
{
    return mInterObservations;
}

void Graph::updateGlobal(double **cam_param, double **point_param)
{
    for(int i = 0; i < mpChildGraphVec.size(); i++)
    {
        double* cam = cam_param[i];
        double norm = cam[0]*cam[0] + cam[1]*cam[1] + cam[2]*cam[2];
        norm = sqrt(norm);
        auto pSubGraph = mpChildGraphVec[i];
        if(norm==0)
        {
            Eigen::AngleAxisd axisAngle(0,Eigen::Vector3d(0,0,0));
            pSubGraph->setBaseFramePoseByAngleAxisAndPoint(axisAngle,Eigen::Vector3d(cam[3],cam[4],cam[5]));
        }
        else
        {
            Eigen::AngleAxisd axisAngle(norm,Eigen::Vector3d(cam[0]/norm,cam[1]/norm,cam[2]/norm));
            pSubGraph->setBaseFramePoseByAngleAxisAndPoint(axisAngle,Eigen::Vector3d(cam[3],cam[4],cam[5]));
        }

    }
    for(int i = 0; i < mpPointVec.size(); i++)
    {
        double* point = point_param[i];
        mpSeparatorPointVec[i]->setPoint(Eigen::Vector3d(point[0],point[1],point[2]));
    }
}

void Graph::applyLocal()
{
    Frame baseFrame;
    if(mpFrameVec.empty())
    {
        std::cout<<"mpFrameVec empty\n";
        return;
    }
    baseFrame = mLocalFrameVec[0];
    auto Twc0 = mpFrameVec[0]->getConstTwc();
    auto Tc0w = mpFrameVec[0]->getConstTcw();
    auto Tc0_c0 = baseFrame.getConstTcw(); //c0_ represents the changed camera c0'
    auto Tc0c0_ = baseFrame.getConstTwc();

    Twc0 = Twc0 * Tc0c0_;
    Tc0w = Tc0_c0 * Tc0w;
    Eigen::Affine3d Tc0w_affine;
    Eigen::Affine3d Twc0_affine;
    Tc0w_affine.matrix() = Tc0w;
    Twc0_affine.matrix() = Twc0;

    mpFrameVec[0]->setFromAffine3d(Tc0w_affine);
    for(int i = 1; i < mpFrameVec.size(); i++)
    {
        Frame localFrame = mLocalFrameVec[i];
        auto Tcic0 = localFrame.getConstTcw();
        Eigen::Affine3d Tciw;

        Tciw = Tcic0 * Tc0w;
        mpFrameVec[i]->setFromAffine3d(Tciw);
    }
    for(int i = 0; i<mpPointVec.size(); i++)
    {
        auto point = mLocalPointVec[i];//todo not update separators
        auto c0P = point.getPoseInWorld(); // pose relative to base node
        auto wP = Twc0_affine * c0P;
        mpPointVec[i]->setPoint(wP);
    }
}

void Graph::applyGlobal()
{
    for(auto pSubGraph : mpChildGraphVec)
    {
        pSubGraph->applyLocal();
    }
}

void Graph::setBaseFramePoseByAngleAxisAndPoint(Eigen::AngleAxisd angleAxis, Eigen::Vector3d point)
{
    Eigen::Affine3d I;
    I.matrix() = Eigen::Matrix4d::Identity();
    mLocalFrameVec[0].setFromAffine3d(I);
//    auto R = angleAxis.inverse().matrix();
//    point = R*point;
//    point = -point;
//    mpFrameVec[0]->setAngleAxisAndPoint(angleAxis.inverse(),point);

    mpFrameVec[0]->setAngleAxisAndPoint(angleAxis,point);
}

Graph* Graph::mpRootGraph;
std::map<size_t,Frame*> Graph::mpGlobalIndexedFrames;
std::map<size_t,Point*> Graph::mpGlobalIndexedPoints;
std::map<Graph::pair,Eigen::Vector3d> Graph::mObservations;