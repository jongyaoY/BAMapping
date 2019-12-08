//
// Created by jojo on 06.12.19.
//

#ifndef BAMAPPING_GRAPH_H
#define BAMAPPING_GRAPH_H

#include <map>
#include <memory>
#include "Frame.h"
#include "Point.h"

namespace BAMapping
{
    class Graph
    {
    public:
        typedef std::pair<size_t ,size_t> pair;
        typedef std::shared_ptr<Graph> Ptr;
        Graph();
        static void setAsRootGraph(Graph* pRootGraph);
        static void addGlobalFrame(const Frame frame);
        static void addGlobalPoint(const Point point);

        void splitInto(const unsigned int num_subgraphs);
        void setParent(Ptr parent);
        void addFrame(Frame* pFrame);
        void addPoint(Point* pPoint);
        void addEdge(const pair ids,const Eigen::Vector3d observation);
        std::vector<Ptr> getSubmaps();

        const FrameVector getConstFrames();
        const PointVector getConstPoints();
        const FrameVector getConstGlobalFrames();
        const PointVector getConstGlobalPoints();

        void getOptParameters(double** cam_param, double** point_param);
        size_t getFrameVectorSize();
        size_t getPointVectorSize();

        size_t getFrameBlockSize();
        size_t getPointBlockSize();
        const std::map<pair,Eigen::Vector3d> getEdges();
        void update(double** cam_param,double** point_param);


    protected:
        //Todo
        //trim points that are observed by less then N frames
        void trimPointsAndEdges(unsigned int threshold);
        //
        void alignToBaseFrame();
        bool hasFrame(size_t FrameIndex);
        bool hasPoint(size_t PointIndex);

        Ptr mpParentGraph;
        std::vector<Ptr> mpChildGraphVec;
        FramePtrVector mpFrameVec;
        PointPtrVector mpPointVec;
        PointPtrVector mpSeparatorPointVec;
        std::vector<size_t> mTrackedPointIndexes;

//        (frame_id , point_id) -> observation
        std::map<pair,Eigen::Vector3d> mEdges;
    private:
        Ptr mPointer;

        //global
        static Graph* mpRootGraph;
        static std::map<size_t,Frame*> mpGlobalIndexedFrames;
        static std::map<size_t,Point*> mpGlobalIndexedPoints;
        static std::map<pair,Eigen::Vector3d> mObservations;
    };

}//end of namspace



#endif //BAMAPPING_GRAPH_H
