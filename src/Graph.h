//
// Created by jojo on 06.12.19.
//

#ifndef BAMAPPING_GRAPH_H
#define BAMAPPING_GRAPH_H

#include <map>

#include "Frame.h"
#include "Point.h"

namespace BAMapping
{
    class Graph
    {
    public:
        typedef std::pair<size_t ,size_t> pair;

        Graph() = default;
        void addFrame(const Frame frame);
        void addPoint(const Point point);
        void addEdge(const pair ids,const Eigen::Vector3d observation);

        const FrameVector getConstFrames();
        const PointVector getConstPoints();

        void getOptParameters(double** cam_param, double** point_param);
        size_t getFrameVectorSize();
        size_t getPointVectorSize();

        size_t getFrameBlockSize();
        size_t getPointBlockSize();
        const std::map<pair,Eigen::Vector3d> getEdges();
        void update(double** cam_param,double** point_param);


    protected:
        //trim points that are observed by less then N frames
        void trimPointsAndEdges(unsigned int threshold);
        //
        void alignToBaseFrame();
        bool hasFrame(size_t FrameIndex);
        bool hasPoint(size_t PointIndex);

        Graph* mParentGraph;
        std::vector<Graph*> mChildGraphVec;
        FramePtrVector mpFrameVec;
        PointPtrVector mpPointVec;
//        FrameVector mFrameVec;
//        PointVector mPointVec;
        //(frame_id , point_id) -> observation
        std::map<pair,Eigen::Vector3d> mEdges;

        //global
        static std::map<size_t,Frame*> mpIndexedFrames;
        static std::map<size_t,Point*> mpIndexedPoints;
        static std::map<pair,Eigen::Vector3d> mObservations;
    };

}//end of namspace



#endif //BAMAPPING_GRAPH_H
