//
// Created by jojo on 06.12.19.
//

#ifndef BAMAPPING_GRAPH_H
#define BAMAPPING_GRAPH_H

#include <map>
#include <memory>
#include "Frame.h"
#include "Point.h"
#include "math/Types.h"
namespace BAMapping
{
    class Graph
    {
    public:
        static std::vector<Graph> spliteIntoSubgraphs(const size_t n_nodes_per_graph, const size_t n_overlap, const Graph& graph);
        static Graph generateGlobalGraph(const size_t n_overlap, const Graph& graph, const std::vector<Graph>& subgraphs);
        static Graph generateResultGraph(const size_t n_overlap, const Graph& globalgraph, const std::vector<Graph>& subgraphs);
        void setGraph(FrameVector frameVector, PointVector pointVector);
        void setNodesFromeFrames(FrameVector frameVector);
        void setPoints(PointVector pointVector, Mat4 Tc0w = Mat4::Identity());
        void updateFrames(FrameVector& frameVector);
        void updatePoints(PointVector& pointVector);
        PointVector copyPoints(const Eigen::Matrix4d Twc0);
        FrameVector copyFrames(const Eigen::Matrix4d Twc0);
        class Node
        {
        public:
            Node(Mat4 Twc) : pose_(Twc){}
            Mat4 pose_;
            std::string rgb_path_;
            std::string depth_path_;
            std::string ir_path_;
        };
        class Point
        {
        public:
            Point(Vec3 pose) : pose_(pose){ }
            Vec3 pose_;
            size_t global_id;
        };
        class Edge
        {
        public:
            Edge(size_t node_id, size_t point_id, Vec3 obs) :
                    node_id_(node_id),
                    point_id_(point_id),
                    obs_(obs){}
            size_t node_id_;
            size_t point_id_;
            Vec3 obs_;
        };
        std::vector<Node> nodes_;
        std::vector<Point> points_;
        std::vector<Edge> edges_;

    };

}//end of namspace



#endif //BAMAPPING_GRAPH_H
