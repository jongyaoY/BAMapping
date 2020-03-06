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
        static std::vector<Graph> spliteIntoSubgraphs(const size_t n_nodes_per_graph, const Graph& graph);

        static Graph generateGlobalGraph(const size_t n_overlap, const Graph& graph, const std::vector<Graph>& subgraphs);
        static Graph generateGlobalGraph(const Graph& graph, const std::vector<Graph>& subgraphs);
        static Graph generateResultGraph(const size_t n_overlap, const Graph& globalgraph, const std::vector<Graph>& subgraphs);
        static void WriteToFile(const Graph& graph,const char* filename);
        static void ReadFromeFile(Graph& graph, const char* filename);
        void setGraph(FrameVector frameVector, PointVector pointVector);

        int isPointObserved(size_t global_point_id);
        PointVector copyPoints(const Eigen::Matrix4d Twc0) const;
        FrameVector copyFrames(const Eigen::Matrix4d Twc0) const;

        static void markSeperators(const Graph& global_graph,std::vector<Graph>& subgraphs);
        class Node
        {
        public:
            Node(Mat4 Twc) : pose_(Twc){}
            Mat4 pose_;
            int ite_frame_id;
            std::string rgb_path_;
            std::string depth_path_;
            std::string ir_path_;
        };
        class Point
        {
        public:
            Point(Vec3 pose) : pose_(pose),is_seperator_(false){ }
            Vec3 pose_;
            size_t global_id;
            bool is_seperator_;
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
