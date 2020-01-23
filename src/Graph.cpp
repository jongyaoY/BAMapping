#include "Graph.h"

using namespace BAMapping;
void Graph::setNodesFromeFrames(FrameVector frameVector)
{
    auto Tc0w = frameVector.front().getConstTcw();
    for(int i = 0; i < frameVector.size(); i++)
    {
        auto frame = frameVector[i];
        if(frame.getObservations().empty())
            continue;

        Graph::Node node(Tc0w * frame.getConstTwc());

        node.rgb_path_ = frame.getRGBImagePath();
        node.depth_path_ = frame.getDepthImagePath();
        node.ir_path_ = frame.getInfraRedImagePath();
        for(auto obs : frame.getObservations())
        {
//            printf("%d %u \n", i, frame.mGlobalIndex);
            Graph::Edge edge(nodes_.size(),obs.first,obs.second);
            edges_.push_back(edge);
        }
        nodes_.push_back(node);
    }
}

void Graph::updateFrames(FrameVector &frameVector)
{
    for(int i = 0; i < nodes_.size(); i++)
    {
        Eigen::Affine3d Twc;
        Twc.matrix() = nodes_[i].pose_;
        frameVector[i].setFromAffine3d(Twc.inverse());
    }
}

void Graph::updatePoints(PointVector &pointVector)
{
    for(int i = 0; i < points_.size(); i++)
    {
        auto p = points_[i].pose_;
        pointVector[i].setPoint(p);
    }
}

PointVector Graph::copyPoints(const Eigen::Matrix4d Twc0)
{
    PointVector pointVector;

    for(int i = 0; i < points_.size(); i++)
    {
        auto p = points_[i].pose_;
        Eigen::Affine3d Twc0_affine;
        Twc0_affine.matrix() = Twc0;
        p = Twc0_affine * p;
        pointVector.push_back(BAMapping::Point(p));
    }
    return pointVector;
}

void Graph::setPoints(PointVector pointVector, Mat4 Tc0w)
{

    for(auto point : pointVector)
    {
        Graph::Point p(point.getPoseInFrame(Tc0w));
        points_.push_back(p);
    }
}

void Graph::setGraph(FrameVector frameVector, PointVector pointVector)
{
    auto Tc0w = frameVector.front().getConstTcw();
    for(int i = 0; i < frameVector.size(); i++)
    {
        auto frame = frameVector[i];
        if(frame.getObservations().empty())
            continue;

        Graph::Node node(Tc0w * frame.getConstTwc());

        node.rgb_path_ = frame.getRGBImagePath();
        node.depth_path_ = frame.getDepthImagePath();
        node.ir_path_ = frame.getInfraRedImagePath();
        for(auto obs : frame.getObservations())
        {
            Graph::Edge edge(nodes_.size(),obs.first,obs.second);
            edges_.push_back(edge);
        }
        nodes_.push_back(node);
    }

    for(auto& edge : edges_)
    {
        size_t global_point_id = edge.point_id_;
        auto point = pointVector[global_point_id];
        Graph::Point p(point.getPoseInFrame(Tc0w));
        edge.point_id_ = points_.size();
        p.global_id = points_.size();
        points_.push_back(p);
    }
}


FrameVector Graph::copyFrames(const Eigen::Matrix4d Twc0)
{
    FrameVector frameVector;
    for(int i = 0; i < nodes_.size(); i++)
    {
        Frame frame;
        auto Tc0cn = nodes_[i].pose_;
        Eigen::Affine3d Twc_affine;
        Twc_affine.matrix() = Twc0 * Tc0cn ;

        frame.setFromAffine3d(Twc_affine.inverse());
        frame.setImagePaths(nodes_[i].rgb_path_.c_str(),nodes_[i].depth_path_.c_str(),nodes_[i].ir_path_.c_str());
        frameVector.push_back(frame);
    }
    return frameVector;
}

std::vector<Graph> Graph::spliteIntoSubgraphs(const size_t n_nodes_per_graph, const size_t n_overlap, const Graph& graph)
{
    std::vector<Graph> subgraphs;
    for(size_t node_id = 0; node_id < graph.nodes_.size();)
    {
        Graph subgraph;
        auto Tc0w = graph.nodes_[node_id].pose_.inverse();
        for(size_t i = 0; i < n_nodes_per_graph&&node_id < graph.nodes_.size(); i++,node_id++)
        {
            auto node = graph.nodes_[node_id];
            node.pose_ = Tc0w * node.pose_;
            for(auto edge : graph.edges_)
            {
                if(edge.node_id_ == node_id)
                {
                    Edge edge_sub(subgraph.nodes_.size(),subgraph.points_.size(),edge.obs_);
                    Point point_sub = graph.points_[edge.point_id_];
                    subgraph.points_.push_back(point_sub);
                    subgraph.edges_.push_back(edge_sub);
                }
            }
            subgraph.nodes_.push_back(node);
        }
        subgraphs.push_back(subgraph);
        if(node_id >= graph.nodes_.size()-1)
            break;
        node_id -= n_overlap;
    }

    return subgraphs;
}