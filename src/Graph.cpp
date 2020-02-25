#include "Graph.h"

using namespace BAMapping;

Graph Graph::generateGlobalGraph(const size_t n_overlap, const Graph &graph, const std::vector<Graph> &subgraphs)
{
    Graph globalGraph;
    size_t n_nodes_per_graph = subgraphs.front().nodes_.size();
    for(size_t node_id = 0; node_id < graph.nodes_.size();)
    {
        auto Twc0 = graph.nodes_[node_id].pose_;
        Node node(Twc0);
        globalGraph.nodes_.push_back(node);
        node_id += n_nodes_per_graph;
        node_id -= n_overlap;
    }

    std::vector<std::list<size_t>> subgraph_point_id;
    subgraph_point_id.reserve(globalGraph.nodes_.size());
    for(size_t node_id = 0; node_id < globalGraph.nodes_.size(); node_id++)
    {
        auto subgraph = subgraphs[node_id];
        std::list<size_t> point_ids;
        for(auto point : subgraph.points_)
        {
            Edge edge(node_id,point.global_id,point.pose_);
            globalGraph.edges_.push_back(edge);

            point_ids.push_back(point.global_id);
        }
        subgraph_point_id.push_back(point_ids);
    }

    globalGraph.points_ = graph.points_;
    std::list<size_t > seperater_ids;
    for(int i = 0; i < subgraph_point_id.size(); i++)
    {
        for(int j = i + 1; j < subgraph_point_id.size(); j++)
        {
            std::list<size_t> list_i = subgraph_point_id[i];
            std::list<size_t> list_j = subgraph_point_id[j];
            list_i.sort();
            list_j.sort();
            std::list<size_t> intersect;
            std::set_intersection(
                    list_i.begin(),list_i.end(),
                    list_j.begin(),list_j.end(),
                    std::back_inserter(intersect));
            for(auto id : intersect)
            {
                globalGraph.points_[id].is_seperator_ = true;
            }
        }
    }


    return globalGraph;
}

Graph Graph::generateResultGraph(const size_t n_overlap, const Graph& globalgraph, const std::vector<Graph>& subgraphs)
{
    Graph resultGraph;
    resultGraph.points_ = globalgraph.points_;
    bool is_first_graph = true;
    for(size_t graph_id = 0; graph_id < subgraphs.size(); graph_id++)
    {
        const Graph& graph = subgraphs[graph_id];
        if(is_first_graph)
        {
            for(size_t i = 0; i < graph.nodes_.size(); i++)
            {
                auto node = graph.nodes_[i];
                auto Twc0 = globalgraph.nodes_[graph_id].pose_;
                node.pose_ = Twc0 * node.pose_;
                for(const Edge& edge : graph.edges_)
                {
                    if(i == edge.node_id_)
                    {
                        size_t globalPoint_id = graph.points_[edge.point_id_].global_id;
                        Edge globalEdge(resultGraph.nodes_.size(),globalPoint_id,edge.obs_);
                        resultGraph.edges_.push_back(edge);
                    }
                }
                resultGraph.nodes_.push_back(node);

            }
            is_first_graph = false;
        }
        else
        {
            for(size_t i = n_overlap; i < graph.nodes_.size(); i++)
            {
                auto node = graph.nodes_[i];
                auto Twc0 = globalgraph.nodes_[graph_id].pose_;
                node.pose_ = Twc0 * node.pose_;

                for(const Edge& edge : graph.edges_)
                {
                    if(i == edge.node_id_)
                    {
                        size_t globalPoint_id = graph.points_[edge.point_id_].global_id;
                        Edge globalEdge(resultGraph.nodes_.size(),globalPoint_id,edge.obs_);
                        resultGraph.edges_.push_back(edge);
                    }
                }
                resultGraph.nodes_.push_back(node);
            }
        }
    }

    return resultGraph;
}

void Graph::markSeperators(const Graph& global_graph,std::vector<Graph>& subgraphs)
{
    std::list<size_t> sep_ids;
    for(auto point : global_graph.points_)
    {
        if(point.is_seperator_)
        {
            sep_ids.push_back(point.global_id);
        }
    }
    for(auto& subgraph : subgraphs)
    {
        for(auto id : sep_ids)
        {
            auto local_id = subgraph.isPointObserved(id);
            if(local_id != -1)
            {
                subgraph.points_[local_id].is_seperator_ = true;
            }
        }
    }
}

void Graph::setGraph(FrameVector frameVector, PointVector pointVector)
{
    if(frameVector.empty())
    {
        printf("frame vector empty\n");
        return;
    }
    auto Tc0w = frameVector.front().getConstTcw();
    for(int i = 0; i < frameVector.size(); i++)
    {
        auto frame = frameVector[i];
//        if(frame.getObservations().empty())
//            continue;

        Graph::Node node(Tc0w * frame.getConstTwc());

        node.ite_frame_id = frame.mITEId;
        node.rgb_path_ = frame.getRGBImagePath();
        node.depth_path_ = frame.getDepthImagePath();
        node.ir_path_ = frame.getInfraRedImagePath();
        auto obs_vec = frame.getObservations();
        for(auto obs : obs_vec)
        {
            Graph::Edge edge(nodes_.size(),obs.first,obs.second);
            edges_.push_back(edge);
        }
        nodes_.push_back(node);
    }
//    for(size_t i = 0; i < frameVector.size(); i++)
//    {
//        auto frame = frameVector[i];
//        auto obs_vec = frame.getObservations();
//
//        for(auto obs : obs_vec)
//        {
//            Graph::Edge edge(i,obs.first,obs.second);
//            edges_.push_back(edge);
//        }
//    }

    for(auto point : pointVector)
    {
        Graph::Point p(point.getPoseInFrame(Tc0w));
        p.global_id = points_.size();
        points_.push_back(p);
    }
//    std::vector<size_t> global_point_id_vec;
//    for(auto& edge : edges_)
//    {
//        size_t global_point_id = edge.point_id_;
//        int local_id = -1;
//        for(int i = 0; i < global_point_id_vec.size(); i++)
//        {
//            if(global_point_id == global_point_id_vec[i])
//            {
//                local_id = i;
//                break;
//            }
//        }
//
//        if(local_id != -1)
//        {
//            edge.point_id_ = local_id;
//        }
//        else
//        {
//            auto point = pointVector[global_point_id];
//            Graph::Point p(point.getPoseInFrame(Tc0w));
//            edge.point_id_ = points_.size();
//            p.global_id = points_.size();
//            points_.push_back(p);
//            global_point_id_vec.push_back(global_point_id);
//        }
//    }
}

int Graph::isPointObserved(size_t global_point_id)
{
    for(int i = 0; i < points_.size(); i++)
    {
        auto point = points_[i];
        if(point.global_id == global_point_id)
            return i;
    }

    return -1;
}

std::vector<Graph> Graph::spliteIntoSubgraphs(const size_t n_nodes_per_graph, const size_t n_overlap, const Graph& graph)
{
    std::vector<Graph> subgraphs;
    for(size_t node_id = 0; node_id < graph.nodes_.size();)
    {
        Graph subgraph;
        auto Tc0w = graph.nodes_[node_id].pose_.inverse();
        Eigen::Affine3d Tc0w_aff;
        Tc0w_aff.matrix() = Tc0w;
        for(size_t i = 0; i < n_nodes_per_graph&&node_id < graph.nodes_.size(); i++,node_id++)
        {
            auto node = graph.nodes_[node_id];
            node.pose_ = Tc0w * node.pose_;
            for(auto edge : graph.edges_)
            {
                if(edge.node_id_ == node_id)
                {
                    auto global_id = graph.points_[edge.point_id_].global_id;
                    auto local_id = subgraph.isPointObserved(global_id);
                    if(local_id != -1)
                    {
                        Edge edge_sub(subgraph.nodes_.size(),local_id,edge.obs_);
                        subgraph.edges_.push_back(edge_sub);
                    }
                    else
                    {
                        Point point_sub = graph.points_[edge.point_id_];
                        point_sub.pose_ = Tc0w_aff * point_sub.pose_;
                        Edge edge_sub(subgraph.nodes_.size(),subgraph.points_.size(),edge.obs_);
                        subgraph.points_.push_back(point_sub);
                        subgraph.edges_.push_back(edge_sub);
                    }
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

FrameVector Graph::copyFrames(const Eigen::Matrix4d Twc0) const
{
    FrameVector frameVector;
    for(int i = 0; i < nodes_.size(); i++)
    {
        Frame frame;
        auto node = nodes_[i];
        auto Tc0cn = nodes_[i].pose_;
        Eigen::Affine3d Twc_affine;
        Twc_affine.matrix() = Twc0 * Tc0cn ;

        frame.mITEId = node.ite_frame_id;
        frame.setFromAffine3d(Twc_affine.inverse());
        frame.setImagePaths(nodes_[i].rgb_path_.c_str(),nodes_[i].depth_path_.c_str(),nodes_[i].ir_path_.c_str());
        frameVector.push_back(frame);
    }
    return frameVector;
}

PointVector Graph::copyPoints(const Eigen::Matrix4d Twc0) const
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
