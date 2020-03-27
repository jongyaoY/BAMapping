#include "Graph.h"
#include "rapidjson/document.h"
//#include "rapidjson/writer.h"
#include "rapidjson/prettywriter.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/filereadstream.h"
#include "rapidjson/filewritestream.h"
#include <iostream>
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

Graph Graph::generateGlobalGraph(const Graph &graph, const std::vector<Graph> &subgraphs)
{
    Graph globalGraph;

    int global_id = 0;
    Mat4 Twc = Mat4::Identity();
    for(int i = 0; i < subgraphs.size(); i++)
    {
        Node node = graph.nodes_[global_id];
        node.pose_ = Twc;
        globalGraph.nodes_.push_back(node);
        global_id += subgraphs[i].nodes_.size();
        Twc = Twc * subgraphs[i].nodes_.back().pose_;
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

        node.timeStamp_ = frame.getTimeStamp();
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

    for(auto point : pointVector)
    {
        Graph::Point p(point.getPoseInFrame(Tc0w));
        p.global_id = points_.size();
        points_.push_back(p);
    }
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

std::vector<Graph> Graph::spliteIntoSubgraphs(const size_t n_nodes_per_graph, const Graph &graph)
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

        frame.setTimeStamp(node.timeStamp_);
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

void Graph::WriteToFile(const Graph &graph, const char *filename)
{
    using namespace rapidjson;

    Document d;
    Document::AllocatorType& allocator = d.GetAllocator();
    d.SetObject();

    Value nodes(kArrayType);
    for(const auto& node : graph.nodes_)
    {
        Value node_val(kObjectType);
        Value pose(kArrayType);
        Value id;
        Value timeStamp;
        for(int i = 0; i < node.pose_.rows(); i++)
        {
            for(int j = 0; j < node.pose_.cols(); j++)
            {
                pose.PushBack(Value().SetDouble(node.pose_(i,j)),allocator);
            }

        }
        id.SetInt(node.ite_frame_id);
        timeStamp.SetDouble(node.timeStamp_);
        node_val.AddMember("id",id,allocator);
        node_val.AddMember("timeStamp",timeStamp,allocator);
        node_val.AddMember("pose",pose,allocator);
        node_val.AddMember("ir_path",Value().SetString(node.ir_path_.c_str(),node.ir_path_.length()),allocator);
        node_val.AddMember("rgb_path",Value().SetString(node.rgb_path_.c_str(),node.rgb_path_.length()),allocator);
        node_val.AddMember("depth_path",Value().SetString(node.depth_path_.c_str(),node.depth_path_.length()),allocator);

        nodes.PushBack(node_val,allocator);
    }

    Value points(kArrayType);
    for(const auto& point : graph.points_)
    {
        Value point_val(kObjectType);
        Value pose(kArrayType);
        point_val.AddMember("global_id",Value().SetInt(point.global_id),allocator);
        point_val.AddMember("is_seperator",Value().SetBool(point.is_seperator_),allocator);
        for(int i = 0; i < point.pose_.size(); i++)
            pose.PushBack(Value().SetDouble(point.pose_[i]),allocator);

        point_val.AddMember("pose",pose,allocator);
        points.PushBack(point_val,allocator);
    }
    Value edges(kArrayType);
    for(const auto& edge : graph.edges_)
    {
        Value edge_val(kObjectType);
        edge_val.AddMember("node_id",Value().SetInt(edge.node_id_),allocator);
        edge_val.AddMember("point_id",Value().SetInt(edge.point_id_),allocator);
        Value obs(kArrayType);
        for(int i = 0; i < edge.obs_.size(); i++)
        {
            obs.PushBack(Value().SetDouble(edge.obs_[i]),allocator);
        }
        edge_val.AddMember("obs",obs,allocator);
        edges.PushBack(edge_val,allocator);
    }

    d.AddMember("Nodes",nodes,allocator);
    d.AddMember("Points",points,allocator);
    d.AddMember("Edges",edges,allocator);

    FILE* pf = fopen(filename,"w");

    char writeBuffer[65536];
    FileWriteStream os(pf, writeBuffer, sizeof(writeBuffer));

//    StringBuffer buffer;
    PrettyWriter<FileWriteStream> writer(os);
    d.Accept(writer);
    fclose(pf);
//    fprintf(pf,"%s",buffer.GetString());
}

void Graph::ReadFromeFile(Graph &graph, const char *filename)
{
    using namespace rapidjson;
    FILE* pf = fopen(filename,"r");
    if(pf==nullptr)
    {
        printf("graph file doesn't exist\n");
        return;;
    }
    char readBuffer[65536];
    FileReadStream is(pf,readBuffer, sizeof(readBuffer));

    Document d;
    d.ParseStream(is);

    Value nodes = d["Nodes"].GetArray();
    Value points = d["Points"].GetArray();
    Value edges = d["Edges"].GetArray();
    printf("read graph file.\nnodes size: %i, points size: %i, edges size: %i\n",nodes.Size(),points.Size(),edges.Size());
    for(const auto& node_val : nodes.GetArray())
    {
        Mat4 pose;
        int ele = 0;
        for(const auto& i : node_val["pose"].GetArray())
        {
            int row = ele / 4;
            int col = ele % 4;
            pose(row,col) = i.GetDouble();
            ele++;
        }
        Node node(pose);
        node.ite_frame_id = node_val["id"].GetInt();
        node.timeStamp_ = node_val["timeStamp"].GetDouble();
        node.ir_path_ = node_val["ir_path"].GetString();
        node.rgb_path_ = node_val["rgb_path"].GetString();
        node.depth_path_ = node_val["depth_path"].GetString();

        graph.nodes_.push_back(node);
    }

    for(const auto& point_val : points.GetArray())
    {
        Vec3 pose;
        int i = 0;
        for(const auto& p : point_val["pose"].GetArray())
        {
            pose[i] = p.GetDouble();
            i++;
        }
        Point point(pose);
        point.global_id = point_val["global_id"].GetInt();
        point.is_seperator_ = point_val["is_seperator"].GetBool();
        graph.points_.push_back(point);
    }

    for(const auto& edge_val : edges.GetArray())
    {
        Vec3 obs;
        int i = 0;
        for(const auto& obs_ : edge_val["obs"].GetArray())
        {
            obs[i] = obs_.GetDouble();
            i++;
        }
        Edge edge(edge_val["node_id"].GetInt(),edge_val["point_id"].GetInt(),obs);
        graph.edges_.push_back(edge);
    }
    fclose(pf);
}