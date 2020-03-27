#include "Writer.h"

using namespace BAMapping;

Writer::Writer()
{

}

void Writer::writeToFile(const Graph& graph,const char* cam_filename,const char* point_filename, const char* image_path_file)
{

}

void Writer::writeToFileTUMFormat(const Graph &graph, const char *cam_filename)
{
    FILE* fptr = fopen(cam_filename, "w");
    float fake_timestamp = 0;
    for (auto node : graph.nodes_)
    {

        auto Twc = node.pose_;
        Eigen::Quaterniond q(Twc.block<3,3>(0,0));
        Eigen::Vector3d translation = Twc.block<3,1>(0,3);

        fprintf(fptr,"%lf %lf %lf %lf %lf %lf %lf %lf\n",
                node.timeStamp_,
                translation[0],
                translation[1],
                translation[2],
                q.x(),
                q.y(),
                q.z(),
                q.w()
        );
        fake_timestamp++;
    }

    fclose(fptr);
}

void Writer::writeToFileITEFormat(const Graph &graph, const char *cam_filename, const char *point_filename)
{
    FILE *fptr = fopen(point_filename, "w");

    if (fptr == NULL) {
        return;
    }
    PointVector points = graph.copyPoints(Eigen::Matrix4d::Identity());
    FrameVector frames = graph.copyFrames(Eigen::Matrix4d::Identity());

    for (auto point : points) {
        fprintf(fptr, "%lf %lf %lf\n", point.getPoseInWorld()[0],
                point.getPoseInWorld()[1],
                point.getPoseInWorld()[2]);
    }
    fclose(fptr);

    fptr = fopen(cam_filename, "w");

    for (auto node : graph.nodes_)
    {
//        auto ir_path = node.ir_path_;
//        auto rgb_path = node.rgb_path_;
//        auto depth_path = node.depth_path_;
//        fprintf(fptr_img,"%s %s %s\n",ir_path.c_str(),rgb_path.c_str(),depth_path.c_str());
        auto Twc = node.pose_;
        Eigen::Quaterniond q(Twc.block<3,3>(0,0));
        Eigen::Vector3d translation = Twc.block<3,1>(0,3);

        fprintf(fptr,"%d %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
                node.ite_frame_id,
                q.w(),
                q.x(),
                q.y(),
                q.z(),
                translation[0],
                translation[1],
                translation[2],
                0.0,
                0.0);
    }

    fclose(fptr);
}

void Writer::writePoses(const FrameVector &frameVector, const char *cam_filename)
{
    FILE* fptr = fopen(cam_filename, "w");

    for(const auto& frame : frameVector)
    {
        auto Twc = frame.getConstTwc();
        Eigen::Quaterniond q(Twc.block<3,3>(0,0));
        Eigen::Vector3d translation = Twc.block<3,1>(0,3);
        fprintf(fptr,"%lu %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
                frame.mITEId,
                q.w(),
                q.x(),
                q.y(),
                q.z(),
                translation[0],
                translation[1],
                translation[2],
                0.0,
                0.0);
    }
    fclose(fptr);

}

void Writer::writeObservations(const FrameVector &frameVector, const char *obs_filename)
{
    FILE* fptr = fopen(obs_filename, "w");
    for(const auto& frame : frameVector)
    {
        auto frame_id = frame.mITEId;
        for(const auto& obs : frame.mObservations)
        {
            auto point_id = obs.first;
            auto obs_vec = obs.second;
            int u = static_cast<int>(obs_vec[0]);
            int v = static_cast<int>(obs_vec[1]);
            float d = static_cast<float>(obs_vec[2]);

            fprintf(fptr,"%lu %lu %d %d %f\n",point_id,frame_id,u,v,d);
        }
    }
    fclose(fptr);
}

void Writer::writeObservationsNormal(const FrameVector &frameVector, const char *obs_filename)
{
    FILE* fptr = fopen(obs_filename, "w");
    size_t frame_id = 0;
    for(const auto& frame : frameVector)
    {

        for(const auto& obs : frame.mObservations)
        {
            auto point_id = obs.first;
            auto obs_vec = obs.second;
            int u = static_cast<int>(obs_vec[0]);
            int v = static_cast<int>(obs_vec[1]);
            float d = static_cast<float>(obs_vec[2]);

            fprintf(fptr,"%lu %lu %d %d %f\n",point_id,frame_id,u,v,d);
        }
        frame_id++;
    }
    fclose(fptr);
}

void Writer::writePoints(const std::vector<MapPoint> points, const char *point_filename)
{
    FILE* fptr = fopen(point_filename, "w");
    for(int i = 0; i < points.size(); i++)
    {
        const auto& point = points[i];
        auto x = static_cast<float> (point.pose_[0]);
        auto y = static_cast<float> (point.pose_[1]);
        auto z = static_cast<float> (point.pose_[2]);

        fprintf(fptr,"%lu %f %f %f\n", point.id,x,y,z);
    }
    fclose(fptr);
}

void Writer::writePoints(const Frontend::Map &map, const char* point_filename)
{
    FILE* fptr = fopen(point_filename, "w");
    for(int i = 0; i < map.mapPoints_.size(); i++)
    {
        const auto& point = map.mapPoints_[i];
        auto x = static_cast<float> (point.pose_[0]);
        auto y = static_cast<float> (point.pose_[1]);
        auto z = static_cast<float> (point.pose_[2]);

        fprintf(fptr,"%lu %f %f %f\n", map.mapPoints_[i].id,x,y,z);
    }
    fclose(fptr);


}