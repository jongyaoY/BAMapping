#include "Writer.h"

using namespace BAMapping;

Writer::Writer()
{

}

void Writer::writeToFile(const Graph& graph,const char* cam_filename,const char* point_filename, const char* image_path_file) {
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

    FILE* fptr_img = fopen(image_path_file, "w");
    fptr = fopen(cam_filename, "w");

    for (auto node : graph.nodes_)
    {
        auto ir_path = node.ir_path_;
        auto rgb_path = node.rgb_path_;
        auto depth_path = node.depth_path_;
        fprintf(fptr_img,"%s %s %s\n",ir_path.c_str(),rgb_path.c_str(),depth_path.c_str());
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

//    for(auto frame : frames)
//    {
//        auto Twc = frame.getConstTwc();
//        int frame_id = frame.mITEId;
//        Eigen::Quaterniond q;
//        q.matrix() = Twc.block<3,3>(0,0);
//        const Eigen::Vector3d translation = frame.getConstTranslation();
//
//        fprintf(fptr,"%d %lf %lf %lf %lf %lf %lf %lf\n",frame_id,
//                q.w(),
//                q.x(),
//                q.y(),
//                q.z(),
//                translation[0],
//                translation[1],
//                translation[2]);
//    }
//    for(auto frame : frames)
//    {
//        const double angle = frame.getConstAngleAxis().angle();
//        const double angleAxis[3] = {angle*frame.getConstAngleAxis().axis()[0],
//                                     angle*frame.getConstAngleAxis().axis()[1],
//                                     angle*frame.getConstAngleAxis().axis()[2]};
//        const Eigen::Vector3d translation = frame.getConstTranslation();
//
//        fprintf(fptr,"%lf %lf %lf %lf %lf %lf\n",angleAxis[0],
//                                                 angleAxis[1],
//                                                 angleAxis[2],
//                                                 translation[0],
//                                                 translation[1],
//                                                 translation[2]);
//    }
    fclose(fptr);
    fclose(fptr_img);
}
