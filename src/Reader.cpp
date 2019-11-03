#include "Reader.h"

Reader::Reader()
{
}

bool Reader::readFrames(Graph* pGraph,const char* cam_file,const char* obs_file)
{
    FILE* pF_cam;
    FILE* pF_obs;
    pF_cam = fopen(cam_file,"r");
    pF_obs = fopen(obs_file,"r");
    FrameVector frameVec;
    Frame::ObservationVector obsVec;
    while(!feof(pF_cam))
    {
        Frame frame;
        double t;
        double x,y,z,qx,qy,qz,qw;
        double k1,k2,k3;
        int num_scanned = fscanf(pF_cam,"%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",&qx,&qy,&qz,&qw,&x,&y,&z,&k1,&k2,&k3);
//        if(num_scanned != 10)
//            break;
        Eigen::Quaternionf q(qw,qx,qy,qz);
        Frame::Pose Tcw;
        Tcw.topLeftCorner(3,3) = q.toRotationMatrix();
        Tcw.topRightCorner(3,1) << x,y,z;
        frameVec.push_back(frame);
    }
    while(!feof(pF_obs))
    {
        Frame::Observation indexedObs;
        Eigen::Vector3d obs;
        unsigned int cam_id;
        unsigned int point_id;
        int num_scanned = fscanf(pF_obs,"%d %d %lf %lf",&cam_id,&point_id,&obs[0],&obs[1]);
        if(num_scanned != 4)
            break;
        indexedObs.first = point_id;
        indexedObs.second = obs;
        frameVec[cam_id].addObservation(indexedObs);
    }
    fclose(pF_cam);
    fclose(pF_obs);
    return true;
}
