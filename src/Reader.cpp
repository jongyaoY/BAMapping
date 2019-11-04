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
        double f,k1,k2;
        int num_scanned = fscanf(pF_cam,"%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",&qx,&qy,&qz,&qw,&x,&y,&z,&f,&k1,&k2);
        if(num_scanned != 10)
            break;
        Eigen::Quaternionf q(qw,qx,qy,qz);
        Frame::Pose Tcw;
        Tcw.topLeftCorner(3,3) = q.toRotationMatrix();
        Tcw.topRightCorner(3,1) << x,y,z;
        frame.setPose(Tcw);
        //just for test
        //assume fx = fy = f,cx=cy=0
        frame.setIntrinsics(f,f,0.,0.);
        frame.setDistortionFactors(k1,k2);
        //
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
    for(auto frame : frameVec)
    {
        pGraph->addFrame(frame);
    }
    fclose(pF_cam);
    fclose(pF_obs);
    return true;
}

bool Reader::readPoints(Graph *pGraph, const char* point_file)
{
    FILE* pF;
    pF = fopen(point_file,"r");
    while(!feof(pF))
    {
        double x,y,z;
        int num_scanned = fscanf(pF,"%lf %lf %lf",&x,&y,&z);
        if(num_scanned != 3)
            break;
        Point point(x,y,z);
        pGraph->addPoint(point);
    }
    fclose(pF);
    return true;
}
