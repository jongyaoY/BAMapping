#include "Reader.h"
#include "ceres/rotation.h"

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
        double a[3];
        double f,k1,k2;
        int num_scanned = fscanf(pF_cam,"%lf %lf %lf %lf %lf %lf %lf %lf %lf",&a[0],&a[1],&a[2],&x,&y,&z,&f,&k1,&k2);
        if(num_scanned != 9)
            break;
        double norm = sqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);
        Eigen::AngleAxisd axisAngle(norm,Eigen::Vector3d(a[0]/norm,a[1]/norm,a[2]/norm));
//        Frame::Pose Tcw;
//        Eigen::Matrix3d R;
//        R = axisAngle.toRotationMatrix();
//        Tcw.topLeftCorner(3,3)<<R(0,0),R(0,1),R(0,2),
//                                R(1,0),R(1,1),R(1,2),
//                                R(2,0),R(2,1),R(2,2);
//        Tcw.topRightCorner(3,1) << x,y,z;
//        Tcw(3,3) = 1;

        frame.setAngleAxisAndPoint(axisAngle,Eigen::Vector3d(x,y,z));
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

bool Reader::readTUMFrames(FrameVector& frames,const std::string dataSetPath, const std::string assoFileName, const std::string poseFileName)
{
    FILE* assoFilePtr;
    FILE* poseFilePtr;
    assoFilePtr = fopen((dataSetPath+assoFileName).c_str(),"r");
    while(!feof(assoFilePtr))
    {
        Frame frame;
        double rgb_timeStamp,depth_timeStamp;
        char* rgb_path = new char[1024];
        char* depth_path = new char[1024];
        fscanf(assoFilePtr,"%lf %s %lf %s",&rgb_timeStamp,rgb_path,&depth_timeStamp,depth_path);
        frame.setImagePaths((dataSetPath+std::string(rgb_path)).c_str(),(dataSetPath+std::string(depth_path)).c_str());
        frame.setTimeStamp(rgb_timeStamp);
        frames.push_back(frame);
        delete[] rgb_path;
        delete[] depth_path;
    }
    fclose(assoFilePtr);
    int line = 0;
    poseFilePtr = fopen((dataSetPath+poseFileName).c_str(),"r");
    for(auto &frame : frames)
    {
        double timeStamp;
        float tx,ty,tz,qx,qy,qz,qw;
        while(!feof(poseFilePtr))
        {
            fscanf(poseFilePtr,"%lf %f %f %f %f %f %f %f",&timeStamp,
                                        &tx,&ty,&tz,&qx,&qy,&qz,&qw);

            if(std::fabs(frame.getTimeStamp()-timeStamp)<0.01)
            {
                frame.setFromQuaternionAndPoint(Eigen::Quaterniond(qw,qx,qy,qz),Eigen::Vector3d(tx,ty,tz));

                break;
            }
        }
    }

    return true;
}
