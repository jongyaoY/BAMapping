//
// Created by jojo on 06.12.19.
//

#include "Reader.h"
using namespace BAMapping::io;

BAMapping::FrameVector Reader::readTUMFrames(const std::string dataSetPath, const std::string assoFileName)
{
    FILE* assoFilePtr;
    assoFilePtr = fopen((dataSetPath+assoFileName).c_str(),"r");
    FrameVector frameVector;
    while(!feof(assoFilePtr))
    {
        Frame frame;
        double rgb_timeStamp,depth_timeStamp;
        char* rgb_path = new char[1024];
        char* depth_path = new char[1024];
        int scan = fscanf(assoFilePtr,"%lf %s %lf %s",&rgb_timeStamp,rgb_path,&depth_timeStamp,depth_path);

        if(scan!=4)
            break;

        frame.setTimeStamp(rgb_timeStamp);
        frame.setImagePaths((dataSetPath+std::string(rgb_path)).c_str(),(dataSetPath+std::string(depth_path)).c_str());
        frameVector.push_back(frame);
        delete[] rgb_path;
        delete[] depth_path;
    }
    return frameVector;
}

BAMapping::FrameVector Reader::readITEFormat(const char *cam_file, const char *img_path_file)
{
    FILE* pf_camera_file = fopen(cam_file,"r");
    FILE* pf_image_paths_file = fopen(img_path_file,"r");
    FrameVector frameVector;
    size_t id = 0;
    if(pf_camera_file != NULL && pf_image_paths_file != NULL)
    {
        while(!feof(pf_camera_file))
        {
            Frame frame;
            frame.mGlobalIndex = id;
            id++;
            double ax,ay,az,x,y,z;
            char rgb[256],ir[256],depth[256];
            int scan = 0;
            scan = fscanf(pf_camera_file,"%lf %lf %lf %lf %lf %lf",&ax,&ay,&az,&x,&y,&z);
            if(scan!=6)
                break;
            scan = fscanf(pf_image_paths_file,"%s %s %s",rgb,ir,depth);
            if(scan!=3)
                ;//todo
            //read in Twc -> Tcw
            Eigen::Matrix3d R;
            Eigen::Vector3d trans;
            auto angle = ax*ax + ay*ay + az*az;
            angle = sqrt(angle);
            R = Eigen::AngleAxisd(angle,Eigen::Vector3d(ax,ay,az)).inverse().toRotationMatrix();
            trans = R*Eigen::Vector3d(x,y,z);
            trans *= -1;
            frame.setAngleAxisAndPoint(Eigen::AngleAxisd(angle,Eigen::Vector3d(ax,ay,az)).inverse(),trans);
            frame.setImagePaths(rgb,depth,ir);

            frameVector.push_back(frame);
        }

        fclose(pf_camera_file);
        fclose(pf_image_paths_file);
    }
    else
    {

    }

    return frameVector;
}

BAMapping::FrameVector
Reader::readITEFrames(const char *cam_file, const char *obs_file, const char *dataset_path,const size_t every_n_frame)
{
    FILE* pF_cam;
    FrameVector frameVec;
    pF_cam = fopen(cam_file,"r");
    if(pF_cam == NULL) {
        printf("camera file not found!\n");
        return frameVec;
    }

    int cam_id_offset = -1;
    unsigned int line = 0;
    int id = 0;
    int i = 0;
    while(!feof(pF_cam))
    {

//FrameId qw qx qy qz px py pz absoluteTime timeIndex
        Frame frame;
        unsigned int frame_id;
        double frame_id_temp,time_id;
        char* cam_id_str = new char[7];
        double t;
        double x,y,z,qx,qy,qz,qw;
        int num_scanned = fscanf(pF_cam,"%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",&frame_id_temp,&qw,&qx,&qy,&qz,&x,&y,&z,&t,&time_id);

        if(i % every_n_frame != 0)
        {
            i++;
            continue;
        }
        i++;

        frame_id = frame_id_temp;

        frame.mGlobalIndex = id;
        id++;

        if(cam_id_offset < 0)
        {
            cam_id_offset = static_cast<int>(frame_id_temp);
            frame_id = cam_id_offset;
        }
        else
            frame_id = cam_id_offset + line;

        frame.mITEId = frame_id;

        line++;

        sprintf(cam_id_str,"%06d",frame_id);
        std::string rgbImg_path = std::string(dataset_path);
        rgbImg_path += "rgb/";
        rgbImg_path += "ColorRealsense";
        rgbImg_path += std::string(cam_id_str);
        rgbImg_path += ".png";
        std::string depthImg_path = std::string(dataset_path);
        depthImg_path += "depth/";
        depthImg_path += "DepthRealsense";
        depthImg_path += std::string(cam_id_str);
        depthImg_path +=    ".png";

        std::string infraRedImg_path = std::string(dataset_path);
        infraRedImg_path += "ir/";
        infraRedImg_path += "InfraredRealsense";
        infraRedImg_path += std::string(cam_id_str);
        infraRedImg_path += ".png";

        frame.setImagePaths(rgbImg_path.c_str(),depthImg_path.c_str(),infraRedImg_path.c_str());
        if(num_scanned != 10)
            break;

//read in Twc -> Tcw
        Eigen::Matrix3d R;
        Eigen::Vector3d trans;
        auto q = Eigen::Quaterniond(qw,qx,qy,qz);
        R = Eigen::AngleAxisd(q).inverse().toRotationMatrix();
        trans = R*Eigen::Vector3d(x,y,z);
        trans *= -1;

        frame.setFromQuaternionAndPoint(q.inverse(),trans);
        frame.setTimeStamp(t);


        frameVec.push_back(frame);
    }

    fclose(pF_cam);

    FILE* pF_obs;
    pF_obs = fopen(obs_file,"r");
    if(pF_obs == NULL)
    {
        printf("failed to read observation file\n");
        return frameVec;
    }

    while(!feof(pF_obs))
    {
        Eigen::Vector3d obs;
        int u,v;
        float d;
        unsigned int cam_id;
        unsigned int point_id;
        int num_scanned = fscanf(pF_obs,"%d %d %d %d %f",&point_id,&cam_id,&u,&v,&d);
        if(num_scanned != 5)
            break;
        obs[0] = static_cast<double>(u);
        obs[1] = static_cast<double>(v);
        obs[2] = d;

        for(auto& frame : frameVec)
        {
            if(frame.mITEId == cam_id)
            {
                frame.addObservation(point_id,obs);
                break;
            }
        }
//        if((cam_id - cam_id_offset) < frameVec.size())
//        {
//            frameVec[cam_id - cam_id_offset].addObservation(point_id,obs);
//        }
    }

    return frameVec;
}

BAMapping::PointVector Reader::readPoints(const char *point_file)
{
    PointVector pointVector;
    FILE* pF;
    pF = fopen(point_file,"r");
    if(pF == NULL)
        return pointVector;

    while(!feof(pF))
    {
        float x,y,z;
        unsigned int id;
        int num_scanned = fscanf(pF,"%u %f %f %f",&id,&x,&y,&z);
        if(num_scanned != 4)
            break;
        Point point(Eigen::Vector3d(x,y,z));
        point.mGlobalIndex = id;
        id++;
        pointVector.push_back(point);
    }
    fclose(pF);
}




