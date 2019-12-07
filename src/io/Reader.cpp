//
// Created by jojo on 06.12.19.
//

#include "Reader.h"
using namespace BAMapping::io;

bool Reader::readITEData(BAMapping::Graph *pGraph, const char *cam_file, const char *obs_file,
                                    const char *point_file, const char *dataset_path, const char *config_file)
{
    readITEFrames(pGraph,cam_file,obs_file,dataset_path,config_file);
    readITEPoints(pGraph,point_file);
    return true;
}


bool Reader::readITEFrames(BAMapping::Graph *pGraph, const char *cam_file, const char *obs_file, const char *dataset_path,
                      const char *config_file)
{
    FILE* pF_cam;
    FILE* pF_obs;
    pF_cam = fopen(cam_file,"r");
    if(pF_cam == NULL)
        return false;

    FrameVector frameVec;
    int cam_id_offset = -1;
    unsigned int line = 0;
    while(!feof(pF_cam))
    {
//FrameId qw qx qy qz px py pz absoluteTime timeIndex
        Frame frame;
        unsigned int frame_id;
        double frame_id_temp,time_id;
        char* cam_id_str = new char[7];
        double t;
        double x,y,z,qx,qy,qz,qw;
        double fx,fy,cx,cy,k1,k2;
        int num_scanned = fscanf(pF_cam,"%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",&frame_id_temp,&qw,&qx,&qy,&qz,&x,&y,&z,&t,&time_id);
        frame_id = frame_id_temp;

        if(cam_id_offset < 0)
        {
            cam_id_offset = static_cast<int>(frame_id_temp);
            frame_id = cam_id_offset;
        }
        else
            frame_id = cam_id_offset + line;

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

        frame.setImagePaths(rgbImg_path.c_str(),depthImg_path.c_str());
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

        Parser parser;
        parser.load(config_file);
        fx = parser.getValue<double>("Camera.fx");
        fy = parser.getValue<double>("Camera.fy");
        cx = parser.getValue<double>("Camera.cx");
        cy = parser.getValue<double>("Camera.cy");

        k1 = parser.getValue<double>("Camera.k1:");
        k2 = parser.getValue<double>("Camera.k2:");

        frame.setIntrinsics(fx,fy,cx,cy);
        frame.setDistortionFactors(k1,k2);

        frameVec.push_back(frame);
    }
    pF_obs = fopen(obs_file,"r");
    if(pF_obs == NULL)
        return false;

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

        if((cam_id - cam_id_offset) < frameVec.size())
        {
            frameVec[cam_id - cam_id_offset].addObservation(point_id,obs);
        }
    }
    for(auto frame : frameVec)
    {
        pGraph->addGlobalFrame(frame);
    }
    fclose(pF_cam);
    fclose(pF_obs);
    return true;
}

bool Reader::readITEPoints(BAMapping::Graph *pGraph, const char *point_file)
{
    FILE* pF;
    pF = fopen(point_file,"r");
    while(!feof(pF))
    {
        unsigned int id;
        double x,y,z;
        int num_scanned = fscanf(pF,"%d %lf %lf %lf",&id,&x,&y,&z);
        if(num_scanned != 4)
            break;
        Point point(Eigen::Vector3d(x,y,z));
        pGraph->addGlobalPoint(point);
    }
    fclose(pF);
    return true;
}



