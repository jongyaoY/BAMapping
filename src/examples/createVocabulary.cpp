//
// Created by jojo on 08.03.20.
//

#include "io/Reader.h"
#include "Frontend.h"
#include "opencv2/core.hpp"

int main(int argc, char** argv)
{
    using namespace BAMapping;
    if(argc < 2)
    {
        printf("usage: ../path_to_data_set/ \n");
        return 0;
    }
    std::string dataset_path = argv[1];
    std::string voc_path_gz = dataset_path +"ORBvoc.yml.gz";
    std::string obs_path = dataset_path + "observations.txt";
    std::string cam_file = dataset_path + "cameras.txt";
    auto frameVec = io::Reader::readITEFrames(cam_file.c_str(),
                                              (dataset_path + "observations.txt").c_str(),
                                              dataset_path.c_str(),1);

    Frontend frontend;
    std::vector<std::vector<cv::Mat > > features;
    int id = 0;
    for(auto& frame : frameVec)
    {
        std::cout<<"extracting: "<<id<<std::endl;
        cv::Ptr<cv::ORB> orb_detector = cv::ORB::create(500);
        frontend.ExtractORB(frame,orb_detector);
        features.push_back(frame.mKepoint_descriptors);
        id++;
    }
    OrbVocabulary voc(9,3);
    voc.create(features);
    voc.save(voc_path_gz);
}