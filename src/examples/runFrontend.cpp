//
// Created by jojo on 08.03.20.
//

#include "io/Reader.h"
#include "Frontend.h"
#include "io/Writer.h"

int main(int argc, char** argv)
{
    using namespace BAMapping;
    if(argc < 2)
    {
        printf("usage: ../path_to_data_set/ \n");
        return 0;
    }
    std::string dataset_path = argv[1];
    std::string voc_path = dataset_path + "ORBvoc.yml.gz";
    std::string obs_path = dataset_path + "observations.txt";
    std::string point_path = dataset_path + "points.txt";
    std::string cam_file = dataset_path + "cameras.txt";

    auto frameVec = io::Reader::readITEFrames(cam_file.c_str(),
                                              (dataset_path + "observations.txt").c_str(),
                                              dataset_path.c_str(),1);

    Frontend frontend;
    frontend.ExtractAndMatchFeatures(frameVec,voc_path);
    Writer::writeObservations(frameVec,obs_path.c_str());
    Writer::writePoints(frontend.mMap,point_path.c_str());
//    Viewer viewer;
//    viewer.setMap(&frontend.mMap);
//    viewer.visualizeMap();
//    viewer.setFrames(frameVec);
//    viewer.visualize();
}
