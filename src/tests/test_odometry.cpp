//
// Created by jojo on 24.03.20.
//
#include "../io/Reader.h"
//#include "../Frontend.h"
#include "../io/Writer.h"
#include "../Viewer.h"
#include "../frontend/System.h"

int main(int argc, char** argv)
{
    using namespace BAMapping;
    if(argc < 2)
    {
        printf("usage: ../path_to_data_set/ \n");
        return 0;
    }
    std::string dataset_path = argv[1];
    std::string asso_path = "ass.txt";
    std::string config_file = dataset_path + "TUM_frontend.yaml";
    auto frameVec = io::Reader::readTUMFrames(dataset_path,asso_path);
    std::cout<<frameVec.size()<<std::endl;
    FrontEnd::System frontend;
    frontend.run(frameVec,config_file);

    for(auto& frame : frameVec)
    {
        Eigen::Affine3d T;
        T.matrix() = frame.Tcw_;
        frame.setFromAffine3d(T);//Tcw;
    }
    Viewer viewer;
    viewer.setFrames(frameVec);
    viewer.setPoints(frontend.mMap.getMapPoints());
    viewer.visualize();

}
