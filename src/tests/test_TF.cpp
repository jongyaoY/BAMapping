//
// Created by jojo on 05.12.19.
//
#include <stdio.h>
#include "../Viewer.h"
#include "../Reader.h"
#include "../util/Graph.h"
#include "../util/Frame.h"
int main(int argc, char** argv)
{
    Viewer viewer;
    Graph graph;

    Reader::readITEFrames(&graph,"../dataset_local/ITE_dataset/cameras.txt",
                          "../dataset_local/ITE_dataset/observations.txt",
                          "../dataset_local/ITE_dataset/",
                          "../dataset_local/ITE_dataset/ITE.yaml");
    Reader::readITEPoints(&graph,"../dataset_local/ITE_dataset/points.txt");
    viewer.setRefPoints(graph.getConstPoints());

    viewer.setFrames(graph.getConstFrames());
    viewer.visualize();
//    Frame frame = graph.getConstFrames()[0];
//    auto m_angleAxis = frame.getConstAngleAxis();
//    auto m_translation = frame.getConstTranslation();
//    Frame::Pose Twc;
//    Eigen::Matrix3d R;
//    Eigen::Vector3d t;
//    R = m_angleAxis.inverse().toRotationMatrix();
//    t = R*m_translation;
//    t*=-1;
//    Twc.topLeftCorner(3,3)<<R(0,0),R(0,1),R(0,2),
//            R(1,0),R(1,1),R(1,2),
//            R(2,0),R(2,1),R(2,2);
//    Twc.topRightCorner(3,1)<<t[0],t[1],t[2];
//    Twc(3,3) = 1;

    return 0;
}