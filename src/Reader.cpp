#include "Reader.h"

Reader::Reader()
{
}

bool Reader::read(FrameVector &frame_vec,const char* fileName)
{
    FILE* pF;
    pF = fopen(fileName,"r");
    frame_vec.clear();
    while(!feof(pF))
    {
        Frame frame;
        double t;
        float x,y,z,qx,qy,qz,qw;
        int num_scanned = fscanf(pF,"%lf %f %f %f %f %f %f %f",&t,&x,&y,&z,&qx,&qy,&qz,&qw);
        if(num_scanned != 8)
            return false;
        Eigen::Quaternionf q(qw,qx,qy,qz);
        frame.Tcw.topLeftCorner(3,3) = q.toRotationMatrix();
        frame.Tcw.topRightCorner(3,1) << x,y,z;
        frame_vec.push_back(frame);
    }
    fclose(pF);
    return true;
}
