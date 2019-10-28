#include "Frame.h"

Frame::Frame()
{
    Tcw = Pose::Identity();
    frameSize = 0.01;
    timeStamp = 0;
}

