#ifndef READER_H
#define READER_H

#include <stdio.h>
#include <Eigen/Geometry>

#include "util/Frame.h"

class Reader
{
public:
    Reader();
    static bool readFrames(FrameVector &frame_vec,const char* fileName);
};

#endif // READER_H
