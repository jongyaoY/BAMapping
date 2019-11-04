#ifndef READER_H
#define READER_H

#include <stdio.h>
#include <Eigen/Geometry>

#include "util/Graph.h"

class Reader
{
public:
    Reader();
    static bool readFrames(Graph *pGraph, const char* cam_file, const char *obs_file);
    static bool readPoints(Graph *pGraph, const char* point_file);
};

#endif // READER_H
