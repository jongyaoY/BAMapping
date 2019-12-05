#ifndef WRITER_H
#define WRITER_H
#include "util/Graph.h"

class Writer
{
public:
    Writer();
    static void writeToFile(const Graph graph, const char *cam_filename, const char *point_filename);
};

#endif // WRITER_H
