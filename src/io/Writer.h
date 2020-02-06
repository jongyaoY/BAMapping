#ifndef WRITER_H
#define WRITER_H
#include "../Graph.h"

namespace BAMapping
{
    class Writer
    {
    public:
        Writer();
        static void writeToFile(const Graph& graph, const char* cam_filename, const char* point_filename, const char* image_path_file);
    };
}


#endif // WRITER_H
