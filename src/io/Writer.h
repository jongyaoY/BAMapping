#ifndef WRITER_H
#define WRITER_H
#include "../Graph.h"
#include "../Frame.h"
#include "../Frontend.h"
namespace BAMapping
{
    class Writer
    {
    public:
        Writer();
        static void writeToFile(const Graph& graph, const char* cam_filename, const char* point_filename, const char* image_path_file);
        static void writeToFileITEFormat(const Graph& graph, const char* cam_filename, const char* point_filename);
        static void writeToFileTUMFormat(const Graph& graph, const char* cam_filename);

        static void writePoses(const FrameVector& frameVector, const char* cam_filename);
        static void writeObservations(const FrameVector& frameVector, const char* obs_filename);
        static void writePoints(const Frontend::Map& map, const char* point_filename);
    };
}


#endif // WRITER_H
