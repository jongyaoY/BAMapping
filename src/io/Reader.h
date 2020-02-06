//
// Created by jojo on 06.12.19.
//

#ifndef BAMAPPING_READER_H
#define BAMAPPING_READER_H

#include "../Frame.h"
#include "../Point.h"
namespace BAMapping
{
    namespace io
    {
        class Reader
        {
        public:
//            static FrameVector readITEFrames(const char *cam_file,const char *dataset_path, const size_t every_n_frame = 1);
            static FrameVector readITEFrames(const char* cam_file,const char* obs_file, const char* dataset_path, const size_t every_n_frame = 1);
            static FrameVector readITEFormat(const char* cam_file, const char* img_path_file);
            static PointVector readPoints(const char *point_file);
        };
    }


}


#endif //BAMAPPING_READER_H
