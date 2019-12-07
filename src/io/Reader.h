//
// Created by jojo on 06.12.19.
//

#ifndef BAMAPPING_READER_H
#define BAMAPPING_READER_H

#include "../Graph.h"
#include "Parser.h"
namespace BAMapping
{
    namespace io
    {
        class Reader
        {
        public:
            static bool readITEData(Graph *pGraph,
                                    const char* cam_file, const char* obs_file,const char* point_file,
                                    const char* dataset_path,const char *config_file);

        private:
            static bool readITEFrames(Graph *pGraph,
                                      const char* cam_file, const char* obs_file,
                                      const char* dataset_path,const char* config_file);
            static bool readITEPoints(Graph *pGraph,const char* point_file);
        };
    }


}


#endif //BAMAPPING_READER_H
