//
// Created by jojo on 25.03.20.
//

#ifndef BAMAPPING_MAPPING_H
#define BAMAPPING_MAPPING_H

#include "../Frame.h"
#include "../MapPoint.h"
#include "../math/Types.h"

namespace BAMapping
{
    namespace FrontEnd
    {
        class Mapping
        {

        public:
            void updateMap(Frame& frame, Frame& ref_frame, std::vector<cv::DMatch> matches,bool loopClosure);
            inline std::vector<MapPoint> getMapPoints(){return m_map_points;}
        private:
            std::vector<MapPoint> m_map_points;
        };
    }
}


#endif //BAMAPPING_MAPPING_H
