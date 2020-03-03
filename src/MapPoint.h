//
// Created by jojo on 27.02.20.
//

#ifndef BAMAPPING_MAPPOINT_H
#define BAMAPPING_MAPPOINT_H

#include "math/Types.h"
#include "opencv2/core.hpp"
#include <memory>
namespace BAMapping
{
    class MapPoint
    {
    public:
        typedef std::shared_ptr<MapPoint> Ptr;
        MapPoint() {}
        ~MapPoint() {}
        MapPoint(Vec3 pose):pose_(pose){ }
        Ptr getPointer()
        {
            return Ptr(this);
        }
        size_t id;
        Vec3 pose_;
        cv::Mat descritor_;
    };
}

#endif //BAMAPPING_MAPPOINT_H
