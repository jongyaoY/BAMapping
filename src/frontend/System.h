//
// Created by jojo on 25.03.20.
//

#ifndef BAMAPPING_SYSTEM_H
#define BAMAPPING_SYSTEM_H

#include "../Frame.h"
#include "Tracking.h"
#include "Mapping.h"

namespace BAMapping
{
    namespace FrontEnd
    {
        class System
        {
        public:
            void run(FrameVector& frameVector,const std::string config_file);

            Mapping mMap;
        private:
        };
    }
}



#endif //BAMAPPING_SYSTEM_H
