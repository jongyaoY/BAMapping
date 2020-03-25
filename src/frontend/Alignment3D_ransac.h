//
// Created by jojo on 25.03.20.
//

#ifndef BAMAPPING_ALIGNMENT3D_RANSAC_H
#define BAMAPPING_ALIGNMENT3D_RANSAC_H

#include "../math/Types.h"

namespace BAMapping
{
    namespace FrontEnd
    {
        class Alignment3D_ransac
        {
        public:
            struct Params
            {
                Params()
                {
                    distance_threshold = 0.02;
                    inlier_threshold = 0.9;
                    iterations = 200;
                }
                double distance_threshold;
                float inlier_threshold;
                size_t iterations;
            };
            static bool Align(const std::vector<Vec3>& source,const std::vector<Vec3>& target,Mat4& Tst,std::vector<size_t>& inliers,const Params params);

        private:
            static std::vector<size_t> generateRandomIds(int num, int max_id);
            static bool isInlier(Vec3 p, Vec3 q, Mat3 R, Vec3 t,double thres);
            static bool estimate(const std::vector<Vec3>& p, const std::vector<Vec3>& q, Mat3& R, Vec3& t);

        };
    }
}




#endif //BAMAPPING_ALIGNMENT3D_RANSAC_H
