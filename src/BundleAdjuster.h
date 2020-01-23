//
// Created by jojo on 22.01.20.
//

#ifndef BAMAPPING_BUNDLEADJUSTER_H
#define BAMAPPING_BUNDLEADJUSTER_H

#include "Graph.h"
#include "Error.h"
#include "util/Parser.h"

#include "ceres/ceres.h"
#include <vector>
namespace BAMapping
{
    class BundleAdjuster
    {
    public:
        static void optimize(Graph& graph, const char* config_file);
        static std::vector<Vec6> packCameraParam(const Graph& graph);
        static std::vector<Vec3> packPointParam(const Graph& graph);
        static std::vector<Vec4> getIntrinsics(Parser config, size_t n);

        static void unpackCameraParam(Graph& graph, const std::vector<Vec6>& extrinsics);
        static void unpackPointParam(Graph& graph, const std::vector<Vec3>& points);
    };
}



#endif //BAMAPPING_BUNDLEADJUSTER_H