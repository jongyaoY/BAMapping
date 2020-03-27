//
// Created by jojo on 07.01.20.
//

#ifndef RECONSTRUCTION3D_GEOMETRYMETHODS_H
#define RECONSTRUCTION3D_GEOMETRYMETHODS_H

#include "util/Parser.h"
#include "Open3D/Open3D.h"
#include "Graph.h"

namespace BAMapping
{
    class GeometryMethods
    {
    public:
        static bool createRGBDImageFromNode(const Graph::Node& node, double depth_factor,double depth_truncate, open3d::geometry::RGBDImage& rgbd, bool useIRImg = true);

        static bool createPointCloundFromNodes(
                const std::vector<Graph::Node> nodes,
                const Parser config,
                std::shared_ptr<open3d::geometry::PointCloud>& pcd,
                bool color = false
                );
        static bool createPointCloudFromNode(
                const Graph::Node node,
                const Parser config,
                std::shared_ptr<open3d::geometry::PointCloud>& pcd,
                bool color = false
        );
        static bool createPointCloudFromNode(
                const Graph::Node node,
                const Vec4& intrisics,
                const size_t width,
                const size_t height,
                std::shared_ptr<open3d::geometry::PointCloud>& pcd,
                bool color = false
        );
    };
}



#endif //RECONSTRUCTION3D_GEOMETRYMETHODS_H
