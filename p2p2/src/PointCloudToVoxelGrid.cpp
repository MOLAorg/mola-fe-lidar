/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <p2p2/PointCloudToVoxelGrid.h>

using namespace p2p2;

void PointCloudToVoxelGrid::resize(
    const mrpt::math::TPoint3D& min_corner,
    const mrpt::math::TPoint3D& max_corner, const float voxel_size)
{
    pts_voxels.clear();
    pts_voxels.setSize(
        min_corner.x, max_corner.x, min_corner.y, max_corner.y, min_corner.z,
        max_corner.z, voxel_size, voxel_size);
}

void PointCloudToVoxelGrid::processPointCloud(const mrpt::maps::CPointsMap& p)
{
    const auto& xs   = p.getPointsBufferRef_x();
    const auto& ys   = p.getPointsBufferRef_y();
    const auto& zs   = p.getPointsBufferRef_z();
    const auto  npts = xs.size();

    for (std::size_t i = 0; i < npts; i++)
    {
        auto* c = pts_voxels.cellByPos(xs[i], ys[i], zs[i]);
        if (c) c->indices.push_back(i);  // only if not out of grid range
    }
}

void PointCloudToVoxelGrid::clear()
{
    for (auto& c : pts_voxels) c.indices.clear();
}
