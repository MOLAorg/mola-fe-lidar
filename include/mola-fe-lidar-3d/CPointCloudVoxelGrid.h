/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mrpt/containers/CDynamicGrid3D.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/math/TPoint3D.h>

class CPointCloudVoxelGrid
{
   public:
    CPointCloudVoxelGrid() = default;

    void resize(
        const mrpt::math::TPoint3D& min_corner,
        const mrpt::math::TPoint3D& max_corner, const float voxel_size);
    void processPointCloud(const mrpt::maps::CPointsMap& p);
    void clear();

    /** The list of point indices in each voxel */
    struct voxel_t
    {
        std::vector<std::size_t> indices;
    };
    using grid_t = mrpt::containers::CDynamicGrid3D<voxel_t, float>;

    /** The point indices in each voxel. Directly access to each desired cell,
     * use its iterator, etc. */
    grid_t pts_voxels;
};
