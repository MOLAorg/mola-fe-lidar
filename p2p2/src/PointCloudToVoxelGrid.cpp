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

    voxel_is_empty_.assign(pts_voxels.getVoxelCount(), true);
    used_voxel_indices.clear();
    used_voxel_indices.reserve(pts_voxels.getVoxelCount() / 4);
}

void PointCloudToVoxelGrid::processPointCloud(const mrpt::maps::CPointsMap& p)
{
    const auto& xs   = p.getPointsBufferRef_x();
    const auto& ys   = p.getPointsBufferRef_y();
    const auto& zs   = p.getPointsBufferRef_z();
    const auto  npts = xs.size();

    for (std::size_t i = 0; i < npts; i++)
    {
        const auto cx      = pts_voxels.x2idx(xs[i]);
        const auto cy      = pts_voxels.y2idx(ys[i]);
        const auto cz      = pts_voxels.z2idx(zs[i]);
        const auto vxl_idx = pts_voxels.cellAbsIndexFromCXCYCZ(cx, cy, cz);
        if (vxl_idx == grid_t::INVALID_VOXEL_IDX) continue;

        auto* c = pts_voxels.cellByIndex(cx, cy, cz);
        if (!c) continue;
        c->indices.push_back(i);  // only if not out of grid range

        if (voxel_is_empty_[vxl_idx])
        {
            voxel_is_empty_[vxl_idx] = false;
            used_voxel_indices.push_back(vxl_idx);
        }
    }
}

void PointCloudToVoxelGrid::clear()
{
    // for (auto& c : pts_voxels) c.indices.clear();
    for (auto idx : used_voxel_indices)
        pts_voxels.cellByIndex(idx)->indices.clear();

    voxel_is_empty_.assign(pts_voxels.getVoxelCount(), true);
    used_voxel_indices.clear();
}
