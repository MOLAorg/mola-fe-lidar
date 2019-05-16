/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   PointsPlanesICP.h
 * @brief  ICP registration for points and planes
 * @author Jose Luis Blanco Claraco
 * @date   May 11, 2019
 */
#pragma once

#include <mrpt/img/TColor.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/opengl/opengl_frwds.h>
#include <mrpt/poses/CPose3D.h>
#include <cstdint>
#include <memory>
#include "IterTermReason.h"
#include "Parameters.h"
#include "Results.h"

/** ICP registration for points and planes
 * Refer to technical report: XXX
 *
 * \ingroup mola_fe_lidar_icp_grp */
namespace p2p2::PointsPlanesICP
{
struct plane_patch_t
{
    mrpt::math::TPlane3D plane;
    mrpt::math::TPoint3D centroid;

    plane_patch_t() = default;
    plane_patch_t(
        const mrpt::math::TPlane3D& pl, const mrpt::math::TPoint3D& center)
        : plane(pl), centroid(center)
    {
    }
};

struct render_params_t
{
    render_params_t() = default;

    mrpt::img::TColor plane_color{0xff, 0xff, 0xff, 0xff};
    double            plane_half_width{1.0}, plane_grid_spacing{0.25};
};

struct pointcloud_t
{
    /** Different point layers, indexed by a descriptive name.
     * Known layer names:
     * - `raw`: reserved to the original, full point cloud (if kept here).
     * - `plane_centroids`: a point for each plane in `planes` (same order).
     */
    std::map<std::string, mrpt::maps::CPointsMap::Ptr> point_layers;
    std::vector<mrpt::math::TLine3D>                   lines;
    std::vector<plane_patch_t>                         planes;

    /** Gets a renderizable view of all planes. The target container `o` is not
     * cleared(), clear() it manually if needed before calling. */
    void planesAsRenderizable(
        mrpt::opengl::CSetOfObjects& o,
        const render_params_t&       p = render_params_t());
};

void align(
    const pointcloud_t& pcs1, const pointcloud_t& pcs2,
    const mrpt::math::TPose3D& init_guess_m2_wrt_m1, const Parameters& p,
    Results& result);

struct matched_plane_t
{
    /// \note "this"=global, "other"=local, while finding the transformation
    /// local wrt global
    plane_patch_t p_this, p_other;

    matched_plane_t() = default;
    matched_plane_t(const plane_patch_t& pl_this, const plane_patch_t& pl_other)
        : p_this(pl_this), p_other(pl_other)
    {
    }
};
using TMatchedPlaneList = std::vector<matched_plane_t>;

struct matched_line_t
{
    /// \note "this"=global, "other"=local, while finding the transformation
    /// local wrt global
    mrpt::math::TLine3D l_this, l_other;
};
using TMatchedLineList = std::vector<matched_line_t>;

struct OLAE_Match_Input
{
    /// We reuse MRPT struct to allow using their matching functions.
    /// \note on MRPT naming convention: "this"=global; "other"=local.
    mrpt::tfest::TMatchingPairList paired_points;
    TMatchedLineList               paired_lines;
    TMatchedPlaneList              paired_planes;

    /** Relative weight of points, lines, and planes. They will be automatically
     * normalized to sum the unity, so feel free of setting weights at any
     * convenient scale. Weights are used in two steps: in the orientation cost
     * function, and in the evaluation of the centroids to find the final
     * translation; hence we have two sets of weights.
     */
    struct Weights
    {
        struct Attitude
        {
            double points{1.0};  //!< Weight for points in attitude estimation
            double lines{1.0};  //!< Weight for lines in attitude estimation
            double planes{1.0};  //!< Weight for planes in attitude estimation
        } attitude;

        struct Translation
        {
            double points{1.0};  //!< Points weight in translation estimation
            double planes{1.0};  //!< Planes weight in translation estimation
        } translation;
    };

    /// See docs for Weights
    Weights weights;

    bool empty() const
    {
        return paired_points.empty() && paired_planes.empty();
    }
};

struct OLAE_Match_Result
{
    mrpt::poses::CPose3D optimal_pose;
};

/** The points-and-planes optimal pose solver.
 * Refer to technical report: XXX
 */
void olae_match(const OLAE_Match_Input& in, OLAE_Match_Result& result);

}  // namespace p2p2::PointsPlanesICP
