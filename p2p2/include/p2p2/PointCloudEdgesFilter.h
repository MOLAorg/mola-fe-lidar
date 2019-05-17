/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mrpt/maps/CPointsMap.h>
#include <mrpt/math/lightweight_geom_data.h>

namespace p2p2
{
class PointCloudEdgesFilter
{
   public:
    PointCloudEdgesFilter() = default;

    void processPointCloud(
        const mrpt::maps::CPointsMap& in, mrpt::maps::CPointsMap& out);

    struct Parameters
    {
        uint8_t search_window{3};
    };

    Parameters params_;
};

}  // namespace p2p2
