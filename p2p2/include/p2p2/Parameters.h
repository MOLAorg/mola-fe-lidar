/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mrpt/core/bits_math.h>  // DEG2RAD()
#include <cstddef>
#include <cstdint>
#include <set>
#include <string>

namespace p2p2
{
struct Parameters
{
    /** @name Termination criteria
        @{ */
    /** Maximum number of iterations to run. */
    size_t maxIterations{40};
    /** If the correction in all translation coordinates (X,Y,Z) is below
     * this threshold (in meters), iterations are terminated (Default:1e-6)
     */
    double minAbsStep_trans{1e-6};
    /** If the correction in all rotation coordinates (yaw,pitch,roll) is
     * below this threshold (in radians), iterations are terminated
     * (Default:1e-6) */
    double minAbsStep_rot{1e-6};
    /** @} */

    /** Treshold distance for pair two near points */
    double thresholdDist{0.75}, thresholdAng{mrpt::DEG2RAD(0.15)};

    /** Cauchy kernel rho, for estimating the optimal transformation
     * covariance, in meters (default = 0.07m) */
    double kernel_rho{0.07};
    /** Whether to use kernel_rho to smooth distances, or use distances
     * directly (default=true) */
    bool use_kernel{false};

    /** Decimation of the point cloud being registered against the reference
     * one. The speed-up comes from a decimation of the number of KD-tree
     * queries, the most expensive step in ICP */
    uint32_t corresponding_points_decimation{5};
    // TODO: Remove, not used for PlanesPointsICP!

    uint32_t max_corresponding_points{100000};

    std::set<std::string> pt2pt_layers;
    std::string           pt2pl_layer;
};

}  // namespace p2p2
