/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   MultiCloudICP.h
 * @brief  ICP registration for pointclouds split in different "layers"
 * @author Jose Luis Blanco Claraco
 * @date   Jan 20, 2019
 */
#pragma once

#include <mrpt/maps/CPointsMap.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <cstdint>
#include <vector>

/** ICP registration for pointclouds split in different "layers"
 *
 * \ingroup mola_fe_lidar_icp_grp */
namespace mola::MultiCloudICP
{
/** Reason of iterating termination */
enum class IterTermReason
{
    Undefined = 0,
    NoPairings,
    MaxIterations,
    Stalled
};

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
};

struct Results
{
    /** The found value (mean + covariance) of the optimal transformation of
     * m2 wrt m1. */
    mrpt::poses::CPose3DPDFGaussian optimal_tf;

    /** The number of executed iterations until convergence */
    size_t nIterations{0};

    IterTermReason terminationReason{IterTermReason::Undefined};

    /** A goodness measure for the alignment, it is a [0,1] range indicator
     * of percentage of correspondences. */
    double goodness{0};

    /** A measure of the 'quality' of the local minimum of the sqr. error
     * found by the method. Higher values are better. Low values will be
     * found in ill-conditioned situations (e.g. a corridor) */
    double quality{0};
};

using clouds_t = std::vector<mrpt::maps::CPointsMap::Ptr>;

/** Compute the displacement (relative pose) between
 *   two maps: the relative pose of pcs2 with respect to pcs1.
 */
void align(
    const clouds_t& pcs1, const clouds_t& pcs2,
    const mrpt::math::TPose3D& init_guess_m2_wrt_m1, const Parameters& p,
    Results& result);

}  // namespace mola::MultiCloudICP
