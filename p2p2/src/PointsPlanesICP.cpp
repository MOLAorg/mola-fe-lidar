/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   PointsPlanesICP.cpp
 * @brief  ICP registration for points and planes
 * @author Jose Luis Blanco Claraco
 * @date   May 11, 2019
 */

#include <mrpt/core/exceptions.h>
#include <mrpt/poses/Lie/SE.h>
#include <mrpt/tfest/se3.h>
#include <p2p2/PointsPlanesICP.h>

using namespace p2p2;

void PointsPlanesICP::align(
    const PointsPlanesICP::clouds_t& pcs1,
    const PointsPlanesICP::clouds_t& pcs2,
    const mrpt::math::TPose3D& init_guess_m2_wrt_m1, const Parameters& p,
    Results& result)
{
    MRPT_START
    // ICP uses KD-trees.
    // kd-trees have each own mutexes to ensure well-defined behavior in
    // multi-threading apps.

    ASSERT_EQUAL_(pcs1.point_layers.size(), pcs2.point_layers.size());
    ASSERT_(
        !pcs1.point_layers.empty() ||
        (!pcs1.planes.empty() && !pcs2.planes.empty()));
    const auto nPointLayers = pcs1.point_layers.size();

    // Reset output:
    result = Results();

    // Matching params:
    mrpt::maps::TMatchingParams mp;
    // Distance threshold
    mp.maxDistForCorrespondence = p.thresholdDist;
    // Angular threshold
    mp.maxAngularDistForCorrespondence = p.thresholdAng;
    mp.onlyKeepTheClosest              = true;
    mp.onlyUniqueRobust                = false;
    mp.decimation_other_map_points     = p.corresponding_points_decimation;

    // For decimation: cycle through all possible points, even if we decimate
    // them, in such a way that different points are used in each iteration.
    mp.offset_other_map_points = 0;

    // Count of points:
    size_t pointcount1 = 0, pointcount2 = 0;
    for (size_t layer = 0; layer < nPointLayers; layer++)
    {
        pointcount1 += pcs1.point_layers[layer]->size();
        pointcount2 += pcs2.point_layers[layer]->size();
    }
    ASSERT_(pointcount1 > 0 || !pcs1.planes.empty());
    ASSERT_(pointcount2 > 0 || !pcs2.planes.empty());

    // ------------------------------------------------------
    // The P2P2 ICP loop
    // ------------------------------------------------------
    auto solution      = mrpt::poses::CPose3D(init_guess_m2_wrt_m1);
    auto prev_solution = solution;

    for (; result.nIterations < p.maxIterations; result.nIterations++)
    {
        std::vector<mrpt::maps::TMatchingExtraResults> mres(nPointLayers);

        // Measure angle distances from the current estimate:
        mp.angularDistPivotPoint = mrpt::math::TPoint3D(solution.asTPose());

        // the global list of pairings:
        OLAE_Match_Input pairings;

        // Correspondences 1/2: point layers:
        // -------------------------------------
        // Find correspondences for each point cloud "layer":
        for (size_t layer = 0; layer < nPointLayers; layer++)
        {
            const auto &m1 = pcs1.point_layers[layer],
                       &m2 = pcs2.point_layers[layer];
            ASSERT_(m1);
            ASSERT_(m2);

            // Find closest pairings
            mrpt::tfest::TMatchingPairList mpl;
            m1->determineMatching3D(m2.get(), solution, mpl, mp, mres[layer]);

            // merge lists:
            pairings.paired_points.insert(
                pairings.paired_points.end(), mpl.begin(), mpl.end());
        }

        // Correspondences 2/2: planes to planes
        // ---------------------------------------
        MRPT_TODO("find plane correspondences");
        THROW_EXCEPTION("to do!");

        // Ratio of points with a valid pairing:
        result.goodness = p.corresponding_points_decimation *
                          pairings.paired_points.size() /
                          static_cast<double>(pointcount1);

        if (pairings.empty())
        {
            // Nothing we can do !!
            result.terminationReason = IterTermReason::NoPairings;
            break;
        }

        // Compute the optimal pose, using the OLAE method
        // (Optimal linear attitude estimator)
        // ------------------------------------------------
        OLAE_Match_Result res;
        olae_match(pairings, res);

        solution = mrpt::poses::CPose3D(res.optimal_pose);

        // If matching has not changed, we are done:
        const auto                        deltaSol = solution - prev_solution;
        const mrpt::math::CArrayDouble<6> dSol =
            mrpt::poses::Lie::SE<3>::log(deltaSol);
        const double delta_xyz = dSol.head<3>().norm();
        const double delta_rot = dSol.tail<3>().norm();

        if (std::abs(delta_xyz) < p.minAbsStep_trans &&
            std::abs(delta_rot) < p.minAbsStep_rot)
        {
            result.terminationReason = IterTermReason::Stalled;
            break;
        }

        // Shuffle decimated points for next iter:
        if (++mp.offset_other_map_points >= p.corresponding_points_decimation)
            mp.offset_other_map_points = 0;

        prev_solution = solution;
    }

    if (result.nIterations >= p.maxIterations)
        result.terminationReason = IterTermReason::MaxIterations;

    // Store output:
    result.optimal_tf.mean = solution;
    MRPT_TODO("covariance of the estimation");

    MRPT_END
}

// Core of the OLAE algorithm.
// Factored out here so it can be re-evaluated if the solution is near the Gibbs
// vector singularity (|Phi|~= \pi)
// (Refer to technical report for details)
static std::tuple<Eigen::Matrix3d, Eigen::Vector3d> olae_build_linear_system(
    const PointsPlanesICP::OLAE_Match_Input& in,
    const mrpt::math::TPoint3D& ct_other, const mrpt::math::TPoint3D& ct_this,
    const bool do_relinearize_singularity)
{
    MRPT_START

    using mrpt::math::TPoint3D;
    using mrpt::math::TVector3D;

    // Build the linear system: M g = v
    Eigen::Matrix3d M = Eigen::Matrix3d::Zero();
    Eigen::Vector3d v = Eigen::Vector3d::Zero();

    const auto nPts        = in.paired_points.size();
    const auto nPlanes     = in.paired_planes.size();
    const auto nAllMatches = nPts + nPlanes;

    // Terms contributed by points & vectors have now the uniform form of
    // unit vectors:
    MRPT_TODO("different weights");
    const double wi_all = 1.0 / nAllMatches;

    for (std::size_t i = 0; i < nAllMatches; i++)
    {
        // Get "bi" (this/global) & "ri" (other/local) vectors:
        TVector3D bi, ri;

        if (i < nPts)
        {
            // point-to-point pairing:  normalize(point-centroid)
            const auto& p = in.paired_points[i];

            bi = TVector3D(p.this_x, p.this_y, p.this_z) - ct_this;
            ri = TVector3D(p.other_x, p.other_y, p.other_z) - ct_other;
            const auto bi_n = bi.norm(), ri_n = ri.norm();
            ASSERT_(bi_n > 1e-3);
            ASSERT_(ri_n > 1e-3);
            // (Note: ideally, both norms should be equal if noiseless and a
            // real pairing )

            bi *= 1.0 / bi_n;
            ri *= 1.0 / ri_n;
        }
        else
        {
            // plane-to-plane pairing:
            bi = in.paired_planes[i].p_this.plane.getNormalVector();
            ri = in.paired_planes[i].p_other.plane.getNormalVector();

            ASSERTDEB_BELOW_(std::abs(bi.norm() - 1.0), 0.01);
            ASSERTDEB_BELOW_(std::abs(ri.norm() - 1.0), 0.01);
        }

        // If we are in a second stage, let's relinearize around +PI, to avoid
        // working near the Gibbs vector singularity:
        if (do_relinearize_singularity)
        {
            // 180 deg change:
            ri = -ri;
        }

        const double wi = wi_all;

        // M+=(1/2)* ([s_i]_{x})^2
        // with: s_i = b_i + r_i
        const double sx = bi.x + ri.x, sy = bi.y + ri.y, sz = bi.z + ri.z;

        /* ([s_i]_{x})^2 is:
         *
         *  ⎡    2     2                          ⎤
         *  ⎢- sy  - sz      sx⋅sy        sx⋅sz   ⎥
         *  ⎢                                     ⎥
         *  ⎢                 2     2             ⎥
         *  ⎢   sx⋅sy     - sx  - sz      sy⋅sz   ⎥
         *  ⎢                                     ⎥
         *  ⎢                              2     2⎥
         *  ⎣   sx⋅sz        sy⋅sz     - sx  - sy ⎦
         */
        const double c00 = -sy * sy - sz * sz;
        const double c11 = -sx * sx - sz * sz;
        const double c22 = -sx * sx - sy * sy;
        const double c01 = sx * sy;
        const double c02 = sx * sz;
        const double c12 = sy * sz;

        MRPT_TODO("vectorize");

        M(0, 0) += wi * c00;
        M(1, 1) += wi * c11;
        M(2, 2) += wi * c22;
        M(0, 1) += wi * c01;
        M(1, 0) += wi * c01;
        M(0, 2) += wi * c02;
        M(2, 0) += wi * c02;
        M(1, 2) += wi * c12;
        M(2, 1) += wi * c12;

        /* v-= [b_i]_{x}  r_i
         *  Each term is:
         *  ⎡by⋅rz - bz⋅ry ⎤
         *  ⎢              ⎥
         *  ⎢-bx⋅rz + bz⋅rx⎥
         *  ⎢              ⎥
         *  ⎣bx⋅ry - by⋅rx ⎦
         */
        v[0] += wi * (bi.y * ri.z - bi.z * ri.y);
        v[1] += wi * (-bi.x * ri.z + bi.z * ri.x);
        v[2] += wi * (bi.x * ri.y - bi.y * ri.x);
    }

    // The missing (1/2) from the formulas above:
    M *= 0.5;

    return {M, v};
    //
    MRPT_END
}

// "Markley, F. L., & Mortari, D. (1999). How to estimate attitude from
// vector observations."
static double olae_estimate_Phi(const double M_det, std::size_t n)
{
    return std::acos((M_det / (n == 2 ? -1.0 : -1.178)) - 1.);
}

// See .h docs, and associated technical report.
void PointsPlanesICP::olae_match(
    const OLAE_Match_Input& in, OLAE_Match_Result& result)
{
    MRPT_START

    using mrpt::math::TPoint3D;
    using mrpt::math::TVector3D;

    // Note on notation: we are search the relative transformation of
    // the "other" frame wrt to "this", i.e. "this"="global",
    // "other"="local"

    const auto nPts        = in.paired_points.size();
    const auto nPlanes     = in.paired_planes.size();
    const auto nAllMatches = nPts + nPlanes;

    // Reset output to defaults:
    result = OLAE_Match_Result();

    // Find centroids:

    // Compute the centroids:
    TPoint3D ct_other(0, 0, 0), ct_this(0, 0, 0);

    // Add global coordinate of points for now, we'll convert them later to unit
    // vectors relative to the centroids:
    for (const auto& pair : in.paired_points)
    {
        ct_this += TPoint3D(pair.this_x, pair.this_y, pair.this_z);
        ct_other += TPoint3D(pair.other_x, pair.other_y, pair.other_z);
    }
    // Add plane centroids to the computation of centroids as well:
    for (const auto& pair : in.paired_planes)
    {
        ct_this += pair.p_this.centroid;
        ct_other += pair.p_other.centroid;
    }

    // Normalize sum of centroids:
    const double F = 1.0 / nAllMatches;
    ct_other *= F;
    ct_this *= F;

    // Build the linear system: M g = v
    const auto [M, v] = olae_build_linear_system(
        in, ct_other, ct_this, false /*dont relinearize singularity*/);

    // We are finding this optimal rotation, as a Gibbs vector:
    Eigen::Vector3d optimal_rot;

    // Solve linear system for optimal rotation:
    const double Md = M.det();

    // Estimate |Phi|:
    const double estPhi1 = olae_estimate_Phi(Md, nAllMatches);

    std::cout << "|M|=" << Md << " estimated |Phi|=" << mrpt::RAD2DEG(estPhi1)
              << "\n";
    // Any threshold [90-180] degrees would work.
    if (estPhi1 > mrpt::DEG2RAD(160.0))
    {
        // relinearize on +180 degrees:
        const auto [M2, v2] = olae_build_linear_system(
            in, ct_other, ct_this, true /*DO relinearize singularity*/);

        const double M2d     = M2.det();
        const double estPhi2 = olae_estimate_Phi(M2d, nAllMatches);

        std::cout << "|M2|=" << M2d
                  << " estimated |Phi2|=" << mrpt::RAD2DEG(estPhi2) << "\n";

        // Find the optimal Gibbs vector:
        optimal_rot = M2.colPivHouseholderQr().solve(v2);

        // Undo the 180 deg rotation:
        optimal_rot = -optimal_rot;
    }
    else
    {
        // Find the optimal Gibbs vector:
        optimal_rot = M.colPivHouseholderQr().solve(v);
    }

    // Convert to quaternion by normalizing q=[1, optim_rot]:
    {
        auto       x = optimal_rot[0], y = optimal_rot[1], z = optimal_rot[2];
        const auto r = 1.0 / std::sqrt(1.0 + x * x + y * y + z * z);
        x *= r;
        y *= r;
        z *= r;
        auto q = mrpt::math::CQuaternionDouble(r, x, y, z);

        // Quaternion to 3x3 rot matrix:
        result.optimal_pose = mrpt::poses::CPose3D(q, .0, .0, .0);
    }

    // Use centroids to solve for optimal translation:
    mrpt::math::TPoint3D pp;
    result.optimal_pose.composePoint(
        ct_other.x, ct_other.y, ct_other.z, pp.x, pp.y, pp.z);

    result.optimal_pose.x(ct_this.x - pp.x);
    result.optimal_pose.y(ct_this.y - pp.y);
    result.optimal_pose.z(ct_this.z - pp.z);

    MRPT_END
}
