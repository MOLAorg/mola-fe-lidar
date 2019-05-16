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
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/poses/Lie/SE.h>
#include <mrpt/tfest/se3.h>
#include <p2p2/PointsPlanesICP.h>

using namespace p2p2;

void PointsPlanesICP::align(
    const PointsPlanesICP::pointcloud_t& pcs1,
    const PointsPlanesICP::pointcloud_t& pcs2,
    const mrpt::math::TPose3D& init_guess_m2_wrt_m1, const Parameters& p,
    Results& result)
{
    using namespace std::string_literals;

    MRPT_START
    // ICP uses KD-trees.
    // kd-trees have each own mutexes to ensure well-defined behavior in
    // multi-threading apps.

    ASSERT_EQUAL_(pcs1.point_layers.size(), pcs2.point_layers.size());
    ASSERT_(
        !pcs1.point_layers.empty() ||
        (!pcs1.planes.empty() && !pcs2.planes.empty()));

    // Reset output:
    result = Results();

    // Matching params for point-to-point:
    mrpt::maps::TMatchingParams mp;
    // Distance threshold
    mp.maxDistForCorrespondence = p.thresholdDist;
    // Angular threshold
    mp.maxAngularDistForCorrespondence = p.thresholdAng;
    mp.onlyKeepTheClosest              = true;
    mp.onlyUniqueRobust                = false;

    // For decimation: cycle through all possible points, even if we decimate
    // them, in such a way that different points are used in each iteration.
    mp.offset_other_map_points = 0;

    // Matching params for plane-to-plane (their centroids only at this point):
    mrpt::maps::TMatchingParams mp_planes;
    // Distance threshold
    mp_planes.maxDistForCorrespondence =
        p.thresholdDist +
        1.0 /* since plane centroids must not show up at the same location */;
    // Angular threshold
    mp_planes.maxAngularDistForCorrespondence = p.thresholdAng;
    mp_planes.onlyKeepTheClosest              = true;
    mp_planes.decimation_other_map_points     = 1;

    // Count of points:
    size_t pointcount1 = 0, pointcount2 = 0;
    for (const auto& kv1 : pcs1.point_layers)
    {
        pointcount1 += kv1.second->size();
        pointcount2 += pcs2.point_layers.at(kv1.first)->size();
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
        std::map<std::string, mrpt::maps::TMatchingExtraResults> mres;

        // Measure angle distances from the current estimate:
        mp.angularDistPivotPoint = mrpt::math::TPoint3D(solution.asTPose());

        // the global list of pairings:
        OLAE_Match_Input pairings;

        // Correspondences for each point layer:
        // ---------------------------------------
        // Find correspondences for each point cloud "layer":
        for (const auto& kv1 : pcs1.point_layers)
        {
            const auto &m1 = kv1.second, &m2 = pcs2.point_layers.at(kv1.first);
            ASSERT_(m1);
            ASSERT_(m2);

            const bool is_layer_of_planes = (kv1.first == "plane_centroids"s);

            // Decimation rate:
            mp.decimation_other_map_points = std::max(
                1U,
                static_cast<unsigned>(m1->size() / p.max_corresponding_points));

            // Find closest pairings
            mrpt::tfest::TMatchingPairList mpl;
            m1->determineMatching3D(
                m2.get(), solution, mpl, is_layer_of_planes ? mp_planes : mp,
                mres[kv1.first]);

            // merge lists:
            // handle specially the plane-to-plane matching:
            if (!is_layer_of_planes)
            {
                // A standard layer: point-to-point correspondences:
                pairings.paired_points.insert(
                    pairings.paired_points.end(), mpl.begin(), mpl.end());
            }
            else
            {
                // Plane-to-plane correspondence:

                // We have pairs of planes whose centroids are quite close.
                // Check their normals too:
                for (const auto& pair : mpl)
                {
                    // 1) Check fo pairing sanity:
                    ASSERTDEB_(pair.this_idx < pcs1.planes.size());
                    ASSERTDEB_(pair.other_idx < pcs2.planes.size());

                    const auto& p1 = pcs1.planes[pair.this_idx];
                    const auto& p2 = pcs2.planes[pair.other_idx];

                    const mrpt::math::TVector3D n1 = p1.plane.getNormalVector();
                    const mrpt::math::TVector3D n2 = p2.plane.getNormalVector();

                    // dot product to find the angle between normals:
                    const double dp = n1.x * n2.x + n1.y * n2.y + n1.z * n2.z;
                    const double n2n_ang = std::acos(dp);

                    // 2) append to list of plane pairs:
                    MRPT_TODO("planes disabled here!");
                    if (n2n_ang < 0)  // mrpt::DEG2RAD(5.0))
                    {
                        // Accept pairing:
                        pairings.paired_planes.emplace_back(p1, p2);
                    }
                }
            }
        }

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

        // Weights: translation => trust points; attitude => trust planes
        pairings.weights.translation.planes = 0.0;
        pairings.weights.translation.points = 1.0;
        pairings.weights.attitude.planes    = 0.2;
        pairings.weights.attitude.points    = 1.0;

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
        // if (++mp.offset_other_map_points >=
        // p.corresponding_points_decimation) mp.offset_other_map_points = 0;

        prev_solution = solution;
    }

    if (result.nIterations >= p.maxIterations)
        result.terminationReason = IterTermReason::MaxIterations;

    // Store output:
    result.optimal_tf.mean = solution;
    MRPT_TODO("covariance of the estimation");

    MRPT_END
}

static auto vector_rot_Z_90d_CCW(const mrpt::math::TVector3D& v)
{
    return mrpt::math::TVector3D(-v.y, v.x, v.z);
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

    const auto nPoints     = in.paired_points.size();
    const auto nLines      = in.paired_lines.size();
    const auto nPlanes     = in.paired_planes.size();
    const auto nAllMatches = nPoints + nPlanes;

    // Normalized weights for attitude "waXX":
    double waPoints, waLines, waPlanes;
    {
        const auto wPt = in.weights.attitude.points,
                   wLi = in.weights.attitude.lines,
                   wPl = in.weights.attitude.planes;

        ASSERTMSG_(
            wPt + wLi + wPl > .0,
            "All, point, line, plane attidude weights, are <=0 (!)");

        const auto k = 1.0 / (wPt * nPoints + wLi * nLines + wPl * nPlanes);
        waPoints     = wPt * k;
        waLines      = wLi * k;
        waPlanes     = wPl * k;
    }

    // Accumulator of robust kernel terms to normalize at the end:
    double robust_w_sum = .0;

    // Terms contributed by points & vectors have now the uniform form of
    // unit vectors:

    for (std::size_t i = 0; i < nAllMatches; i++)
    {
        // Get "bi" (this/global) & "ri" (other/local) vectors:
        TVector3D bi, ri;
        double    wi = .0;

        // Points, lines, planes, are all stored in sequence:
        if (i < nPoints)
        {
            // point-to-point pairing:  normalize(point-centroid)
            const auto& p = in.paired_points[i];
            wi            = waPoints;

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
        else if (i < nPoints + nLines)
        {
            // line-to-line pairing:
            wi = waLines;

            const auto idxLine = i - nPoints;
            MRPT_TODO("handle lines");
            THROW_EXCEPTION("handle lines");
        }
        else
        {
            // plane-to-plane pairing:
            wi = waPlanes;

            const auto idxPlane = i - (nPoints + nLines);
            bi = in.paired_planes[idxPlane].p_this.plane.getNormalVector();
            ri = in.paired_planes[idxPlane].p_other.plane.getNormalVector();

            ASSERTDEB_BELOW_(std::abs(bi.norm() - 1.0), 0.01);
            ASSERTDEB_BELOW_(std::abs(ri.norm() - 1.0), 0.01);
        }

        // If we are in a second stage, let's relinearize around a rotation
        // of +PI/2 along +Z, to avoid working near the Gibbs vector
        // singularity:
        if (do_relinearize_singularity)
        {
            // Rotate:
            ri = vector_rot_Z_90d_CCW(ri);
        }

        // Robust kernel:
        if (in.use_robust_kernel)
        {
            const double A = in.robust_kernel_param;
            const double B = in.robust_kernel_scale;
            const double ang =
                std::acos(ri.x * bi.x + ri.y * bi.y + ri.z * bi.z);
            double f = 1.0;
            if (ang > A) f = 1.0 / (1.0 + B * mrpt::square(ang - A));
            wi *= f;
        }

        ASSERT_(wi > .0);
        robust_w_sum += wi;

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

        // clang-format off
        const auto dM = (Eigen::Matrix3d() <<
           c00, c01, c02,
           c01, c11, c12,
           c02, c12, c22 ).finished();
        // clang-format on

        /* v-= [b_i]_{x}  r_i
         *  Each term is:
         *  ⎡by⋅rz - bz⋅ry ⎤
         *  ⎢              ⎥
         *  ⎢-bx⋅rz + bz⋅rx⎥
         *  ⎢              ⎥
         *  ⎣bx⋅ry - by⋅rx ⎦
         */

        // clang-format off
        const auto dV = (Eigen::Vector3d() <<
           (bi.y * ri.z - bi.z * ri.y),
           (-bi.x * ri.z + bi.z * ri.x),
           (bi.x * ri.y - bi.y * ri.x) ).finished();
        // clang-format on

        M += wi * dM;
        v -= wi * dV;
    }

    if (robust_w_sum)
    {
        M *= (1.0 / robust_w_sum);
        v *= (1.0 / robust_w_sum);
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
    // "other"="local":
    //   p_this = pose \oplus p_other
    //   p_A    = pose \oplus p_B      --> pB = p_A \ominus pose

    const auto nPoints     = in.paired_points.size();
    const auto nLines      = in.paired_lines.size();
    const auto nPlanes     = in.paired_planes.size();
    const auto nAllMatches = nPoints + nLines + nPlanes;

    // Reset output to defaults:
    result = OLAE_Match_Result();

    // Normalize weights for each feature type and for each target (attitude
    // / translation):
    ASSERT_(in.weights.attitude.points >= .0);
    ASSERT_(in.weights.attitude.lines >= .0);
    ASSERT_(in.weights.attitude.planes >= .0);
    ASSERT_(in.weights.translation.points >= .0);
    ASSERT_(in.weights.translation.planes >= .0);

    // Normalized weights for centroid "wcXX":
    double wcPoints, wcPlanes;
    {
        const auto wPt = in.weights.translation.points,
                   wPl = in.weights.translation.planes;

        ASSERTMSG_(
            wPt + wPl > .0,
            "Both, point and plane translation weights, are <=0 (!)");

        const auto k = 1.0 / (wPt * nPoints + wPl * nPlanes);
        wcPoints     = wPt * k;
        wcPlanes     = wPl * k;
    }

    // Compute the centroids:
    TPoint3D ct_other(0, 0, 0), ct_this(0, 0, 0);

    // Add global coordinate of points for now, we'll convert them later to
    // unit vectors relative to the centroids:
    {
        TPoint3D ct_other_pt(0, 0, 0), ct_this_pt(0, 0, 0);

        for (const auto& pair : in.paired_points)
        {
            ct_this_pt += TPoint3D(pair.this_x, pair.this_y, pair.this_z);
            ct_other_pt += TPoint3D(pair.other_x, pair.other_y, pair.other_z);
        }
        ct_other_pt *= wcPoints;
        ct_this_pt *= wcPoints;

        // Add plane centroids to the computation of centroids as well:
        TPoint3D ct_other_pl(0, 0, 0), ct_this_pl(0, 0, 0);
        if (wcPlanes > 0)
        {
            for (const auto& pair : in.paired_planes)
            {
                ct_this_pl += pair.p_this.centroid;
                ct_other_pl += pair.p_other.centroid;
            }
            ct_this_pl *= wcPlanes;
            ct_other_pl *= wcPlanes;
        }

        // Normalize sum of centroids:
        ct_other = ct_other_pt + ct_other_pl;
        ct_this  = ct_this_pt + ct_this_pl;
    }

    // Build the linear system: M g = v
    const auto [M, v] = olae_build_linear_system(
        in, ct_other, ct_this, false /*dont relinearize singularity*/);

    // We are finding this optimal rotation, as a Gibbs vector:
    Eigen::Vector3d optimal_rot;

    // Solve linear system for optimal rotation:
    const double Md = M.det();

    // Estimate |Phi|:
    const double estPhi1 = olae_estimate_Phi(Md, nAllMatches);

    // Threshold to decide whether to do a re-linearization:
    const bool do_relinearize = (estPhi1 > mrpt::DEG2RAD(150.0));

    if (do_relinearize)
    {
        // relinearize on a different orientation:
        const auto [M2, v2] = olae_build_linear_system(
            in, ct_other, ct_this, true /*DO relinearize singularity*/);

        // Find the optimal Gibbs vector:
        optimal_rot = M2.colPivHouseholderQr().solve(v2);
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
        auto q = mrpt::math::CQuaternionDouble(r, -x, -y, -z);

        // Quaternion to 3x3 rot matrix:
        result.optimal_pose = mrpt::poses::CPose3D(q, .0, .0, .0);
    }
    // Undo transformation above:
    if (do_relinearize)
    {
        result.optimal_pose = result.optimal_pose +
                              mrpt::poses::CPose3D(0, 0, 0, M_PI * 0.5, 0, 0);
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

/** Gets a renderizable view of all planes */
void PointsPlanesICP::pointcloud_t::planesAsRenderizable(
    mrpt::opengl::CSetOfObjects& o, const PointsPlanesICP::render_params_t& p)
{
    MRPT_START

    const float pw = p.plane_half_width, pf = p.plane_grid_spacing;

    for (const auto& plane : planes)
    {
        auto gl_pl =
            mrpt::opengl::CGridPlaneXY::Create(-pw, pw, -pw, pw, .0, pf);
        gl_pl->setColor_u8(p.plane_color);
        mrpt::math::TPose3D planePose;
        plane.plane.getAsPose3DForcingOrigin(plane.centroid, planePose);
        gl_pl->setPose(planePose);
        o.insert(gl_pl);
    }

    MRPT_END
}
