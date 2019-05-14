/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * @file   mola-test-p2p2-olae.cpp
 * @brief  Unit tests for the P2P2 OLAE solver
 * @author Jose Luis Blanco Claraco
 * @date   May 12, 2019
 */

#include <mrpt/core/exceptions.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/poses/Lie/SE.h>
#include <mrpt/random.h>
#include <mrpt/system/CTicTac.h>
#include <p2p2/PointsPlanesICP.h>
#include <sstream>

using TPoints = std::vector<mrpt::math::TPoint3D>;
using TPlanes = std::vector<p2p2::PointsPlanesICP::plane_patch_t>;

TPoints generate_points(const size_t nPts)
{
    auto& rnd = mrpt::random::getRandomGenerator();

    TPoints pA;
    pA.resize(nPts);

    for (size_t i = 0; i < nPts; i++)
    {
        pA[i].x = rnd.drawUniform(-10.0, 10.0);
        pA[i].y = rnd.drawUniform(-10.0, 10.0);
        pA[i].z = rnd.drawUniform(-10.0, 10.0);
    }
    return pA;
}

TPlanes generate_planes(const size_t nPlanes)
{
    auto& rnd = mrpt::random::getRandomGenerator();

    TPlanes plA;
    plA.resize(nPlanes);

    for (size_t i = 0; i < nPlanes; i++)
    {
        plA[i].centroid.x = rnd.drawUniform(-10.0, 10.0);
        plA[i].centroid.y = rnd.drawUniform(-10.0, 10.0);
        plA[i].centroid.z = rnd.drawUniform(-10.0, 10.0);

        auto n = mrpt::math::TVector3D(
            rnd.drawUniform(-1.0, 1.0), rnd.drawUniform(-1.0, 1.0),
            rnd.drawUniform(-1.0, 1.0));
        n *= 1.0 / n.norm();

        plA[i].plane = mrpt::math::TPlane(plA[i].centroid, n);
    }
    return plA;
}

// transform features
mrpt::poses::CPose3D transform_points_planes(
    const TPoints& pA, TPoints& pB, mrpt::tfest::TMatchingPairList& pointsPairs,
    const TPlanes& plA, TPlanes& plB,
    std::vector<p2p2::PointsPlanesICP::matched_planes_t>& planePairs,
    const double xyz_noise_std, const double n_err_std /* normals noise*/)
{
    auto& rnd = mrpt::random::getRandomGenerator();

    const double Dx = rnd.drawUniform(-10.0, 10.0);
    const double Dy = rnd.drawUniform(-10.0, 10.0);
    const double Dz = rnd.drawUniform(-10.0, 10.0);

    const double yaw   = mrpt::DEG2RAD(rnd.drawUniform(-180.0, 180.0));
    const double pitch = mrpt::DEG2RAD(rnd.drawUniform(-90.0, 90.0));
    const double roll  = mrpt::DEG2RAD(rnd.drawUniform(-90.0, 90.0));

    const auto pose = mrpt::poses::CPose3D(Dx, Dy, Dz, yaw, pitch, roll);
    // just the rotation, to transform vectors (vs. R^3 points):
    const auto pose_rot_only = mrpt::poses::CPose3D(0, 0, 0, yaw, pitch, roll);

    // Points:
    pB.resize(pA.size());
    for (std::size_t i = 0; i < pA.size(); ++i)
    {
        // Transform + noise:
        pose.inverseComposePoint(pA[i], pB[i]);
        pB[i].x += rnd.drawGaussian1D(0, xyz_noise_std);
        pB[i].y += rnd.drawGaussian1D(0, xyz_noise_std);
        pB[i].z += rnd.drawGaussian1D(0, xyz_noise_std);

        // Add pairing:
        mrpt::tfest::TMatchingPair pair;
        pair.this_idx = pair.other_idx = i;
        pair.this_x                    = pA[i][0];
        pair.this_y                    = pA[i][1];
        pair.this_z                    = pA[i][2];

        pair.other_x = pB[i][0];
        pair.other_y = pB[i][1];
        pair.other_z = pB[i][2];

        pointsPairs.push_back(pair);
    }

    // Planes: transform + noise
    plB.resize(plA.size());
    for (std::size_t i = 0; i < plA.size(); ++i)
    {
        // Centroid: transform + noise
        plB[i].centroid = pose.inverseComposePoint(plA[i].centroid);

        plB[i].centroid.x += rnd.drawGaussian1D(0, xyz_noise_std);
        plB[i].centroid.y += rnd.drawGaussian1D(0, xyz_noise_std);
        plB[i].centroid.z += rnd.drawGaussian1D(0, xyz_noise_std);

        // Plane: rotate + noise
        plB[i].plane = plA[i].plane;
        {
            const mrpt::math::TVector3D ug = plA[i].plane.getNormalVector();
            mrpt::math::TVector3D       ul;
            pose_rot_only.inverseComposePoint(ug, ul);

            auto& coefs = plB[i].plane.coefs;

            // Ax+By+Cz+D=0
            coefs[0] = ul.x + rnd.drawGaussian1D(0, n_err_std);
            coefs[1] = ul.y + rnd.drawGaussian1D(0, n_err_std);
            coefs[2] = ul.z + rnd.drawGaussian1D(0, n_err_std);
            coefs[3] = 0;  // temporary.
            plB[i].plane.unitarize();

            coefs[3] =
                -(coefs[0] * plB[i].centroid.x + coefs[1] * plB[i].centroid.y +
                  coefs[2] * plB[i].centroid.z);
        }

        // Add pairing:
        p2p2::PointsPlanesICP::matched_planes_t pair;
        pair.p_this  = plA[i];
        pair.p_other = plB[i];
        planePairs.push_back(pair);
    }

    return pose;
}

bool TEST_p2p2_olae(
    const size_t numPts, const size_t numPlanes,
    const double xyz_noise_std = .0, const double n_err_std = .0)
{
    MRPT_START

    std::cout << "[TEST_p2p2_olae] numPts=" << numPts
              << " numPlanes=" << numPlanes
              << " xyz_noise_std=" << xyz_noise_std
              << " n_err_std= " << n_err_std << "\n";

    // The input points & planes
    const TPoints pA  = generate_points(numPts);
    const TPlanes plA = generate_planes(numPlanes);

    TPoints pB;
    TPlanes plB;

    mrpt::tfest::TMatchingPairList            pointPairs;
    p2p2::PointsPlanesICP::TMatchedPlanesList planePairs;

    const mrpt::poses::CPose3D gt_pose = transform_points_planes(
        pA, pB, pointPairs, plA, plB, planePairs, xyz_noise_std, n_err_std);

    // test with the OLEA method:
    {
        p2p2::PointsPlanesICP::OLAE_Match_Result res;
        p2p2::PointsPlanesICP::OLAE_Match_Input  in;
        in.paired_points = pointPairs;
        in.paired_planes = planePairs;

        mrpt::system::CTicTac timer;
        timer.Tic();

        size_t num_reps = 1000;
        for (size_t rep = 0; rep < num_reps; rep++)
        {
            //
            p2p2::PointsPlanesICP::olae_match(in, res);
        }

        const double dt = timer.Tac() / num_reps;

        const auto pos_error = gt_pose - res.optimal_pose;
        const auto err_log   = mrpt::poses::Lie::SE<3>::log(pos_error);

        std::cout << " -Ground_truth : " << gt_pose.asString() << "\n"
                  << " -OLEA_output  : " << res.optimal_pose.asString() << "\n"
                  << " -Error        : " << err_log.norm()
                  << "  Time: " << dt * 1e6 << " [us]\n";

        ASSERT_BELOW_(err_log.norm(), 1e-3 + xyz_noise_std + 40 * n_err_std);
    }

    return true;  // all ok.
    MRPT_END
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        auto& rnd = mrpt::random::getRandomGenerator();
        rnd.randomize();

        const double nXYZ = 0.1;  // [meters] std. noise of XYZ points
        const double nN   = mrpt::DEG2RAD(0.1);  // normals noise

        // Points only. Noiseless:
        ASSERT_(TEST_p2p2_olae(3 /*nPts*/, 0 /*nPlanes*/));
        ASSERT_(TEST_p2p2_olae(4 /*nPts*/, 0 /*nPlanes*/));
        ASSERT_(TEST_p2p2_olae(10 /*nPts*/, 0 /*nPlanes*/));
        ASSERT_(TEST_p2p2_olae(100 /*nPts*/, 0 /*nPlanes*/));
        ASSERT_(TEST_p2p2_olae(1000 /*nPts*/, 0 /*nPlanes*/));

        // Points only. Noisy:
        ASSERT_(TEST_p2p2_olae(100 /*numPts*/, 0 /*nPlanes*/, nXYZ));
        ASSERT_(TEST_p2p2_olae(1000 /*numPts*/, 0 /*nPlanes*/, nXYZ));

        // Planes only. Noiseless:
        ASSERT_(TEST_p2p2_olae(0 /*nPts*/, 3 /*nPlanes*/));
        ASSERT_(TEST_p2p2_olae(0 /*nPts*/, 10 /*nPlanes*/));
        ASSERT_(TEST_p2p2_olae(0 /*nPts*/, 100 /*nPlanes*/));

        // Planes only. Noisy:
        ASSERT_(TEST_p2p2_olae(0 /*nPts*/, 3 /*nPlanes*/, 0, nN));
        ASSERT_(TEST_p2p2_olae(0 /*nPts*/, 10 /*nPlanes*/, 0, nN));
        ASSERT_(TEST_p2p2_olae(0 /*nPts*/, 100 /*nPlanes*/, 0, nN));

        // Points and planes, noisy.
        ASSERT_(TEST_p2p2_olae(1 /*nPts*/, 3 /*nPlanes*/));
        ASSERT_(TEST_p2p2_olae(2 /*nPts*/, 1 /*nPlanes*/));
        ASSERT_(TEST_p2p2_olae(20 /*nPts*/, 10 /*nPlanes*/, nXYZ, nN));
        ASSERT_(TEST_p2p2_olae(400 /*nPts*/, 100 /*nPlanes*/, nXYZ, nN));
    }
    catch (std::exception& e)
    {
        std::cerr << mrpt::exception_to_str(e) << "\n";
        return 1;
    }
}
