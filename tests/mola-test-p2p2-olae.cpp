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
#include <p2p2/PointsPlanesICP.h>
#include <sstream>
// To refactor!
#include <mrpt/math/ops_vectors.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/poses/Lie/SE.h>
#include <mrpt/random.h>
#include <mrpt/system/CTicTac.h>

using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::tfest;
using namespace mrpt::random;
using namespace mrpt::poses;
using namespace std;

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

        for (int j = 0; j < 4; j++)
            plA[i].plane = mrpt::math::TPlane(
                plA[i].centroid,
                mrpt::math::TVector3D(
                    rnd.drawUniform(-1.0, 1.0), rnd.drawUniform(-1.0, 1.0),
                    rnd.drawUniform(-1.0, 1.0)));
        plA[i].plane.unitarize();
    }
    return plA;
}

// transform features
mrpt::poses::CPose3D transform_points_planes(
    const TPoints& pA, TPoints& pB, TMatchingPairList& pointsPairs,
    const TPlanes& plA, TPlanes& plB,
    std::vector<p2p2::PointsPlanesICP::matched_planes_t>& planePairs,
    const double xyz_noise_std, const double plane_normal_noise_std)
{
    auto& rnd = mrpt::random::getRandomGenerator();

    const double Dx = rnd.drawUniform(-10.0, 10.0);
    const double Dy = rnd.drawUniform(-10.0, 10.0);
    const double Dz = rnd.drawUniform(-10.0, 10.0);

    const double yaw   = DEG2RAD(rnd.drawUniform(-180.0, 180.0));
    const double pitch = DEG2RAD(rnd.drawUniform(-90.0, 90.0));
    const double roll  = DEG2RAD(rnd.drawUniform(-90.0, 90.0));

    const auto pose = CPose3D(Dx, Dy, Dz, yaw, pitch, roll);
    // just the rotation, to transform vectors (vs. R^3 points):
    const auto pose_rot_only = CPose3D(0, 0, 0, yaw, pitch, roll);

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
        TMatchingPair pair;
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
        // Transform + noise:
        pose.inverseComposePoint(plA[i].centroid, plB[i].centroid);
        plB[i].plane = plA[i].plane;

        // Rotate a plane:
        {
            const mrpt::math::TVector3D ul = plB[i].plane.getNormalVector();
            mrpt::math::TVector3D       ug;
            pose_rot_only.inverseComposePoint(ul, ug);

            // Ax+By+Cz+D=0
            plB[i].plane.coefs[0] = ug.x;
            plB[i].plane.coefs[1] = ug.y;
            plB[i].plane.coefs[2] = ug.z;
            plB[i].plane.coefs[3] =
                -(ug.x * plB[i].centroid.x + ug.y * plB[i].centroid.y +
                  ug.z * plB[i].centroid.z);
        }

        // noise:
        plB[i].centroid.x += rnd.drawGaussian1D(0, xyz_noise_std);
        plB[i].centroid.y += rnd.drawGaussian1D(0, xyz_noise_std);
        plB[i].centroid.z += rnd.drawGaussian1D(0, xyz_noise_std);

        plB[i].plane.unitarize();
        for (int k = 0; k < 3; k++)
            plB[i].plane.coefs[k] +=
                rnd.drawGaussian1D(0, plane_normal_noise_std);

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
    const double xyz_noise_std = .0, const double normals_noise_std = .0)
{
    MRPT_START

    std::cout << "[TEST_p2p2_olae] numPts=" << numPts
              << " numPlanes=" << numPlanes
              << " xyz_noise_std=" << xyz_noise_std
              << " normals_noise_std= " << normals_noise_std << "\n";

    // The input points & planes
    const TPoints pA  = generate_points(numPts);
    const TPlanes plA = generate_planes(numPlanes);

    TPoints pB;
    TPlanes plB;

    TMatchingPairList                         pointPairs;
    p2p2::PointsPlanesICP::TMatchedPlanesList planePairs;

    const CPose3D gt_pose = transform_points_planes(
        pA, pB, pointPairs, plA, plB, planePairs, xyz_noise_std,
        normals_noise_std);

    // test with the OLEA method:
    {
        p2p2::PointsPlanesICP::OLAE_Match_Result res;
        p2p2::PointsPlanesICP::OLAE_Match_Input  in;
        in.paired_points = pointPairs;
        in.paired_planes = planePairs;

        mrpt::system::CTicTac timer;
        timer.Tic();

        size_t num_reps = 1;  // 1000
        for (size_t rep = 0; rep < num_reps; rep++)
        {
            //
            p2p2::PointsPlanesICP::olae_match(in, res);
        }

        const double dt = timer.Tac() / num_reps;

        std::cout << "OLEA output       : " << res.optimal_pose.asString()
                  << "\n";
        std::cout << "Ground truth      : " << gt_pose.asString() << "\n";
        std::cout << "OLEA time: " << dt * 1e6
                  << " microseconds. numPts=" << numPts << "\n";

        const auto pos_error = gt_pose - res.optimal_pose;
        const auto err_log   = mrpt::poses::Lie::SE<3>::log(pos_error);
        ASSERT_BELOW_(err_log.norm(), 1e-3 + xyz_noise_std + normals_noise_std);
    }

    return true;  // all ok.
    MRPT_END
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        // Points only. Noiseless:
        ASSERT_(TEST_p2p2_olae(3 /*nPts*/, 0 /*nPlanes*/));
        ASSERT_(TEST_p2p2_olae(4 /*nPts*/, 0 /*nPlanes*/));
        ASSERT_(TEST_p2p2_olae(10 /*nPts*/, 0 /*nPlanes*/));
        ASSERT_(TEST_p2p2_olae(100 /*nPts*/, 0 /*nPlanes*/));
        ASSERT_(TEST_p2p2_olae(1000 /*nPts*/, 0 /*nPlanes*/));

        // Points only. Noisy:
        ASSERT_(TEST_p2p2_olae(100 /*numPts*/, 0 /*nPlanes*/, .1 /* noise*/));
        ASSERT_(TEST_p2p2_olae(1000 /*numPts*/, 0 /*nPlanes*/, .1 /* noise*/));

        // Planes only. Noiseless:
        ASSERT_(TEST_p2p2_olae(0 /*nPts*/, 3 /*nPlanes*/));
        ASSERT_(TEST_p2p2_olae(0 /*nPts*/, 10 /*nPlanes*/));
        ASSERT_(TEST_p2p2_olae(0 /*nPts*/, 100 /*nPlanes*/));

        // Planes only. Noisy:
        ASSERT_(TEST_p2p2_olae(0 /*nPts*/, 3 /*nPlanes*/, 0, 0.02));
        ASSERT_(TEST_p2p2_olae(0 /*nPts*/, 10 /*nPlanes*/, 0, 0.02));
        ASSERT_(TEST_p2p2_olae(0 /*nPts*/, 100 /*nPlanes*/, 0, 0.02));
    }
    catch (std::exception& e)
    {
        std::cerr << mrpt::exception_to_str(e) << "\n";
        return 1;
    }
}
