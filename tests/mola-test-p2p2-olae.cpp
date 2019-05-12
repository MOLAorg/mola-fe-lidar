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

// ------------------------------------------------------
//				Generate both sets of points
// ------------------------------------------------------
CPose3DQuat generate_points(
    TPoints& pA, TPoints& pB, const size_t nPts, const double noise_std)
{
    auto& rnd = mrpt::random::getRandomGenerator();

    const double Dx = rnd.drawUniform(-10.0, 10.0);
    const double Dy = rnd.drawUniform(-10.0, 10.0);
    const double Dz = rnd.drawUniform(-10.0, 10.0);

    const double yaw   = DEG2RAD(rnd.drawUniform(-180.0, 180.0));
    const double pitch = DEG2RAD(rnd.drawUniform(-90.0, 90.0));
    const double roll  = DEG2RAD(rnd.drawUniform(-90.0, 90.0));

    const auto qPose = CPose3DQuat(CPose3D(Dx, Dy, Dz, yaw, pitch, roll));

    pA.resize(nPts);

    for (size_t i = 0; i < nPts; i++)
    {
        pA[i].x = rnd.drawUniform(-10.0, 10.0);
        pA[i].y = rnd.drawUniform(-10.0, 10.0);
        pA[i].z = rnd.drawUniform(-10.0, 10.0);
    }

    pB.resize(nPts);
    for (unsigned int i = 0; i < nPts; ++i)
    {
        qPose.inverseComposePoint(
            pA[i][0], pA[i][1], pA[i][2], pB[i][0], pB[i][1], pB[i][2]);
        pB[i].x += rnd.drawGaussian1D(0, noise_std);
        pB[i].y += rnd.drawGaussian1D(0, noise_std);
        pB[i].z += rnd.drawGaussian1D(0, noise_std);
    }

    return qPose;
}

// ------------------------------------------------------
//				Generate a list of matched points
// ------------------------------------------------------
void generate_list_of_points(
    const TPoints& pA, const TPoints& pB, TMatchingPairList& list)
{
    TMatchingPair pair;
    for (unsigned int i = 0; i < pA.size(); ++i)
    {
        pair.this_idx = pair.other_idx = i;
        pair.this_x                    = pA[i][0];
        pair.this_y                    = pA[i][1];
        pair.this_z                    = pA[i][2];

        pair.other_x = pB[i][0];
        pair.other_y = pB[i][1];
        pair.other_z = pB[i][2];

        list.push_back(pair);
    }
}

// ------------------------------------------------------
//				Genreate a vector of matched points
// ------------------------------------------------------
void generate_vector_of_points(
    const TPoints& pA, const TPoints& pB, vector<mrpt::math::TPoint3D>& ptsA,
    vector<mrpt::math::TPoint3D>& ptsB)
{
    // The input vector: inV = [pA1x, pA1y, pA1z, pB1x, pB1y, pB1z, ... ]
    ptsA.resize(pA.size());
    ptsB.resize(pA.size());
    for (unsigned int i = 0; i < pA.size(); ++i)
    {
        ptsA[i] = mrpt::math::TPoint3D(pA[i][0], pA[i][1], pA[i][2]);
        ptsB[i] = mrpt::math::TPoint3D(pB[i][0], pB[i][1], pB[i][2]);
    }
}  // end generate_vector_of_points

bool TEST_p2p2_olae(const size_t numPts, const double noise_std)
{
    MRPT_START

    std::cout << "[TEST_p2p2_olae] numPts=" << numPts
              << " noise_std=" << noise_std << "\n";

    TPoints           pA, pB;  // The input points
    const CPose3DQuat qPose   = generate_points(pA, pB, numPts, noise_std);
    const CPose3D     gt_pose = CPose3D(qPose);

    TMatchingPairList list;
    generate_list_of_points(pA, pB, list);  // Generate a list of matched points

    // test with the OLEA method:
    {
        p2p2::PointsPlanesICP::OLAE_Match_Result res;
        p2p2::PointsPlanesICP::OLAE_Match_Input  in;
        in.paired_points = list;

        mrpt::system::CTicTac timer;
        timer.Tic();

        size_t num_reps = 1000;
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
        ASSERT_BELOW_(err_log.norm(), 1e-3 + noise_std);
    }

    return true;  // all ok.
    MRPT_END
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        // Noiseless:
        ASSERT_(TEST_p2p2_olae(3 /*numPts*/, .0 /* noise*/));
        ASSERT_(TEST_p2p2_olae(4 /*numPts*/, .0 /* noise*/));
        ASSERT_(TEST_p2p2_olae(10 /*numPts*/, .0 /* noise*/));
        ASSERT_(TEST_p2p2_olae(100 /*numPts*/, .0 /* noise*/));
        ASSERT_(TEST_p2p2_olae(1000 /*numPts*/, .0 /* noise*/));

        // Noisy:
        ASSERT_(TEST_p2p2_olae(100 /*numPts*/, .1 /* noise*/));
        ASSERT_(TEST_p2p2_olae(1000 /*numPts*/, .1 /* noise*/));
    }
    catch (std::exception& e)
    {
        std::cerr << mrpt::exception_to_str(e) << "\n";
        return 1;
    }
}
