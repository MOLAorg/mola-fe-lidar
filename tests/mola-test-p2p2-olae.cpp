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
#include <mrpt/random.h>
#include <mrpt/tfest/se3.h>

using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::tfest;
using namespace mrpt::random;
using namespace mrpt::poses;
using namespace std;

using TPoints = std::vector<std::vector<double>>;

// ------------------------------------------------------
//				Generate both sets of points
// ------------------------------------------------------
CPose3DQuat generate_points(TPoints& pA, TPoints& pB)
{
    const double Dx = 10.1;
    const double Dy = 15.2;
    const double Dz = 18.3;

    const double yaw   = DEG2RAD(10.);
    const double pitch = DEG2RAD(0.);
    const double roll  = DEG2RAD(0.);

    pA.resize(5);  // A set of points at "A" reference system
    pB.resize(5);  // A set of points at "B" reference system

    pA[0].resize(3);
    pA[0][0] = 0.0;
    pA[0][1] = 0.5;
    pA[0][2] = 0.4;
    pA[1].resize(3);
    pA[1][0] = 1.0;
    pA[1][1] = 1.5;
    pA[1][2] = -0.1;
    pA[2].resize(3);
    pA[2][0] = 1.2;
    pA[2][1] = 1.1;
    pA[2][2] = 0.9;
    pA[3].resize(3);
    pA[3][0] = 0.7;
    pA[3][1] = 0.3;
    pA[3][2] = 3.4;
    pA[4].resize(3);
    pA[4][0] = 1.9;
    pA[4][1] = 2.5;
    pA[4][2] = -1.7;

    CPose3DQuat qPose = CPose3DQuat(CPose3D(Dx, Dy, Dz, yaw, pitch, roll));
    for (unsigned int i = 0; i < 5; ++i)
    {
        pB[i].resize(3);
        qPose.inverseComposePoint(
            pA[i][0], pA[i][1], pA[i][2], pB[i][0], pB[i][1], pB[i][2]);
    }

    return qPose;

}  // end generate_points

// ------------------------------------------------------
//				Generate a list of matched points
// ------------------------------------------------------
void generate_list_of_points(
    const TPoints& pA, const TPoints& pB, TMatchingPairList& list)
{
    TMatchingPair pair;
    for (unsigned int i = 0; i < 5; ++i)
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
}  // end generate_list_of_points

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

int TEST_p2p2_olae_basic()
{
    MRPT_START

    TPoints           pA, pB;  // The input points
    const CPose3DQuat qPose   = generate_points(pA, pB);
    const CPose3D     gt_pose = CPose3D(qPose);

    TMatchingPairList list;
    generate_list_of_points(pA, pB, list);  // Generate a list of matched points

    // 1st: test with the classic Horn method (from MRPT)
    {
        // Output CPose3DQuat for the LSRigidTransformation
        CPose3DQuat outQuat;
        // Output scale value
        double scale;

        bool res = mrpt::tfest::se3_l2(list, outQuat, scale);
        ASSERT_EQUAL_(res, true);

        double err = 0.0;
        if ((qPose[3] * outQuat[3] > 0 && qPose[4] * outQuat[4] > 0 &&
             qPose[5] * outQuat[5] > 0 && qPose[6] * outQuat[6] > 0) ||
            (qPose[3] * outQuat[3] < 0 && qPose[4] * outQuat[4] < 0 &&
             qPose[5] * outQuat[5] < 0 && qPose[6] * outQuat[6] < 0))
        {
            for (unsigned int i = 0; i < 7; ++i)
                err += square(std::fabs(qPose[i]) - std::fabs(outQuat[i]));
            err = sqrt(err);
            if (err > 1e-6)
            {
                std::stringstream s;
                s << "Applied quaternion: " << endl
                  << qPose << endl
                  << "Out CPose3DQuat: " << endl
                  << outQuat << " [Err: " << err << "]" << endl;
                THROW_EXCEPTION(s.str());
            }
        }
    }

    // Now, with the OLEA method:
    {
        p2p2::PointsPlanesICP::OLAE_Match_Result res;
        p2p2::PointsPlanesICP::OLAE_Match_Input  in;
        in.paired_points = list;

        p2p2::PointsPlanesICP::olae_match(in, res);

        std::cout << "OLEA output : " << res.optimal_pose.asString() << "\n";
        std::cout << "Ground truth: " << gt_pose.asString() << "\n";
    }

    return 0;
    MRPT_END
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
    try
    {
        return TEST_p2p2_olae_basic();
    }
    catch (std::exception& e)
    {
        std::cerr << mrpt::exception_to_str(e) << "\n";
        return 1;
    }
}
