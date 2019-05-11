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
    //

    MRPT_END
}
