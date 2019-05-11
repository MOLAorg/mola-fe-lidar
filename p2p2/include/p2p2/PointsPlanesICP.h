/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   PointsPlanesICP.h
 * @brief  ICP registration for points and planes
 * @author Jose Luis Blanco Claraco
 * @date   May 11, 2019
 */
#pragma once

#include <mrpt/maps/CPointsMap.h>
#include <cstdint>
#include "IterTermReason.h"
#include "Parameters.h"
#include "Results.h"

/** ICP registration for points and planes
 *
 * \ingroup mola_fe_lidar_icp_grp */
namespace p2p2::PointsPlanesICP
{
using clouds_t = int;  // XXX

void align(
    const clouds_t& pcs1, const clouds_t& pcs2,
    const mrpt::math::TPose3D& init_guess_m2_wrt_m1, const Parameters& p,
    Results& result);

}  // namespace p2p2::PointsPlanesICP
