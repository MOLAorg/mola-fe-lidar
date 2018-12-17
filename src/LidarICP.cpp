/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   LidarICP.cpp
 * @brief  Simple SLAM FrontEnd for point-cloud sensors via ICP registration
 * @author Jose Luis Blanco Claraco
 * @date   Dec 17, 2018
 */

/** \defgroup mola_fe_lidar_icp_grp mola-fe-lidar-icp.
 * Simple SLAM FrontEnd for point-cloud sensors via ICP registration.
 *
 *
 */

#include <mola-fe-lidar-icp/LidarICP.h>
#include <yaml-cpp/yaml.h>

using namespace mola;

MOLA_REGISTER_FRONTEND(LidarICP)

LidarICP::LidarICP() = default;

void LidarICP::initialize(const std::string& cfg_block)
{
    //
}
void LidarICP::spinOnce()
{
    //
}
void LidarICP::onNewObservation(CObservation::Ptr& o)
{
    //
}
