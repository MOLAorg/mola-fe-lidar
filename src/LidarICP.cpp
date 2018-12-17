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
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/system/datetime.h>
#include <yaml-cpp/yaml.h>

using namespace mola;

MOLA_REGISTER_FRONTEND(LidarICP)

LidarICP::LidarICP() = default;

void LidarICP::initialize(const std::string& cfg_block)
{
    //
    MRPT_TODO("Load params into params_");

    // Default params:
    params_.mrpt_icp.corresponding_points_decimation = 10;
}
void LidarICP::spinOnce()
{
    //
}

void LidarICP::reset() { state_ = MethodState(); }

void LidarICP::onNewObservation(CObservation::Ptr& o)
{
    worker_pool_.enqueue(&LidarICP::doProcessNewObservation, this, o);
}

// here happens the main stuff:
void LidarICP::doProcessNewObservation(CObservation::Ptr& o)
{
    MRPT_TRY_START
    ASSERT_(o);

    // Only process pointclouds that are sufficiently apart in time:
    const auto this_obs_tim = o->timestamp;
    if (state_.last_obs_tim != mrpt::Clock::time_point() &&
        mrpt::system::timeDifference(state_.last_obs_tim, this_obs_tim) <
            params_.min_time_between_scans_)
    {
        // Drop observation.
        return;
    }

    auto       this_obs_points = mrpt::maps::CSimplePointsMap::Create();
    const bool have_points     = this_obs_points->insertObservationPtr(o);

    // Store for next step:
    // auto last_obs_tim   = state_.last_obs_tim;
    auto last_obs       = state_.last_obs;
    auto last_points    = state_.last_points;
    state_.last_obs     = o;
    state_.last_obs_tim = this_obs_tim;
    state_.last_points  = this_obs_points;

    // First time we cannot do ICP since we need at least two pointclouds:
    if (!last_points)
    {
        MRPT_LOG_DEBUG("First pointcloud: skipping ICP.");
        return;
    }

    if (!have_points)
    {
        MRPT_LOG_WARN_STREAM(
            "Observation of type `"
            << o->GetRuntimeClass()->className
            << "` could not be converted into a pointcloud. Doing nothing.");
        return;
    }

    // Register point clouds using any of the available ICP algorithms:
    mrpt::poses::CPose3DPDFGaussian initial_guess;
    mrpt::slam::CICP::TReturnInfo   ret_info;
    // Call ICP:
    auto rel_pose_pdf = state_.mrpt_icp.Align3DPDF(
        last_points.get(), this_obs_points.get(), initial_guess,
        nullptr /*running_time*/, &ret_info);

    const mrpt::poses::CPose3D rel_pose = rel_pose_pdf->getMeanVal();

    MRPT_LOG_DEBUG_FMT(
        "MRPT ICP done: goodness=%.03f rel_pose=%s", ret_info.goodness,
        rel_pose.asString().c_str());

    MRPT_TRY_END
}
