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
    params_.mrpt_icp.corresponding_points_decimation = 20;
    params_.mrpt_icp.maxIterations                   = 50;
    params_.mrpt_icp.skip_cov_calculation            = true;
    params_.mrpt_icp.thresholdDist                   = 1.25;
    params_.mrpt_icp.thresholdAng                    = mrpt::DEG2RAD(1.0);
    params_.mrpt_icp.ALFA                            = 0.01;
}
void LidarICP::spinOnce()
{
    MRPT_TRY_START

    ProfilerEntry tleg(profiler_, "spinOnce");

    //
    MRPT_TRY_END
}

void LidarICP::reset() { state_ = MethodState(); }

void LidarICP::onNewObservation(CObservation::Ptr& o)
{
    MRPT_TRY_START
    ProfilerEntry tleg(profiler_, "onNewObservation");

    // Only process "my" sensor source:
    ASSERT_(o);
    if (o->sensorLabel != raw_sensor_label_) return;

    const auto queued = worker_pool_.pendingTasks();
    if (queued > 1)
    {
        MRPT_LOG_THROTTLE_WARN(
            5.0, "Dropping observation due to worker threads too busy.");
        return;
    }

    // Enqueue task:
    worker_pool_.enqueue(&LidarICP::doProcessNewObservation, this, o);

    MRPT_TRY_END
}

// here happens the main stuff:
void LidarICP::doProcessNewObservation(CObservation::Ptr& o)
{
    MRPT_TRY_START
    ASSERT_(o);

    ProfilerEntry tleg(profiler_, "doProcessNewObservation");

    // Only process pointclouds that are sufficiently apart in time:
    const auto this_obs_tim = o->timestamp;
    if (state_.last_obs_tim != mrpt::Clock::time_point() &&
        mrpt::system::timeDifference(state_.last_obs_tim, this_obs_tim) <
            params_.min_time_between_scans_)
    {
        // Drop observation.
        return;
    }

    profiler_.enter("doProcessNewObservation.obs2pointcloud");

    auto       this_obs_points = mrpt::maps::CSimplePointsMap::Create();
    const bool have_points     = this_obs_points->insertObservationPtr(o);

    profiler_.leave("doProcessNewObservation.obs2pointcloud");

    // Store for next step:
    auto last_obs_tim   = state_.last_obs_tim;
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
    // Use velocity model for the initial guess:
    double dt = .0;
    if (last_obs_tim != mrpt::Clock::time_point())
        dt = mrpt::system::timeDifference(last_obs_tim, this_obs_tim);
    initial_guess.mean.setFromValues(
        state_.last_iter_twist.vx * dt, state_.last_iter_twist.vy * dt,
        state_.last_iter_twist.vz * dt);
    MRPT_TODO("do omega_xyz part!");

    mrpt::slam::CICP::TReturnInfo ret_info;
    // Call ICP:
    state_.mrpt_icp.options = params_.mrpt_icp;
    auto rel_pose_pdf       = state_.mrpt_icp.Align3DPDF(
        last_points.get(), this_obs_points.get(), initial_guess,
        nullptr /*running_time*/, &ret_info);

    const mrpt::poses::CPose3D rel_pose = rel_pose_pdf->getMeanVal();

    // Update velocity model:
    state_.last_iter_twist.vx = rel_pose.x() / dt;
    state_.last_iter_twist.vy = rel_pose.y() / dt;
    state_.last_iter_twist.vz = rel_pose.z() / dt;
    MRPT_TODO("do omega_xyz part!");

    MRPT_LOG_DEBUG_FMT(
        "MRPT ICP: goodness=%.03f iters=%u", ret_info.goodness,
        ret_info.nIterations);

    MRPT_LOG_DEBUG_STREAM("ICP rel_pose=" << rel_pose.asString());
    MRPT_LOG_DEBUG_STREAM("Est.twist=" << state_.last_iter_twist.asString());
    MRPT_LOG_DEBUG_STREAM(
        "Time since last scan=" << mrpt::system::formatTimeInterval(dt));

    MRPT_TRY_END
}
